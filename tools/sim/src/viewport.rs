//! The 3D view: an offscreen wgpu render (colour + depth) that egui
//! shows as an image. Z is up, the light is fixed relative to the
//! world, every brick instance is drawn through one storage-buffer
//! slot holding its model matrix and colour.

use crate::bundle::MeshData;
use eframe::egui_wgpu::{self, wgpu};
use glam::{Mat4, Vec3};
use std::collections::HashMap;

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    pos: [f32; 3],
    nrm: [f32; 3],
    uv: [f32; 2],
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct LineVertex {
    pos: [f32; 3],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    light_dir: [f32; 4],
    camera_pos: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct InstanceData {
    model: [[f32; 4]; 4],
    color: [f32; 4],
    flags: [u32; 4],
}

pub struct GpuMesh {
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    index_count: u32,
    pub bbox: (Vec3, Vec3),
}

/// One instance to draw: which mesh, where, what colour, and which
/// texture (none = flat colour).
#[derive(Clone, Debug)]
pub struct DrawItem {
    pub mesh: String,
    pub model: Mat4,
    pub color: [f32; 4],
    pub texture: Option<String>,
}

struct GpuTexture {
    bind_group: wgpu::BindGroup,
}

/// A line segment in world space (grid, axes, markers).
pub struct Line {
    pub a: Vec3,
    pub b: Vec3,
    pub color: [f32; 4],
}

/// Where the ghosts render before they are blended over the frame.
struct GhostTargets {
    color: wgpu::TextureView,
    depth: wgpu::TextureView,
    bind_group: wgpu::BindGroup,
}

/// Everything one frame draws: the scene, its lines, translucent ghost
/// items blended over it (a chassis where a drag would put it), and
/// overlay items (handles) drawn on top of everything with a fresh
/// depth buffer.
#[derive(Default)]
pub struct Scene<'a> {
    pub items: &'a [DrawItem],
    pub lines: &'a [Line],
    pub ghost: &'a [DrawItem],
    pub overlay: &'a [DrawItem],
    pub background: [f64; 4],
}

/// A `0xRRGGBB` colour as linear RGBA.
pub fn srgb(hex: u32) -> [f32; 4] {
    let c = |v: u32| ((v & 0xff) as f32 / 255.0).powf(2.2);
    [c(hex >> 16), c(hex >> 8), c(hex), 1.0]
}

#[derive(Clone, Debug)]
pub struct Camera {
    pub target: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub fov_deg: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Camera {
            target: Vec3::ZERO,
            yaw: -128.0,
            pitch: 28.0,
            distance: 400.0,
            fov_deg: 38.0,
        }
    }
}

impl Camera {
    pub fn direction(&self) -> Vec3 {
        let (y, p) = (self.yaw.to_radians(), self.pitch.to_radians());
        Vec3::new(p.cos() * y.cos(), p.cos() * y.sin(), p.sin())
    }
    pub fn eye(&self) -> Vec3 {
        self.target + self.direction() * self.distance
    }
    pub fn view(&self) -> Mat4 {
        glam::camera::rh::view::look_at_mat4(self.eye(), self.target, Vec3::Z)
    }
    pub fn proj(&self, aspect: f32) -> Mat4 {
        let near = (self.distance / 200.0).max(0.5);
        glam::camera::rh::proj::directx::perspective(self.fov_deg.to_radians(), aspect.max(0.01), near, self.distance * 40.0 + 1000.0)
    }
    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        self.proj(aspect) * self.view()
    }
    pub fn right(&self) -> Vec3 {
        Vec3::Z.cross(self.direction()).normalize_or_zero()
    }
    pub fn up(&self) -> Vec3 {
        self.direction().cross(self.right()).normalize_or_zero()
    }
    /// Frame a bounding box: look at its centre from the current angles.
    pub fn fit(&mut self, min: Vec3, max: Vec3) {
        self.target = (min + max) * 0.5;
        let diag = (max - min).length().max(40.0);
        self.distance = diag / (2.0 * (self.fov_deg.to_radians() / 2.0).tan()) * 1.25;
    }
    /// The pixel of a `w × h` view a world point lands on; None behind the eye.
    pub fn project(&self, p: Vec3, w: f32, h: f32) -> Option<glam::Vec2> {
        let clip = self.view_proj(w / h) * p.extend(1.0);
        if clip.w <= 1e-6 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        Some(glam::Vec2::new((ndc.x + 1.0) * 0.5 * w, (1.0 - ndc.y) * 0.5 * h))
    }
    /// A world-space ray through a pixel of a `w × h` view.
    pub fn ray(&self, px: f32, py: f32, w: f32, h: f32) -> (Vec3, Vec3) {
        let inv = self.view_proj(w / h).inverse();
        let ndc = |z: f32| glam::Vec4::new(px / w * 2.0 - 1.0, 1.0 - py / h * 2.0, z, 1.0);
        let near = inv * ndc(0.0);
        let far = inv * ndc(1.0);
        let near = near.truncate() / near.w;
        let far = far.truncate() / far.w;
        (near, (far - near).normalize_or_zero())
    }
}

/// Where a ray meets the plane z = z0 (None when parallel).
pub fn ray_plane_z(origin: Vec3, dir: Vec3, z0: f32) -> Option<Vec3> {
    if dir.z.abs() < 1e-6 {
        return None;
    }
    let t = (z0 - origin.z) / dir.z;
    if t < 0.0 {
        return None;
    }
    Some(origin + dir * t)
}

/// Ray against an oriented box (the mesh bbox under `model`): the hit distance.
pub fn ray_obb(origin: Vec3, dir: Vec3, model: &Mat4, bbox: (Vec3, Vec3)) -> Option<f32> {
    let inv = model.inverse();
    let o = inv.transform_point3(origin);
    let d = inv.transform_vector3(dir);
    let mut tmin = 0.0f32;
    let mut tmax = f32::INFINITY;
    for i in 0..3 {
        let (lo, hi) = (bbox.0[i], bbox.1[i]);
        if d[i].abs() < 1e-9 {
            if o[i] < lo || o[i] > hi {
                return None;
            }
        } else {
            let t1 = (lo - o[i]) / d[i];
            let t2 = (hi - o[i]) / d[i];
            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
            if tmin > tmax {
                return None;
            }
        }
    }
    Some(tmin)
}

pub struct Viewport {
    pipeline: wgpu::RenderPipeline,
    /// Lays the ghost image over the frame where the ghost is nearer than the scene.
    composite_pipeline: wgpu::RenderPipeline,
    composite_layout: wgpu::BindGroupLayout,
    line_pipeline: wgpu::RenderPipeline,
    globals: wgpu::Buffer,
    globals_bg: wgpu::BindGroup,
    instance_layout: wgpu::BindGroupLayout,
    instance_buf: wgpu::Buffer,
    instance_bg: wgpu::BindGroup,
    instance_capacity: usize,
    line_buf: wgpu::Buffer,
    line_capacity: usize,
    meshes: HashMap<String, GpuMesh>,
    texture_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    textures: HashMap<String, GpuTexture>,
    color: Option<wgpu::TextureView>,
    color_tex: Option<wgpu::Texture>,
    depth: Option<wgpu::TextureView>,
    /// The ghosts' own colour and depth, composited over the frame.
    ghost_color: Option<wgpu::TextureView>,
    ghost_depth: Option<wgpu::TextureView>,
    composite_bg: Option<wgpu::BindGroup>,
    size: (u32, u32),
    tex_id: Option<eframe::egui::TextureId>,
    /// Small renders by key (library thumbnails), kept alive for egui.
    thumbs: HashMap<String, (wgpu::Texture, eframe::egui::TextureId)>,
    pub camera: Camera,
}

const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

impl Viewport {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("bricks"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let globals_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("globals"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let instance_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("instances"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let texture_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("texture"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("bricks"),
            bind_group_layouts: &[Some(&globals_layout), Some(&instance_layout), Some(&texture_layout)],
            ..Default::default()
        });
        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 0,
                    shader_location: 0,
                },
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 12,
                    shader_location: 1,
                },
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x2,
                    offset: 24,
                    shader_location: 2,
                },
            ],
        };
        let depth_state = wgpu::DepthStencilState {
            format: DEPTH,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: Default::default(),
            bias: Default::default(),
        };
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("bricks"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: std::slice::from_ref(&vertex_layout),
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(FORMAT.into())],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(depth_state.clone()),
            multisample: Default::default(),
            multiview_mask: None,
            cache: None,
        });
        // the ghost composite: a screen triangle that blends the ghost image in wherever the
        // ghost is nearer than the scene (the ghost's own depth buffer resolved its faces)
        let composite_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ghost composite"),
            source: wgpu::ShaderSource::Wgsl(COMPOSITE_SHADER.into()),
        });
        let composite_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("ghost composite"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Depth,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Depth,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
            ],
        });
        let composite_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("ghost composite"),
            bind_group_layouts: &[Some(&composite_layout)],
            ..Default::default()
        });
        let composite_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("ghost composite"),
            layout: Some(&composite_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &composite_shader,
                entry_point: Some("vs_composite"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &composite_shader,
                entry_point: Some("fs_composite"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: FORMAT,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            multiview_mask: None,
            cache: None,
        });
        let line_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("lines"),
            bind_group_layouts: &[Some(&globals_layout)],
            ..Default::default()
        });
        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("lines"),
            layout: Some(&line_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_line"),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<LineVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x3,
                            offset: 0,
                            shader_location: 0,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x4,
                            offset: 12,
                            shader_location: 1,
                        },
                    ],
                }],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_line"),
                targets: &[Some(FORMAT.into())],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                depth_write_enabled: Some(false),
                ..depth_state
            }),
            multisample: Default::default(),
            multiview_mask: None,
            cache: None,
        });
        let globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("globals"),
            size: std::mem::size_of::<Globals>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let globals_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("globals"),
            layout: &globals_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: globals.as_entire_binding(),
            }],
        });
        let instance_capacity = 256;
        let instance_buf = Self::instance_buffer(device, instance_capacity);
        let instance_bg = Self::instance_bind_group(device, &instance_layout, &instance_buf);
        let line_capacity = 4096;
        let line_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("lines"),
            size: (line_capacity * std::mem::size_of::<LineVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("texture"),
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            ..Default::default()
        });
        let mut vp = Viewport {
            pipeline,
            composite_pipeline,
            composite_layout,
            line_pipeline,
            globals,
            globals_bg,
            instance_layout,
            instance_buf,
            instance_bg,
            instance_capacity,
            line_buf,
            line_capacity,
            meshes: HashMap::new(),
            color: None,
            color_tex: None,
            depth: None,
            ghost_color: None,
            ghost_depth: None,
            composite_bg: None,
            size: (0, 0),
            tex_id: None,
            thumbs: HashMap::new(),
            texture_layout,
            sampler,
            textures: HashMap::new(),
            camera: Camera::default(),
        };
        vp.add_texture(device, queue, "white", 1, 1, &[255, 255, 255, 255]);
        vp
    }

    /// Upload an RGBA8 image as a texture the draw items can name.
    pub fn add_texture(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, key: &str, width: u32, height: u32, rgba: &[u8]) {
        let size = wgpu::Extent3d {
            width: width.max(1),
            height: height.max(1),
            depth_or_array_layers: 1,
        };
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some(key),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            rgba,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(4 * size.width),
                rows_per_image: Some(size.height),
            },
            size,
        );
        let view = texture.create_view(&Default::default());
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(key),
            layout: &self.texture_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
            ],
        });
        self.textures.insert(key.to_string(), GpuTexture { bind_group });
    }

    fn instance_buffer(device: &wgpu::Device, capacity: usize) -> wgpu::Buffer {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("instances"),
            size: (capacity * std::mem::size_of::<InstanceData>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    }

    fn instance_bind_group(device: &wgpu::Device, layout: &wgpu::BindGroupLayout, buf: &wgpu::Buffer) -> wgpu::BindGroup {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("instances"),
            layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: buf.as_entire_binding(),
            }],
        })
    }

    pub fn has_mesh(&self, key: &str) -> bool {
        self.meshes.contains_key(key)
    }

    pub fn add_mesh(&mut self, device: &wgpu::Device, key: &str, data: &MeshData) {
        use wgpu::util::DeviceExt;
        let verts: Vec<Vertex> = data
            .positions
            .iter()
            .zip(&data.normals)
            .enumerate()
            .map(|(i, (p, n))| Vertex {
                pos: *p,
                nrm: *n,
                uv: data.uvs.get(i).copied().unwrap_or([0.0, 0.0]),
            })
            .collect();
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = Vec3::splat(f32::NEG_INFINITY);
        for p in &data.positions {
            lo = lo.min(Vec3::from(*p));
            hi = hi.max(Vec3::from(*p));
        }
        if verts.is_empty() {
            lo = Vec3::ZERO;
            hi = Vec3::ZERO;
        }
        let vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(key),
            contents: bytemuck::cast_slice(&verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let ibuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(key),
            contents: bytemuck::cast_slice(&data.indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.meshes.insert(
            key.to_string(),
            GpuMesh {
                vbuf,
                ibuf,
                index_count: data.indices.len() as u32,
                bbox: (lo, hi),
            },
        );
    }

    fn ensure_targets(&mut self, device: &wgpu::Device, renderer: &mut egui_wgpu::Renderer, size: (u32, u32)) {
        if self.size == size && self.tex_id.is_some() {
            return;
        }
        let desc = |format, usage| wgpu::TextureDescriptor {
            label: Some("viewport"),
            size: wgpu::Extent3d {
                width: size.0.max(1),
                height: size.1.max(1),
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage,
            view_formats: &[],
        };
        let color = device.create_texture(&desc(
            FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_SRC,
        ));
        let bindable = wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING;
        let depth = device.create_texture(&desc(DEPTH, bindable));
        let ghost_color = device.create_texture(&desc(FORMAT, bindable));
        let ghost_depth = device.create_texture(&desc(DEPTH, bindable));
        let color_view = color.create_view(&Default::default());
        let depth_view = depth.create_view(&Default::default());
        let ghost_color_view = ghost_color.create_view(&Default::default());
        let ghost_depth_view = ghost_depth.create_view(&Default::default());
        match self.tex_id {
            Some(id) => renderer.update_egui_texture_from_wgpu_texture(device, &color_view, wgpu::FilterMode::Linear, id),
            None => self.tex_id = Some(renderer.register_native_texture(device, &color_view, wgpu::FilterMode::Linear)),
        }
        self.composite_bg = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("ghost composite"),
            layout: &self.composite_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&ghost_color_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&ghost_depth_view),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(&depth_view),
                },
            ],
        }));
        self.color = Some(color_view);
        self.color_tex = Some(color);
        self.depth = Some(depth_view);
        self.ghost_color = Some(ghost_color_view);
        self.ghost_depth = Some(ghost_depth_view);
        self.size = size;
    }

    /// The last frame as tightly packed RGBA8 rows, top to bottom;
    /// None before the first render.
    #[cfg(test)]
    pub fn read_pixels(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> Option<(u32, u32, Vec<u8>)> {
        let texture = self.color_tex.as_ref()?;
        let (w, h) = self.size;
        Self::read_texture(device, queue, texture, w, h)
    }

    /// A thumbnail's pixels, for tests.
    #[cfg(test)]
    pub fn read_thumb(&self, device: &wgpu::Device, queue: &wgpu::Queue, key: &str) -> Option<(u32, u32, Vec<u8>)> {
        let (texture, _) = self.thumbs.get(key)?;
        let size = texture.size();
        Self::read_texture(device, queue, texture, size.width, size.height)
    }

    #[cfg(test)]
    fn read_texture(device: &wgpu::Device, queue: &wgpu::Queue, texture: &wgpu::Texture, w: u32, h: u32) -> Option<(u32, u32, Vec<u8>)> {
        let row = (4 * w).div_ceil(256) * 256;
        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: (row * h) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("readback") });
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &buf,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(row),
                    rows_per_image: Some(h),
                },
            },
            wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
        );
        queue.submit(Some(encoder.finish()));
        let slice = buf.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        device.poll(wgpu::PollType::wait_indefinitely()).ok()?;
        rx.recv().ok()?.ok()?;
        let data = slice.get_mapped_range();
        let mut out = Vec::with_capacity((4 * w * h) as usize);
        for y in 0..h {
            let s = (y * row) as usize;
            out.extend_from_slice(&data[s..s + (4 * w) as usize]);
        }
        drop(data);
        buf.unmap();
        Some((w, h, out))
    }

    /// Draw the scene into the offscreen target; returns the egui texture to show.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        renderer: &mut egui_wgpu::Renderer,
        size: (u32, u32),
        scene: &Scene<'_>,
    ) -> eframe::egui::TextureId {
        self.ensure_targets(device, renderer, size);
        let (color, depth) = (self.color.clone().unwrap(), self.depth.clone().unwrap());
        let ghost = GhostTargets {
            color: self.ghost_color.clone().unwrap(),
            depth: self.ghost_depth.clone().unwrap(),
            bind_group: self.composite_bg.clone().unwrap(),
        };
        let camera = self.camera.clone();
        self.render_to(device, queue, &color, &depth, size, &camera, scene, Some(&ghost));
        self.tex_id.unwrap()
    }

    /// Render `items` alone into a small square texture the library can
    /// show, framed from the usual angle; the texture stays alive under
    /// `key` and the same key returns the same image.
    #[allow(clippy::too_many_arguments)]
    pub fn thumbnail(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        renderer: &mut egui_wgpu::Renderer,
        key: &str,
        items: &[DrawItem],
        bbox: (Vec3, Vec3),
        size: u32,
        background: [f64; 4],
    ) -> eframe::egui::TextureId {
        if let Some((_, id)) = self.thumbs.get(key) {
            return *id;
        }
        let desc = |format, usage| wgpu::TextureDescriptor {
            label: Some("thumbnail"),
            size: wgpu::Extent3d {
                width: size.max(1),
                height: size.max(1),
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage,
            view_formats: &[],
        };
        let color = device.create_texture(&desc(
            FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_SRC,
        ));
        let depth = device.create_texture(&desc(DEPTH, wgpu::TextureUsages::RENDER_ATTACHMENT));
        let color_view = color.create_view(&Default::default());
        let depth_view = depth.create_view(&Default::default());
        let mut camera = Camera::default();
        camera.fit(bbox.0, bbox.1);
        camera.distance *= 0.8;
        let scene = Scene {
            items,
            lines: &[],
            ghost: &[],
            overlay: &[],
            background,
        };
        self.render_to(device, queue, &color_view, &depth_view, (size, size), &camera, &scene, None);
        let id = renderer.register_native_texture(device, &color_view, wgpu::FilterMode::Linear);
        self.thumbs.insert(key.to_string(), (color, id));
        id
    }

    pub fn thumb(&self, key: &str) -> Option<eframe::egui::TextureId> {
        self.thumbs.get(key).map(|t| t.1)
    }

    /// Drop the thumbnails whose key `keep` rejects (stale components).
    pub fn retain_thumbs(&mut self, renderer: &mut egui_wgpu::Renderer, keep: impl Fn(&str) -> bool) {
        let gone: Vec<String> = self.thumbs.keys().filter(|k| !keep(k)).cloned().collect();
        for k in gone {
            if let Some((_, id)) = self.thumbs.remove(&k) {
                renderer.free_texture(&id);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::too_many_arguments)]
    fn render_to(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        size: (u32, u32),
        camera: &Camera,
        scene: &Scene<'_>,
        ghost: Option<&GhostTargets>,
    ) {
        let (items, lines, background) = (scene.items, scene.lines, scene.background);
        let aspect = size.0 as f32 / size.1.max(1) as f32;
        let eye = camera.eye();
        let light = (camera.direction() + Vec3::Z * 0.8 + camera.right() * 0.3).normalize();
        let globals = Globals {
            view_proj: camera.view_proj(aspect).to_cols_array_2d(),
            light_dir: [light.x, light.y, light.z, 0.0],
            camera_pos: [eye.x, eye.y, eye.z, 1.0],
        };
        queue.write_buffer(&self.globals, 0, bytemuck::bytes_of(&globals));
        let data: Vec<InstanceData> = items
            .iter()
            .chain(scene.overlay.iter())
            .chain(scene.ghost.iter())
            .map(|it| InstanceData {
                model: it.model.to_cols_array_2d(),
                color: it.color,
                flags: [
                    if it.texture.as_deref().map(|t| self.textures.contains_key(t)).unwrap_or(false) {
                        1
                    } else {
                        0
                    },
                    0,
                    0,
                    0,
                ],
            })
            .collect();
        if data.len() > self.instance_capacity {
            self.instance_capacity = data.len().next_power_of_two();
            self.instance_buf = Self::instance_buffer(device, self.instance_capacity);
            self.instance_bg = Self::instance_bind_group(device, &self.instance_layout, &self.instance_buf);
        }
        if !data.is_empty() {
            queue.write_buffer(&self.instance_buf, 0, bytemuck::cast_slice(&data));
        }
        let mut lv: Vec<LineVertex> = Vec::with_capacity(lines.len() * 2);
        for l in lines {
            lv.push(LineVertex {
                pos: l.a.to_array(),
                color: l.color,
            });
            lv.push(LineVertex {
                pos: l.b.to_array(),
                color: l.color,
            });
        }
        if lv.len() > self.line_capacity {
            self.line_capacity = lv.len().next_power_of_two();
            self.line_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("lines"),
                size: (self.line_capacity * std::mem::size_of::<LineVertex>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
        }
        if !lv.is_empty() {
            queue.write_buffer(&self.line_buf, 0, bytemuck::cast_slice(&lv));
        }
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("viewport") });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: color,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: background[0],
                            g: background[1],
                            b: background[2],
                            a: background[3],
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            pass.set_bind_group(0, &self.globals_bg, &[]);
            if !lv.is_empty() {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, self.line_buf.slice(..));
                pass.draw(0..lv.len() as u32, 0..1);
            }
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(1, &self.instance_bg, &[]);
            let white = &self.textures["white"].bind_group;
            for (i, it) in items.iter().enumerate() {
                let tex = it
                    .texture
                    .as_deref()
                    .and_then(|t| self.textures.get(t))
                    .map(|t| &t.bind_group)
                    .unwrap_or(white);
                pass.set_bind_group(2, tex, &[]);
                let Some(mesh) = self.meshes.get(&it.mesh) else { continue };
                if mesh.index_count == 0 {
                    continue;
                }
                pass.set_vertex_buffer(0, mesh.vbuf.slice(..));
                pass.set_index_buffer(mesh.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.index_count, 0, i as u32..i as u32 + 1);
            }
        }
        if let (false, Some(gt)) = (scene.ghost.is_empty(), ghost) {
            // the ghosts, opaque, into their own colour and depth (so a ghost's nearest
            // surface wins over its back faces) ...
            {
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("ghost"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &gt.color,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &gt.depth,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(1.0),
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
                pass.set_pipeline(&self.pipeline);
                pass.set_bind_group(0, &self.globals_bg, &[]);
                pass.set_bind_group(1, &self.instance_bg, &[]);
                pass.set_bind_group(2, &self.textures["white"].bind_group, &[]);
                for (k, it) in scene.ghost.iter().enumerate() {
                    let Some(mesh) = self.meshes.get(&it.mesh) else { continue };
                    if mesh.index_count == 0 {
                        continue;
                    }
                    let i = (items.len() + scene.overlay.len() + k) as u32;
                    pass.set_vertex_buffer(0, mesh.vbuf.slice(..));
                    pass.set_index_buffer(mesh.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.index_count, 0, i..i + 1);
                }
            }
            // ... then blended over the frame by their alpha, where the scene is not nearer
            {
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("ghost composite"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: color,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: None,
                    ..Default::default()
                });
                pass.set_pipeline(&self.composite_pipeline);
                pass.set_bind_group(0, &gt.bind_group, &[]);
                pass.draw(0..3, 0..1);
            }
        }
        if !scene.overlay.is_empty() {
            // handles: on top of everything, but still occluding each other
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("overlay"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: color,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.globals_bg, &[]);
            pass.set_bind_group(1, &self.instance_bg, &[]);
            pass.set_bind_group(2, &self.textures["white"].bind_group, &[]);
            for (k, it) in scene.overlay.iter().enumerate() {
                let Some(mesh) = self.meshes.get(&it.mesh) else { continue };
                if mesh.index_count == 0 {
                    continue;
                }
                let i = (items.len() + k) as u32;
                pass.set_vertex_buffer(0, mesh.vbuf.slice(..));
                pass.set_index_buffer(mesh.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.index_count, 0, i..i + 1);
            }
        }
        queue.submit(Some(encoder.finish()));
    }

    /// The nearest item under a pixel, by oriented bounding box.
    pub fn pick(&self, items: &[DrawItem], px: f32, py: f32) -> Option<usize> {
        let (w, h) = (self.size.0 as f32, self.size.1 as f32);
        let (o, d) = self.camera.ray(px, py, w, h);
        let mut best: Option<(f32, usize)> = None;
        for (i, it) in items.iter().enumerate() {
            let Some(m) = self.meshes.get(&it.mesh) else { continue };
            if let Some(t) = ray_obb(o, d, &it.model, m.bbox)
                && best.map(|b| t < b.0).unwrap_or(true)
            {
                best = Some((t, i));
            }
        }
        best.map(|b| b.1)
    }
}

const SHADER: &str = r#"
struct Globals { view_proj: mat4x4<f32>, light_dir: vec4<f32>, camera_pos: vec4<f32> };
struct Inst { model: mat4x4<f32>, color: vec4<f32>, flags: vec4<u32> };
@group(0) @binding(0) var<uniform> g: Globals;
@group(1) @binding(0) var<storage, read> insts: array<Inst>;
@group(2) @binding(0) var tex: texture_2d<f32>;
@group(2) @binding(1) var samp: sampler;

struct VOut { @builtin(position) pos: vec4<f32>, @location(0) nrm: vec3<f32>, @location(1) color: vec4<f32>, @location(2) wpos: vec3<f32>, @location(3) uv: vec2<f32>, @location(4) @interpolate(flat) textured: u32 };

@vertex fn vs_main(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>, @location(2) uv: vec2<f32>, @builtin(instance_index) ii: u32) -> VOut {
  let inst = insts[ii];
  let wp = inst.model * vec4<f32>(p, 1.0);
  var o: VOut;
  o.pos = g.view_proj * wp;
  o.nrm = normalize((inst.model * vec4<f32>(n, 0.0)).xyz);
  o.color = inst.color;
  o.wpos = wp.xyz;
  o.uv = uv;
  o.textured = inst.flags.x;
  return o;
}

@fragment fn fs_main(i: VOut) -> @location(0) vec4<f32> {
  let n0 = normalize(i.nrm);
  let v = normalize(g.camera_pos.xyz - i.wpos);
  let n = select(n0, -n0, dot(n0, v) < 0.0);
  let l = normalize(g.light_dir.xyz);
  let diff = max(dot(n, l), 0.0);
  let sky = max(n.z, 0.0);
  let k = 0.32 + 0.48 * diff + 0.2 * sky;
  var base = i.color;
  if (i.textured != 0u) {
    base = base * textureSample(tex, samp, i.uv);
  }
  return vec4<f32>(base.rgb * k, base.a);
}

struct LOut { @builtin(position) pos: vec4<f32>, @location(0) color: vec4<f32> };
@vertex fn vs_line(@location(0) p: vec3<f32>, @location(1) c: vec4<f32>) -> LOut {
  var o: LOut;
  o.pos = g.view_proj * vec4<f32>(p, 1.0);
  o.color = c;
  return o;
}
@fragment fn fs_line(i: LOut) -> @location(0) vec4<f32> { return i.color; }
"#;

/// The ghost composite: one triangle over the screen; each pixel takes
/// the ghost image's colour and alpha where the ghost drew and is not
/// behind the scene.
const COMPOSITE_SHADER: &str = r#"
@group(0) @binding(0) var ghost_color: texture_2d<f32>;
@group(0) @binding(1) var ghost_depth: texture_depth_2d;
@group(0) @binding(2) var scene_depth: texture_depth_2d;

struct COut { @builtin(position) pos: vec4<f32> };
@vertex fn vs_composite(@builtin(vertex_index) vi: u32) -> COut {
  var o: COut;
  let x = f32(i32(vi & 1u) * 4 - 1);
  let y = f32(i32(vi >> 1u) * 4 - 1);
  o.pos = vec4<f32>(x, y, 0.0, 1.0);
  return o;
}
@fragment fn fs_composite(i: COut) -> @location(0) vec4<f32> {
  let xy = vec2<i32>(i.pos.xy);
  let g = textureLoad(ghost_color, xy, 0);
  if (g.a <= 0.0) { discard; }
  if (textureLoad(ghost_depth, xy, 0) > textureLoad(scene_depth, xy, 0)) { discard; }
  return g;
}
"#;

/// Offscreen GPU access for tests in every module.
#[cfg(test)]
pub mod testing {
    use super::*;

    /// A GPU device for offscreen tests: any adapter (CI's Linux leg
    /// installs mesa's lavapipe). Without one the test is skipped on a
    /// developer machine but fails on CI, so a missing driver cannot
    /// silently turn the test off there.
    pub fn test_device() -> Option<(wgpu::Device, wgpu::Queue)> {
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()));
        let adapter = match adapter {
            Ok(a) => a,
            Err(e) => {
                assert!(
                    std::env::var_os("CI").is_none(),
                    "CI must provide a GPU adapter for the render test: {e}"
                );
                eprintln!("no GPU adapter ({e}): skipping the offscreen render test");
                return None;
            }
        };
        let info = adapter.get_info();
        eprintln!(
            "GPU adapter: {} ({:?}, {:?}) driver {} {}",
            info.name, info.backend, info.device_type, info.driver, info.driver_info
        );
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).expect("a device");
        Some((device, queue))
    }

    /// An egui renderer for the offscreen colour format.
    pub fn test_renderer(device: &wgpu::Device) -> egui_wgpu::Renderer {
        egui_wgpu::Renderer::new(device, FORMAT, egui_wgpu::RendererOptions::default())
    }
}

#[cfg(test)]
mod tests {
    use super::testing::*;
    use super::*;
    use crate::geometry;
    use crate::gizmo::{self, Gizmo, Handle, Mode};

    #[test]
    fn renders_bricks_and_handles_offscreen() {
        let Some((device, queue)) = test_device() else { return };
        let mut renderer = test_renderer(&device);
        let mut vp = Viewport::new(&device, &queue);
        vp.camera = Camera {
            target: Vec3::ZERO,
            yaw: 0.0,
            pitch: 0.0,
            distance: 500.0,
            fov_deg: 38.0,
        };
        let (w, h) = (320u32, 240u32);
        vp.add_mesh(&device, "box", &geometry::box_mesh([100.0; 3], [0.0; 3]));
        vp.add_mesh(
            &device,
            gizmo::MESH_SHAFT,
            &geometry::cylinder_mesh(0.03, 0.8, "z", [0.0, 0.0, 0.4], 12),
        );
        vp.add_mesh(&device, gizmo::MESH_CONE, &geometry::cone_mesh(0.05, 0.2, [0.0, 0.0, 0.8], 16));
        let items = [DrawItem {
            mesh: "box".into(),
            model: Mat4::IDENTITY,
            color: srgb(0x5B7A9C),
            texture: None,
        }];
        // the y arrow crosses the box's face; the z arrow is the lit one
        let g = Gizmo::new(Vec3::ZERO, Mode::Move, &vp.camera, h as f32);
        let overlay = g.draw(Some(Handle::Axis(2)));
        // a ghost box beside it, 70 % transparent: seen, but faint
        let ghost = [DrawItem {
            mesh: "box".into(),
            model: Mat4::from_translation(Vec3::new(0.0, 120.0, 0.0)),
            color: [srgb(0x5B7A9C)[0], srgb(0x5B7A9C)[1], srgb(0x5B7A9C)[2], 0.3],
            texture: None,
        }];
        let scene = Scene {
            items: &items,
            lines: &[],
            ghost: &ghost,
            overlay: &overlay,
            background: [0.0, 0.0, 0.0, 1.0],
        };
        vp.render(&device, &queue, &mut renderer, (w, h), &scene);
        let (rw, rh, px) = vp.read_pixels(&device, &queue).unwrap();
        assert_eq!((rw, rh), (w, h));
        if let Some(p) = std::env::var_os("OPENBRICKS_SIM_RENDER_PNG") {
            image::save_buffer(p, &px, w, h, image::ColorType::Rgba8).unwrap();
        }
        let at = |x: u32, y: u32| {
            let i = ((y * w + x) * 4) as usize;
            [px[i], px[i + 1], px[i + 2]]
        };
        let column = |x: u32, y0: u32, y1: u32| (y0..=y1).map(|y| at(x, y)).collect::<Vec<_>>();
        assert_eq!(at(10, 10), [0, 0, 0], "the corner is background");
        // the box: blue-grey, in front of everything but the handles
        let b = at(160, 140);
        assert!(b[2] > b[1] && b[2] > 40, "box pixel {b:?}");
        // the ghost: the same colour blended at 30 % over the background. Every pixel that
        // only the ghost box covers (black without it, the box's hue with it drawn solid) is
        // lit in the ghost render, and fainter than the solid box there.
        let sum = |c: [u8; 3]| c[0] as u32 + c[1] as u32 + c[2] as u32;
        let bare_scene = Scene {
            items: &items,
            lines: &[],
            ghost: &[],
            overlay: &overlay,
            background: [0.0, 0.0, 0.0, 1.0],
        };
        vp.render(&device, &queue, &mut renderer, (w, h), &bare_scene);
        let (_, _, bare_px) = vp.read_pixels(&device, &queue).unwrap();
        let solid_items: Vec<DrawItem> = items
            .iter()
            .cloned()
            .chain(ghost.iter().map(|g| DrawItem {
                color: [g.color[0], g.color[1], g.color[2], 1.0],
                ..g.clone()
            }))
            .collect();
        let solid_scene = Scene {
            items: &solid_items,
            lines: &[],
            ghost: &[],
            overlay: &overlay,
            background: [0.0, 0.0, 0.0, 1.0],
        };
        vp.render(&device, &queue, &mut renderer, (w, h), &solid_scene);
        let (_, _, solid_px) = vp.read_pixels(&device, &queue).unwrap();
        let pick = |buf: &[u8], x: u32, y: u32| {
            let i = ((y * w + x) * 4) as usize;
            [buf[i], buf[i + 1], buf[i + 2]]
        };
        // 30 % of the solid pixel, blended in linear light and stored as sRGB
        let expected = |s: u8| -> u8 {
            let lin = ((s as f64 / 255.0 + 0.055) / 1.055).powf(2.4) * 0.3;
            ((1.055 * lin.powf(1.0 / 2.4) - 0.055) * 255.0).round() as u8
        };
        // the report: how the ghost-only pixels come out at three alphas (a driver's blend
        // going wrong shows up here, with the adapter named above)
        let ghost_only: Vec<(u32, u32)> = (0..h)
            .flat_map(|y| (0..w).map(move |x| (x, y)))
            .filter(|&(x, y)| {
                let (bare, solid) = (pick(&bare_px, x, y), pick(&solid_px, x, y));
                sum(bare) == 0 && solid[2] > solid[1] && solid[2] > solid[0] && solid[2] > 40
            })
            .collect();
        let bbox = |pts: &[(u32, u32)]| {
            pts.iter().fold((u32::MAX, u32::MAX, 0u32, 0u32), |b, &(x, y)| {
                (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y))
            })
        };
        eprintln!("ghost-only pixels: {} in bbox {:?}", ghost_only.len(), bbox(&ghost_only));
        for alpha in [0.0f32, 0.3, 1.0] {
            let g = [DrawItem {
                color: [ghost[0].color[0], ghost[0].color[1], ghost[0].color[2], alpha],
                ..ghost[0].clone()
            }];
            let sc = Scene {
                items: &items,
                lines: &[],
                ghost: &g,
                overlay: &overlay,
                background: [0.0, 0.0, 0.0, 1.0],
            };
            vp.render(&device, &queue, &mut renderer, (w, h), &sc);
            let (_, _, gpx) = vp.read_pixels(&device, &queue).unwrap();
            let want = |s: u8| -> u8 {
                let lin = ((s as f64 / 255.0 + 0.055) / 1.055).powf(2.4) * alpha as f64;
                ((1.055 * lin.powf(1.0 / 2.4) - 0.055).max(0.0) * 255.0).round() as u8
            };
            let (mut black, mut as_expected, mut as_solid, mut other) = (0, 0, 0, 0);
            let mut samples = Vec::new();
            for &(x, y) in &ghost_only {
                let (gp, solid) = (pick(&gpx, x, y), pick(&solid_px, x, y));
                let exp = solid.map(want);
                let near = |a: [u8; 3], b: [u8; 3]| (0..3).all(|c| (a[c] as i32 - b[c] as i32).abs() <= 3);
                if sum(gp) == 0 {
                    black += 1;
                } else if near(gp, exp) {
                    as_expected += 1;
                } else if near(gp, solid) {
                    as_solid += 1;
                } else {
                    other += 1;
                    if samples.len() < 4 {
                        samples.push(((x, y), gp, exp, solid));
                    }
                }
            }
            let lit: Vec<(u32, u32)> = (0..h)
                .flat_map(|y| (0..w).map(move |x| (x, y)))
                .filter(|&(x, y)| sum(pick(&gpx, x, y)) != sum(pick(&bare_px, x, y)))
                .collect();
            eprintln!(
                "alpha {alpha}: black {black}, as expected {as_expected}, as solid {as_solid}, other {other} {samples:?}; the ghost changed {} pixels in bbox {:?}",
                lit.len(),
                bbox(&lit)
            );
        }
        let (mut compared, mut exact) = (0, 0);
        let mut off = Vec::new();
        for y in 0..h {
            for x in 0..w {
                let (bare, solid) = (pick(&bare_px, x, y), pick(&solid_px, x, y));
                if sum(bare) != 0 || !(solid[2] > solid[1] && solid[2] > solid[0] && solid[2] > 40) {
                    continue;
                }
                let gp = at(x, y);
                assert!(sum(gp) > 0, "the ghost is visible at ({x}, {y})");
                assert!(
                    sum(gp) * 10 < sum(solid) * 9,
                    "fainter than the solid box at ({x}, {y}): {gp:?} vs {solid:?}"
                );
                let want = solid.map(expected);
                if (0..3).all(|c| (gp[c] as i32 - want[c] as i32).abs() <= 3) {
                    exact += 1;
                } else if off.len() < 8 {
                    off.push(((x, y), gp, want));
                }
                compared += 1;
            }
        }
        assert!(compared > 200, "{compared} ghost-only pixels compared");
        // the box's edges, where two faces share a depth, may blend twice; everywhere else it is the 30 % blend
        assert!(
            exact * 20 >= compared * 19,
            "{exact} of {compared} pixels are the 30 % blend; off: {off:?}"
        );
        vp.render(&device, &queue, &mut renderer, (w, h), &scene);
        // the y arrow points right across the box and is drawn over it (green beats blue)
        assert!(
            column(200, 116, 124).iter().any(|c| c[1] > c[2] && c[1] > c[0] && c[1] > 40),
            "no green shaft over the box at x=200: {:?}",
            column(200, 116, 124)
        );
        // the lit z arrow's cone, above the box: amber (red over green over blue)
        let cone = cam_cone_row(&vp.camera, &g, w as f32, h as f32);
        assert!(
            column(160, cone - 3, cone + 3)
                .iter()
                .any(|c| c[0] > c[1] && c[1] > c[2] && c[0] > 120),
            "no amber cone at y={cone}: {:?}",
            column(160, cone - 3, cone + 3)
        );
    }

    fn cam_cone_row(cam: &Camera, g: &Gizmo, w: f32, h: f32) -> u32 {
        cam.project(g.center + Vec3::Z * g.length * 0.9, w, h).unwrap().y.round() as u32
    }

    #[test]
    fn thumbnails_frame_the_item_and_are_kept_by_key() {
        let Some((device, queue)) = test_device() else { return };
        let mut renderer = test_renderer(&device);
        let mut vp = Viewport::new(&device, &queue);
        vp.add_mesh(&device, "box", &geometry::box_mesh([40.0, 40.0, 40.0], [0.0; 3]));
        let items = [DrawItem {
            mesh: "box".into(),
            model: Mat4::IDENTITY,
            color: srgb(0x5B7A9C),
            texture: None,
        }];
        let bbox = (Vec3::splat(-20.0), Vec3::splat(20.0));
        let id = vp.thumbnail(&device, &queue, &mut renderer, "t:box", &items, bbox, 64, [0.0, 0.0, 0.0, 1.0]);
        assert!(vp.thumb("t:box").is_some());
        assert_eq!(
            vp.thumbnail(&device, &queue, &mut renderer, "t:box", &[], bbox, 64, [0.0; 4]),
            id,
            "the same key: the same image"
        );
        let (w, h, px) = vp.read_thumb(&device, &queue, "t:box").unwrap();
        assert_eq!((w, h), (64, 64));
        let at = |x: u32, y: u32| {
            let i = ((y * w + x) * 4) as usize;
            [px[i], px[i + 1], px[i + 2]]
        };
        assert_ne!(at(32, 32), [0, 0, 0], "the box fills the middle");
        assert_eq!(at(1, 1), [0, 0, 0], "the corner is background");
        assert!(vp.read_thumb(&device, &queue, "nope").is_none());
        vp.retain_thumbs(&mut renderer, |k| k != "t:box");
        assert!(vp.thumb("t:box").is_none());
        // the main view still renders after thumbnails used the shared buffers
        vp.render(
            &device,
            &queue,
            &mut renderer,
            (32, 24),
            &Scene {
                items: &items,
                lines: &[],
                ghost: &[],
                overlay: &[],
                background: [0.0; 4],
            },
        );
        let (_, _, px) = vp.read_pixels(&device, &queue).unwrap();
        assert_eq!(px.len(), 32 * 24 * 4);
    }

    #[test]
    fn camera_ray_passes_through_the_target_at_the_centre_pixel() {
        let cam = Camera {
            target: Vec3::new(10.0, 20.0, 30.0),
            ..Default::default()
        };
        let (o, d) = cam.ray(320.0, 240.0, 640.0, 480.0);
        let to_target = (cam.target - o).normalize();
        assert!((d - to_target).length() < 1e-3, "{d:?} vs {to_target:?}");
    }

    #[test]
    fn obb_and_plane_hits() {
        let model = Mat4::from_translation(Vec3::new(100.0, 0.0, 0.0));
        let bbox = (Vec3::splat(-5.0), Vec3::splat(5.0));
        let t = ray_obb(Vec3::new(0.0, 0.0, 0.0), Vec3::X, &model, bbox).unwrap();
        assert!((t - 95.0).abs() < 1e-4);
        assert!(ray_obb(Vec3::new(0.0, 20.0, 0.0), Vec3::X, &model, bbox).is_none());
        let p = ray_plane_z(Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, -1.0), 0.0).unwrap();
        assert_eq!(p, Vec3::ZERO);
        assert!(ray_plane_z(Vec3::ZERO, Vec3::X, 5.0).is_none());
    }

    #[test]
    fn project_is_the_inverse_of_ray() {
        let cam = Camera {
            target: Vec3::new(10.0, 20.0, 30.0),
            ..Default::default()
        };
        let centre = cam.project(cam.target, 640.0, 480.0).unwrap();
        assert!((centre - glam::Vec2::new(320.0, 240.0)).length() < 1e-2, "{centre:?}");
        let p = Vec3::new(-25.0, 40.0, 12.0);
        let px = cam.project(p, 640.0, 480.0).unwrap();
        let (o, d) = cam.ray(px.x, px.y, 640.0, 480.0);
        let along = (p - o).dot(d);
        assert!((o + d * along - p).length() < 1e-2);
        assert!(cam.project(cam.eye() + cam.direction() * 10.0, 640.0, 480.0).is_none());
        assert_eq!(srgb(0xFFFFFF), [1.0, 1.0, 1.0, 1.0]);
        assert_eq!(srgb(0x000000), [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn fit_frames_the_box() {
        let mut cam = Camera::default();
        cam.fit(Vec3::new(-50.0, -50.0, 0.0), Vec3::new(50.0, 50.0, 20.0));
        assert_eq!(cam.target, Vec3::new(0.0, 0.0, 10.0));
        assert!(cam.distance > 100.0);
    }
}
