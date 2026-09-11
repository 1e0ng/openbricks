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
}

pub struct GpuMesh {
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    index_count: u32,
    pub bbox: (Vec3, Vec3),
}

/// One brick instance to draw: which mesh, where, what colour.
pub struct DrawItem {
    pub mesh: String,
    pub model: Mat4,
    pub color: [f32; 4],
}

/// A line segment in world space (grid, axes, markers).
pub struct Line {
    pub a: Vec3,
    pub b: Vec3,
    pub color: [f32; 4],
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
    color: Option<wgpu::TextureView>,
    depth: Option<wgpu::TextureView>,
    size: (u32, u32),
    tex_id: Option<eframe::egui::TextureId>,
    pub camera: Camera,
}

const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

impl Viewport {
    pub fn new(device: &wgpu::Device) -> Self {
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
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("bricks"),
            bind_group_layouts: &[Some(&globals_layout), Some(&instance_layout)],
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
                buffers: &[vertex_layout],
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
        Viewport {
            pipeline,
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
            depth: None,
            size: (0, 0),
            tex_id: None,
            camera: Camera::default(),
        }
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
            .map(|(p, n)| Vertex { pos: *p, nrm: *n })
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
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        ));
        let depth = device.create_texture(&desc(DEPTH, wgpu::TextureUsages::RENDER_ATTACHMENT));
        let color_view = color.create_view(&Default::default());
        let depth_view = depth.create_view(&Default::default());
        match self.tex_id {
            Some(id) => renderer.update_egui_texture_from_wgpu_texture(device, &color_view, wgpu::FilterMode::Linear, id),
            None => self.tex_id = Some(renderer.register_native_texture(device, &color_view, wgpu::FilterMode::Linear)),
        }
        self.color = Some(color_view);
        self.depth = Some(depth_view);
        self.size = size;
    }

    /// Draw the scene into the offscreen target; returns the egui texture to show.
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        renderer: &mut egui_wgpu::Renderer,
        size: (u32, u32),
        items: &[DrawItem],
        lines: &[Line],
        background: [f64; 4],
    ) -> eframe::egui::TextureId {
        self.ensure_targets(device, renderer, size);
        let aspect = size.0 as f32 / size.1.max(1) as f32;
        let eye = self.camera.eye();
        let light = (self.camera.direction() + Vec3::Z * 0.8 + self.camera.right() * 0.3).normalize();
        let globals = Globals {
            view_proj: self.camera.view_proj(aspect).to_cols_array_2d(),
            light_dir: [light.x, light.y, light.z, 0.0],
            camera_pos: [eye.x, eye.y, eye.z, 1.0],
        };
        queue.write_buffer(&self.globals, 0, bytemuck::bytes_of(&globals));
        let data: Vec<InstanceData> = items
            .iter()
            .map(|it| InstanceData {
                model: it.model.to_cols_array_2d(),
                color: it.color,
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
            let color = self.color.as_ref().unwrap();
            let depth = self.depth.as_ref().unwrap();
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
                            a: 1.0,
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
            for (i, it) in items.iter().enumerate() {
                let Some(mesh) = self.meshes.get(&it.mesh) else { continue };
                if mesh.index_count == 0 {
                    continue;
                }
                pass.set_vertex_buffer(0, mesh.vbuf.slice(..));
                pass.set_index_buffer(mesh.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.index_count, 0, i as u32..i as u32 + 1);
            }
        }
        queue.submit(Some(encoder.finish()));
        self.tex_id.unwrap()
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
struct Inst { model: mat4x4<f32>, color: vec4<f32> };
@group(0) @binding(0) var<uniform> g: Globals;
@group(1) @binding(0) var<storage, read> insts: array<Inst>;

struct VOut { @builtin(position) pos: vec4<f32>, @location(0) nrm: vec3<f32>, @location(1) color: vec4<f32>, @location(2) wpos: vec3<f32> };

@vertex fn vs_main(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>, @builtin(instance_index) ii: u32) -> VOut {
  let inst = insts[ii];
  let wp = inst.model * vec4<f32>(p, 1.0);
  var o: VOut;
  o.pos = g.view_proj * wp;
  o.nrm = normalize((inst.model * vec4<f32>(n, 0.0)).xyz);
  o.color = inst.color;
  o.wpos = wp.xyz;
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
  return vec4<f32>(i.color.rgb * k, i.color.a);
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

#[cfg(test)]
mod tests {
    use super::*;

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
    fn fit_frames_the_box() {
        let mut cam = Camera::default();
        cam.fit(Vec3::new(-50.0, -50.0, 0.0), Vec3::new(50.0, 50.0, 20.0));
        assert_eq!(cam.target, Vec3::new(0.0, 0.0, 10.0));
        assert!(cam.distance > 100.0);
    }
}
