//! The application: the Workbench tab (library, viewport, tree,
//! inspector) drawn over the editor's state, and the Simulate tab.
//! Everything that changes the document lives in `editor`; this file
//! only draws it and feeds it input.

use crate::assembly::{self, Geometry, Instance, Part, Props};
use crate::bundle::MeshData;
use crate::editor::Editor;
use crate::geometry;
use crate::gizmo::{self, Gizmo, Handle, Mode};
use crate::route::Pose2;
use crate::simulate::SimulateTab;
use crate::stl;
use crate::viewport::{self, DrawItem, Line, Viewport, srgb};
use eframe::egui;
use eframe::egui_wgpu::{self, RenderState, wgpu};
use glam::{DVec3, Mat4, Quat, Vec3};
use std::collections::HashSet;
use std::sync::Arc;

/// The GPU handles the app draws with: eframe's render state, or a
/// test device.
#[derive(Clone)]
pub struct Gpu {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub renderer: Arc<egui::mutex::RwLock<egui_wgpu::Renderer>>,
}

impl From<&RenderState> for Gpu {
    fn from(s: &RenderState) -> Self {
        Gpu {
            device: s.device.clone(),
            queue: s.queue.clone(),
            renderer: s.renderer.clone(),
        }
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
enum Tab {
    Workbench,
    Simulate,
}

enum Drag {
    None,
    Orbit,
    Pan,
    Move {
        z0: f32,
        start: Vec3,
        starts: Vec<(String, [f64; 3])>,
    },
    /// A handle drag: the axis position or ring angle it started at,
    /// the pivot, and every dragged instance's starting pose.
    Handle {
        handle: Handle,
        start: f32,
        pivot: Vec3,
        starts: Vec<(String, [f64; 3], [f64; 3])>,
    },
    /// The chassis dragged on the map: where the press hit the ground,
    /// the pointer's x there, and the pose it started from.
    Chassis {
        start: Vec3,
        px0: f32,
        pose0: Pose2,
    },
}

pub struct App {
    editor: Editor,
    tab: Tab,
    search: String,
    group_name: String,
    gizmo_mode: Mode,
    hot: Option<Handle>,
    show_grid: bool,
    show_com: bool,
    viewport: Viewport,
    drag: Drag,
    /// Where the 3D view was drawn last frame, in screen points.
    view_rect: egui::Rect,
    items: Vec<DrawItem>,
    item_tops: Vec<String>,
    simulate: SimulateTab,
    /// An STL file being imported: the form and what it works out to.
    stl: Option<stl::Import>,
    stl_prepared: Option<stl::Prepared>,
}

pub fn cat_color(category: &str, dark: bool) -> [f32; 4] {
    let hex = match (category, dark) {
        ("lego", false) => 0x5B7A9C,
        ("lego", true) => 0x6F90B4,
        ("electronics", false) => 0x4A8A69,
        ("electronics", true) => 0x5FA382,
        ("servo", false) => 0x46525C,
        ("servo", true) => 0x7C8A96,
        ("wheel", false) => 0x2E3438,
        ("wheel", true) => 0x5A6369,
        (_, false) => 0x9A8562,
        (_, true) => 0xB49C74,
    };
    srgb(hex)
}

const ACCENT: [f32; 4] = [0.71, 0.29, 0.005, 1.0];
/// Library thumbnails: rendered pixels, shown at half size; how many
/// are rendered per frame so the first frames stay quick.
const THUMB_PX: u32 = 88;
const THUMB_SIZE: egui::Vec2 = egui::vec2(44.0, 33.0);
const THUMBS_PER_FRAME: usize = 4;
const RED: egui::Color32 = egui::Color32::from_rgb(196, 68, 42);

impl App {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        bundle: crate::bundle::Bundle,
        doc: Option<(std::path::PathBuf, assembly::Document)>,
        python: Option<String>,
    ) -> Self {
        let gpu = Gpu::from(cc.wgpu_render_state.as_ref().expect("the wgpu renderer is required"));
        Self::with_gpu(&gpu, bundle, doc, python)
    }

    pub fn with_gpu(
        gpu: &Gpu,
        bundle: crate::bundle::Bundle,
        doc: Option<(std::path::PathBuf, assembly::Document)>,
        python: Option<String>,
    ) -> Self {
        let viewport = Viewport::new(&gpu.device, &gpu.queue);
        App {
            editor: Editor::new(bundle, doc),
            tab: Tab::Workbench,
            search: String::new(),
            group_name: String::new(),
            gizmo_mode: Mode::Move,
            hot: None,
            show_grid: true,
            show_com: true,
            viewport,
            drag: Drag::None,
            view_rect: egui::Rect::ZERO,
            items: vec![],
            item_tops: vec![],
            simulate: SimulateTab::new(python),
            stl: None,
            stl_prepared: None,
        }
    }

    // --------------------------------------------------------- viewport

    fn mesh_key(part_id: &str, part: &Part) -> String {
        match &part.ldraw {
            Some(n) => format!("ld:{n}"),
            None => format!("part:{part_id}"),
        }
    }

    fn mesh_for(&self, part: &Part) -> Option<MeshData> {
        match assembly::geometry_of(part, &self.editor.bundle) {
            Geometry::Record(rec) => rec.mesh.decode().ok(),
            Geometry::Imported { .. } => {
                let m: crate::bundle::MeshRecord = serde_json::from_value(part.extra.get("mesh")?.clone()).ok()?;
                m.decode().ok()
            }
            Geometry::Shapes => {
                let mut out = MeshData::default();
                for s in &part.shapes {
                    let m = geometry::shape_mesh(s);
                    let base = out.positions.len() as u32;
                    out.positions.extend(m.positions);
                    out.normals.extend(m.normals);
                    out.indices.extend(m.indices.iter().map(|i| i + base));
                }
                Some(out)
            }
            Geometry::None => None,
        }
    }

    /// The viewport mesh of a part, built on first use; None without geometry.
    fn ensure_part_mesh(&mut self, device: &eframe::egui_wgpu::wgpu::Device, part_id: &str, part: &Part) -> Option<String> {
        let key = Self::mesh_key(part_id, part);
        if !self.viewport.has_mesh(&key) {
            let m = self.mesh_for(part)?;
            self.viewport.add_mesh(device, &key, &m);
        }
        Some(key)
    }

    // ------------------------------------------------------- thumbnails

    fn thumb_background(dark: bool) -> [f64; 4] {
        if dark {
            [0.0067, 0.0093, 0.0122, 0.0]
        } else {
            [0.83, 0.87, 0.89, 0.0]
        }
    }

    /// A Technic part from the bundle, rendered alone.
    fn part_thumb(&mut self, gpu: Option<&Gpu>, budget: &mut usize, num: &str, dark: bool) -> Option<egui::TextureId> {
        let key = format!("thumb:ld:{num}:{dark}");
        if let Some(id) = self.viewport.thumb(&key) {
            return Some(id);
        }
        let gpu = gpu?;
        if *budget == 0 {
            return None;
        }
        let rec = self.editor.bundle.parts.get(num)?;
        let bbox = (Vec3::from(rec.bbox[0].map(|v| v as f32)), Vec3::from(rec.bbox[1].map(|v| v as f32)));
        let part = Part {
            name: rec.name.clone(),
            category: "lego".into(),
            mass_g: rec.mass_g,
            source: String::new(),
            source_note: String::new(),
            ldraw: Some(num.to_string()),
            shapes: vec![],
            extra: Default::default(),
        };
        let mesh = self.ensure_part_mesh(&gpu.device, num, &part)?;
        let items = [DrawItem {
            mesh,
            model: Mat4::IDENTITY,
            color: cat_color("lego", dark),
            texture: None,
        }];
        *budget -= 1;
        let mut renderer = gpu.renderer.write();
        Some(self.viewport.thumbnail(
            &gpu.device,
            &gpu.queue,
            &mut renderer,
            &key,
            &items,
            bbox,
            THUMB_PX,
            Self::thumb_background(dark),
        ))
    }

    /// A brick recorded in the document (shapes or an imported mesh).
    fn other_thumb(&mut self, gpu: Option<&Gpu>, budget: &mut usize, id: &str, dark: bool) -> Option<egui::TextureId> {
        let key = format!("thumb:part:{id}:{}:{dark}", self.editor.edits);
        if let Some(id) = self.viewport.thumb(&key) {
            return Some(id);
        }
        let gpu = gpu?;
        if *budget == 0 {
            return None;
        }
        let part = self.editor.doc.parts.get(id)?.clone();
        let pr = assembly::part_props(&part, &self.editor.bundle);
        let bb = pr.bbox?;
        let mesh = self.ensure_part_mesh(&gpu.device, id, &part)?;
        let items = [DrawItem {
            mesh,
            model: Mat4::IDENTITY,
            color: cat_color(&part.category, dark),
            texture: None,
        }];
        *budget -= 1;
        let mut renderer = gpu.renderer.write();
        Some(self.viewport.thumbnail(
            &gpu.device,
            &gpu.queue,
            &mut renderer,
            &key,
            &items,
            (bb.min.as_vec3(), bb.max.as_vec3()),
            THUMB_PX,
            Self::thumb_background(dark),
        ))
    }

    /// A component as it is now: every brick under it, in its frame.
    fn component_thumb(&mut self, gpu: Option<&Gpu>, budget: &mut usize, id: &str, dark: bool) -> Option<egui::TextureId> {
        let key = format!("thumb:comp:{id}:{}:{dark}", self.editor.edits);
        if let Some(id) = self.viewport.thumb(&key) {
            return Some(id);
        }
        let gpu = gpu?;
        if *budget == 0 {
            return None;
        }
        let bb = self.editor.memo.get(id)?.bbox.clone()?;
        let mut items = Vec::new();
        for leaf in assembly::flatten(&self.editor.doc, id) {
            let Some(part) = self.editor.doc.parts.get(&leaf.part_id).cloned() else {
                continue;
            };
            let Some(mesh) = self.ensure_part_mesh(&gpu.device, &leaf.part_id, &part) else {
                continue;
            };
            let q = Quat::from_mat3(&leaf.rot.as_mat3());
            items.push(DrawItem {
                mesh,
                model: Mat4::from_rotation_translation(q, leaf.pos.as_vec3()),
                color: cat_color(&part.category, dark),
                texture: None,
            });
        }
        *budget -= 1;
        let mut renderer = gpu.renderer.write();
        Some(self.viewport.thumbnail(
            &gpu.device,
            &gpu.queue,
            &mut renderer,
            &key,
            &items,
            (bb.min.as_vec3(), bb.max.as_vec3()),
            THUMB_PX,
            Self::thumb_background(dark),
        ))
    }

    /// Thumbnails of components and document bricks are keyed on the
    /// edit count; the stale ones go when the document changes.
    fn prune_thumbs(&mut self, gpu: Option<&Gpu>) {
        let Some(gpu) = gpu else { return };
        let stamp = format!(":{}:", self.editor.edits);
        let mut renderer = gpu.renderer.write();
        self.viewport
            .retain_thumbs(&mut renderer, |k| k.starts_with("thumb:ld:") || k.contains(&stamp));
    }

    fn ensure_gizmo_meshes(&mut self, device: &eframe::egui_wgpu::wgpu::Device) {
        if self.viewport.has_mesh(gizmo::MESH_SHAFT) {
            return;
        }
        let shaft = geometry::cylinder_mesh(0.012, 0.8, "z", [0.0, 0.0, 0.4], 12);
        let cone = geometry::cone_mesh(0.05, 0.2, [0.0, 0.0, 0.8], 16);
        let ring = geometry::torus_mesh(gizmo::RING_FRACTION, 0.014, 64, 8);
        self.viewport.add_mesh(device, gizmo::MESH_SHAFT, &shaft);
        self.viewport.add_mesh(device, gizmo::MESH_CONE, &cone);
        self.viewport.add_mesh(device, gizmo::MESH_RING, &ring);
    }

    /// The handles on the selection: at the first selected instance's origin.
    fn gizmo(&self, h: f32) -> Option<Gizmo> {
        let first = self.editor.selected_instances().into_iter().next()?;
        let c = Vec3::new(first.pos[0] as f32, first.pos[1] as f32, first.pos[2] as f32);
        Some(Gizmo::new(c, self.gizmo_mode, &self.viewport.camera, h))
    }

    fn build_items(&mut self, device: &eframe::egui_wgpu::wgpu::Device, dark: bool) {
        let sel: HashSet<String> = self.editor.selection.iter().cloned().collect();
        let locked: HashSet<String> = self.editor.children().iter().filter(|c| c.locked).map(|c| c.name.clone()).collect();
        let mut items = Vec::new();
        let mut tops = Vec::new();
        for leaf in self.editor.leaves.clone() {
            let Some(part) = self.editor.doc.parts.get(&leaf.part_id).cloned() else {
                continue;
            };
            let Some(key) = self.ensure_part_mesh(device, &leaf.part_id, &part) else {
                continue;
            };
            let selected = sel.contains(&leaf.path[0]);
            let q = Quat::from_mat3(&leaf.rot.as_mat3());
            let model = Mat4::from_rotation_translation(q, leaf.pos.as_vec3());
            let mut color = cat_color(&part.category, dark);
            if locked.contains(&leaf.path[0]) {
                // locked: faded towards the ground colour
                let g = if dark { 0.05 } else { 0.55 };
                color = [
                    color[0] * 0.55 + g * 0.45,
                    color[1] * 0.55 + g * 0.45,
                    color[2] * 0.55 + g * 0.45,
                    1.0,
                ];
            }
            if selected {
                color = [
                    color[0] * 0.5 + ACCENT[0] * 0.6,
                    color[1] * 0.5 + ACCENT[1] * 0.6,
                    color[2] * 0.5 + ACCENT[2] * 0.6,
                    1.0,
                ];
            }
            items.push(DrawItem {
                mesh: key,
                model,
                color,
                texture: None,
            });
            tops.push(leaf.path[0].clone());
        }
        self.items = items;
        self.item_tops = tops;
    }

    fn scene_lines(&mut self, dark: bool) -> Vec<Line> {
        let mut lines = Vec::new();
        let pr = self.editor.edited_props();
        let bb = pr.bbox.clone().unwrap_or(assembly::Bbox {
            min: DVec3::splat(-40.0),
            max: DVec3::splat(40.0),
        });
        let gz = bb.min.z as f32;
        if self.show_grid {
            let (fine, strong) = if dark {
                (srgb(0x242C31), srgb(0x344047))
            } else {
                (srgb(0xDCE3E7), srgb(0xC3CDD3))
            };
            let x0 = ((bb.min.x - 48.0) / 8.0).floor() * 8.0;
            let x1 = ((bb.max.x + 48.0) / 8.0).ceil() * 8.0;
            let y0 = ((bb.min.y - 48.0) / 8.0).floor() * 8.0;
            let y1 = ((bb.max.y + 48.0) / 8.0).ceil() * 8.0;
            let mut x = x0;
            while x <= x1 {
                lines.push(Line {
                    a: Vec3::new(x as f32, y0 as f32, gz),
                    b: Vec3::new(x as f32, y1 as f32, gz),
                    color: if (x as i64) % 40 == 0 { strong } else { fine },
                });
                x += 8.0;
            }
            let mut y = y0;
            while y <= y1 {
                lines.push(Line {
                    a: Vec3::new(x0 as f32, y as f32, gz),
                    b: Vec3::new(x1 as f32, y as f32, gz),
                    color: if (y as i64) % 40 == 0 { strong } else { fine },
                });
                y += 8.0;
            }
        }
        let axes = [(Vec3::X, srgb(0xC4442A)), (Vec3::Y, srgb(0x3F7D5B)), (Vec3::Z, srgb(0x1F6FB2))];
        for (a, c) in axes {
            lines.push(Line {
                a: Vec3::ZERO,
                b: a * 24.0,
                color: c,
            });
        }
        if self.show_com && pr.count > 0 {
            let c = pr.com.as_vec3();
            let col = srgb(0xD9910F);
            for a in [Vec3::X, Vec3::Y, Vec3::Z] {
                lines.push(Line {
                    a: c - a * 12.0,
                    b: c + a * 12.0,
                    color: col,
                });
            }
        }
        lines
    }

    fn fit_view(&mut self) {
        let pr = self.editor.edited_props();
        if let Some(b) = pr.bbox {
            self.viewport.camera.fit(b.min.as_vec3(), b.max.as_vec3());
        } else {
            self.viewport.camera.fit(Vec3::splat(-40.0), Vec3::splat(40.0));
        }
    }

    fn viewport_ui(&mut self, ui: &mut egui::Ui, gpu: Option<&Gpu>) {
        let dark = ui.visuals().dark_mode;
        let avail = ui.available_size();
        let size = ((avail.x.max(1.0)) as u32, (avail.y.max(1.0)) as u32);
        let Some(gpu) = gpu else {
            ui.label("wgpu is not available");
            return;
        };
        self.ensure_gizmo_meshes(&gpu.device);
        self.build_items(&gpu.device, dark);
        if self.editor.fit_pending {
            self.fit_view();
            self.editor.fit_pending = false;
        }
        let lines = self.scene_lines(dark);
        let (w, h) = (size.0 as f32, size.1 as f32);
        let gizmo = self.gizmo(h);
        let overlay: Vec<DrawItem> = gizmo.as_ref().map(|g| g.draw(self.hot)).unwrap_or_default();
        let bg = if dark {
            [0.0067, 0.0093, 0.0122, 1.0]
        } else {
            [0.83, 0.87, 0.89, 1.0]
        };
        let tex = {
            let mut renderer = gpu.renderer.write();
            let scene = viewport::Scene {
                items: &self.items,
                lines: &lines,
                overlay: &overlay,
                background: bg,
            };
            self.viewport.render(&gpu.device, &gpu.queue, &mut renderer, size, &scene)
        };
        let response = ui.add(egui::Image::new((tex, egui::vec2(w, h))).sense(egui::Sense::click_and_drag()));
        let rect = response.rect;
        self.view_rect = rect;
        let local = |p: egui::Pos2| (p.x - rect.min.x, p.y - rect.min.y);
        let cam = self.viewport.camera.clone();
        let handle_under = |x: f32, y: f32| gizmo.as_ref().and_then(|g| g.handle_at(&cam, x, y, w, h));
        let shift = ui.input(|i| i.modifiers.shift);

        // zoom
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                let f = (1.0 - scroll * 0.002).clamp(0.5, 2.0);
                self.viewport.camera.distance = (self.viewport.camera.distance * f).clamp(20.0, 20000.0);
            }
        }
        // press: a handle first, then a brick — both judged where the button went down,
        // since a drag is only recognised once the pointer has moved a few points
        let pressed = response.drag_started_by(egui::PointerButton::Primary) || response.clicked_by(egui::PointerButton::Primary);
        let pointer = response.interact_pointer_pos().map(local);
        let origin = ui.input(|i| i.pointer.press_origin()).map(local).or(pointer);
        let on_handle = if pressed {
            origin.and_then(|(x, y)| handle_under(x, y))
        } else {
            None
        };
        if let (Some(handle), Some(g), true) = (on_handle, gizmo.as_ref(), response.drag_started_by(egui::PointerButton::Primary)) {
            let (x, y) = origin.unwrap_or((0.0, 0.0));
            let (o, d) = cam.ray(x, y, w, h);
            if let Some(start) = g.param(handle, o, d) {
                let starts = self.editor.begin_handle();
                if !starts.is_empty() {
                    self.drag = Drag::Handle {
                        handle,
                        start,
                        pivot: g.center,
                        starts,
                    };
                }
            }
        } else if pressed && on_handle.is_none() {
            let hit = origin
                .and_then(|(x, y)| self.viewport.pick(&self.items, x, y))
                .map(|i| self.item_tops[i].clone());
            match hit {
                Some(top) => {
                    // a click selects (shift toggles); a drag only makes sure what it drags is selected
                    if response.clicked_by(egui::PointerButton::Primary) || !self.editor.selection.contains(&top) {
                        self.editor.select(&top, shift);
                    }
                    if response.drag_started_by(egui::PointerButton::Primary) {
                        let z0 = self.editor.selected_instances().first().map(|i| i.pos[2] as f32).unwrap_or(0.0);
                        let (x, y) = origin.unwrap_or((0.0, 0.0));
                        let (o, d) = cam.ray(x, y, w, h);
                        if let Some(start) = viewport::ray_plane_z(o, d, z0) {
                            let starts = self.editor.begin_move();
                            if !starts.is_empty() {
                                self.drag = Drag::Move { z0, start, starts };
                            }
                        }
                    }
                }
                None => {
                    if response.clicked_by(egui::PointerButton::Primary) && !shift {
                        self.editor.selection.clear();
                    }
                    if response.drag_started_by(egui::PointerButton::Primary) {
                        self.drag = Drag::Orbit;
                    }
                }
            }
        }
        if response.drag_started_by(egui::PointerButton::Secondary) || response.drag_started_by(egui::PointerButton::Middle) {
            self.drag = Drag::Pan;
        }
        // a double-click opens a component (a third quick click still counts as one)
        if response.double_clicked() || response.triple_clicked() {
            let hit = pointer
                .and_then(|(x, y)| self.viewport.pick(&self.items, x, y))
                .map(|i| self.item_tops[i].clone());
            if let Some(top) = hit
                && let Some(cid) = self.editor.component_of(&top)
            {
                self.editor.open_component(&cid, true);
                return;
            }
        }
        let delta = response.drag_delta();
        match &self.drag {
            Drag::Orbit if response.dragged() => {
                self.viewport.camera.yaw -= delta.x * 0.5;
                self.viewport.camera.pitch = (self.viewport.camera.pitch + delta.y * 0.5).clamp(-89.0, 89.0);
            }
            Drag::Pan if response.dragged() => {
                let c = &mut self.viewport.camera;
                let scale = c.distance * 0.0015;
                let (r, u) = (c.right(), c.up());
                c.target = c.target - r * delta.x * scale + u * delta.y * scale;
            }
            Drag::Handle {
                handle,
                start,
                pivot,
                starts,
            } if response.dragged() => {
                let (handle, start, pivot, starts) = (*handle, *start, *pivot, starts.clone());
                if let Some((x, y)) = pointer {
                    let (o, d) = cam.ray(x, y, w, h);
                    let g = Gizmo {
                        center: pivot,
                        mode: self.gizmo_mode,
                        length: 1.0,
                    };
                    if let Some(now) = g.param(handle, o, d) {
                        let pv = DVec3::new(pivot.x as f64, pivot.y as f64, pivot.z as f64);
                        self.editor.drag_handle(handle, now - start, pv, &starts, shift);
                    }
                }
            }
            Drag::Move { z0, start, starts } if response.dragged() => {
                let (z0, start, starts) = (*z0, *start, starts.clone());
                if let Some((x, y)) = pointer {
                    let (o, d) = cam.ray(x, y, w, h);
                    if shift {
                        let dz = -delta.y * cam.distance * 0.0015;
                        self.editor.lift_by(&starts, dz as f64);
                    } else if let Some(p) = viewport::ray_plane_z(o, d, z0) {
                        let dp = p - start;
                        self.editor.move_by(&starts, dp.x, dp.y);
                    }
                }
            }
            _ => {}
        }
        if response.drag_stopped() {
            match self.drag {
                Drag::Move { .. } => self.editor.end_move(),
                Drag::Handle { .. } => self.editor.end_handle(),
                _ => {}
            }
            self.drag = Drag::None;
        }
        // the handle under the pointer lights up (drawn next frame)
        self.hot = match &self.drag {
            Drag::Handle { handle, .. } => Some(*handle),
            Drag::None => response.hover_pos().map(local).and_then(|(x, y)| handle_under(x, y)),
            _ => None,
        };
        if self.hot.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
        }
        self.keys(ui, shift);
        let hud = format!(
            "editing {}{} · 1 module = 8 mm · snap {} · orbit: drag · pan: right-drag · zoom: wheel · drag a brick to move it, shift lifts · W/E move/rotate handles (shift: free) · R turns 90° · S snaps · ⌘C/⌘V copy/paste · ⌘L locks, ⌘⇧L unlocks · Del · ⌘Z",
            self.editor.editing,
            if self.editor.selection.is_empty() {
                String::new()
            } else {
                format!(" · {} selected", self.editor.selection.len())
            },
            if self.editor.snap_mm > 0.0 {
                format!("{} mm", self.editor.snap_mm)
            } else {
                "off".into()
            }
        );
        ui.painter().text(
            rect.left_bottom() + egui::vec2(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            hud,
            egui::FontId::monospace(11.0),
            ui.visuals().weak_text_color(),
        );
    }

    /// The keys, and the copy / cut / paste events the platform turns
    /// ⌘C / ⌘X / ⌘V into, while no text field has the keyboard.
    fn keys(&mut self, ui: &mut egui::Ui, shift: bool) {
        if ui.ctx().egui_wants_keyboard_input() {
            return;
        }
        let mut copy = false;
        let mut cut = false;
        let mut paste = None;
        let (del, rot, esc, undo, dup, fit, snap_now, arrows, move_mode, rotate_mode, lock, unlock) = ui.input(|i| {
            for ev in &i.events {
                match ev {
                    egui::Event::Copy => copy = true,
                    egui::Event::Cut => cut = true,
                    egui::Event::Paste(t) => paste = Some(t.clone()),
                    _ => {}
                }
            }
            (
                i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace),
                i.key_pressed(egui::Key::R),
                i.key_pressed(egui::Key::Escape),
                i.modifiers.command && i.key_pressed(egui::Key::Z),
                i.modifiers.command && i.key_pressed(egui::Key::D),
                i.key_pressed(egui::Key::F),
                i.key_pressed(egui::Key::S) && !i.modifiers.command,
                [
                    i.key_pressed(egui::Key::ArrowUp),
                    i.key_pressed(egui::Key::ArrowDown),
                    i.key_pressed(egui::Key::ArrowLeft),
                    i.key_pressed(egui::Key::ArrowRight),
                ],
                i.key_pressed(egui::Key::W),
                i.key_pressed(egui::Key::E),
                i.modifiers.command && !i.modifiers.shift && i.key_pressed(egui::Key::L),
                i.modifiers.command && i.modifiers.shift && i.key_pressed(egui::Key::L),
            )
        });
        if undo {
            self.editor.undo();
        }
        if move_mode {
            self.gizmo_mode = Mode::Move;
        }
        if rotate_mode {
            self.gizmo_mode = Mode::Rotate;
        }
        if dup {
            self.editor.duplicate_selection();
        }
        if esc {
            self.editor.selection.clear();
        }
        if fit {
            self.fit_view();
        }
        if del {
            self.editor.remove_selection();
        }
        if rot {
            self.editor.rotate_selection(2, 90.0);
        }
        if snap_now {
            self.editor.snap_selection(true);
        }
        if lock {
            self.editor.lock_selection(true);
        }
        if unlock {
            self.editor.lock_selection(false);
        }
        if copy {
            self.copy(ui.ctx());
        }
        if cut {
            self.cut(ui.ctx());
        }
        if let Some(text) = paste {
            self.editor.paste(&text);
        }
        let step = self.editor.nudge_step(shift);
        if arrows[0] {
            self.editor.nudge_selection([step, 0.0, 0.0]);
        }
        if arrows[1] {
            self.editor.nudge_selection([-step, 0.0, 0.0]);
        }
        if arrows[2] {
            self.editor.nudge_selection([0.0, step, 0.0]);
        }
        if arrows[3] {
            self.editor.nudge_selection([0.0, -step, 0.0]);
        }
    }

    /// The selection goes to the system clipboard as text, so ⌘V brings
    /// it back here, into another component, or into another window.
    fn copy(&mut self, ctx: &egui::Context) {
        match self.editor.copy_selection() {
            Some(text) => {
                ctx.copy_text(text);
                self.editor.status = format!("Copied {}", self.editor.selection.len());
            }
            None => self.editor.status = "Nothing selected to copy".into(),
        }
    }

    fn cut(&mut self, ctx: &egui::Context) {
        match self.editor.cut_selection() {
            Some(text) => ctx.copy_text(text),
            None => self.editor.status = "Nothing selected to cut".into(),
        }
    }

    // --------------------------------------------------------------- ui

    fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.selectable_value(&mut self.tab, Tab::Workbench, "Workbench");
            ui.selectable_value(&mut self.tab, Tab::Simulate, "Simulate");
            ui.separator();
            if ui.button("Open…").clicked()
                && let Some(p) = rfd::FileDialog::new().add_filter("assembly", &["json"]).pick_file()
            {
                self.editor.load_path(p);
            }
            if ui.button(if self.editor.dirty { "Save *" } else { "Save" }).clicked() {
                self.save_file(false);
            }
            if ui.button("Save as…").clicked() {
                self.save_file(true);
            }
            if ui.button("Example").clicked() {
                self.editor.reset_to_example();
            }
            ui.separator();
            for (i, id) in self.editor.crumbs.clone().iter().enumerate() {
                if i > 0 {
                    ui.label("›");
                }
                let label = if id == &self.editor.doc.robot.root {
                    format!(
                        "{} (robot)",
                        if self.editor.doc.robot.name.is_empty() {
                            id.clone()
                        } else {
                            self.editor.doc.robot.name.clone()
                        }
                    )
                } else {
                    id.clone()
                };
                if i + 1 == self.editor.crumbs.len() {
                    ui.strong(label);
                } else if ui.link(label).clicked() {
                    self.editor.open_component(id, false);
                }
            }
            ui.separator();
            if ui.button("Fit").on_hover_text("F").clicked() {
                self.fit_view();
            }
            for (name, yaw, pitch) in [
                ("Iso", -128.0f32, 28.0f32),
                ("Top", -90.0, 89.0),
                ("Side", -90.0, 0.0),
                ("Front", 180.0, 0.0),
            ] {
                if ui.button(name).clicked() {
                    self.viewport.camera.yaw = yaw;
                    self.viewport.camera.pitch = pitch;
                }
            }
            ui.selectable_value(&mut self.gizmo_mode, Mode::Move, "Move")
                .on_hover_text("W: arrows on the selection");
            ui.selectable_value(&mut self.gizmo_mode, Mode::Rotate, "Rotate")
                .on_hover_text("E: rings on the selection");
            ui.label("snap");
            egui::ComboBox::from_id_salt("snap")
                .selected_text(if self.editor.snap_mm > 0.0 {
                    format!("{} mm", self.editor.snap_mm)
                } else {
                    "off".into()
                })
                .show_ui(ui, |ui| {
                    for (label, v) in [("8 mm", 8.0), ("4 mm", 4.0), ("1 mm", 1.0), ("off", 0.0)] {
                        ui.selectable_value(&mut self.editor.snap_mm, v, label);
                    }
                });
            ui.checkbox(&mut self.editor.magnet, "snap to holes");
            ui.checkbox(&mut self.show_grid, "ground");
            ui.checkbox(&mut self.show_com, "COM");
            if ui.button("Snap").on_hover_text("S").clicked() {
                self.editor.snap_selection(true);
            }
        });
    }

    fn save_file(&mut self, ask: bool) {
        let path = if ask || self.editor.path.is_none() {
            let mut dlg = rfd::FileDialog::new()
                .add_filter("assembly", &["json"])
                .set_file_name("robot.assembly.json");
            if let Some(p) = self.editor.path.as_ref().and_then(|p| p.parent()) {
                dlg = dlg.set_directory(p);
            }
            match dlg.save_file() {
                Some(p) => p,
                None => return,
            }
        } else {
            self.editor.path.clone().unwrap()
        };
        self.editor.save_to(&path);
    }

    fn library_ui(&mut self, ui: &mut egui::Ui, gpu: Option<&Gpu>) {
        ui.heading("Library");
        ui.add(egui::TextEdit::singleline(&mut self.search).hint_text("search bricks and components"));
        let q = self.search.trim().to_lowercase();
        let dark = ui.visuals().dark_mode;
        self.prune_thumbs(gpu);
        let mut budget = THUMBS_PER_FRAME;
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.add_space(6.0);
            ui.strong("Components");
            let ids: Vec<String> = self.editor.doc.components.keys().cloned().collect();
            let mut to_add: Option<(Option<String>, Option<String>)> = None;
            let mut to_open: Option<String> = None;
            let mut to_ldraw: Option<String> = None;
            for id in ids {
                if !q.is_empty() && !id.contains(&q) {
                    continue;
                }
                let mass = self.editor.memo.get(&id).map(|p| p.mass).unwrap_or(0.0);
                let uses = assembly::usage_count(&self.editor.doc, &id);
                let is_root = id == self.editor.doc.robot.root;
                let can_add = !is_root && !assembly::component_contains(&self.editor.doc, &id, &self.editor.editing, &mut vec![]);
                let thumb = self.component_thumb(gpu, &mut budget, &id, dark);
                ui.horizontal(|ui| {
                    thumb_slot(ui, thumb);
                    ui.add(egui::Label::new(egui::RichText::new(&id).monospace()).truncate());
                    ui.add(egui::Label::new(egui::RichText::new(format!("{} g Σ · used ×{uses}", assembly::fmt(mass))).weak()).truncate());
                    if ui.small_button("open").clicked() {
                        to_open = Some(id.clone());
                    }
                    if can_add && ui.small_button("+ add").clicked() {
                        to_add = Some((None, Some(id.clone())));
                    }
                });
            }
            ui.add_space(8.0);
            ui.strong(format!(
                "LEGO Technic  ({} parts, exact LDraw geometry)",
                self.editor.bundle.parts.len()
            ));
            let mut nums: Vec<String> = self.editor.bundle.parts.keys().cloned().collect();
            nums.sort_by_key(|n| self.editor.bundle.parts[n].name.to_lowercase());
            for num in nums {
                let rec = &self.editor.bundle.parts[&num];
                if !(q.is_empty() || num.contains(&q) || rec.name.to_lowercase().contains(&q)) {
                    continue;
                }
                let dens = if rec.volume_mm3 > 0.0 {
                    rec.mass_g / (rec.volume_mm3 / 1000.0)
                } else {
                    0.0
                };
                let (name, mass) = (rec.name.clone(), rec.mass_g);
                let thumb = self.part_thumb(gpu, &mut budget, &num, dark);
                ui.horizontal(|ui| {
                    thumb_slot(ui, thumb);
                    if ui.small_button("+").on_hover_text("add to the view").clicked() {
                        to_ldraw = Some(num.clone());
                    }
                    ui.add(egui::Label::new(&name).truncate());
                    ui.add(
                        egui::Label::new(egui::RichText::new(format!("{num} · {} g · {:.2} g/cm³", assembly::fmt(mass), dens)).weak())
                            .truncate(),
                    );
                });
            }
            ui.add_space(8.0);
            let mut import = false;
            ui.horizontal(|ui| {
                ui.strong("Other bricks");
                if ui.small_button("Import STL…").on_hover_text("a part from a mesh file").clicked() {
                    import = true;
                }
            });
            let ids: Vec<String> = self
                .editor
                .doc
                .parts
                .iter()
                .filter(|(_, p)| p.ldraw.is_none())
                .map(|(k, _)| k.clone())
                .collect();
            for id in ids {
                let p = self.editor.doc.parts[&id].clone();
                if !(q.is_empty() || p.name.to_lowercase().contains(&q) || id.contains(&q)) {
                    continue;
                }
                let thumb = self.other_thumb(gpu, &mut budget, &id, dark);
                ui.horizontal(|ui| {
                    thumb_slot(ui, thumb);
                    if ui.small_button("+").clicked() {
                        to_add = Some((Some(id.clone()), None));
                    }
                    ui.add(egui::Label::new(&p.name).truncate());
                    ui.add(
                        egui::Label::new(egui::RichText::new(format!("{} g · {}", assembly::fmt(p.mass_g), p.source)).weak()).truncate(),
                    );
                });
            }
            if let Some(num) = to_ldraw
                && let Some(id) = self.editor.ensure_ldraw_part(&num)
            {
                self.editor.add_instance(Some(id), None, [0.0; 3]);
            }
            if let Some((p, c)) = to_add {
                self.editor.add_instance(p, c, [0.0; 3]);
            }
            if let Some(id) = to_open {
                let push = id != self.editor.doc.robot.root && !self.editor.crumbs.contains(&id);
                self.editor.open_component(&id, push);
            }
            if import {
                self.import_stl();
            }
        });
    }

    fn import_stl(&mut self) {
        let Some(p) = rfd::FileDialog::new().add_filter("STL", &["stl"]).pick_file() else {
            return;
        };
        match stl::Import::from_path(&p) {
            Ok(imp) => {
                self.stl_prepared = Some(imp.prepare());
                self.stl = Some(imp);
            }
            Err(e) => self.editor.status = format!("Could not import {}: {e}", p.display()),
        }
    }

    /// The import form: name, category, units, origin, a weighed mass
    /// or a density; what it works out to is shown as it changes.
    fn stl_window(&mut self, ctx: &egui::Context) {
        let Some(imp) = self.stl.as_mut() else { return };
        let mut open = true;
        let mut changed = false;
        let mut add = false;
        let mut cancel = false;
        let prepared = &self.stl_prepared;
        egui::Window::new("Import a part from an STL file")
            .collapsible(false)
            .resizable(false)
            .open(&mut open)
            .show(ctx, |ui| {
                ui.label(format!("{} · {} triangles in the file", imp.file, imp.raw.len()));
                egui::Grid::new("stl-form").num_columns(2).show(ui, |ui| {
                    ui.weak("name");
                    ui.text_edit_singleline(&mut imp.name);
                    ui.end_row();
                    ui.weak("category");
                    egui::ComboBox::from_id_salt("stl-cat")
                        .selected_text(imp.category.clone())
                        .show_ui(ui, |ui| {
                            for c in stl::CATEGORIES {
                                ui.selectable_value(&mut imp.category, c.to_string(), c);
                            }
                        });
                    ui.end_row();
                    ui.weak("units in the file");
                    egui::ComboBox::from_id_salt("stl-units")
                        .selected_text(imp.units.label())
                        .show_ui(ui, |ui| {
                            for u in stl::Units::ALL {
                                changed |= ui.selectable_value(&mut imp.units, u, u.label()).changed();
                            }
                        });
                    ui.end_row();
                    ui.weak("origin");
                    egui::ComboBox::from_id_salt("stl-origin")
                        .selected_text(imp.origin.label())
                        .show_ui(ui, |ui| {
                            for o in stl::Origin::ALL {
                                changed |= ui.selectable_value(&mut imp.origin, o, o.label()).changed();
                            }
                        });
                    ui.end_row();
                    ui.weak("weighed mass");
                    ui.horizontal(|ui| {
                        changed |= ui
                            .add(egui::DragValue::new(&mut imp.mass_g).speed(0.1).range(0.0..=100000.0).suffix(" g"))
                            .changed();
                        ui.weak("0 = from the volume and density");
                    });
                    ui.end_row();
                    ui.weak("density");
                    ui.horizontal(|ui| {
                        changed |= ui
                            .add(
                                egui::DragValue::new(&mut imp.density)
                                    .speed(0.01)
                                    .range(0.01..=30.0)
                                    .suffix(" g/cm³"),
                            )
                            .changed();
                        for (name, d) in stl::DENSITIES {
                            if ui.small_button(name).clicked() {
                                imp.density = d;
                                changed = true;
                            }
                        }
                    });
                    ui.end_row();
                });
                if let Some(p) = prepared {
                    ui.add_space(4.0);
                    ui.label(&p.summary);
                }
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    let can_add = prepared.as_ref().map(|p| p.mass_g.is_some()).unwrap_or(false);
                    if ui.add_enabled(can_add, egui::Button::new("Add to the library")).clicked() {
                        add = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
        if changed {
            self.stl_prepared = Some(imp.prepare());
        }
        if add
            && let (Some(imp), Some(p)) = (self.stl.as_ref(), self.stl_prepared.as_ref())
            && let Some(part) = imp.part(p)
        {
            self.editor.import_part(part);
            cancel = true;
        }
        if cancel || !open {
            self.stl = None;
            self.stl_prepared = None;
        }
    }

    fn tree_ui(&mut self, ui: &mut egui::Ui) {
        ui.heading(format!("Contents of {}", self.editor.editing));
        let children: Vec<Instance> = self.editor.children().to_vec();
        let mut open: Option<String> = None;
        for ch in &children {
            let selected = self.editor.selection.contains(&ch.name);
            let (sub, mass) = match (&ch.part, &ch.component) {
                (Some(p), _) => (
                    self.editor
                        .doc
                        .parts
                        .get(p)
                        .map(|x| x.name.clone())
                        .unwrap_or_else(|| format!("missing {p}")),
                    self.editor.doc.parts.get(p).map(|x| x.mass_g).unwrap_or(0.0),
                ),
                (_, Some(c)) => (
                    format!("{c} · used ×{}", assembly::usage_count(&self.editor.doc, c)),
                    self.editor.memo.get(c).map(|p| p.mass).unwrap_or(0.0),
                ),
                _ => (String::new(), 0.0),
            };
            let label = format!(
                "{}{}   {}   {} g",
                if ch.locked { "🔒 " } else { "" },
                ch.name,
                sub,
                assembly::fmt(mass)
            );
            let resp = ui.selectable_label(selected, label);
            if resp.clicked() {
                if ui.input(|i| i.modifiers.shift) {
                    self.editor.select(&ch.name, true);
                } else {
                    self.editor.selection = vec![ch.name.clone()];
                }
            }
            if resp.double_clicked()
                && let Some(c) = &ch.component
            {
                open = Some(c.clone());
            }
        }
        if let Some(c) = open {
            self.editor.open_component(&c, true);
        }
    }

    fn computed_block(ui: &mut egui::Ui, pr: &Props, frame: &str) {
        ui.strong(format!("Computed  (Σ over everything inside, {frame})"));
        egui::Grid::new(format!("computed-{frame}")).num_columns(2).show(ui, |ui| {
            ui.weak("mass");
            ui.monospace(format!("{} g", assembly::fmt(pr.mass)));
            ui.end_row();
            ui.weak("centre of mass");
            ui.monospace(format!("{} mm", vec_text(pr.com)));
            ui.end_row();
            if let Some(b) = &pr.bbox {
                ui.weak("extents");
                ui.monospace(format!("{} mm", vec_text(b.size())));
                ui.end_row();
            }
            ui.weak("bricks");
            ui.monospace(pr.count.to_string());
            ui.end_row();
            ui.weak("inertia about COM");
            ui.vertical(|ui| {
                for r in 0..3 {
                    ui.monospace(format!(
                        "{:>12} {:>12} {:>12}",
                        inertia_text(pr.inertia.col(0)[r]),
                        inertia_text(pr.inertia.col(1)[r]),
                        inertia_text(pr.inertia.col(2)[r])
                    ));
                }
                ui.weak("g·mm²");
            });
            ui.end_row();
        });
    }

    /// The buttons every selection gets: copy, lock or unlock, duplicate, remove.
    fn selection_actions(&mut self, ui: &mut egui::Ui) {
        let locked = self.editor.locked_count();
        let total = self.editor.selection.len();
        if ui.button("Copy").on_hover_text("⌘C, then ⌘V pastes").clicked() {
            self.copy(ui.ctx());
        }
        if locked < total && ui.button("Lock").on_hover_text("⌘L: locked items stay where they are").clicked() {
            self.editor.lock_selection(true);
        }
        if locked > 0 && ui.button("Unlock").on_hover_text("⌘⇧L").clicked() {
            self.editor.lock_selection(false);
        }
        if ui.button("Duplicate").on_hover_text("⌘D").clicked() {
            self.editor.duplicate_selection();
        }
        if ui.button("Remove").on_hover_text("Delete").clicked() {
            self.editor.remove_selection();
        }
    }

    fn robot_ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("Robot");
        let mut name = self.editor.doc.robot.name.clone();
        if ui.text_edit_singleline(&mut name).changed() {
            self.editor.doc.robot.name = name;
            self.editor.dirty = true;
        }
        let pr = self.editor.root_props.clone();
        Self::computed_block(ui, &pr, "assembly frame");
        ui.add_space(6.0);
        ui.strong("Roles  (what the simulator binds)");
        let options = self.editor.role_options();
        let mut changed = None;
        egui::Grid::new("roles").num_columns(2).show(ui, |ui| {
            for (role, help) in assembly::ROLES {
                ui.monospace(role).on_hover_text(help);
                let current = self.editor.doc.robot.roles.get(role).cloned().unwrap_or_default();
                let mut pick = current.clone();
                egui::ComboBox::from_id_salt(role)
                    .selected_text(if pick.is_empty() { "—".to_string() } else { pick.clone() })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut pick, String::new(), "—");
                        for o in &options {
                            ui.selectable_value(&mut pick, o.clone(), o);
                        }
                    });
                if pick != current {
                    changed = Some((role.to_string(), pick));
                }
                ui.end_row();
            }
        });
        if let Some((role, v)) = changed {
            self.editor.set_role(&role, v);
        }
        ui.add_space(6.0);
        ui.strong("Spawn on the mat");
        ui.horizontal(|ui| {
            let spawn = &mut self.editor.doc.robot.spawn;
            let mut changed = false;
            ui.weak("x");
            changed |= ui
                .add(egui::DragValue::new(&mut spawn.pos_mm[0]).speed(1.0).suffix(" mm"))
                .changed();
            ui.weak("y");
            changed |= ui
                .add(egui::DragValue::new(&mut spawn.pos_mm[1]).speed(1.0).suffix(" mm"))
                .changed();
            ui.weak("yaw");
            changed |= ui.add(egui::DragValue::new(&mut spawn.yaw_deg).speed(1.0).suffix("°")).changed();
            if changed {
                self.editor.dirty = true;
            }
        });
        ui.add_space(6.0);
        ui.strong("Check against the scale");
        let mut weighed = self.editor.doc.robot.measured_mass_g.unwrap_or(0.0);
        ui.horizontal(|ui| {
            ui.weak("weighed");
            if ui.add(egui::DragValue::new(&mut weighed).speed(1.0).suffix(" g")).changed() {
                self.editor.doc.robot.measured_mass_g = if weighed > 0.0 { Some(weighed) } else { None };
                self.editor.dirty = true;
            }
        });
        if let Some(w) = self.editor.doc.robot.measured_mass_g {
            let d = pr.mass - w;
            ui.label(format!(
                "computed {} g vs weighed {} g: {}{} g ({:.1} %). Cables, screws and pins are usually the gap.",
                assembly::fmt(pr.mass),
                assembly::fmt(w),
                if d >= 0.0 { "+" } else { "" },
                assembly::fmt(d),
                100.0 * d / w
            ));
        }
    }

    fn inspector_ui(&mut self, ui: &mut egui::Ui) {
        let insts = self.editor.selected_instances();
        if insts.is_empty() {
            if self.editor.is_root() {
                self.robot_ui(ui);
            } else {
                ui.heading(format!("Component {}", self.editor.editing));
                ui.label("Its origin is the frame every instance is placed by. Add bricks, arrange them, then use it from the library.");
                let pr = self.editor.edited_props();
                Self::computed_block(ui, &pr, &format!("{} frame", self.editor.editing));
                ui.weak(format!("used ×{}", assembly::usage_count(&self.editor.doc, &self.editor.editing)));
                if ui.button("Back to the robot").clicked() {
                    let root = self.editor.doc.robot.root.clone();
                    self.editor.open_component(&root, false);
                }
            }
            if !self.editor.errors.is_empty() {
                ui.add_space(6.0);
                for e in &self.editor.errors {
                    ui.colored_label(RED, e);
                }
            }
            return;
        }
        if insts.len() > 1 {
            ui.heading(format!("{} selected", insts.len()));
            let together: f64 = insts
                .iter()
                .map(|i| {
                    i.part
                        .as_ref()
                        .and_then(|p| self.editor.doc.parts.get(p))
                        .map(|p| p.mass_g)
                        .or_else(|| i.component.as_ref().and_then(|c| self.editor.memo.get(c)).map(|p| p.mass))
                        .unwrap_or(0.0)
                })
                .sum();
            ui.monospace(format!("together {} g", assembly::fmt(together)));
            let locked = self.editor.locked_count();
            if locked > 0 {
                ui.weak(format!("🔒 {locked} locked"));
            }
            self.group_form(ui, insts.len());
            ui.horizontal_wrapped(|ui| {
                if ui.button("Turn 90°").clicked() {
                    self.editor.rotate_selection(2, 90.0);
                }
                self.selection_actions(ui);
            });
            return;
        }
        let inst = insts[0].clone();
        ui.heading(format!(
            "{}{}",
            if inst.part.is_some() {
                "Brick instance"
            } else {
                "Component instance"
            },
            if inst.locked { "  🔒 locked" } else { "" }
        ));
        let mut name = inst.name.clone();
        if ui.text_edit_singleline(&mut name).lost_focus() && name != inst.name && self.editor.rename_selected(&name) {
            return;
        }
        ui.weak(format!(
            "in {}{}",
            self.editor.editing,
            if assembly::usage_count(&self.editor.doc, &self.editor.editing) > 1 && !self.editor.is_root() {
                " — used more than once: a change here moves it in every use"
            } else {
                ""
            }
        ));
        ui.strong(format!("Pose  (in the {} frame)", self.editor.editing));
        let mut pos = inst.pos;
        let mut rot = inst.rot;
        let mut changed = false;
        let step = self.editor.snap_mm.max(1.0);
        ui.add_enabled_ui(!inst.locked, |ui| {
            egui::Grid::new("pose").num_columns(4).show(ui, |ui| {
                for (k, label) in ["x", "y", "z"].iter().enumerate() {
                    ui.weak(*label);
                    changed |= ui.add(egui::DragValue::new(&mut pos[k]).speed(1.0).suffix(" mm")).changed();
                    if ui.small_button(format!("−{step}")).clicked() {
                        pos[k] -= step;
                        changed = true;
                    }
                    if ui.small_button(format!("+{step}")).clicked() {
                        pos[k] += step;
                        changed = true;
                    }
                    ui.end_row();
                }
                for (k, label) in ["roll", "pitch", "yaw"].iter().enumerate() {
                    ui.weak(*label);
                    changed |= ui.add(egui::DragValue::new(&mut rot[k]).speed(1.0).suffix("°")).changed();
                    if ui.small_button("−90").clicked() {
                        rot[k] -= 90.0;
                        changed = true;
                    }
                    if ui.small_button("+90").clicked() {
                        rot[k] += 90.0;
                        changed = true;
                    }
                    ui.end_row();
                }
            });
        });
        if changed {
            self.editor.set_pose(&inst.name, pos, rot);
        }
        if let Some(pid) = &inst.part {
            if let Some(part) = self.editor.doc.parts.get(pid).cloned() {
                ui.add_space(6.0);
                ui.strong("Recorded  (the brick's own facts)");
                egui::Grid::new("recorded").num_columns(2).show(ui, |ui| {
                    ui.weak("name");
                    ui.label(&part.name);
                    ui.end_row();
                    ui.weak("mass");
                    ui.monospace(format!("{} g", assembly::fmt(part.mass_g)));
                    ui.end_row();
                    match assembly::geometry_of(&part, &self.editor.bundle) {
                        Geometry::Record(rec) => {
                            ui.weak("geometry");
                            ui.add(
                                egui::Label::new(format!(
                                    "LDraw {} exact mesh, {} triangles, {} mm",
                                    rec.ldraw,
                                    rec.mesh.tris,
                                    bbox_text(&rec.bbox)
                                ))
                                .wrap(),
                            );
                            ui.end_row();
                            ui.weak("volume");
                            let dens = part.mass_g / (rec.volume_mm3 / 1000.0);
                            ui.label(format!(
                                "{:.2} cm³ → {:.2} g/cm³{}",
                                rec.volume_mm3 / 1000.0,
                                dens,
                                if (dens - 1.05).abs() < 0.2 {
                                    " (ABS ✓)"
                                } else {
                                    " (check: ABS is ≈1.05)"
                                }
                            ));
                            ui.end_row();
                            ui.weak("features");
                            ui.label(connector_summary(&rec.connectors));
                            ui.end_row();
                        }
                        Geometry::Imported {
                            volume_mm3,
                            tris,
                            connectors,
                            bbox,
                            ..
                        } => {
                            ui.weak("geometry");
                            ui.label(format!("STL mesh, {tris} triangles, {} mm", vec_text(bbox.size())));
                            ui.end_row();
                            ui.weak("volume");
                            ui.label(format!(
                                "{:.2} cm³ → {:.2} g/cm³",
                                volume_mm3 / 1000.0,
                                part.mass_g / (volume_mm3 / 1000.0).max(1e-9)
                            ));
                            ui.end_row();
                            ui.weak("features");
                            ui.label(connector_summary(&connectors));
                            ui.end_row();
                        }
                        _ => {
                            for (i, s) in part.shapes.iter().enumerate() {
                                ui.weak(format!("shape {}", i + 1));
                                ui.label(s.text());
                                ui.end_row();
                            }
                        }
                    }
                    ui.weak("source");
                    ui.add(
                        egui::Label::new(format!(
                            "{}{}",
                            part.source,
                            if part.source_note.is_empty() {
                                String::new()
                            } else {
                                format!(" — {}", part.source_note)
                            }
                        ))
                        .wrap(),
                    );
                    ui.end_row();
                });
                let cons = assembly::connections_of(&self.editor.doc, &self.editor.bundle, &self.editor.editing, &inst.name);
                ui.add_space(4.0);
                ui.strong("Connections  (mated features)");
                if cons.is_empty() {
                    ui.weak("none: drag it next to a hole and let go, or press S");
                } else {
                    for (m, o, path) in cons {
                        ui.label(format!("{} → {} of {}", m.replace('_', " "), o.replace('_', " "), path.join("/")));
                    }
                }
                let pr = assembly::part_props(&part, &self.editor.bundle);
                Self::computed_block(ui, &pr, "brick frame");
            }
        } else if let Some(cid) = &inst.component {
            let pr = self.editor.memo.get(cid).cloned();
            if let Some(pr) = pr {
                Self::computed_block(ui, &pr, &format!("{cid} frame"));
            }
            ui.horizontal(|ui| {
                if ui.button(format!("Open {cid}")).clicked() {
                    self.editor.open_component(cid, true);
                    return;
                }
                if ui.button("Ungroup").clicked() {
                    self.editor.ungroup_selection();
                }
            });
        }
        self.group_form(ui, 1);
        ui.horizontal_wrapped(|ui| {
            if ui.button("Turn 90°").clicked() {
                self.editor.rotate_selection(2, 90.0);
            }
            if ui.button("Pitch 90°").clicked() {
                self.editor.rotate_selection(1, 90.0);
            }
            if ui.button("Roll 90°").clicked() {
                self.editor.rotate_selection(0, 90.0);
            }
            self.selection_actions(ui);
        });
    }

    fn group_form(&mut self, ui: &mut egui::Ui, n: usize) {
        ui.add_space(6.0);
        ui.strong(format!(
            "Make a component from {}",
            if n == 1 {
                "this item".to_string()
            } else {
                format!("these {n} items")
            }
        ));
        ui.horizontal(|ui| {
            let resp = ui.add(
                egui::TextEdit::singleline(&mut self.group_name)
                    .hint_text("drive_unit")
                    .desired_width(160.0),
            );
            let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if (ui.button("Group").clicked() || enter) && self.editor.group_selection(&self.group_name.clone()) {
                self.group_name.clear();
            }
        });
        ui.weak("The first selected item's frame becomes the new component's origin; the component joins the library.");
    }

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        let pr = self.editor.root_props.clone();
        let placeholders = self.editor.doc.parts.values().filter(|p| p.source == "placeholder").count();
        ui.horizontal(|ui| {
            ui.monospace(format!(
                "robot {} g · {} bricks · COM {} mm",
                assembly::fmt(pr.mass),
                pr.count,
                vec_text(pr.com)
            ));
            if placeholders > 0 {
                ui.colored_label(
                    RED,
                    format!("{placeholders} brick{} still to measure", if placeholders > 1 { "s" } else { "" }),
                );
            }
            if let Some(p) = &self.editor.path {
                ui.weak(p.display().to_string());
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(&self.editor.status);
            });
        });
    }

    fn simulate_ui(&mut self, ui: &mut egui::Ui, gpu: Option<&Gpu>) {
        self.simulate.ensure_loaded();
        if self.simulate.pump() || self.simulate.is_live() {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(16));
        }
        egui::Panel::top("sim-controls").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                let can_use = self.editor.path.is_some() && !self.editor.dirty;
                if ui
                    .add_enabled(can_use, egui::Button::new("Use the workbench's build"))
                    .on_hover_text(if self.editor.dirty {
                        "save the assembly first"
                    } else {
                        "the assembly open in the Workbench tab"
                    })
                    .clicked()
                    && let Some(p) = self.editor.path.clone()
                {
                    self.simulate.set_chassis(p, self.editor.doc.clone());
                }
                self.simulate.controls(ui);
            });
        });
        egui::Panel::bottom("sim-log")
            .default_size(160.0)
            .resizable(true)
            .show(ui, |ui| self.simulate.log_ui(ui));
        egui::Panel::right("route")
            .default_size(360.0)
            .size_range(300.0..=520.0)
            .resizable(true)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.simulate.route_ui(ui));
            });
        egui::CentralPanel::default().show(ui, |ui| self.sim_view_ui(ui, gpu));
    }

    fn sim_view_ui(&mut self, ui: &mut egui::Ui, gpu: Option<&Gpu>) {
        let dark = ui.visuals().dark_mode;
        let avail = ui.available_size();
        let size = ((avail.x.max(1.0)) as u32, (avail.y.max(1.0)) as u32);
        let Some(gpu) = gpu else {
            ui.label("wgpu is not available");
            return;
        };
        let (items, mut lines) = self
            .simulate
            .draw_items(&mut self.viewport, &gpu.device, &gpu.queue, &self.editor.bundle, dark);
        lines.extend(self.simulate.route_lines(dark));
        if let Some((lo, hi)) = self.simulate.frame_target() {
            self.viewport.camera.fit(lo, hi);
            if self.simulate.follows() {
                self.viewport.camera.distance = self.viewport.camera.distance.max(600.0);
            }
        }
        let bg = if dark {
            [0.0067, 0.0093, 0.0122, 1.0]
        } else {
            [0.83, 0.87, 0.89, 1.0]
        };
        let tex = {
            let mut renderer = gpu.renderer.write();
            let scene = viewport::Scene {
                items: &items,
                lines: &lines,
                overlay: &[],
                background: bg,
            };
            self.viewport.render(&gpu.device, &gpu.queue, &mut renderer, size, &scene)
        };
        let response = ui.add(egui::Image::new((tex, egui::vec2(size.0 as f32, size.1 as f32))).sense(egui::Sense::click_and_drag()));
        self.view_rect = response.rect;
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                let f = (1.0 - scroll * 0.002).clamp(0.5, 2.0);
                self.viewport.camera.distance = (self.viewport.camera.distance * f).clamp(20.0, 50000.0);
            }
        }
        let rect = response.rect;
        let (w, h) = (size.0 as f32, size.1 as f32);
        let local = |p: egui::Pos2| (p.x - rect.min.x, p.y - rect.min.y);
        let cam = self.viewport.camera.clone();
        let ground = |x: f32, y: f32| {
            let (o, d) = cam.ray(x, y, w, h);
            viewport::ray_plane_z(o, d, 0.0)
        };
        let shift = ui.input(|i| i.modifiers.shift);
        // a press on the chassis starts moving it; anywhere else orbits
        if response.drag_started_by(egui::PointerButton::Primary) {
            let origin = ui
                .input(|i| i.pointer.press_origin())
                .map(local)
                .or(response.interact_pointer_pos().map(local));
            let on_chassis = origin
                .and_then(|(x, y)| self.viewport.pick(&items, x, y))
                .map(|i| self.simulate.chassis_items.contains(&i))
                .unwrap_or(false);
            self.drag = Drag::Orbit;
            if on_chassis
                && let Some((x, y)) = origin
                && let Some(hit) = ground(x, y)
                && let Some(pose0) = self.simulate.begin_chassis_drag()
            {
                self.drag = Drag::Chassis { start: hit, px0: x, pose0 };
            }
        }
        if response.drag_started_by(egui::PointerButton::Secondary) || response.drag_started_by(egui::PointerButton::Middle) {
            self.drag = Drag::Pan;
        }
        // a click on the map ends the armed segment
        if response.clicked_by(egui::PointerButton::Primary)
            && self.simulate.pick.is_some()
            && let Some((x, y)) = response.interact_pointer_pos().map(local)
            && let Some(hit) = ground(x, y)
        {
            self.simulate.map_click(hit.x as f64, hit.y as f64);
        }
        let delta = response.drag_delta();
        match &self.drag {
            Drag::Orbit if response.dragged() => {
                self.viewport.camera.yaw -= delta.x * 0.5;
                self.viewport.camera.pitch = (self.viewport.camera.pitch + delta.y * 0.5).clamp(-89.0, 89.0);
            }
            Drag::Pan if response.dragged() => {
                let c = &mut self.viewport.camera;
                let scale = c.distance * 0.0015;
                let (r, u) = (c.right(), c.up());
                c.target = c.target - r * delta.x * scale + u * delta.y * scale;
            }
            Drag::Chassis { start, px0, pose0 } if response.dragged() => {
                let (start, px0, pose0) = (*start, *px0, *pose0);
                if let Some((x, y)) = response.interact_pointer_pos().map(local) {
                    let pose = if shift {
                        // shift turns it: a screen-width drag is a full turn
                        Pose2 {
                            yaw_deg: pose0.yaw_deg - ((x - px0) * 360.0 / w.max(1.0)) as f64,
                            ..pose0
                        }
                    } else if let Some(hit) = ground(x, y) {
                        Pose2 {
                            x_mm: pose0.x_mm + (hit.x - start.x) as f64,
                            y_mm: pose0.y_mm + (hit.y - start.y) as f64,
                            yaw_deg: pose0.yaw_deg,
                        }
                    } else {
                        pose0
                    };
                    self.simulate.drag_chassis(pose);
                }
            }
            _ => {}
        }
        if response.drag_stopped() {
            if matches!(self.drag, Drag::Chassis { .. }) {
                self.simulate.end_chassis_drag();
            }
            self.drag = Drag::None;
        }
        if ui.input(|i| i.key_pressed(egui::Key::Escape)) && !ui.ctx().egui_wants_keyboard_input() {
            self.simulate.pick = None;
        }
        if items.is_empty() {
            ui.painter().text(
                response.rect.center(),
                egui::Align2::CENTER_CENTER,
                "The map appears here once the run server has built it",
                egui::FontId::proportional(14.0),
                ui.visuals().weak_text_color(),
            );
        }
        ui.painter().text(
            response.rect.left_bottom() + egui::vec2(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            if self.simulate.pick.is_some() {
                "click the map where the segment ends · Esc cancels"
            } else {
                "orbit: drag · pan: right-drag · zoom: wheel · drag the chassis to place it (shift turns it)"
            },
            egui::FontId::monospace(11.0),
            ui.visuals().weak_text_color(),
        );
    }

    fn title_name(&self) -> String {
        self.editor
            .path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|f| f.to_string_lossy().to_string())
            .unwrap_or_else(|| "example".into())
    }
}

/// A thumbnail, or the space one takes while it is not rendered yet.
fn thumb_slot(ui: &mut egui::Ui, thumb: Option<egui::TextureId>) {
    match thumb {
        Some(id) => {
            ui.image((id, THUMB_SIZE));
        }
        None => {
            ui.add_space(THUMB_SIZE.x + ui.spacing().item_spacing.x);
        }
    }
}

fn vec_text(v: DVec3) -> String {
    format!("{}, {}, {}", assembly::fmt(v.x), assembly::fmt(v.y), assembly::fmt(v.z))
}

fn bbox_text(b: &[[f64; 3]; 2]) -> String {
    format!(
        "{} × {} × {}",
        assembly::fmt(b[1][0] - b[0][0]),
        assembly::fmt(b[1][1] - b[0][1]),
        assembly::fmt(b[1][2] - b[0][2])
    )
}

fn inertia_text(v: f64) -> String {
    if v.abs() < 0.5 {
        "0".into()
    } else {
        format!("{}", v.round() as i64)
    }
}

fn connector_summary(cs: &[crate::bundle::Connector]) -> String {
    let mut counts: Vec<(String, usize)> = Vec::new();
    for c in cs {
        match counts.iter_mut().find(|(k, _)| k == &c.kind) {
            Some(e) => e.1 += 1,
            None => counts.push((c.kind.clone(), 1)),
        }
    }
    if counts.is_empty() {
        return "no connectors".into();
    }
    counts
        .iter()
        .map(|(k, n)| {
            let label = match k.as_str() {
                "pin_hole" => "pin hole",
                "axle_hole" => "axle hole",
                "stud_hole" => "stud tube",
                other => other,
            };
            format!("{n} {label}{}", if *n > 1 { "s" } else { "" })
        })
        .collect::<Vec<_>>()
        .join(", ")
}

impl App {
    /// One frame of the whole window, drawn with `gpu` (None shows a
    /// notice where the 3D views would be).
    pub fn frame_ui(&mut self, ui: &mut egui::Ui, gpu: Option<&Gpu>) {
        egui::Panel::top("toolbar").show(ui, |ui| self.toolbar(ui));
        egui::Panel::bottom("status").show(ui, |ui| self.status_bar(ui));
        match self.tab {
            Tab::Simulate => {
                self.simulate_ui(ui, gpu);
            }
            Tab::Workbench => {
                self.stl_window(ui.ctx());
                egui::Panel::left("library")
                    .default_size(320.0)
                    .size_range(220.0..=520.0)
                    .resizable(true)
                    .show(ui, |ui| self.library_ui(ui, gpu));
                egui::Panel::right("inspector")
                    .default_size(380.0)
                    .size_range(320.0..=560.0)
                    .resizable(true)
                    .show(ui, |ui| {
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            self.tree_ui(ui);
                            ui.separator();
                            self.inspector_ui(ui);
                        });
                    });
                egui::CentralPanel::default().show(ui, |ui| self.viewport_ui(ui, gpu));
            }
        }
        if self.editor.dirty {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::Title(format!("Openbricks Sim — {}*", self.title_name())));
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let gpu = frame.wgpu_render_state().map(Gpu::from);
        self.frame_ui(ui, gpu.as_ref());
    }
}

impl Drop for App {
    fn drop(&mut self) {
        self.simulate.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::editor::testing::real_bundle;
    use crate::sim::testing::fake_server;
    use crate::viewport::testing::{test_device, test_renderer};
    use egui::{Event, Key, Modifiers, PointerButton, Pos2};
    use egui_kittest::Harness;
    use egui_kittest::kittest::{NodeT, Queryable};

    #[test]
    fn colours_and_labels() {
        assert_ne!(cat_color("lego", false), cat_color("lego", true));
        assert_eq!(cat_color("mystery", false), cat_color("other", false));
        assert_eq!(vec_text(DVec3::new(1.0, 2.5, -3.0)), "1, 2.5, -3");
        assert_eq!(bbox_text(&[[0.0, 0.0, 0.0], [10.0, 20.0, 30.0]]), "10 × 20 × 30");
        assert_eq!(inertia_text(0.4), "0");
        assert_eq!(inertia_text(1234.6), "1235");
        let c = |kind: &str| crate::bundle::Connector {
            kind: kind.into(),
            centre: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            length: 8.0,
            r: 2.4,
        };
        assert_eq!(connector_summary(&[]), "no connectors");
        assert_eq!(
            connector_summary(&[c("pin_hole"), c("pin_hole"), c("axle_hole"), c("stud_hole"), c("pin")]),
            "2 pin holes, 1 axle hole, 1 stud tube, 1 pin"
        );
    }

    /// The whole app in a headless window, drawn with the test GPU.
    fn gpu() -> Option<Gpu> {
        let (device, queue) = test_device()?;
        let renderer = test_renderer(&device);
        Some(Gpu {
            device,
            queue,
            renderer: Arc::new(egui::mutex::RwLock::new(renderer)),
        })
    }

    fn harness(gpu: &Gpu, python: Option<String>) -> Harness<'_, App> {
        let app = App::with_gpu(gpu, real_bundle(), None, python);
        Harness::builder()
            .with_size(egui::vec2(1800.0, 2400.0))
            .with_step_dt(1.0 / 60.0)
            .build_ui_state(|ui, app: &mut App| app.frame_ui(ui, Some(gpu)), app)
    }

    fn steps(h: &mut Harness<'_, App>, n: usize) {
        for _ in 0..n {
            h.step();
        }
    }

    /// The screen point a world position lands on in the 3D view.
    fn on_screen(app: &App, world: Vec3) -> Pos2 {
        let r = app.view_rect;
        let p = app
            .viewport
            .camera
            .project(world, r.width(), r.height())
            .expect("in front of the camera");
        Pos2::new(r.min.x + p.x, r.min.y + p.y)
    }

    /// The pointer arrives, then the button goes down, each in its own
    /// frame as they do from a real mouse.
    fn press(h: &mut Harness<'_, App>, pos: Pos2, button: PointerButton, modifiers: Modifiers) {
        h.input_mut().events.push(Event::PointerMoved(pos));
        h.step();
        h.input_mut().modifiers = modifiers;
        h.input_mut().events.push(Event::PointerButton {
            pos,
            button,
            pressed: true,
            modifiers,
        });
        h.step();
    }

    fn drag_to(h: &mut Harness<'_, App>, pos: Pos2, modifiers: Modifiers) {
        h.input_mut().modifiers = modifiers;
        h.input_mut().events.push(Event::PointerMoved(pos));
        h.step();
    }

    fn release(h: &mut Harness<'_, App>, pos: Pos2, button: PointerButton) {
        h.input_mut().modifiers = Modifiers::NONE;
        h.input_mut().events.push(Event::PointerButton {
            pos,
            button,
            pressed: false,
            modifiers: Modifiers::NONE,
        });
        h.step();
        h.step();
    }

    #[test]
    fn toolbar_buttons_and_mode_keys_drive_the_view() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        assert!(h.state().view_rect.width() > 300.0, "{:?}", h.state().view_rect);
        assert!(!h.state().items.is_empty(), "the example robot is drawn");
        h.get_by_label("Rotate").click();
        h.step();
        assert_eq!(h.state().gizmo_mode, Mode::Rotate);
        h.get_by_label("Move").click();
        h.step();
        assert_eq!(h.state().gizmo_mode, Mode::Move);
        h.key_press(Key::E);
        h.step();
        assert_eq!(h.state().gizmo_mode, Mode::Rotate);
        h.key_press(Key::W);
        h.step();
        assert_eq!(h.state().gizmo_mode, Mode::Move);
        for (name, yaw, pitch) in [
            ("Top", -90.0, 89.0),
            ("Side", -90.0, 0.0),
            ("Front", 180.0, 0.0),
            ("Iso", -128.0, 28.0),
        ] {
            h.get_by_label(name).click();
            h.step();
            assert_eq!(
                (h.state().viewport.camera.yaw, h.state().viewport.camera.pitch),
                (yaw, pitch),
                "{name}"
            );
        }
        h.state_mut().viewport.camera.distance = 5000.0;
        h.get_by_label("Fit").click();
        h.step();
        assert!(h.state().viewport.camera.distance < 2000.0);
        h.state_mut().viewport.camera.distance = 5000.0;
        h.key_press(Key::F);
        h.step();
        assert!(h.state().viewport.camera.distance < 2000.0);
        h.get_by_label("ground").click();
        h.get_by_label("COM").click();
        h.get_by_label("snap to holes").click();
        h.step();
        assert!(!h.state().show_grid && !h.state().show_com && !h.state().editor.magnet);
        h.get_by_label("Snap").click();
        h.step();
        assert_eq!(h.state().editor.status, "Select one item to snap");
        h.get_by_label("Example").click();
        h.step();
        assert!(h.state().editor.dirty);
        assert_eq!(h.state().title_name(), "example");
        h.get_by_label("Simulate").click();
        h.step();
        assert_eq!(h.state().tab, Tab::Simulate);
        h.get_by_label("Workbench").click();
        h.step();
        assert_eq!(h.state().tab, Tab::Workbench);
    }

    #[test]
    fn the_library_adds_bricks_and_components_and_shows_thumbnails() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        let n = h.state().editor.children().len();
        h.get_all_by_label("+ add").next().unwrap().click();
        h.step();
        assert_eq!(h.state().editor.children().len(), n + 1);
        assert!(h.state().editor.selected_instances()[0].component.is_some(), "a component instance");
        h.get_all_by_label("+").next().unwrap().click();
        h.step();
        assert_eq!(h.state().editor.children().len(), n + 2);
        assert!(h.state().editor.selected_instances()[0].part.is_some(), "a brick instance");
        // thumbnails arrive a few per frame
        steps(&mut h, 60);
        let dark = h.ctx.theme() == egui::Theme::Dark;
        assert!(
            h.state().viewport.thumb(&format!("thumb:ld:32278:{dark}")).is_some(),
            "a Technic beam's thumbnail"
        );
        let edits = h.state().editor.edits;
        let root = h.state().editor.doc.robot.root.clone();
        assert!(
            h.state().viewport.thumb(&format!("thumb:comp:{root}:{edits}:{dark}")).is_some(),
            "the robot's thumbnail"
        );
        // an edit stamps new component thumbnails and prunes the old
        h.state_mut().editor.nudge_selection([8.0, 0.0, 0.0]);
        steps(&mut h, 3);
        assert!(
            h.state().viewport.thumb(&format!("thumb:comp:{root}:{edits}:{dark}")).is_none(),
            "stale thumbnail dropped"
        );
        // the search narrows the list
        h.state_mut().search = "zzzz-nothing".into();
        h.step();
        assert!(h.query_by_label("+ add").is_none());
        h.state_mut().search.clear();
        h.step();
    }

    #[test]
    fn the_inspector_edits_the_selection_and_makes_components() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        let brick = h.state().editor.children().iter().find(|c| c.part.is_some()).unwrap().name.clone();
        h.state_mut().editor.selection = vec![brick.clone()];
        steps(&mut h, 3);
        let rot0 = h.state().editor.selected_instances()[0].rot;
        for label in ["Turn 90°", "Pitch 90°", "Roll 90°"] {
            h.get_by_label(label).click();
            steps(&mut h, 2);
        }
        let rot = h.state().editor.selected_instances()[0].rot;
        assert_ne!(rot, rot0);
        h.get_by_label("Copy").click();
        steps(&mut h, 2);
        assert_eq!(h.state().editor.status, "Copied 1");
        h.get_by_label("Lock").click();
        steps(&mut h, 3);
        assert!(h.state().editor.is_locked(&brick));
        assert!(h.query_by_label("Lock").is_none() && h.query_by_label("Unlock").is_some());
        h.get_by_label("Unlock").click();
        steps(&mut h, 3);
        assert!(!h.state().editor.is_locked(&brick));
        let n = h.state().editor.children().len();
        h.get_by_label("Duplicate").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.children().len(), n + 1);
        h.get_by_label("Remove").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.children().len(), n);
        // two selected: grouped into a new component, then opened, ungrouped and left
        let names: Vec<String> = h.state().editor.children().iter().take(2).map(|c| c.name.clone()).collect();
        h.state_mut().editor.selection = names.clone();
        h.state_mut().group_name = "Sensor Mast".into();
        steps(&mut h, 3);
        assert!(h.query_by_label("2 selected").is_some());
        h.get_by_label("Group").click();
        steps(&mut h, 3);
        assert!(h.state().editor.doc.components.contains_key("sensor_mast"));
        assert!(h.state().group_name.is_empty());
        h.get_by_label("Open sensor_mast").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.editing, "sensor_mast");
        h.get_by_label("Back to the robot").click();
        steps(&mut h, 3);
        assert!(h.state().editor.is_root());
        let inst = h
            .state()
            .editor
            .children()
            .iter()
            .find(|c| c.component.as_deref() == Some("sensor_mast"))
            .unwrap()
            .name
            .clone();
        h.state_mut().editor.selection = vec![inst];
        steps(&mut h, 3);
        h.get_by_label("Ungroup").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.selection, names);
        h.state_mut().editor.selection.clear();
        steps(&mut h, 2);
        assert!(
            h.query_by_label("Robot").is_some(),
            "no selection at the root: the robot's own page"
        );
    }

    #[test]
    fn keys_and_clipboard_events_edit_the_selection() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        h.state_mut().editor.magnet = false;
        let brick = h.state().editor.children().iter().find(|c| c.part.is_some()).unwrap().name.clone();
        h.state_mut().editor.selection = vec![brick.clone()];
        h.step();
        let start = h.state().editor.selected_instances()[0].clone();
        h.key_press(Key::R);
        h.step();
        assert_eq!(
            h.state().editor.selected_instances()[0].rot[2],
            crate::editor::wrap_deg(start.rot[2] + 90.0)
        );
        h.key_press(Key::ArrowUp);
        h.key_press(Key::ArrowRight);
        h.step();
        let p = h.state().editor.selected_instances()[0].pos;
        assert_eq!((p[0], p[1]), (start.pos[0] + 8.0, start.pos[1] - 8.0));
        h.key_press_modifiers(Modifiers::COMMAND, Key::Z);
        h.key_press_modifiers(Modifiers::COMMAND, Key::Z);
        h.step();
        assert_eq!(h.state().editor.selected_instances()[0].pos, start.pos);
        h.key_press_modifiers(Modifiers::COMMAND, Key::L);
        h.step();
        assert!(h.state().editor.is_locked(&brick));
        h.key_press_modifiers(Modifiers::COMMAND | Modifiers::SHIFT, Key::L);
        h.step();
        assert!(!h.state().editor.is_locked(&brick));
        let n = h.state().editor.children().len();
        h.key_press_modifiers(Modifiers::COMMAND, Key::D);
        h.step();
        assert_eq!(h.state().editor.children().len(), n + 1);
        h.key_press(Key::Delete);
        h.step();
        assert_eq!(h.state().editor.children().len(), n);
        // copy puts our text on the clipboard; paste brings it back
        h.state_mut().editor.selection = vec![brick.clone()];
        h.input_mut().events.push(Event::Copy);
        h.step();
        assert_eq!(h.state().editor.status, "Copied 1");
        let text = h.state().editor.copy_selection().unwrap();
        h.input_mut().events.push(Event::Paste(text));
        h.step();
        assert_eq!(h.state().editor.children().len(), n + 1);
        assert!(h.state().editor.status.starts_with("Pasted 1"));
        h.input_mut().events.push(Event::Paste("just some text".into()));
        h.step();
        assert_eq!(h.state().editor.status, "The clipboard holds no bricks");
        h.input_mut().events.push(Event::Cut);
        h.step();
        assert_eq!(h.state().editor.children().len(), n);
        h.key_press(Key::S);
        h.step();
        assert_eq!(h.state().editor.status, "Select one item to snap");
        h.key_press(Key::Escape);
        h.step();
        assert!(h.state().editor.selection.is_empty());
        h.input_mut().events.push(Event::Copy);
        h.step();
        assert_eq!(h.state().editor.status, "Nothing selected to copy");
        h.input_mut().events.push(Event::Cut);
        h.step();
        assert_eq!(h.state().editor.status, "Nothing selected to cut");
    }

    #[test]
    fn the_pointer_orbits_pans_zooms_picks_moves_and_pulls_handles() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        h.state_mut().editor.magnet = false;
        let rect = h.state().view_rect;
        let corner = Pos2::new(rect.min.x + 12.0, rect.min.y + 12.0);
        // orbit: a drag on empty space turns the camera
        let yaw0 = h.state().viewport.camera.yaw;
        press(&mut h, corner, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, corner + egui::vec2(10.0, 0.0), Modifiers::NONE);
        drag_to(&mut h, corner + egui::vec2(60.0, 0.0), Modifiers::NONE);
        release(&mut h, corner + egui::vec2(60.0, 0.0), PointerButton::Primary);
        assert_ne!(h.state().viewport.camera.yaw, yaw0);
        assert!(h.state().editor.selection.is_empty());
        // pan: the right button moves the target
        let target0 = h.state().viewport.camera.target;
        press(&mut h, corner, PointerButton::Secondary, Modifiers::NONE);
        drag_to(&mut h, corner + egui::vec2(10.0, 10.0), Modifiers::NONE);
        drag_to(&mut h, corner + egui::vec2(50.0, 40.0), Modifiers::NONE);
        release(&mut h, corner + egui::vec2(50.0, 40.0), PointerButton::Secondary);
        assert_ne!(h.state().viewport.camera.target, target0);
        // zoom: the wheel changes the distance
        let d0 = h.state().viewport.camera.distance;
        h.input_mut().events.push(Event::PointerMoved(rect.center()));
        h.input_mut().events.push(Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point,
            delta: egui::vec2(0.0, -40.0),
            modifiers: Modifiers::NONE,
            phase: egui::TouchPhase::Move,
        });
        steps(&mut h, 2);
        assert!(
            h.state().viewport.camera.distance > d0,
            "{} vs {d0}",
            h.state().viewport.camera.distance
        );
        h.get_by_label("Fit").click();
        steps(&mut h, 2);
        // a drag from a brick selects the one in front and moves it on the ground plane in 8 mm steps
        let before: std::collections::HashMap<String, [f64; 3]> =
            h.state().editor.children().iter().map(|c| (c.name.clone(), c.pos)).collect();
        let leaf = h.state().editor.leaves.iter().find(|l| l.path.len() == 1).unwrap().clone();
        let at = on_screen(h.state(), leaf.pos.as_vec3());
        assert!(rect.contains(at), "{at:?} in {rect:?}");
        press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(8.0, 0.0), Modifiers::NONE);
        assert_eq!(h.state().editor.selection.len(), 1, "the brick under the pointer");
        let top = h.state().editor.selection[0].clone();
        let start = before[&top];
        drag_to(&mut h, at + egui::vec2(90.0, 0.0), Modifiers::NONE);
        release(&mut h, at + egui::vec2(90.0, 0.0), PointerButton::Primary);
        assert_eq!(h.state().editor.selection, vec![top.clone()]);
        let moved = h.state().editor.children().iter().find(|c| c.name == top).unwrap().pos;
        assert_ne!(moved, start);
        assert!((moved[0] / 8.0).fract() == 0.0 && (moved[1] / 8.0).fract() == 0.0, "{moved:?}");
        // shift lifts: pressed on the brick's body, away from its origin where the handles meet
        let body = {
            let leaf = h.state().editor.leaves.iter().find(|l| l.path == [top.clone()]).unwrap().clone();
            let part = h.state().editor.doc.parts[&leaf.part_id].clone();
            let pr = assembly::part_props(&part, &h.state().editor.bundle);
            let bb = pr.bbox.unwrap();
            let size = bb.size();
            let axis = if size.x >= size.y && size.x >= size.z {
                0
            } else if size.y >= size.z {
                1
            } else {
                2
            };
            let mut local = (bb.min + bb.max) * 0.5;
            local[axis] = bb.min[axis] + size[axis] * 0.85;
            (leaf.pos + leaf.rot * local).as_vec3()
        };
        let at = on_screen(h.state(), body);
        let before: std::collections::HashMap<String, [f64; 3]> =
            h.state().editor.children().iter().map(|c| (c.name.clone(), c.pos)).collect();
        press(&mut h, at, PointerButton::Primary, Modifiers::SHIFT);
        drag_to(&mut h, at + egui::vec2(0.0, -8.0), Modifiers::SHIFT);
        // whichever brick is nearest along that ray is the one lifted
        let top = h.state().editor.selection[0].clone();
        drag_to(&mut h, at + egui::vec2(0.0, -60.0), Modifiers::SHIFT);
        release(&mut h, at + egui::vec2(0.0, -60.0), PointerButton::Primary);
        let moved = before[&top];
        let lifted = h.state().editor.children().iter().find(|c| c.name == top).unwrap().pos;
        assert!(lifted[2] > moved[2], "{lifted:?} above {moved:?}");
        // the z arrow: a handle drag along the axis
        let centre = Vec3::new(lifted[0] as f32, lifted[1] as f32, lifted[2] as f32);
        let g = Gizmo::new(centre, Mode::Move, &h.state().viewport.camera, rect.height());
        let tip = on_screen(h.state(), centre + Vec3::Z * g.length * 0.5);
        press(&mut h, tip, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, tip + egui::vec2(0.0, -10.0), Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::Handle { .. }), "the arrow was grabbed");
        assert_eq!(h.state().hot, Some(Handle::Axis(2)));
        drag_to(&mut h, tip + egui::vec2(0.0, -80.0), Modifiers::NONE);
        release(&mut h, tip + egui::vec2(0.0, -80.0), PointerButton::Primary);
        let pulled = h.state().editor.children().iter().find(|c| c.name == top).unwrap().pos;
        assert!(pulled[2] > lifted[2] + 4.0, "{pulled:?} above {lifted:?}");
        assert!(matches!(h.state().drag, Drag::None));
        // a ring turns it: seen from the top, the z ring is a circle and the others are lines
        h.state_mut().gizmo_mode = Mode::Rotate;
        h.get_by_label("Top").click();
        h.get_by_label("Fit").click();
        steps(&mut h, 2);
        h.state_mut().viewport.camera.distance *= 1.6; // room for the ring around a brick near the edge
        steps(&mut h, 2);
        let centre = Vec3::new(pulled[0] as f32, pulled[1] as f32, pulled[2] as f32);
        let g = Gizmo::new(centre, Mode::Rotate, &h.state().viewport.camera, rect.height());
        // 45° along the z ring: a point no other ring passes through
        let on_ring = on_screen(h.state(), centre + (Vec3::X + Vec3::Y) * (0.5f32.sqrt() * g.ring_radius()));
        assert!(rect.contains(on_ring), "{on_ring:?} in {rect:?}");
        let rot0 = h.state().editor.children().iter().find(|c| c.name == top).unwrap().rot;
        press(&mut h, on_ring, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, on_ring + egui::vec2(0.0, 10.0), Modifiers::NONE);
        assert!(
            matches!(
                h.state().drag,
                Drag::Handle {
                    handle: Handle::Ring(2),
                    ..
                }
            ),
            "{:?}",
            h.state().hot
        );
        drag_to(&mut h, on_ring + egui::vec2(0.0, 70.0), Modifiers::NONE);
        release(&mut h, on_ring + egui::vec2(0.0, 70.0), PointerButton::Primary);
        let rot = h.state().editor.children().iter().find(|c| c.name == top).unwrap().rot;
        assert_ne!(rot, rot0);
        // a double-click on a component instance opens it: find a brick of one that is in front
        h.get_by_label("Iso").click();
        h.get_by_label("Fit").click();
        steps(&mut h, 2);
        let rect = h.state().view_rect;
        let mut target = None;
        for leaf in h.state().editor.leaves.iter().filter(|l| l.path.len() > 1) {
            let at = on_screen(h.state(), leaf.pos.as_vec3());
            if !rect.contains(at) {
                continue;
            }
            let app = h.state();
            if let Some(i) = app.viewport.pick(&app.items, at.x - rect.min.x, at.y - rect.min.y)
                && let Some(cid) = app.editor.component_of(&app.item_tops[i])
            {
                target = Some((at, cid));
                break;
            }
        }
        let (at, cid) = target.expect("a component instance in view");
        steps(&mut h, 40); // well clear of the toolbar clicks, which egui would chain into the count
        for _ in 0..2 {
            press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
            release(&mut h, at, PointerButton::Primary);
        }
        h.step();
        assert_eq!(h.state().editor.editing, cid);
    }

    #[test]
    fn the_simulate_tab_runs_a_program_on_the_stand_in_server() {
        let Some(gpu) = gpu() else { return };
        let Some(fake) = fake_server("app") else { return };
        let mut h = harness(&gpu, None);
        h.state_mut().simulate = SimulateTab::new_with_env(Some(fake.python.clone()), fake.env.clone());
        h.get_by_label("Simulate").click();
        h.step();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        while !h.state().simulate.scene_loaded() && std::time::Instant::now() < deadline {
            h.step();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert!(h.state().simulate.scene_loaded(), "{}", h.state().simulate.status());
        assert!(h.state().view_rect.width() > 300.0);
        let script = fake.dir.join("main.py");
        std::fs::write(&script, "print('hi')\n").unwrap();
        h.state_mut().simulate.set_script(script);
        h.step();
        h.get_by_label("▶ Run").click();
        let wait = |h: &mut Harness<'_, App>, status: &str| {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
            while h.state().simulate.status() != status && std::time::Instant::now() < deadline {
                h.step();
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            assert_eq!(h.state().simulate.status(), status);
        };
        wait(&mut h, "running");
        h.get_by_label("⏸ Pause").click();
        wait(&mut h, "paused");
        h.get_by_label("▶ Resume").click();
        wait(&mut h, "running");
        h.get_by_label("⏹ Stop").click();
        wait(&mut h, "stopped");
        assert!(
            h.state()
                .simulate
                .log
                .iter()
                .any(|(s, t)| s == "stdout" && t.starts_with("hello from"))
        );
        h.get_by_label("clear").click();
        h.step();
        assert!(h.state().simulate.log.is_empty());
        // the sim view orbits and pans like the workbench's
        let rect = h.state().view_rect;
        let yaw0 = h.state().viewport.camera.yaw;
        // away from the chassis, which the run parked near the mat's centre
        let at = rect.center() + egui::vec2(300.0, 0.0);
        press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(10.0, 0.0), Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::Orbit), "empty map: orbit");
        drag_to(&mut h, at + egui::vec2(50.0, 0.0), Modifiers::NONE);
        release(&mut h, at + egui::vec2(50.0, 0.0), PointerButton::Primary);
        assert_ne!(h.state().viewport.camera.yaw, yaw0);
        let target0 = h.state().viewport.camera.target;
        press(&mut h, at, PointerButton::Secondary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(10.0, 10.0), Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(40.0, 40.0), Modifiers::NONE);
        release(&mut h, at + egui::vec2(40.0, 40.0), PointerButton::Secondary);
        assert_ne!(h.state().viewport.camera.target, target0);
        // following keeps the camera on the chassis whatever the pan
        h.get_by_label("follow the robot").click();
        steps(&mut h, 2);
        assert!(h.state().simulate.follows());
        let on_robot = h.state().viewport.camera.target;
        press(&mut h, at, PointerButton::Secondary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(10.0, 10.0), Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(40.0, 40.0), Modifiers::NONE);
        release(&mut h, at + egui::vec2(40.0, 40.0), PointerButton::Secondary);
        assert_eq!(h.state().viewport.camera.target, on_robot);
        // the workbench's build is offered once saved
        assert!(h.get_by_label("Use the workbench's build").accesskit_node().is_disabled());
        let path = fake.dir.join("robot.assembly.json");
        h.state_mut().editor.save_to(&path);
        h.step();
        h.get_by_label("Use the workbench's build").click();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        while !h.state().simulate.log.iter().any(|(_, t)| t.ends_with("robot.assembly.json")) && std::time::Instant::now() < deadline {
            h.step();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert!(
            h.state().simulate.log.iter().any(|(_, t)| t.ends_with("robot.assembly.json")),
            "{:?}",
            h.state().simulate.log
        );
        h.state_mut().simulate.shutdown();
        let _ = std::fs::remove_dir_all(&fake.dir);
    }

    #[test]
    fn the_route_panel_plans_on_the_map_and_the_chassis_drags_into_place() {
        let Some(gpu) = gpu() else { return };
        let Some(fake) = fake_server("route") else { return };
        let mut h = harness(&gpu, None);
        h.state_mut().simulate = SimulateTab::new_with_env(Some(fake.python.clone()), fake.env.clone());
        h.get_by_label("Simulate").click();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        while !h.state().simulate.scene_loaded() && std::time::Instant::now() < deadline {
            h.step();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert!(h.state().simulate.scene_loaded(), "{}", h.state().simulate.status());
        steps(&mut h, 3);
        assert!(h.query_by_label("Route").is_some());
        // the chassis stands at its spawn; a drag on it places it, and the server hears
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        while h.state().simulate.chassis_pose().map(|p| p.x_mm > -500.0).unwrap_or(true) && std::time::Instant::now() < deadline {
            h.step();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let p0 = h.state().simulate.chassis_pose().expect("the chassis's pose from the frames");
        assert!((p0.x_mm + 547.0).abs() < 1e-3 && (p0.yaw_deg - 90.0).abs() < 1e-3, "{p0:?}");
        // seen from the top, framed on the mat as the tab framed it (the toolbar's Fit frames the workbench)
        h.get_by_label("Top").click();
        steps(&mut h, 3);
        let rect = h.state().view_rect;
        let at = on_screen(h.state(), Vec3::new(p0.x_mm as f32, p0.y_mm as f32, 50.0));
        assert!(rect.contains(at), "{at:?} in {rect:?}");
        press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(8.0, 0.0), Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::Chassis { .. }), "the chassis was grabbed");
        drag_to(&mut h, at + egui::vec2(120.0, 0.0), Modifiers::NONE);
        let preview = h.state().simulate.chassis_pose().unwrap();
        assert!(preview.x_mm > p0.x_mm + 50.0, "the preview follows the pointer: {preview:?}");
        release(&mut h, at + egui::vec2(120.0, 0.0), PointerButton::Primary);
        let placed = h.state().simulate.route.start;
        assert!(placed.x_mm > p0.x_mm + 50.0, "{placed:?}");
        assert_eq!(h.state().simulate.sent.last().unwrap()["cmd"], "place");
        // the server's next frame shows it there; then shift turns it
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        while h
            .state()
            .simulate
            .chassis_pose()
            .map(|p| (p.x_mm - placed.x_mm).abs() > 1e-3)
            .unwrap_or(true)
            && std::time::Instant::now() < deadline
        {
            h.step();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        steps(&mut h, 2);
        let at = on_screen(h.state(), Vec3::new(placed.x_mm as f32, placed.y_mm as f32, 50.0));
        press(&mut h, at, PointerButton::Primary, Modifiers::SHIFT);
        drag_to(&mut h, at + egui::vec2(8.0, 0.0), Modifiers::SHIFT);
        drag_to(&mut h, at + egui::vec2(100.0, 0.0), Modifiers::SHIFT);
        release(&mut h, at + egui::vec2(100.0, 0.0), PointerButton::Primary);
        let turned = h.state().simulate.route.start;
        assert!(
            (turned.x_mm - placed.x_mm).abs() < 1e-6 && turned.yaw_deg < placed.yaw_deg,
            "{turned:?} from {placed:?}"
        );
        // arm a straight segment and click the map where it ends
        h.get_by_label("+ Straight to…").click();
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.pick, Some(crate::route::Kind::Straight));
        assert!(h.query_by_label("click the map where the straight segment ends").is_some());
        let end = on_screen(h.state(), Vec3::new((turned.x_mm + 250.0) as f32, turned.y_mm as f32, 0.0));
        press(&mut h, end, PointerButton::Primary, Modifiers::NONE);
        release(&mut h, end, PointerButton::Primary);
        steps(&mut h, 2);
        assert_eq!(
            h.state().simulate.route.segments.len(),
            1,
            "{:?}",
            h.state().simulate.route.segments
        );
        assert!(h.state().simulate.pick.is_none());
        let to = h.state().simulate.route.segments[0].to;
        assert!(
            (to[0] - turned.x_mm - 250.0).abs() < 3.0 && (to[1] - turned.y_mm).abs() < 3.0,
            "{to:?}"
        );
        assert!(h.query_by_label("straight").is_some());
        // Escape cancels an armed pick; a curve is armed the same way
        h.get_by_label("+ Curve to…").click();
        h.step();
        h.key_press(Key::Escape);
        steps(&mut h, 2);
        assert!(h.state().simulate.pick.is_none());
        // the program and the run
        h.get_by_label("show the program").click();
        steps(&mut h, 2);
        assert!(h.query_by_label("show the program").is_some());
        h.get_by_label("▶ Run route").click();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        while h.state().simulate.status() != "running" && std::time::Instant::now() < deadline {
            h.step();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert_eq!(h.state().simulate.status(), "running");
        assert!(
            h.state().simulate.log.iter().any(|(_, t)| t.contains("openbricks-route-")),
            "{:?}",
            h.state().simulate.log
        );
        h.get_by_label("⏹ Stop").click();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        while h.state().simulate.status() != "stopped" && std::time::Instant::now() < deadline {
            h.step();
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        h.get_by_label("Undo last").click();
        steps(&mut h, 2);
        assert!(h.state().simulate.route.segments.is_empty());
        h.state_mut().simulate.shutdown();
        let _ = std::fs::remove_dir_all(&fake.dir);
    }

    #[test]
    fn without_python_the_simulate_tab_says_so() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        h.get_by_label("Simulate").click();
        steps(&mut h, 2);
        assert!(h.query_by_label("The map appears here once the run server has built it").is_some() || !h.state().simulate.scene_loaded());
        assert!(h.state().simulate.status().contains("no run server") || !h.state().simulate.is_live());
    }

    #[test]
    fn the_stl_window_adds_a_part_or_cancels() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        let tris: Vec<stl::Tri> = {
            let m = geometry::box_mesh([20.0, 10.0, 5.0], [0.0; 3]);
            m.indices
                .as_chunks::<3>()
                .0
                .iter()
                .map(|t| [m.positions[t[0] as usize], m.positions[t[1] as usize], m.positions[t[2] as usize]])
                .collect()
        };
        let imp = stl::Import::new("bracket.stl", tris.clone()).unwrap();
        h.state_mut().stl_prepared = Some(imp.prepare());
        h.state_mut().stl = Some(imp);
        steps(&mut h, 3);
        assert!(h.query_by_label("Add to the library").is_some());
        h.get_by_label("PETG").click();
        h.step();
        assert_eq!(h.state().stl.as_ref().unwrap().density, 1.27);
        assert!(h.state().stl_prepared.as_ref().unwrap().summary.contains("1.27"));
        let n = h.state().editor.children().len();
        h.get_by_label("Add to the library").click();
        h.step();
        assert!(h.state().stl.is_none());
        assert_eq!(h.state().editor.children().len(), n + 1);
        assert!(h.state().editor.doc.parts.contains_key("bracket"));
        assert!(h.state().editor.status.ends_with("is in the library and in the view"));
        steps(&mut h, 2);
        assert!(h.state().items.len() > n, "the imported part is drawn");
        let imp = stl::Import::new("other.stl", tris).unwrap();
        h.state_mut().stl_prepared = Some(imp.prepare());
        h.state_mut().stl = Some(imp);
        h.step();
        h.get_by_label("Cancel").click();
        h.step();
        assert!(h.state().stl.is_none());
        assert_eq!(h.state().editor.children().len(), n + 1);
    }

    #[test]
    fn the_inspector_shows_a_component_page_and_a_brick_page() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        let cid = h
            .state()
            .editor
            .doc
            .components
            .keys()
            .find(|k| **k != h.state().editor.doc.robot.root)
            .unwrap()
            .clone();
        h.state_mut().editor.open_component(&cid, true);
        h.step();
        assert!(h.query_by_label(&format!("Component {cid}")).is_some());
        h.get_by_label("Back to the robot").click();
        h.step();
        assert!(h.state().editor.is_root());
        // a brick instance's page lists its recorded facts and connections
        let brick = h.state().editor.children().iter().find(|c| c.part.is_some()).unwrap().name.clone();
        h.state_mut().editor.selection = vec![brick];
        h.step();
        assert!(h.query_by_label("Brick instance").is_some());
        assert!(h.query_by_label("Recorded  (the brick's own facts)").is_some());
        // a locked brick's page says so and its pose fields are disabled
        h.state_mut().editor.lock_selection(true);
        h.step();
        assert!(h.query_by_label("Brick instance  🔒 locked").is_some());
        assert!(h.query_by_label("Unlock").is_some());
    }
}
