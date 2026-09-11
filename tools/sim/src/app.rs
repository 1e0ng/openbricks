//! The application: the Workbench tab (library, viewport, tree,
//! inspector) drawn over the editor's state, and the Simulate tab.
//! Everything that changes the document lives in `editor`; this file
//! only draws it and feeds it input.

use crate::assembly::{self, Geometry, Instance, Part, Props};
use crate::bundle::MeshData;
use crate::editor::Editor;
use crate::geometry;
use crate::gizmo::{self, Gizmo, Handle, Mode};
use crate::simulate::SimulateTab;
use crate::viewport::{self, DrawItem, Line, Viewport, srgb};
use eframe::egui;
use glam::{DVec3, Mat4, Quat, Vec3};
use std::collections::HashSet;

#[derive(PartialEq, Clone, Copy)]
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
    items: Vec<DrawItem>,
    item_tops: Vec<String>,
    simulate: SimulateTab,
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
const RED: egui::Color32 = egui::Color32::from_rgb(196, 68, 42);

impl App {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        bundle: crate::bundle::Bundle,
        doc: Option<(std::path::PathBuf, assembly::Document)>,
        python: Option<String>,
    ) -> Self {
        let state = cc.wgpu_render_state.as_ref().expect("the wgpu renderer is required");
        let viewport = Viewport::new(&state.device, &state.queue);
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
            items: vec![],
            item_tops: vec![],
            simulate: SimulateTab::new(python),
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
            let key = Self::mesh_key(&leaf.part_id, &part);
            if !self.viewport.has_mesh(&key) {
                match self.mesh_for(&part) {
                    Some(m) => self.viewport.add_mesh(device, &key, &m),
                    None => continue,
                }
            }
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

    fn viewport_ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let dark = ui.visuals().dark_mode;
        let avail = ui.available_size();
        let size = ((avail.x.max(1.0)) as u32, (avail.y.max(1.0)) as u32);
        let Some(state) = frame.wgpu_render_state() else {
            ui.label("wgpu is not available");
            return;
        };
        self.ensure_gizmo_meshes(&state.device);
        self.build_items(&state.device, dark);
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
            let mut renderer = state.renderer.write();
            let scene = viewport::Scene {
                items: &self.items,
                lines: &lines,
                overlay: &overlay,
                background: bg,
            };
            self.viewport.render(&state.device, &state.queue, &mut renderer, size, &scene)
        };
        let response = ui.add(egui::Image::new((tex, egui::vec2(w, h))).sense(egui::Sense::click_and_drag()));
        let rect = response.rect;
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
        // press: a handle first, then a brick
        let pressed = response.drag_started_by(egui::PointerButton::Primary) || response.clicked_by(egui::PointerButton::Primary);
        let pointer = response.interact_pointer_pos().map(local);
        let on_handle = if pressed {
            pointer.and_then(|(x, y)| handle_under(x, y))
        } else {
            None
        };
        if let (Some(handle), Some(g), true) = (on_handle, gizmo.as_ref(), response.drag_started_by(egui::PointerButton::Primary)) {
            let (x, y) = pointer.unwrap_or((0.0, 0.0));
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
            let hit = pointer
                .and_then(|(x, y)| self.viewport.pick(&self.items, x, y))
                .map(|i| self.item_tops[i].clone());
            match hit {
                Some(top) => {
                    self.editor.select(&top, shift);
                    if response.drag_started_by(egui::PointerButton::Primary) {
                        let z0 = self.editor.selected_instances().first().map(|i| i.pos[2] as f32).unwrap_or(0.0);
                        let (x, y) = pointer.unwrap_or((0.0, 0.0));
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
        if response.double_clicked() {
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

    fn library_ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("Library");
        ui.add(egui::TextEdit::singleline(&mut self.search).hint_text("search bricks and components"));
        let q = self.search.trim().to_lowercase();
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
                ui.horizontal(|ui| {
                    ui.monospace(&id);
                    ui.weak(format!("{} g Σ · used ×{uses}", assembly::fmt(mass)));
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
            let mut nums: Vec<&String> = self.editor.bundle.parts.keys().collect();
            nums.sort_by_key(|n| self.editor.bundle.parts[*n].name.to_lowercase());
            for num in nums {
                let rec = &self.editor.bundle.parts[num];
                if !(q.is_empty() || num.contains(&q) || rec.name.to_lowercase().contains(&q)) {
                    continue;
                }
                let dens = if rec.volume_mm3 > 0.0 {
                    rec.mass_g / (rec.volume_mm3 / 1000.0)
                } else {
                    0.0
                };
                ui.horizontal(|ui| {
                    if ui.small_button("+").on_hover_text("add to the view").clicked() {
                        to_ldraw = Some(num.clone());
                    }
                    ui.label(&rec.name);
                    ui.weak(format!("{num} · {} g · {:.2} g/cm³", assembly::fmt(rec.mass_g), dens));
                });
            }
            ui.add_space(8.0);
            ui.strong("Other bricks");
            let ids: Vec<String> = self
                .editor
                .doc
                .parts
                .iter()
                .filter(|(_, p)| p.ldraw.is_none())
                .map(|(k, _)| k.clone())
                .collect();
            for id in ids {
                let p = &self.editor.doc.parts[&id];
                if !(q.is_empty() || p.name.to_lowercase().contains(&q) || id.contains(&q)) {
                    continue;
                }
                ui.horizontal(|ui| {
                    if ui.small_button("+").clicked() {
                        to_add = Some((Some(id.clone()), None));
                    }
                    ui.label(&p.name);
                    ui.weak(format!("{} g · {}", assembly::fmt(p.mass_g), p.source));
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
        });
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
                            ui.label(format!(
                                "LDraw {} exact mesh, {} triangles, {} mm",
                                rec.ldraw,
                                rec.mesh.tris,
                                bbox_text(&rec.bbox)
                            ));
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
                    ui.label(format!(
                        "{}{}",
                        part.source,
                        if part.source_note.is_empty() {
                            String::new()
                        } else {
                            format!(" — {}", part.source_note)
                        }
                    ));
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

    fn simulate_ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
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
        egui::CentralPanel::default().show(ui, |ui| self.sim_view_ui(ui, frame));
    }

    fn sim_view_ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let dark = ui.visuals().dark_mode;
        let avail = ui.available_size();
        let size = ((avail.x.max(1.0)) as u32, (avail.y.max(1.0)) as u32);
        let Some(state) = frame.wgpu_render_state() else {
            ui.label("wgpu is not available");
            return;
        };
        let (items, lines) = self
            .simulate
            .draw_items(&mut self.viewport, &state.device, &state.queue, &self.editor.bundle, dark);
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
            let mut renderer = state.renderer.write();
            let scene = viewport::Scene {
                items: &items,
                lines: &lines,
                overlay: &[],
                background: bg,
            };
            self.viewport.render(&state.device, &state.queue, &mut renderer, size, &scene)
        };
        let response = ui.add(egui::Image::new((tex, egui::vec2(size.0 as f32, size.1 as f32))).sense(egui::Sense::click_and_drag()));
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                let f = (1.0 - scroll * 0.002).clamp(0.5, 2.0);
                self.viewport.camera.distance = (self.viewport.camera.distance * f).clamp(20.0, 50000.0);
            }
        }
        let delta = response.drag_delta();
        if response.dragged_by(egui::PointerButton::Primary) {
            self.viewport.camera.yaw -= delta.x * 0.5;
            self.viewport.camera.pitch = (self.viewport.camera.pitch + delta.y * 0.5).clamp(-89.0, 89.0);
        } else if response.dragged_by(egui::PointerButton::Secondary) || response.dragged_by(egui::PointerButton::Middle) {
            let cam = &mut self.viewport.camera;
            let scale = cam.distance * 0.0015;
            let (r, u) = (cam.right(), cam.up());
            cam.target = cam.target - r * delta.x * scale + u * delta.y * scale;
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
            "orbit: drag · pan: right-drag · zoom: wheel",
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

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        egui::Panel::top("toolbar").show(ui, |ui| self.toolbar(ui));
        egui::Panel::bottom("status").show(ui, |ui| self.status_bar(ui));
        match self.tab {
            Tab::Simulate => {
                self.simulate_ui(ui, frame);
            }
            Tab::Workbench => {
                egui::Panel::left("library")
                    .default_size(300.0)
                    .resizable(true)
                    .show(ui, |ui| self.library_ui(ui));
                egui::Panel::right("inspector").default_size(360.0).resizable(true).show(ui, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        self.tree_ui(ui);
                        ui.separator();
                        self.inspector_ui(ui);
                    });
                });
                egui::CentralPanel::default().show(ui, |ui| self.viewport_ui(ui, frame));
            }
        }
        if self.editor.dirty {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::Title(format!("Openbricks Sim — {}*", self.title_name())));
        }
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
}
