//! The application: the Workbench tab (library, viewport, tree,
//! inspector) and the Simulate tab.

use crate::assembly::{self, Document, Geometry, Instance, Part, Props};
use crate::bundle::{Bundle, MeshData};
use crate::geometry;
use crate::gizmo::{self, Gizmo, Handle, Mode};
use crate::simulate::SimulateTab;
use crate::viewport::{self, DrawItem, Line, Viewport, srgb};
use eframe::egui;
use glam::{DMat3, DVec3, Mat4, Quat, Vec3};
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;

const MODULE_MM: f64 = 8.0;

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
    /// the pivot, and every selected instance's starting pose.
    Handle {
        handle: Handle,
        start: f32,
        pivot: Vec3,
        starts: Vec<(String, [f64; 3], [f64; 3])>,
    },
}

pub struct App {
    bundle: Bundle,
    doc: Document,
    path: Option<PathBuf>,
    editing: String,
    crumbs: Vec<String>,
    selection: Vec<String>,
    undo: Vec<Document>,
    dirty: bool,
    tab: Tab,
    search: String,
    group_name: String,
    snap_mm: f64,
    magnet: bool,
    gizmo_mode: Mode,
    hot: Option<Handle>,
    show_grid: bool,
    show_com: bool,
    status: String,
    errors: Vec<String>,
    viewport: Viewport,
    drag: Drag,
    view_size: (u32, u32),
    fit_pending: bool,
    memo: HashMap<String, Props>,
    root_props: Props,
    leaves: Vec<assembly::Leaf>,
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

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>, bundle: Bundle, doc: Option<(PathBuf, Document)>, python: Option<String>) -> Self {
        let state = cc.wgpu_render_state.as_ref().expect("the wgpu renderer is required");
        let viewport = Viewport::new(&state.device, &state.queue);
        let (path, doc) = match doc {
            Some((p, d)) => (Some(p), d),
            None => (None, assembly::example()),
        };
        let editing = doc.robot.root.clone();
        let mut app = App {
            bundle,
            doc,
            path,
            editing: editing.clone(),
            crumbs: vec![editing],
            selection: vec![],
            undo: vec![],
            dirty: false,
            tab: Tab::Workbench,
            search: String::new(),
            group_name: String::new(),
            snap_mm: MODULE_MM,
            magnet: true,
            gizmo_mode: Mode::Move,
            hot: None,
            show_grid: true,
            show_com: true,
            status: String::new(),
            errors: vec![],
            viewport,
            drag: Drag::None,
            view_size: (1, 1),
            fit_pending: true,
            memo: HashMap::new(),
            root_props: Props {
                mass: 0.0,
                com: DVec3::ZERO,
                inertia: DMat3::ZERO,
                bbox: None,
                count: 0,
            },
            leaves: vec![],
            items: vec![],
            item_tops: vec![],
            simulate: SimulateTab::new(python),
        };
        app.errors = assembly::validate(&app.doc, &app.bundle);
        app.recompute();
        app
    }

    // ------------------------------------------------------------ model

    fn recompute(&mut self) {
        self.memo.clear();
        let mut errs = vec![];
        let root = self.doc.robot.root.clone();
        self.root_props = assembly::component_props(&self.doc, &self.bundle, &root, &mut self.memo, &mut vec![], &mut errs);
        if !self.doc.components.contains_key(&self.editing) {
            self.editing = root.clone();
            self.crumbs = vec![root];
            self.selection.clear();
        }
        let editing = self.editing.clone();
        assembly::component_props(&self.doc, &self.bundle, &editing, &mut self.memo, &mut vec![], &mut errs);
        self.leaves = assembly::flatten(&self.doc, &editing);
        let names: HashSet<String> = self.doc.components[&editing].children.iter().map(|c| c.name.clone()).collect();
        self.selection.retain(|n| names.contains(n));
    }

    fn edited_props(&mut self) -> Props {
        let editing = self.editing.clone();
        let mut errs = vec![];
        assembly::component_props(&self.doc, &self.bundle, &editing, &mut self.memo, &mut vec![], &mut errs)
    }

    fn push_undo(&mut self) {
        self.undo.push(self.doc.clone());
        if self.undo.len() > 60 {
            self.undo.remove(0);
        }
        self.dirty = true;
    }

    fn undo(&mut self) {
        if let Some(d) = self.undo.pop() {
            self.doc = d;
            self.recompute();
            self.status = "Undone".into();
        } else {
            self.status = "Nothing to undo".into();
        }
    }

    fn selected_instances(&self) -> Vec<Instance> {
        let comp = &self.doc.components[&self.editing];
        self.selection
            .iter()
            .filter_map(|n| comp.children.iter().find(|c| &c.name == n).cloned())
            .collect()
    }

    fn set_instance(&mut self, name: &str, f: impl FnOnce(&mut Instance)) {
        if let Some(inst) = self
            .doc
            .components
            .get_mut(&self.editing)
            .and_then(|c| c.children.iter_mut().find(|c| c.name == name))
        {
            f(inst);
        }
    }

    fn open_component(&mut self, id: &str, push: bool) {
        if !self.doc.components.contains_key(id) {
            return;
        }
        if push {
            self.crumbs.push(id.to_string());
        } else if let Some(i) = self.crumbs.iter().position(|c| c == id) {
            self.crumbs.truncate(i + 1);
        }
        self.editing = id.to_string();
        self.selection.clear();
        self.fit_pending = true;
        self.recompute();
    }

    fn add_instance(&mut self, part: Option<String>, component: Option<String>, pos: [f64; 3]) {
        if let Some(c) = &component
            && assembly::component_contains(&self.doc, c, &self.editing, &mut vec![])
        {
            self.status = format!("That would put {c} inside itself");
            return;
        }
        self.push_undo();
        let base = part.clone().or(component.clone()).unwrap_or_default();
        let editing = self.editing.clone();
        let comp = self.doc.components.get_mut(&editing).unwrap();
        let name = if comp.children.iter().any(|c| c.name == base) {
            assembly::unique_name(&comp.children, &base)
        } else {
            base
        };
        comp.children.push(Instance {
            name: name.clone(),
            part,
            component,
            pos,
            rot: [0.0; 3],
        });
        self.selection = vec![name.clone()];
        self.recompute();
        if self.magnet {
            self.snap_selection(false);
        }
        self.status = format!("Added {name} to {editing}");
    }

    fn ensure_ldraw_part(&mut self, num: &str) -> Option<String> {
        let rec = self.bundle.parts.get(num)?;
        let id = format!("lego_{num}");
        if !self.doc.parts.contains_key(&id) {
            self.doc.parts.insert(
                id.clone(),
                Part {
                    name: rec.name.clone(),
                    category: "lego".into(),
                    mass_g: rec.mass_g,
                    source: rec.source.clone(),
                    source_note: rec.source_note.clone(),
                    ldraw: Some(num.to_string()),
                    shapes: vec![],
                    extra: Default::default(),
                },
            );
        }
        Some(id)
    }

    fn remove_selection(&mut self) {
        if self.selection.is_empty() {
            return;
        }
        self.push_undo();
        let names: HashSet<String> = self.selection.iter().cloned().collect();
        let editing = self.editing.clone();
        let root = self.doc.robot.root.clone();
        let comps = self.doc.components.clone();
        for value in self.doc.robot.roles.values_mut() {
            let segs: Vec<&str> = value.split('/').collect();
            let mut comp = Some(root.clone());
            for seg in segs {
                if comp.as_deref() == Some(editing.as_str()) && names.contains(seg) {
                    value.clear();
                    break;
                }
                comp = comp
                    .as_deref()
                    .and_then(|c| comps.get(c))
                    .and_then(|c| c.children.iter().find(|ch| ch.name == seg))
                    .and_then(|ch| ch.component.clone());
            }
        }
        self.doc
            .components
            .get_mut(&editing)
            .unwrap()
            .children
            .retain(|c| !names.contains(&c.name));
        self.status = format!("Removed {}", names.len());
        self.selection.clear();
        self.recompute();
    }

    fn duplicate_selection(&mut self) {
        let insts = self.selected_instances();
        if insts.is_empty() {
            return;
        }
        self.push_undo();
        let editing = self.editing.clone();
        let comp = self.doc.components.get_mut(&editing).unwrap();
        let mut made = vec![];
        for inst in insts {
            let mut copy = inst.clone();
            copy.name = assembly::unique_name(&comp.children, &inst.name);
            copy.pos[1] = assembly::round3(copy.pos[1] - 2.0 * MODULE_MM);
            let idx = comp
                .children
                .iter()
                .position(|c| c.name == inst.name)
                .map(|i| i + 1)
                .unwrap_or(comp.children.len());
            comp.children.insert(idx, copy.clone());
            made.push(copy.name);
        }
        self.selection = made;
        self.recompute();
    }

    fn rotate_selection(&mut self, axis: usize, deg: f64) {
        if self.selection.is_empty() {
            return;
        }
        self.push_undo();
        for name in self.selection.clone() {
            self.set_instance(&name, |i| i.rot[axis] = ((i.rot[axis] + deg + 180.0).rem_euclid(360.0)) - 180.0);
        }
        self.recompute();
    }

    fn nudge_selection(&mut self, d: [f64; 3]) {
        if self.selection.is_empty() {
            return;
        }
        self.push_undo();
        for name in self.selection.clone() {
            self.set_instance(&name, |i| {
                for (k, dk) in d.iter().enumerate() {
                    i.pos[k] = assembly::round3(i.pos[k] + dk);
                }
            });
        }
        self.recompute();
    }

    fn snap_selection(&mut self, announce: bool) {
        if self.selection.len() != 1 {
            if announce {
                self.status = "Select one item to snap".into();
            }
            return;
        }
        let name = self.selection[0].clone();
        let before = self.doc.clone();
        match assembly::snap_instance(&mut self.doc, &self.bundle, &self.editing, &name) {
            Some(path) => {
                self.undo.push(before);
                self.dirty = true;
                self.status = format!("Snapped {name} into {}", path.join("/"));
                self.recompute();
            }
            None => {
                if announce {
                    self.status = "No hole within reach".into();
                }
            }
        }
    }

    fn group_selection(&mut self) {
        let before = self.doc.clone();
        let names = self.selection.clone();
        match assembly::group(&mut self.doc, &self.editing, &names, &self.group_name) {
            Ok(inst) => {
                self.undo.push(before);
                self.dirty = true;
                self.status = format!("{} is now in the library", assembly::slug(&self.group_name));
                self.group_name.clear();
                self.selection = vec![inst];
                self.recompute();
            }
            Err(e) => {
                self.doc = before;
                self.status = e;
            }
        }
    }

    fn ungroup_selection(&mut self) {
        if self.selection.len() != 1 {
            return;
        }
        let before = self.doc.clone();
        let name = self.selection[0].clone();
        match assembly::ungroup(&mut self.doc, &self.editing, &name) {
            Ok(names) => {
                self.undo.push(before);
                self.dirty = true;
                self.status = format!("Ungrouped {name}; the component stays in the library");
                self.selection = names;
                self.recompute();
            }
            Err(e) => {
                self.doc = before;
                self.status = e;
            }
        }
    }

    // ------------------------------------------------------------ files

    fn open_file(&mut self) {
        let Some(p) = rfd::FileDialog::new().add_filter("assembly", &["json"]).pick_file() else {
            return;
        };
        self.load_path(p);
    }

    pub fn load_path(&mut self, p: PathBuf) {
        match std::fs::read_to_string(&p)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str::<Document>(&t).map_err(|e| e.to_string()))
        {
            Ok(doc) => {
                let errs = assembly::validate(&doc, &self.bundle);
                if errs.iter().any(|e| e.starts_with("format") || e.starts_with("robot.root")) {
                    self.status = format!("Not an assembly file: {}", errs.join("; "));
                    return;
                }
                self.errors = errs;
                self.push_undo();
                self.doc = doc;
                self.path = Some(p.clone());
                self.editing = self.doc.robot.root.clone();
                self.crumbs = vec![self.editing.clone()];
                self.selection.clear();
                self.dirty = false;
                self.fit_pending = true;
                self.recompute();
                self.status = format!("Opened {}", p.display());
            }
            Err(e) => self.status = format!("Could not open: {e}"),
        }
    }

    fn save_file(&mut self, ask: bool) {
        let path = if ask || self.path.is_none() {
            let mut dlg = rfd::FileDialog::new()
                .add_filter("assembly", &["json"])
                .set_file_name("robot.assembly.json");
            if let Some(p) = self.path.as_ref().and_then(|p| p.parent()) {
                dlg = dlg.set_directory(p);
            }
            match dlg.save_file() {
                Some(p) => p,
                None => return,
            }
        } else {
            self.path.clone().unwrap()
        };
        match serde_json::to_string_pretty(&self.doc)
            .map_err(|e| e.to_string())
            .and_then(|t| std::fs::write(&path, t).map_err(|e| e.to_string()))
        {
            Ok(()) => {
                self.path = Some(path.clone());
                self.dirty = false;
                self.status = format!("Saved {}", path.display());
            }
            Err(e) => self.status = format!("Could not save: {e}"),
        }
    }

    // --------------------------------------------------------- viewport

    fn mesh_key(&self, part_id: &str, part: &Part) -> String {
        match &part.ldraw {
            Some(n) => format!("ld:{n}"),
            None => format!("part:{part_id}"),
        }
    }

    fn mesh_for(&self, part: &Part) -> Option<MeshData> {
        match assembly::geometry_of(part, &self.bundle) {
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
        let first = self.selected_instances().into_iter().next()?;
        let c = Vec3::new(first.pos[0] as f32, first.pos[1] as f32, first.pos[2] as f32);
        Some(Gizmo::new(c, self.gizmo_mode, &self.viewport.camera, h))
    }

    fn build_items(&mut self, device: &eframe::egui_wgpu::wgpu::Device, dark: bool) {
        let sel: HashSet<String> = self.selection.iter().cloned().collect();
        let mut items = Vec::new();
        let mut tops = Vec::new();
        for leaf in &self.leaves {
            let Some(part) = self.doc.parts.get(&leaf.part_id) else { continue };
            let key = self.mesh_key(&leaf.part_id, part);
            if !self.viewport.has_mesh(&key) {
                match self.mesh_for(part) {
                    Some(m) => self.viewport.add_mesh(device, &key, &m),
                    None => continue,
                }
            }
            let selected = sel.contains(&leaf.path[0]);
            let r = leaf.rot.as_mat3();
            let q = Quat::from_mat3(&r);
            let model = Mat4::from_rotation_translation(q, leaf.pos.as_vec3());
            let mut color = cat_color(&part.category, dark);
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
        let pr = self.edited_props();
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
        let pr = self.edited_props();
        if let Some(b) = pr.bbox {
            self.viewport.camera.fit(b.min.as_vec3(), b.max.as_vec3());
        } else {
            self.viewport.camera.fit(Vec3::splat(-40.0), Vec3::splat(40.0));
        }
    }

    fn snap(&self, v: f32) -> f64 {
        if self.snap_mm > 0.0 {
            ((v as f64) / self.snap_mm).round() * self.snap_mm
        } else {
            assembly::round3(v as f64)
        }
    }

    fn viewport_ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let dark = ui.visuals().dark_mode;
        let avail = ui.available_size();
        let size = ((avail.x.max(1.0)) as u32, (avail.y.max(1.0)) as u32);
        self.view_size = size;
        let Some(state) = frame.wgpu_render_state() else {
            ui.label("wgpu is not available");
            return;
        };
        self.ensure_gizmo_meshes(&state.device);
        self.build_items(&state.device, dark);
        if self.fit_pending {
            self.fit_view();
            self.fit_pending = false;
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
        let on_handle = if pressed {
            response.interact_pointer_pos().map(local).and_then(|(x, y)| handle_under(x, y))
        } else {
            None
        };
        if let (Some(handle), Some(g), true) = (on_handle, gizmo.as_ref(), response.drag_started_by(egui::PointerButton::Primary)) {
            let (x, y) = response.interact_pointer_pos().map(local).unwrap_or((0.0, 0.0));
            let (o, d) = self.viewport.camera.ray(x, y, w, h);
            if let Some(start) = g.param(handle, o, d) {
                self.push_undo();
                let starts = self.selected_instances().iter().map(|i| (i.name.clone(), i.pos, i.rot)).collect();
                self.drag = Drag::Handle {
                    handle,
                    start,
                    pivot: g.center,
                    starts,
                };
            }
        } else if pressed && on_handle.is_none() {
            let hit = response
                .interact_pointer_pos()
                .map(local)
                .and_then(|(x, y)| self.viewport.pick(&self.items, x, y))
                .map(|i| self.item_tops[i].clone());
            let shift = ui.input(|i| i.modifiers.shift);
            match hit {
                Some(top) => {
                    if shift {
                        if let Some(i) = self.selection.iter().position(|n| n == &top) {
                            self.selection.remove(i);
                        } else {
                            self.selection.push(top.clone());
                        }
                    } else if !self.selection.contains(&top) {
                        self.selection = vec![top.clone()];
                    }
                    if response.drag_started_by(egui::PointerButton::Primary) {
                        let z0 = self.selected_instances().first().map(|i| i.pos[2] as f32).unwrap_or(0.0);
                        let (x, y) = response.interact_pointer_pos().map(local).unwrap_or((0.0, 0.0));
                        let (o, d) = self.viewport.camera.ray(x, y, size.0 as f32, size.1 as f32);
                        if let Some(start) = viewport::ray_plane_z(o, d, z0) {
                            self.push_undo();
                            let starts = self.selected_instances().iter().map(|i| (i.name.clone(), i.pos)).collect();
                            self.drag = Drag::Move { z0, start, starts };
                        }
                    }
                }
                None => {
                    if response.clicked_by(egui::PointerButton::Primary) && !shift {
                        self.selection.clear();
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
            let hit = response
                .interact_pointer_pos()
                .map(local)
                .and_then(|(x, y)| self.viewport.pick(&self.items, x, y))
                .map(|i| self.item_tops[i].clone());
            if let Some(top) = hit {
                let comp = self.doc.components[&self.editing]
                    .children
                    .iter()
                    .find(|c| c.name == top)
                    .and_then(|c| c.component.clone());
                if let Some(cid) = comp {
                    self.open_component(&cid, true);
                    return;
                }
            }
        }
        let delta = response.drag_delta();
        match &self.drag {
            Drag::Orbit if response.dragged() => {
                self.viewport.camera.yaw -= delta.x * 0.5;
                self.viewport.camera.pitch = (self.viewport.camera.pitch + delta.y * 0.5).clamp(-89.0, 89.0);
            }
            Drag::Pan if response.dragged() => {
                let cam = &mut self.viewport.camera;
                let scale = cam.distance * 0.0015;
                let (r, u) = (cam.right(), cam.up());
                cam.target = cam.target - r * delta.x * scale + u * delta.y * scale;
            }
            Drag::Handle {
                handle,
                start,
                pivot,
                starts,
            } if response.dragged() => {
                let (handle, start, pivot, starts) = (*handle, *start, *pivot, starts.clone());
                if let Some((x, y)) = response.interact_pointer_pos().map(local) {
                    let (o, d) = self.viewport.camera.ray(x, y, w, h);
                    let shift = ui.input(|i| i.modifiers.shift);
                    let g = Gizmo {
                        center: pivot,
                        mode: self.gizmo_mode,
                        length: 1.0,
                    };
                    if let Some(now) = g.param(handle, o, d) {
                        match handle {
                            Handle::Axis(i) => {
                                let snap = if shift { 0.0 } else { self.snap_mm };
                                for (name, p0, _) in &starts {
                                    let np = gizmo::moved(*p0, i, (now - start) as f64, snap);
                                    self.set_instance(name, |inst| inst.pos = np);
                                }
                            }
                            Handle::Ring(i) => {
                                let deg = gizmo::snap_angle(now - start, !shift && self.snap_mm > 0.0);
                                let pivot = DVec3::new(pivot.x as f64, pivot.y as f64, pivot.z as f64);
                                for (name, p0, r0) in &starts {
                                    let (np, nr) = gizmo::rotated(*p0, *r0, pivot, i, deg);
                                    self.set_instance(name, |inst| {
                                        inst.pos = np;
                                        inst.rot = nr;
                                    });
                                }
                            }
                        }
                        self.recompute();
                    }
                }
            }
            Drag::Move { z0, start, starts } if response.dragged() => {
                if let Some((x, y)) = response.interact_pointer_pos().map(local) {
                    let (o, d) = self.viewport.camera.ray(x, y, w, h);
                    let shift = ui.input(|i| i.modifiers.shift);
                    let starts = starts.clone();
                    if shift {
                        let dz = -delta.y * self.viewport.camera.distance * 0.0015;
                        for (name, _) in &starts {
                            self.set_instance(name, |i| i.pos[2] = assembly::round3(i.pos[2] + dz as f64));
                        }
                    } else if let Some(p) = viewport::ray_plane_z(o, d, *z0) {
                        let dp = p - *start;
                        for (name, p0) in &starts {
                            let nx = self.snap(p0[0] as f32 + dp.x);
                            let ny = self.snap(p0[1] as f32 + dp.y);
                            self.set_instance(name, |i| {
                                i.pos[0] = nx;
                                i.pos[1] = ny;
                            });
                        }
                    }
                    self.recompute();
                }
            }
            _ => {}
        }
        if response.drag_stopped() {
            if matches!(self.drag, Drag::Move { .. }) {
                if self.snap_mm > 0.0 {
                    for name in self.selection.clone() {
                        let s = self.snap_mm;
                        self.set_instance(&name, |i| i.pos[2] = (i.pos[2] / s).round() * s);
                    }
                }
                self.recompute();
                if self.magnet {
                    self.snap_selection(false);
                }
            }
            if matches!(self.drag, Drag::Handle { .. }) {
                self.recompute();
                if self.magnet {
                    self.snap_selection(false);
                }
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
        // keyboard
        let (del, rot, esc, undo, dup, fit, snap_now, arrows, shift, move_mode, rotate_mode) = ui.input(|i| {
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
                i.modifiers.shift,
                i.key_pressed(egui::Key::W),
                i.key_pressed(egui::Key::E),
            )
        });
        if !ui.ctx().egui_wants_keyboard_input() {
            if undo {
                self.undo();
            }
            if move_mode {
                self.gizmo_mode = Mode::Move;
            }
            if rotate_mode {
                self.gizmo_mode = Mode::Rotate;
            }
            if dup {
                self.duplicate_selection();
            }
            if esc {
                self.selection.clear();
            }
            if fit {
                self.fit_view();
            }
            if del {
                self.remove_selection();
            }
            if rot {
                self.rotate_selection(2, 90.0);
            }
            if snap_now {
                self.snap_selection(true);
            }
            let step = if shift { 1.0 } else { self.snap_mm.max(1.0) };
            if arrows[0] {
                self.nudge_selection([step, 0.0, 0.0]);
            }
            if arrows[1] {
                self.nudge_selection([-step, 0.0, 0.0]);
            }
            if arrows[2] {
                self.nudge_selection([0.0, step, 0.0]);
            }
            if arrows[3] {
                self.nudge_selection([0.0, -step, 0.0]);
            }
        }
        let hud = format!(
            "editing {}{} · 1 module = 8 mm · snap {} · orbit: drag · pan: right-drag · zoom: wheel · drag a brick to move it, shift lifts · W/E move/rotate handles (shift: free) · R turns 90° · S snaps · Del · ⌘Z",
            self.editing,
            if self.selection.is_empty() {
                String::new()
            } else {
                format!(" · {} selected", self.selection.len())
            },
            if self.snap_mm > 0.0 {
                format!("{} mm", self.snap_mm)
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

    // --------------------------------------------------------------- ui

    fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.selectable_value(&mut self.tab, Tab::Workbench, "Workbench");
            ui.selectable_value(&mut self.tab, Tab::Simulate, "Simulate");
            ui.separator();
            if ui.button("Open…").clicked() {
                self.open_file();
            }
            if ui.button(if self.dirty { "Save *" } else { "Save" }).clicked() {
                self.save_file(false);
            }
            if ui.button("Save as…").clicked() {
                self.save_file(true);
            }
            if ui.button("Example").clicked() {
                self.push_undo();
                self.doc = assembly::example();
                self.path = None;
                self.editing = self.doc.robot.root.clone();
                self.crumbs = vec![self.editing.clone()];
                self.selection.clear();
                self.errors = assembly::validate(&self.doc, &self.bundle);
                self.fit_pending = true;
                self.recompute();
            }
            ui.separator();
            for (i, id) in self.crumbs.clone().iter().enumerate() {
                if i > 0 {
                    ui.label("›");
                }
                let label = if id == &self.doc.robot.root {
                    format!(
                        "{} (robot)",
                        if self.doc.robot.name.is_empty() {
                            id.clone()
                        } else {
                            self.doc.robot.name.clone()
                        }
                    )
                } else {
                    id.clone()
                };
                if i + 1 == self.crumbs.len() {
                    ui.strong(label);
                } else if ui.link(label).clicked() {
                    self.open_component(id, false);
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
                .selected_text(if self.snap_mm > 0.0 {
                    format!("{} mm", self.snap_mm)
                } else {
                    "off".into()
                })
                .show_ui(ui, |ui| {
                    for (label, v) in [("8 mm", 8.0), ("4 mm", 4.0), ("1 mm", 1.0), ("off", 0.0)] {
                        ui.selectable_value(&mut self.snap_mm, v, label);
                    }
                });
            ui.checkbox(&mut self.magnet, "snap to holes");
            ui.checkbox(&mut self.show_grid, "ground");
            ui.checkbox(&mut self.show_com, "COM");
            if ui.button("Snap").on_hover_text("S").clicked() {
                self.snap_selection(true);
            }
        });
    }

    fn library_ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("Library");
        ui.add(egui::TextEdit::singleline(&mut self.search).hint_text("search bricks and components"));
        let q = self.search.trim().to_lowercase();
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.add_space(6.0);
            ui.strong("Components");
            let ids: Vec<String> = self.doc.components.keys().cloned().collect();
            let mut to_add: Option<(Option<String>, Option<String>)> = None;
            let mut to_open: Option<String> = None;
            let mut to_ldraw: Option<String> = None;
            for id in ids {
                if !q.is_empty() && !id.contains(&q) {
                    continue;
                }
                let mass = self.memo.get(&id).map(|p| p.mass).unwrap_or(0.0);
                let uses = assembly::usage_count(&self.doc, &id);
                let is_root = id == self.doc.robot.root;
                let can_add = !is_root && !assembly::component_contains(&self.doc, &id, &self.editing, &mut vec![]);
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
            ui.strong(format!("LEGO Technic  ({} parts, exact LDraw geometry)", self.bundle.parts.len()));
            let mut nums: Vec<&String> = self.bundle.parts.keys().collect();
            nums.sort_by_key(|n| self.bundle.parts[*n].name.to_lowercase());
            for num in nums {
                let rec = &self.bundle.parts[num];
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
                .doc
                .parts
                .iter()
                .filter(|(_, p)| p.ldraw.is_none())
                .map(|(k, _)| k.clone())
                .collect();
            for id in ids {
                let p = &self.doc.parts[&id];
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
                && let Some(id) = self.ensure_ldraw_part(&num)
            {
                self.add_instance(Some(id), None, [0.0; 3]);
            }
            if let Some((p, c)) = to_add {
                self.add_instance(p, c, [0.0; 3]);
            }
            if let Some(id) = to_open {
                let push = id != self.doc.robot.root && !self.crumbs.contains(&id);
                self.open_component(&id, push);
            }
        });
    }

    fn tree_ui(&mut self, ui: &mut egui::Ui) {
        ui.heading(format!("Contents of {}", self.editing));
        let children: Vec<Instance> = self.doc.components[&self.editing].children.clone();
        let mut open: Option<String> = None;
        for ch in &children {
            let selected = self.selection.contains(&ch.name);
            let (sub, mass) = match (&ch.part, &ch.component) {
                (Some(p), _) => (
                    self.doc
                        .parts
                        .get(p)
                        .map(|x| x.name.clone())
                        .unwrap_or_else(|| format!("missing {p}")),
                    self.doc.parts.get(p).map(|x| x.mass_g).unwrap_or(0.0),
                ),
                (_, Some(c)) => (
                    format!("{c} · used ×{}", assembly::usage_count(&self.doc, c)),
                    self.memo.get(c).map(|p| p.mass).unwrap_or(0.0),
                ),
                _ => (String::new(), 0.0),
            };
            let label = format!("{}   {}   {} g", ch.name, sub, assembly::fmt(mass));
            let resp = ui.selectable_label(selected, label);
            if resp.clicked() {
                let shift = ui.input(|i| i.modifiers.shift);
                if shift {
                    if selected {
                        self.selection.retain(|n| n != &ch.name);
                    } else {
                        self.selection.push(ch.name.clone());
                    }
                } else {
                    self.selection = vec![ch.name.clone()];
                }
            }
            if resp.double_clicked()
                && let Some(c) = &ch.component
            {
                open = Some(c.clone());
            }
        }
        if let Some(c) = open {
            self.open_component(&c, true);
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

    fn inspector_ui(&mut self, ui: &mut egui::Ui) {
        let insts = self.selected_instances();
        if insts.is_empty() {
            if self.editing == self.doc.robot.root {
                ui.heading("Robot");
                let mut name = self.doc.robot.name.clone();
                if ui.text_edit_singleline(&mut name).changed() {
                    self.doc.robot.name = name;
                    self.dirty = true;
                }
                let pr = self.root_props.clone();
                Self::computed_block(ui, &pr, "assembly frame");
                ui.add_space(6.0);
                ui.strong("Roles  (what the simulator binds)");
                let options: Vec<String> = assembly::flatten(&self.doc, &self.doc.robot.root)
                    .iter()
                    .map(|l| l.path.join("/"))
                    .collect();
                let mut changed = None;
                egui::Grid::new("roles").num_columns(2).show(ui, |ui| {
                    for (role, help) in assembly::ROLES {
                        ui.monospace(role).on_hover_text(help);
                        let current = self.doc.robot.roles.get(role).cloned().unwrap_or_default();
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
                    self.push_undo();
                    self.doc.robot.roles.insert(role, v);
                }
                ui.add_space(6.0);
                ui.strong("Spawn on the mat");
                ui.horizontal(|ui| {
                    ui.weak("x");
                    if ui
                        .add(egui::DragValue::new(&mut self.doc.robot.spawn.pos_mm[0]).speed(1.0).suffix(" mm"))
                        .changed()
                    {
                        self.dirty = true;
                    }
                    ui.weak("y");
                    if ui
                        .add(egui::DragValue::new(&mut self.doc.robot.spawn.pos_mm[1]).speed(1.0).suffix(" mm"))
                        .changed()
                    {
                        self.dirty = true;
                    }
                    ui.weak("yaw");
                    if ui
                        .add(egui::DragValue::new(&mut self.doc.robot.spawn.yaw_deg).speed(1.0).suffix("°"))
                        .changed()
                    {
                        self.dirty = true;
                    }
                });
                ui.add_space(6.0);
                ui.strong("Check against the scale");
                let mut weighed = self.doc.robot.measured_mass_g.unwrap_or(0.0);
                ui.horizontal(|ui| {
                    ui.weak("weighed");
                    if ui.add(egui::DragValue::new(&mut weighed).speed(1.0).suffix(" g")).changed() {
                        self.doc.robot.measured_mass_g = if weighed > 0.0 { Some(weighed) } else { None };
                        self.dirty = true;
                    }
                });
                if let Some(w) = self.doc.robot.measured_mass_g {
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
            } else {
                ui.heading(format!("Component {}", self.editing));
                ui.label("Its origin is the frame every instance is placed by. Add bricks, arrange them, then use it from the library.");
                let pr = self.edited_props();
                Self::computed_block(ui, &pr, &format!("{} frame", self.editing));
                ui.weak(format!("used ×{}", assembly::usage_count(&self.doc, &self.editing)));
                if ui.button("Back to the robot").clicked() {
                    let root = self.doc.robot.root.clone();
                    self.open_component(&root, false);
                }
            }
            if !self.errors.is_empty() {
                ui.add_space(6.0);
                for e in &self.errors {
                    ui.colored_label(egui::Color32::from_rgb(196, 68, 42), e);
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
                        .and_then(|p| self.doc.parts.get(p))
                        .map(|p| p.mass_g)
                        .or_else(|| i.component.as_ref().and_then(|c| self.memo.get(c)).map(|p| p.mass))
                        .unwrap_or(0.0)
                })
                .sum();
            ui.monospace(format!("together {} g", assembly::fmt(together)));
            self.group_form(ui, insts.len());
            ui.horizontal(|ui| {
                if ui.button("Turn 90°").clicked() {
                    self.rotate_selection(2, 90.0);
                }
                if ui.button("Duplicate").clicked() {
                    self.duplicate_selection();
                }
                if ui.button("Remove").clicked() {
                    self.remove_selection();
                }
            });
            return;
        }
        let inst = insts[0].clone();
        ui.heading(if inst.part.is_some() {
            "Brick instance"
        } else {
            "Component instance"
        });
        let mut name = inst.name.clone();
        if ui.text_edit_singleline(&mut name).lost_focus() && name != inst.name {
            let siblings = &self.doc.components[&self.editing].children;
            if !name.is_empty() && !name.contains('/') && !siblings.iter().any(|c| c.name == name) {
                self.push_undo();
                let editing = self.editing.clone();
                let old = inst.name.clone();
                let new = name.clone();
                assembly::remap_roles(&mut self.doc, &editing, &|seg| {
                    if seg == old { Some(vec![new.clone()]) } else { None }
                });
                self.set_instance(&old, |i| i.name = new.clone());
                self.selection = vec![name];
                self.recompute();
                return;
            } else {
                self.status = "Names must be unique among siblings and contain no /".into();
            }
        }
        ui.weak(format!(
            "in {}{}",
            self.editing,
            if assembly::usage_count(&self.doc, &self.editing) > 1 && self.editing != self.doc.robot.root {
                " — used more than once: a change here moves it in every use"
            } else {
                ""
            }
        ));
        ui.strong(format!("Pose  (in the {} frame)", self.editing));
        let mut pos = inst.pos;
        let mut rot = inst.rot;
        let mut changed = false;
        egui::Grid::new("pose").num_columns(4).show(ui, |ui| {
            for (k, label) in ["x", "y", "z"].iter().enumerate() {
                ui.weak(*label);
                changed |= ui.add(egui::DragValue::new(&mut pos[k]).speed(1.0).suffix(" mm")).changed();
                if ui.small_button(format!("−{}", self.snap_mm.max(1.0))).clicked() {
                    pos[k] -= self.snap_mm.max(1.0);
                    changed = true;
                }
                if ui.small_button(format!("+{}", self.snap_mm.max(1.0))).clicked() {
                    pos[k] += self.snap_mm.max(1.0);
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
        if changed {
            self.push_undo();
            let name = inst.name.clone();
            self.set_instance(&name, |i| {
                i.pos = pos.map(assembly::round3);
                i.rot = rot.map(|a| ((a + 180.0).rem_euclid(360.0)) - 180.0);
            });
            self.recompute();
        }
        if let Some(pid) = &inst.part {
            if let Some(part) = self.doc.parts.get(pid).cloned() {
                ui.add_space(6.0);
                ui.strong("Recorded  (the brick's own facts)");
                egui::Grid::new("recorded").num_columns(2).show(ui, |ui| {
                    ui.weak("name");
                    ui.label(&part.name);
                    ui.end_row();
                    ui.weak("mass");
                    ui.monospace(format!("{} g", assembly::fmt(part.mass_g)));
                    ui.end_row();
                    match assembly::geometry_of(&part, &self.bundle) {
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
                let cons = assembly::connections_of(&self.doc, &self.bundle, &self.editing, &inst.name);
                ui.add_space(4.0);
                ui.strong("Connections  (mated features)");
                if cons.is_empty() {
                    ui.weak("none: drag it next to a hole and let go, or press S");
                } else {
                    for (m, o, path) in cons {
                        ui.label(format!("{} → {} of {}", m.replace('_', " "), o.replace('_', " "), path.join("/")));
                    }
                }
                let pr = assembly::part_props(&part, &self.bundle);
                Self::computed_block(ui, &pr, "brick frame");
            }
        } else if let Some(cid) = &inst.component {
            let pr = self.memo.get(cid).cloned();
            if let Some(pr) = pr {
                Self::computed_block(ui, &pr, &format!("{cid} frame"));
            }
            ui.horizontal(|ui| {
                if ui.button(format!("Open {cid}")).clicked() {
                    self.open_component(cid, true);
                    return;
                }
                if ui.button("Ungroup").clicked() {
                    self.ungroup_selection();
                }
            });
        }
        self.group_form(ui, 1);
        ui.horizontal_wrapped(|ui| {
            if ui.button("Turn 90°").clicked() {
                self.rotate_selection(2, 90.0);
            }
            if ui.button("Pitch 90°").clicked() {
                self.rotate_selection(1, 90.0);
            }
            if ui.button("Roll 90°").clicked() {
                self.rotate_selection(0, 90.0);
            }
            if ui.button("Duplicate").clicked() {
                self.duplicate_selection();
            }
            if ui.button("Remove").clicked() {
                self.remove_selection();
            }
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
            if ui.button("Group").clicked() || enter {
                self.group_selection();
            }
        });
        ui.weak("The first selected item's frame becomes the new component's origin; the component joins the library.");
    }

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        let pr = self.root_props.clone();
        let placeholders = self.doc.parts.values().filter(|p| p.source == "placeholder").count();
        ui.horizontal(|ui| {
            ui.monospace(format!(
                "robot {} g · {} bricks · COM {} mm",
                assembly::fmt(pr.mass),
                pr.count,
                vec_text(pr.com)
            ));
            if placeholders > 0 {
                ui.colored_label(
                    egui::Color32::from_rgb(196, 68, 42),
                    format!("{placeholders} brick{} still to measure", if placeholders > 1 { "s" } else { "" }),
                );
            }
            if let Some(p) = &self.path {
                ui.weak(p.display().to_string());
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(&self.status);
            });
        });
    }

    fn simulate_ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        if self.simulate.pump() || self.simulate.is_live() {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(16));
        }
        egui::Panel::top("sim-controls").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                let can_use = self.path.is_some() && !self.dirty;
                if ui
                    .add_enabled(can_use, egui::Button::new("Use the workbench's build"))
                    .on_hover_text(if self.dirty {
                        "save the assembly first"
                    } else {
                        "the assembly open in the Workbench tab"
                    })
                    .clicked()
                    && let Some(p) = self.path.clone()
                {
                    self.simulate.set_chassis(p, self.doc.clone());
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
            .draw_items(&mut self.viewport, &state.device, &state.queue, &self.bundle, dark);
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
                "Choose a map, a chassis and a program, then Load or Run",
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
        if self.dirty {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::Title(format!("Openbricks Sim — {}*", self.title_name())));
        }
    }
}

impl App {
    fn title_name(&self) -> String {
        self.path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|f| f.to_string_lossy().to_string())
            .unwrap_or_else(|| "example".into())
    }
}

impl Drop for App {
    fn drop(&mut self) {
        self.simulate.shutdown();
    }
}
