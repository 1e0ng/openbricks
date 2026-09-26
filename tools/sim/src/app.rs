//! The application: the Workbench tab (library, viewport, tree,
//! inspector) drawn over the editor's state, and the Simulate tab.
//! Everything that changes the document lives in `editor`; this file
//! only draws it and feeds it input.

use crate::assembly::{self, Geometry, Instance, Part, Props};
use crate::editor::{self, Editor};
use crate::fetch::{Outcome, PartFetch};
use crate::geometry;
use crate::gizmo::{self, Gizmo, Handle, Mode};
use crate::route::Pose2;
use crate::simulate::SimulateTab;
use crate::stl;
use crate::viewport::{self, Camera, DrawItem, Line, Viewport, srgb};
use eframe::egui;
use eframe::egui_wgpu::{self, RenderState, wgpu};
use glam::{DVec3, Mat4, Quat, Vec3};
use std::collections::HashSet;
use std::path::{Path, PathBuf};
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
    /// The map editor: props moved, added, removed; the map saved as the user's own.
    Map,
    Simulate,
}

impl Tab {
    /// The Simulate tab is the fixed plan view; the others are 3D.
    fn plan(self) -> bool {
        self == Tab::Simulate
    }

    /// Each tab keeps a camera of its own.
    fn index(self) -> usize {
        match self {
            Tab::Workbench => 0,
            Tab::Map => 1,
            Tab::Simulate => 2,
        }
    }
}

enum Drag {
    None,
    Orbit,
    Pan,
    /// A prop dragged on the map: which, the pointer x it started at, the
    /// pose it started from, and where the pointer took hold of it.
    Prop {
        i: usize,
        px0: f32,
        pose0: Pose2,
        grab: [f64; 2],
    },
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
    /// The chassis dragged on the map: the pointer's x at the press and
    /// the pose it started from (the ghost's axle centre follows the pointer).
    Chassis {
        px0: f32,
        pose0: Pose2,
    },
    /// An action's handle dragged on the map.
    RouteHandle,
    /// A map marker dragged to a new spot.
    Marker,
}

/// The kinds of file the Workbench's dialogs ask for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FileKind {
    Assembly,
    Stl,
}

impl FileKind {
    fn filter(self) -> (&'static str, &'static [&'static str]) {
        match self {
            FileKind::Assembly => ("assembly", &["json"]),
            FileKind::Stl => ("STL", &["stl"]),
        }
    }
}

/// The file dialogs the toolbar and the library open: the native ones,
/// or, under test, answers set beforehand — the handlers behind the
/// buttons run either way.
type PickFn = Box<dyn Fn(FileKind) -> Option<PathBuf>>;
type SaveFn = Box<dyn Fn(FileKind, &str, Option<&Path>) -> Option<PathBuf>>;

pub struct Dialogs {
    pick: PickFn,
    save: SaveFn,
}

/// The native open dialog.
fn native_pick(kind: FileKind) -> Option<PathBuf> {
    let (name, ext) = kind.filter();
    rfd::FileDialog::new().add_filter(name, ext).pick_file()
}

/// The native save dialog, in `dir` when there is one.
fn native_save(kind: FileKind, file_name: &str, dir: Option<&Path>) -> Option<PathBuf> {
    let (name, ext) = kind.filter();
    let mut dlg = rfd::FileDialog::new().add_filter(name, ext).set_file_name(file_name);
    if let Some(d) = dir {
        dlg = dlg.set_directory(d);
    }
    dlg.save_file()
}

impl Dialogs {
    pub fn native() -> Self {
        Dialogs {
            pick: Box::new(native_pick),
            save: Box::new(native_save),
        }
    }

    /// Dialogs that answer every question with `path` (None: declined).
    #[cfg(test)]
    pub fn answering(path: Option<PathBuf>) -> Self {
        let picked = path.clone();
        Dialogs {
            pick: Box::new(move |_| picked.clone()),
            save: Box::new(move |_, _, _| path.clone()),
        }
    }

    pub fn pick(&self, kind: FileKind) -> Option<PathBuf> {
        (self.pick)(kind)
    }

    pub fn save(&self, kind: FileKind, file_name: &str, dir: Option<&Path>) -> Option<PathBuf> {
        (self.save)(kind, file_name, dir)
    }
}

pub struct App {
    editor: Editor,
    /// The file dialogs, native unless a test answers them.
    dialogs: Dialogs,
    tab: Tab,
    search: String,
    /// The Map tab's search of the brick library.
    map_search: String,
    /// Builds opened for the map editor beside the one open in the
    /// Workbench: any saved assembly, so any component built there can go
    /// on the map.
    map_builds: Vec<(std::path::PathBuf, assembly::Document)>,
    /// Which build the Map tab lists the components of: 0 the
    /// Workbench's, then `map_builds` in order.
    map_build: usize,
    /// What the last attempt to open a build for the map said.
    map_note: String,
    group_name: String,
    /// The name typed beside the library's Components for a new one.
    new_component_name: String,
    /// The LEGO colour picked in the library for each part number: what
    /// the next brick of it is placed in.
    pick_color: std::collections::HashMap<String, u32>,
    gizmo_mode: Mode,
    hot: Option<Handle>,
    show_grid: bool,
    show_com: bool,
    viewport: Viewport,
    /// The camera of the tab not shown: the Workbench orbits in 3D, the
    /// Simulate tab is a fixed top-down plan; switching tabs swaps them.
    /// Every tab's camera but the shown one (which lives in the viewport).
    cameras: [Camera; 3],
    camera_tab: Tab,
    /// The plan view's size last frame: a change refits the map.
    plan_size: (u32, u32),
    drag: Drag,
    /// Set in the frame Escape abandoned a drag, so the key does not also drop the selection.
    escaped_drag: bool,
    /// The Python the sim was started with: what fetches a part by number.
    python: Option<String>,
    /// Environment for the fetcher (tests point PYTHONPATH at a stand-in).
    fetch_env: Vec<(String, String)>,
    /// Where fetched parts are kept (the data directory's `bricks`).
    bricks_dir: std::path::PathBuf,
    /// A fetch by number in progress (one at a time: a second one would kill it).
    fetch: Option<PartFetch>,
    /// Why the last fetch failed: the number, and the reason drawn under
    /// the library's line about that number alone.
    fetch_note: Option<(String, String)>,
    /// Where the 3D view was drawn last frame, in screen points.
    view_rect: egui::Rect,
    items: Vec<DrawItem>,
    item_tops: Vec<String>,
    simulate: SimulateTab,
    /// An STL file being imported: the form and what it works out to.
    stl: Option<stl::Import>,
    stl_prepared: Option<stl::Prepared>,
}

/// A brick category's colour as written, sRGB hex.
pub fn cat_hex(category: &str, dark: bool) -> u32 {
    match (category, dark) {
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
    }
}

pub fn cat_color(category: &str, dark: bool) -> [f32; 4] {
    srgb(cat_hex(category, dark))
}

/// A LEGO colour from the library's palette, for the same pipeline.
pub fn rgb_color(rgb: [u8; 3]) -> [f32; 4] {
    srgb(((rgb[0] as u32) << 16) | ((rgb[1] as u32) << 8) | rgb[2] as u32)
}

/// A LEGO colour chosen among those a part comes in: "—" for the
/// category colour, else a swatch and the colour's name. Returns whether
/// the choice changed.
fn color_combo(ui: &mut egui::Ui, id: &str, choices: &[(u32, String, [u8; 3])], picked: &mut Option<u32>) -> bool {
    let label = match *picked {
        None => "—".to_string(),
        Some(c) => choices
            .iter()
            .find(|(id, ..)| *id == c)
            .map(|(_, name, _)| name.clone())
            .unwrap_or_else(|| format!("colour {c}")),
    };
    let mut changed = false;
    egui::ComboBox::from_id_salt(id)
        .selected_text(label)
        .width(150.0)
        .show_ui(ui, |ui| {
            changed |= ui.selectable_value(picked, None, "—").changed();
            for (id, name, rgb) in choices {
                let resp = ui
                    .horizontal(|ui| {
                        egui::color_picker::show_color(ui, egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]), egui::vec2(14.0, 14.0));
                        ui.selectable_value(picked, Some(*id), name)
                    })
                    .inner;
                changed |= resp.changed();
            }
        });
    changed
}

const ACCENT: [f32; 4] = [0.71, 0.29, 0.005, 1.0];
/// Library thumbnails: rendered pixels, shown at half size; how many
/// are rendered per frame so the first frames stay quick.
const THUMB_PX: u32 = 88;
const THUMB_SIZE: egui::Vec2 = egui::vec2(44.0, 33.0);
const THUMBS_PER_FRAME: usize = 4;
const RED: egui::Color32 = egui::Color32::from_rgb(196, 68, 42);

/// Whether a search looks like an LDraw part number (a LEGO design id,
/// digits with an optional mould suffix: `2458`, `3648b`, `32013`).
fn looks_like_part_number(q: &str) -> bool {
    let b = q.as_bytes();
    b.len() >= 2 && b.iter().take_while(|c| c.is_ascii_digit()).count() >= 2 && b.iter().all(|c| c.is_ascii_alphanumeric())
}

impl App {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        bundle: crate::bundle::Bundle,
        doc: Option<(std::path::PathBuf, assembly::Document)>,
        python: Option<String>,
        notes: Vec<String>,
    ) -> Self {
        let gpu = Gpu::from(cc.wgpu_render_state.as_ref().expect("the wgpu renderer is required"));
        let given = doc.is_some();
        let mut app = Self::with_gpu(&gpu, bundle, doc, python);
        if !given {
            app.restore_drafts(crate::drafts::now_ms());
        }
        // what went wrong on the way in (a fetched part's file that would not read, or one the
        // library ships a better record for) is listed with the build's own problems, and stays
        // listed however often those are checked afresh
        app.editor.notes.extend(notes);
        app
    }

    #[cfg(test)]
    fn note_text(&self) -> String {
        self.fetch_note.as_ref().map(|n| n.1.clone()).unwrap_or_default()
    }

    /// The drafts an earlier session kept come back: the build (unless a
    /// file was named on the command line) and the route.
    pub fn restore_drafts(&mut self, now_ms: i64) {
        self.editor.restore_draft(now_ms);
        self.simulate.restore_draft(now_ms);
    }

    /// Where the drafts are kept: the tests point it at a scratch directory.
    #[cfg(test)]
    pub fn drafts_in(&mut self, dir: std::path::PathBuf) {
        self.editor.draft_dir = dir.clone();
        self.simulate.draft_dir = dir;
    }

    /// Where fetched parts are kept, and how the fetcher is run: the
    /// tests point both at a stand-in.
    #[cfg(test)]
    pub fn fetching_with(&mut self, env: Vec<(String, String)>, dir: std::path::PathBuf) {
        self.fetch_env = env;
        self.bricks_dir = dir;
    }

    // ------------------------------------------------------------ fetch

    /// Start fetching part `number` from ldraw.org into the bricks
    /// directory, with the sim's Python.
    fn start_fetch(&mut self, number: &str) {
        if let Some(f) = &self.fetch {
            // one at a time: starting another would kill this one half way
            self.fetch_note = Some((number.into(), format!("{} is still being fetched", f.number)));
            return;
        }
        let Some(python) = self.python.clone() else {
            self.fetch_note = Some((
                number.into(),
                "fetching needs the openbricks Python: start the sim with `openbricks sim`".into(),
            ));
            return;
        };
        let out = self.bricks_dir.join(format!("{number}.json"));
        match PartFetch::start(&python, &self.fetch_env, number, &out) {
            Ok(f) => {
                self.fetch = Some(f);
                self.fetch_note = None;
            }
            Err(e) => self.fetch_note = Some((number.into(), e)),
        }
    }

    /// The fetch under way is stopped; nothing of it is kept.
    fn cancel_fetch(&mut self) {
        if let Some(f) = self.fetch.take() {
            self.editor.status = format!("The fetch of {} was cancelled", f.number);
        }
    }

    /// Called every frame: a fetch that has ended joins its part to the
    /// library (drawn, measured and pictured afresh) or leaves its
    /// reason under the library's line.
    fn poll_fetch(&mut self, ui: &egui::Ui, gpu: Option<&Gpu>) {
        let Some(f) = &mut self.fetch else { return };
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(100));
        let Some(outcome) = f.poll() else { return };
        let number = f.number.clone();
        self.fetch = None;
        match outcome {
            Outcome::Fetched {
                name,
                files,
                colors,
                note,
                out,
            } => match crate::bundle::load_bundle(&out) {
                Ok(b) => {
                    self.editor.bundle.merge(b);
                    self.viewport.remove_mesh(&format!("ld:{number}"));
                    if let Some(gpu) = gpu {
                        let mut renderer = gpu.renderer.write();
                        let stale = format!("thumb:ld:{number}:");
                        self.viewport.retain_thumbs(&mut renderer, |k| !k.starts_with(&stale));
                    }
                    self.editor.forget_part(&number);
                    let colours = match note {
                        Some(why) => why,
                        None => format!("{colors} colour{}", if colors == 1 { "" } else { "s" }),
                    };
                    self.editor.status = format!(
                        "{number} {name} joined the library ({files} file{}, {colours}); it is kept at {}",
                        if files == 1 { "" } else { "s" },
                        out.display()
                    );
                }
                Err(e) => self.fetch_note = Some((number.clone(), format!("{number} was fetched but its file would not read: {e}"))),
            },
            Outcome::Failed(text) => self.fetch_note = Some((number, text)),
        }
        ui.ctx().request_repaint();
    }

    /// Called every frame: drafts of unsaved work, a moment after a run
    /// of changes settles.
    pub fn autosave(&mut self, now: std::time::Instant, now_ms: i64) {
        self.editor.autosave(now, now_ms);
        self.simulate.autosave(now, now_ms);
    }

    pub fn with_gpu(
        gpu: &Gpu,
        bundle: crate::bundle::Bundle,
        doc: Option<(std::path::PathBuf, assembly::Document)>,
        python: Option<String>,
    ) -> Self {
        let viewport = Viewport::new(&gpu.device, &gpu.queue);
        // the map editor looks at the map from above and aside, in perspective
        let map_camera = {
            let mut c = viewport.camera.clone();
            c.ortho = false;
            c.yaw = -128.0;
            c.pitch = 28.0;
            c.distance = 3000.0;
            c
        };
        let cameras = [viewport.camera.clone(), map_camera, Camera::top_down()];
        App {
            editor: Editor::new(bundle, doc),
            dialogs: Dialogs::native(),
            tab: Tab::Workbench,
            search: String::new(),
            map_search: String::new(),
            map_builds: vec![],
            map_build: 0,
            map_note: String::new(),
            group_name: String::new(),
            new_component_name: String::new(),
            pick_color: Default::default(),
            gizmo_mode: Mode::Move,
            hot: None,
            show_grid: true,
            show_com: true,
            viewport,
            cameras,
            camera_tab: Tab::Workbench,
            plan_size: (0, 0),
            drag: Drag::None,
            escaped_drag: false,
            python: python.clone(),
            fetch_env: vec![],
            bricks_dir: crate::markers::data_dir().join("bricks"),
            fetch: None,
            fetch_note: None,
            view_rect: egui::Rect::ZERO,
            items: vec![],
            item_tops: vec![],
            simulate: SimulateTab::new(python),
            stl: None,
            stl_prepared: None,
        }
    }

    // --------------------------------------------------------- viewport

    /// The viewport mesh of a part, built on first use; None without geometry.
    fn ensure_part_mesh(&mut self, device: &eframe::egui_wgpu::wgpu::Device, part_id: &str, part: &Part) -> Option<String> {
        let key = editor::mesh_key(part_id, part);
        if !self.viewport.has_mesh(&key) {
            let m = self.editor.mesh_of(part)?;
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
                color: leaf
                    .color
                    .and_then(|c| self.editor.bundle.color_rgb(c))
                    .map(rgb_color)
                    .unwrap_or_else(|| cat_color(&part.category, dark)),
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
            let mut color = leaf
                .color
                .and_then(|c| self.editor.bundle.color_rgb(c))
                .map(rgb_color)
                .unwrap_or_else(|| cat_color(&part.category, dark));
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
            if self
                .editor
                .overlapping
                .iter()
                .any(|(a, b, _)| *a == leaf.path[0] || *b == leaf.path[0])
            {
                // would overlap: flushed red while the drag goes on
                color = [color[0] * 0.35 + 0.65, color[1] * 0.35 + 0.03, color[2] * 0.35 + 0.02, 1.0];
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
        // the plan view has no Fit (it always fits the map): off the Workbench, Fit is the map
        // editor's, and frames the map again
        if self.tab != Tab::Workbench {
            self.simulate.frame_map();
            return;
        }
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
                top_lines: &[],
                ghost: &[],
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
        let (shift, command) = ui.input(|i| (i.modifiers.shift, i.modifiers.command));

        // zoom: the wheel, or a pinch (a trackpad's)
        if response.hovered() {
            let (scroll, pinch) = ui.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
            let f = (1.0 - scroll * 0.002).clamp(0.5, 2.0) / pinch.clamp(0.5, 2.0);
            if f != 1.0 {
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
                        // a drag on empty space orbits; with shift or ⌘ held it pans, for a pointer
                        // with no right button
                        self.drag = if shift || command { Drag::Pan } else { Drag::Orbit };
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
            // Escape ends a drag (egui lets go of it): the brick is put back where it was,
            // and the key does not also drop the selection
            let escaped = ui.input(|i| i.key_pressed(egui::Key::Escape));
            match self.drag {
                Drag::Move { .. } | Drag::Handle { .. } if escaped => {
                    self.editor.cancel_change();
                    self.escaped_drag = true;
                }
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
            "editing {}{} · 1 module = 8 mm · snap {} · orbit: drag · pan: shift-drag or right-drag · zoom: wheel or pinch · drag a brick to move it, shift lifts · W/E move/rotate handles (shift: free) · R turns 90° · S snaps · ⌘C/⌘V copy/paste · ⌘L locks, ⌘⇧L unlocks · Del · ⌘Z",
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
        if esc && !std::mem::take(&mut self.escaped_drag) {
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
            ui.selectable_value(&mut self.tab, Tab::Map, "Map");
            ui.selectable_value(&mut self.tab, Tab::Simulate, "Simulate");
            ui.separator();
            if self.tab == Tab::Workbench {
                if ui.button("Open…").clicked()
                    && let Some(p) = self.dialogs.pick(FileKind::Assembly)
                {
                    self.editor.load_path(p);
                }
                if ui
                    .button("Import…")
                    .on_hover_text("a saved build's components and bricks join this library, to add from")
                    .clicked()
                    && let Some(p) = self.dialogs.pick(FileKind::Assembly)
                {
                    self.editor.import_build(p);
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
            } else {
                // the assembly's file buttons and its component path belong to the Workbench: the
                // other tabs name the map they show (the Map tab's panel saves it)
                ui.weak(format!("map: {}", self.simulate.world()));
                ui.separator();
            }
            if self.tab.plan() {
                ui.weak("plan view: the whole map, fitted");
                return;
            }
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
            if self.tab == Tab::Map {
                return;
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
            ui.checkbox(&mut self.viewport.edges, "edges")
                .on_hover_text("the bricks' edges: the seams of a stack, the rims of studs");
            ui.checkbox(&mut self.show_com, "COM");
            if ui.button("Snap").on_hover_text("S").clicked() {
                self.editor.snap_selection(true);
            }
        });
    }

    /// A component saved as a build of its own, where the dialog says.
    fn save_component_dialog(&mut self, id: &str) {
        let dir = self.editor.path.as_ref().and_then(|p| p.parent());
        if let Some(p) = self.dialogs.save(FileKind::Assembly, &format!("{id}.assembly.json"), dir) {
            self.editor.save_component(id, &p);
        }
    }

    fn save_file(&mut self, ask: bool) {
        let path = if ask || self.editor.path.is_none() {
            let dir = self.editor.path.as_ref().and_then(|p| p.parent());
            match self.dialogs.save(FileKind::Assembly, "robot.assembly.json", dir) {
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
            ui.horizontal(|ui| {
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut self.new_component_name)
                        .hint_text("new component")
                        .desired_width(160.0),
                );
                let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                let make = ui
                    .small_button("New")
                    .on_hover_text("an empty component of that name joins the library and opens to build")
                    .clicked();
                if (make || enter) && self.editor.new_component(&self.new_component_name.clone()) {
                    self.new_component_name.clear();
                }
            });
            let ids: Vec<String> = self.editor.doc.components.keys().cloned().collect();
            let mut to_add: Option<(Option<String>, Option<String>)> = None;
            let mut to_open: Option<String> = None;
            let mut to_save: Option<String> = None;
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
                    if !is_root && ui.small_button("save").on_hover_text("as a build of its own").clicked() {
                        to_save = Some(id.clone());
                    }
                });
            }
            ui.add_space(8.0);
            let sets_note = if self.editor.bundle.sets.is_empty() {
                String::new()
            } else {
                format!(
                    "; sets {} complete",
                    self.editor.bundle.sets.keys().cloned().collect::<Vec<_>>().join(" and ")
                )
            };
            ui.strong(format!(
                "LEGO  ({} parts, exact LDraw geometry{sets_note})",
                self.editor.bundle.parts.len()
            ));
            let mut nums: Vec<String> = self.editor.bundle.parts.keys().cloned().collect();
            nums.sort_by_key(|n| self.editor.bundle.parts[n].name.to_lowercase());
            let mut hits = 0;
            for num in nums {
                let rec = &self.editor.bundle.parts[&num];
                // by number, name, an inventory's own number for it, or a set that holds it (id or name)
                // a LEGO element number names the part in one colour: it finds the part and picks the colour
                let element_color = rec.colors.iter().find(|(_, els)| els.contains(&q)).map(|(c, _)| *c);
                let hit = q.is_empty()
                    || num.contains(&q)
                    || rec.name.to_lowercase().contains(&q)
                    || rec.aliases.iter().any(|a| a.contains(&q))
                    || rec
                        .sets
                        .keys()
                        .any(|s| s.contains(&q) || self.editor.bundle.sets.get(s).is_some_and(|i| i.name.to_lowercase().contains(&q)))
                    || element_color.is_some();
                if !hit {
                    continue;
                }
                hits += 1;
                if let Some(c) = element_color {
                    self.pick_color.insert(num.clone(), c);
                }
                let choices: Vec<(u32, String, [u8; 3])> = rec.colors.keys().filter_map(|c| self.editor.bundle.color_choice(*c)).collect();
                let dens = if rec.volume_mm3 > 0.0 {
                    rec.mass_g / (rec.volume_mm3 / 1000.0)
                } else {
                    0.0
                };
                let in_sets: String = rec.sets.iter().map(|(s, n)| format!(" · {s} ×{n}")).collect();
                let (name, mass) = (rec.name.clone(), rec.mass_g);
                let thumb = self.part_thumb(gpu, &mut budget, &num, dark);
                let had = self.pick_color.get(&num).copied();
                let mut picked = had;
                ui.horizontal(|ui| {
                    thumb_slot(ui, thumb);
                    if ui
                        .small_button("+")
                        .on_hover_text("add to the view, in the colour chosen")
                        .clicked()
                    {
                        to_ldraw = Some(num.clone());
                    }
                    ui.add(egui::Label::new(&name).truncate());
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new(format!("{num} · {} g · {:.2} g/cm³{in_sets}", assembly::fmt(mass), dens)).weak(),
                        )
                        .truncate(),
                    );
                    if !choices.is_empty() {
                        color_combo(ui, &format!("lib-color-{num}"), &choices, &mut picked);
                    }
                });
                if picked != had {
                    match picked {
                        Some(c) => {
                            self.pick_color.insert(num.clone(), c);
                        }
                        None => {
                            self.pick_color.remove(&num);
                        }
                    }
                }
            }
            // a number the library lacks: say so, and offer to fetch it from ldraw.org
            if hits == 0 && looks_like_part_number(&q) {
                ui.add_space(4.0);
                ui.label(format!("{q} is not in the library"));
                let mut cancel = false;
                match &self.fetch {
                    Some(f) if f.number == q => {
                        ui.horizontal_wrapped(|ui| {
                            ui.weak(format!("fetching {q}… {}", f.last));
                            if ui
                                .small_button("Cancel")
                                .on_hover_text("stop this fetch; nothing of it is kept")
                                .clicked()
                            {
                                cancel = true;
                            }
                        });
                    }
                    // one at a time: another fetch now would kill that one half way
                    Some(f) => {
                        ui.weak(format!("fetching {} first…", f.number));
                    }
                    None => {
                        if ui
                            .button(format!("Fetch {q} from LDraw"))
                            .on_hover_text("its files from ldraw.org, converted, with the colours it comes in; kept for every later launch")
                            .clicked()
                        {
                            self.start_fetch(&q.clone());
                        }
                    }
                }
                if cancel {
                    self.cancel_fetch();
                }
                if let Some((number, why)) = &self.fetch_note
                    && *number == q
                {
                    ui.colored_label(RED, why);
                }
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
                let color = self.pick_color.get(&num).copied();
                self.editor.add_brick(id, color);
            }
            if let Some((p, c)) = to_add {
                self.editor.add_instance(p, c, [0.0; 3]);
            }
            if let Some(id) = to_save {
                self.save_component_dialog(&id);
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
        let Some(p) = self.dialogs.pick(FileKind::Stl) else {
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
        if children.is_empty() {
            ui.weak("Nothing here yet: add bricks from the library");
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

    /// The colour combo for the selected bricks: the colours every one of
    /// them comes in, the choice applied to them all.
    fn selection_color_ui(&mut self, ui: &mut egui::Ui, insts: &[Instance]) {
        let choices: Vec<(u32, String, [u8; 3])> = self
            .editor
            .common_colors()
            .iter()
            .filter_map(|c| self.editor.bundle.color_choice(*c))
            .collect();
        if choices.is_empty() {
            return;
        }
        let colors: Vec<Option<u32>> = insts.iter().filter(|i| i.part.is_some()).map(|i| i.color).collect();
        let mut picked = if colors.windows(2).all(|w| w[0] == w[1]) {
            colors.first().copied().flatten()
        } else {
            None
        };
        ui.horizontal(|ui| {
            ui.weak("colour");
            if color_combo(ui, "selection-color", &choices, &mut picked) {
                self.editor.set_selection_color(picked);
            }
        });
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
                if ui
                    .button("Save as build…")
                    .on_hover_text("this component as an assembly file of its own, to open or import elsewhere")
                    .clicked()
                {
                    let id = self.editor.editing.clone();
                    self.save_component_dialog(&id);
                }
                if ui.button("Back to the robot").clicked() {
                    let root = self.editor.doc.robot.root.clone();
                    self.editor.open_component(&root, false);
                }
            }
            if !self.editor.errors.is_empty() || !self.editor.notes.is_empty() {
                ui.add_space(6.0);
                for e in self.editor.errors.iter().chain(&self.editor.notes) {
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
            self.selection_color_ui(ui, &insts);
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
        if inst.part.is_some() {
            self.selection_color_ui(ui, std::slice::from_ref(&inst));
            let elements = self.editor.elements_of(&inst);
            if !elements.is_empty() {
                ui.weak(format!("LEGO element {}", elements.join(", ")));
            }
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
                let note = self.editor.overlap_note();
                if !note.is_empty() {
                    ui.colored_label(egui::Color32::from_rgb(214, 62, 48), note);
                }
            });
        });
    }

    /// The Map tab: the map editor's panel and the plan view.
    fn map_ui(&mut self, ui: &mut egui::Ui, gpu: Option<&Gpu>) {
        self.simulate.ensure_loaded();
        if self.simulate.pump() || self.simulate.is_live() {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(16));
        }
        egui::Panel::right("map-editor")
            .default_size(360.0)
            .size_range(300.0..=520.0)
            .resizable(true)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.simulate.map_ui(ui, &self.editor.bundle);
                    self.map_add_ui(ui);
                });
            });
        egui::CentralPanel::default().show(ui, |ui| self.sim_view_ui(ui, gpu));
    }

    /// A saved build, opened for the map editor: its components join the
    /// list the Map tab adds from, and it is the build listed. A file
    /// that is not an assembly is refused with the reason.
    pub fn open_build(&mut self, p: std::path::PathBuf) -> Result<(), String> {
        let text = std::fs::read_to_string(&p).map_err(|e| format!("{}: {e}", p.display()))?;
        let doc: assembly::Document = serde_json::from_str(&text).map_err(|e| format!("{}: not an assembly file ({e})", p.display()))?;
        let errs = assembly::validate(&doc, &self.editor.bundle);
        if errs.iter().any(|e| e.starts_with("format") || e.starts_with("robot.root")) {
            return Err(format!("{}: not an assembly file: {}", p.display(), errs.join("; ")));
        }
        let k = match self.map_builds.iter().position(|(q, _)| q == &p) {
            Some(k) => {
                self.map_builds[k].1 = doc;
                k
            }
            None => {
                self.map_builds.push((p, doc));
                self.map_builds.len() - 1
            }
        };
        self.map_build = k + 1;
        self.map_note.clear();
        Ok(())
    }

    /// `open_build`, a refusal shown in the panel.
    pub fn open_build_noted(&mut self, p: std::path::PathBuf) {
        if let Err(e) = self.open_build(p) {
            self.map_note = e;
        }
    }

    /// Props from elsewhere: a component of a build from the Workbench —
    /// the one open there, or any saved build opened here — or one brick
    /// from the library, placed at the map's origin.
    fn map_add_ui(&mut self, ui: &mut egui::Ui) {
        ui.separator();
        ui.strong("Add to the map");
        ui.weak("a component of a build from the Workbench, at the origin");
        // the builds to add from: the Workbench's, then every saved build opened here; a row
        // chooses the one whose components are listed
        let title = self.title_name();
        let names: Vec<String> = std::iter::once(format!("{title} · open in the Workbench"))
            .chain(self.map_builds.iter().map(|(p, _)| file_name(p)))
            .collect();
        for (k, name) in names.iter().enumerate() {
            if ui.selectable_label(self.map_build == k, name).clicked() {
                self.map_build = k;
            }
        }
        if ui
            .button("Open a build…")
            .on_hover_text("any saved assembly: its components join the list")
            .clicked()
            && let Some(p) = rfd::FileDialog::new().add_filter("assembly", &["json"]).pick_file()
        {
            self.open_build_noted(p);
        }
        if !self.map_note.is_empty() {
            ui.colored_label(RED, &self.map_note);
        }
        // the chosen build's components that hold bricks, the whole build first
        let shown = self.map_build.min(self.map_builds.len());
        let (doc, label) = if shown == 0 {
            (&self.editor.doc, title)
        } else {
            let (p, d) = &self.map_builds[shown - 1];
            (d, file_name(p))
        };
        let root = doc.robot.root.clone();
        let ids: Vec<String> = std::iter::once(root.clone())
            .chain(doc.components.keys().filter(|id| **id != root).cloned())
            .collect();
        let mut to_add: Option<(String, assembly::Document)> = None;
        let mut any = false;
        for id in ids {
            let Some(sub) = assembly::subset(doc, &id) else { continue };
            let bricks = assembly::flatten(&sub, &id).len();
            if bricks == 0 {
                continue;
            }
            any = true;
            ui.horizontal(|ui| {
                if ui.small_button(format!("+ {id}")).clicked() {
                    let name = if id == root { build_stem(&label) } else { id.clone() };
                    to_add = Some((name, sub.clone()));
                }
                let what = if id == root { "the whole build" } else { "a component" };
                ui.weak(format!("{what} · {bricks} brick{}", if bricks == 1 { "" } else { "s" }));
            });
        }
        if !any {
            ui.weak("this build has no bricks yet");
        }
        if let Some((name, sub)) = to_add {
            self.simulate.add_model(&name, &sub);
        }
        ui.weak("or one brick from the library");
        ui.add(egui::TextEdit::singleline(&mut self.map_search).hint_text("search bricks by number or name"));
        let q = self.map_search.trim().to_lowercase();
        if !q.is_empty() {
            let hits: Vec<(String, String)> = self
                .editor
                .bundle
                .parts
                .iter()
                .filter(|(n, r)| n.contains(&q) || r.name.to_lowercase().contains(&q))
                .take(12)
                .map(|(n, r)| (n.clone(), r.name.clone()))
                .collect();
            if hits.is_empty() {
                ui.weak("no brick matches");
            }
            for (num, name) in hits {
                ui.horizontal(|ui| {
                    // the button first: a truncating label takes the width left, not the button's
                    if ui.small_button("+ to map").clicked()
                        && let Some(doc) = assembly::brick_document(&self.editor.bundle, &num)
                    {
                        self.simulate.add_model(&name, &doc);
                    }
                    ui.add(egui::Label::new(format!("{num} · {name}")).truncate());
                });
            }
        }
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
        let draw = self
            .simulate
            .draw_items(&mut self.viewport, &gpu.device, &gpu.queue, &self.editor.bundle, dark);
        // a prop just added is moved off whatever it landed on, once the frame shows it
        self.simulate.settle_new_prop(&self.editor.bundle);
        let (items, lines, ghost) = (draw.items, draw.lines, draw.ghost);
        let editing = self.tab == Tab::Map;
        // over the map, whatever is drawn on it: the props' outlines when editing the map, else
        // the markers and the route
        let top = if editing {
            self.simulate.prop_lines(dark)
        } else {
            self.simulate.route_lines(dark)
        };
        // the plan view never pans or zooms: it fits the map, and again whenever its size changes;
        // the map editor's 3D camera frames a map when the map is new to it (or on Fit), then it
        // is the user's: an edit's reload leaves it where it was put
        if !editing && self.plan_size != size {
            self.plan_size = size;
            self.simulate.refit();
        }
        if editing {
            if let Some((lo, hi)) = self.simulate.map_frame_target() {
                self.viewport.camera.fit(lo, hi);
            }
        } else if let Some((lo, hi)) = self.simulate.frame_target() {
            self.viewport.camera.fit_plan(lo, hi, size.0 as f32 / size.1.max(1) as f32);
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
                top_lines: &top,
                ghost: &ghost,
                overlay: &[],
                background: bg,
            };
            self.viewport.render(&gpu.device, &gpu.queue, &mut renderer, size, &scene)
        };
        let response = ui.add(egui::Image::new((tex, egui::vec2(size.0 as f32, size.1 as f32))).sense(egui::Sense::click_and_drag()));
        self.view_rect = response.rect;
        if editing && response.hovered() {
            // the wheel zooms, and so does a pinch (a trackpad's)
            let (scroll, pinch) = ui.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
            let f = (1.0 - scroll * 0.002).clamp(0.5, 2.0) / pinch.clamp(0.5, 2.0);
            if f != 1.0 {
                self.viewport.camera.distance = (self.viewport.camera.distance * f).clamp(50.0, 50000.0);
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
        let (shift, command) = ui.input(|i| (i.modifiers.shift, i.modifiers.command));
        // the pointer on the map, for the rubber band of a placement, and the prop under it
        self.simulate.hover = response
            .hover_pos()
            .and_then(|p| {
                let (x, y) = local(p);
                ground(x, y)
            })
            .map(|h| [h.x as f64, h.y as f64]);
        // shift frees a straight's end from the heading it nears
        self.simulate.free = shift;
        self.simulate.hover_prop = if editing {
            response.hover_pos().and_then(|p| {
                let (x, y) = local(p);
                self.simulate.prop_of_item(self.viewport.pick(&items, x, y)?)
            })
        } else {
            None
        };
        // a press on a prop (editing the map), a route handle or a path drags it, on the chassis
        // moves it; anywhere else nothing: the plan view does not pan
        if response.drag_started_by(egui::PointerButton::Primary) {
            let origin = ui
                .input(|i| i.pointer.press_origin())
                .map(local)
                .or(response.interact_pointer_pos().map(local));
            self.drag = Drag::None;
            let marker = if editing {
                None
            } else {
                origin.and_then(|(x, y)| self.simulate.marker_at(&cam, x, y, w, h))
            };
            let handle = if editing {
                None
            } else {
                origin.and_then(|(x, y)| Some((self.simulate.route_handle_at(&cam, x, y, w, h)?, ground(x, y)?)))
            };
            let prop = if editing {
                origin.and_then(|(x, y)| Some((self.simulate.prop_of_item(self.viewport.pick(&items, x, y)?)?, ground(x, y)?, x)))
            } else {
                None
            };
            if let Some((i, hit, x)) = prop {
                if let Some(pose0) = self.simulate.begin_prop_drag(i) {
                    self.drag = Drag::Prop {
                        i,
                        px0: x,
                        pose0,
                        grab: [pose0.x_mm - hit.x as f64, pose0.y_mm - hit.y as f64],
                    };
                }
            } else if let Some(i) = marker {
                if self.simulate.begin_marker_drag(i) {
                    self.drag = Drag::Marker;
                }
            } else if let Some(((i, handle), hit)) = handle {
                if self.simulate.begin_handle_drag(i, handle, [hit.x as f64, hit.y as f64]) {
                    self.drag = Drag::RouteHandle;
                }
            } else {
                let on_chassis = origin
                    .and_then(|(x, y)| self.viewport.pick(&items, x, y))
                    .map(|i| self.simulate.chassis_items.contains(&i))
                    .unwrap_or(false);
                if on_chassis
                    && let Some((x, _)) = origin
                    && let Some(pose0) = self.simulate.begin_chassis_drag()
                {
                    self.drag = Drag::Chassis { px0: x, pose0 };
                } else if editing {
                    // a drag on the map orbits; with shift or ⌘ held it pans, for a pointer with no
                    // right button
                    self.drag = if shift || command { Drag::Pan } else { Drag::Orbit };
                }
            }
        }
        if editing && (response.drag_started_by(egui::PointerButton::Secondary) || response.drag_started_by(egui::PointerButton::Middle)) {
            self.drag = Drag::Pan;
        }
        // a click on the map: editing it, selects the prop under the pointer (or nothing); planning
        // a route, places the armed action's next point, or selects the path under it
        if editing && response.clicked_by(egui::PointerButton::Primary) {
            self.simulate.selected_prop = response.interact_pointer_pos().and_then(|pos| {
                let (x, y) = local(pos);
                let i = self.simulate.prop_of_item(self.viewport.pick(&items, x, y)?)?;
                self.simulate.prop_name(i)
            });
        }
        if !editing
            && response.clicked_by(egui::PointerButton::Primary)
            && self.simulate.draft.is_none()
            && let Some(pos) = response.interact_pointer_pos()
            && let Some(hit) = {
                let (x, y) = local(pos);
                ground(x, y)
            }
        {
            let tol = (15.0 * cam.units_per_px(h)) as f64;
            let p = [hit.x as f64, hit.y as f64];
            self.simulate.note_click(p);
            if self.simulate.map_click(p[0], p[1], tol) {
                if let Some(d) = self.simulate.draft.as_mut() {
                    d.at = Some(pos + egui::vec2(12.0, 12.0));
                }
                if let Some(m) = self.simulate.marker_draft.as_mut() {
                    m.2 = Some(pos + egui::vec2(12.0, 12.0));
                }
            } else {
                self.simulate.select_at(p, tol);
            }
        }
        let delta = response.drag_delta();
        match &self.drag {
            Drag::Orbit if response.dragged() => {
                self.viewport.camera.yaw += delta.x * 0.5;
                self.viewport.camera.pitch = (self.viewport.camera.pitch + delta.y * 0.5).clamp(-89.0, 89.0);
            }
            Drag::Pan if response.dragged() => {
                let c = &mut self.viewport.camera;
                let scale = c.distance * 0.0015;
                let (r, u) = (c.right(), c.up());
                c.target = c.target - r * delta.x * scale + u * delta.y * scale;
            }
            Drag::Prop { i, px0, pose0, grab } if response.dragged() => {
                let (i, px0, pose0, grab) = (*i, *px0, *pose0, *grab);
                if let Some((x, y)) = response.interact_pointer_pos().map(local) {
                    let pose = if shift {
                        // shift turns it: a screen-width drag is a full turn
                        Pose2 {
                            yaw_deg: pose0.yaw_deg - ((x - px0) * 360.0 / w.max(1.0)) as f64,
                            ..pose0
                        }
                    } else if let Some(hit) = ground(x, y) {
                        // it keeps the point it was taken hold of under the pointer
                        Pose2 {
                            x_mm: hit.x as f64 + grab[0],
                            y_mm: hit.y as f64 + grab[1],
                            yaw_deg: pose0.yaw_deg,
                        }
                    } else {
                        pose0
                    };
                    self.simulate.drag_prop(i, pose, &self.editor.bundle);
                }
            }
            Drag::Chassis { px0, pose0 } if response.dragged() => {
                let (px0, pose0) = (*px0, *pose0);
                if let Some((x, y)) = response.interact_pointer_pos().map(local) {
                    let pose = if shift {
                        // shift turns it: a screen-width drag is a full turn
                        Pose2 {
                            yaw_deg: pose0.yaw_deg - ((x - px0) * 360.0 / w.max(1.0)) as f64,
                            ..pose0
                        }
                    } else if let Some(hit) = ground(x, y) {
                        // the ghost's axle centre sits under the pointer
                        Pose2 {
                            x_mm: hit.x as f64,
                            y_mm: hit.y as f64,
                            yaw_deg: pose0.yaw_deg,
                        }
                    } else {
                        pose0
                    };
                    self.simulate.drag_chassis(pose);
                }
            }
            Drag::RouteHandle if response.dragged() => {
                if let Some((x, y)) = response.interact_pointer_pos().map(local)
                    && let Some(hit) = ground(x, y)
                {
                    self.simulate.drag_handle(hit.x as f64, hit.y as f64);
                }
            }
            Drag::Marker if response.dragged() => {
                if let Some((x, y)) = response.interact_pointer_pos().map(local)
                    && let Some(hit) = ground(x, y)
                {
                    self.simulate.drag_marker(hit.x as f64, hit.y as f64);
                }
            }
            _ => {}
        }
        if response.drag_stopped() {
            match self.drag {
                Drag::Prop { .. } => self.simulate.end_prop_drag(),
                Drag::Chassis { .. } => self.simulate.end_chassis_drag(),
                Drag::RouteHandle => self.simulate.end_handle_drag(),
                Drag::Marker => self.simulate.end_marker_drag(),
                _ => {}
            }
            self.drag = Drag::None;
        }
        if editing && !ui.ctx().egui_wants_keyboard_input() {
            let (esc, del, dup, fit, turn) = ui.input(|i| {
                (
                    i.key_pressed(egui::Key::Escape),
                    i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace),
                    i.modifiers.command && i.key_pressed(egui::Key::D),
                    i.key_pressed(egui::Key::F),
                    i.key_pressed(egui::Key::R),
                )
            });
            if esc {
                self.simulate.selected_prop = None;
            }
            if del {
                self.simulate.remove_selected_prop();
            }
            if dup {
                self.simulate.duplicate_selected_prop();
            }
            if fit {
                self.simulate.frame_map();
            }
            if turn {
                self.simulate.turn_selected_prop(90.0, &self.editor.bundle);
            }
        }
        if !editing && !ui.ctx().egui_wants_keyboard_input() {
            let mut copy = false;
            let mut cut = false;
            let mut paste = None;
            let (esc, del, undo, lock, unlock, fit) = ui.input(|i| {
                for ev in &i.events {
                    match ev {
                        egui::Event::Copy => copy = true,
                        egui::Event::Cut => cut = true,
                        egui::Event::Paste(t) => paste = Some(t.clone()),
                        _ => {}
                    }
                }
                (
                    i.key_pressed(egui::Key::Escape),
                    i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace),
                    i.modifiers.command && i.key_pressed(egui::Key::Z),
                    i.modifiers.command && !i.modifiers.shift && i.key_pressed(egui::Key::L),
                    i.modifiers.command && i.modifiers.shift && i.key_pressed(egui::Key::L),
                    i.key_pressed(egui::Key::F),
                )
            });
            if esc {
                if self.simulate.placing.is_some() || self.simulate.draft.is_some() || self.simulate.marker_draft.is_some() {
                    self.simulate.cancel();
                } else {
                    self.simulate.selected = None;
                    self.simulate.selected_marker = None;
                }
            }
            if del {
                if self.simulate.selected_marker.is_some() {
                    self.simulate.remove_selected_marker();
                } else {
                    self.simulate.remove_selected();
                }
            }
            if undo {
                self.simulate.undo();
            }
            if lock {
                self.simulate.set_locked(true);
            }
            if unlock {
                self.simulate.set_locked(false);
            }
            if fit {
                self.simulate.refit();
            }
            if (copy || cut)
                && let Some(text) = self.simulate.copy_selected()
            {
                ui.ctx().copy_text(text);
                if cut {
                    self.simulate.remove_selected();
                }
            }
            if let Some(text) = paste {
                self.simulate.paste(&text);
            }
        }
        if items.is_empty() {
            ui.painter().text(
                response.rect.center(),
                egui::Align2::CENTER_CENTER,
                "The map appears here once the run server has built it",
                egui::FontId::proportional(14.0),
                ui.visuals().weak_text_color(),
            );
            // what is holding it up: the status, and the last message when there is one
            ui.painter().text(
                response.rect.center() + egui::vec2(0.0, 22.0),
                egui::Align2::CENTER_CENTER,
                self.simulate.status_line(),
                egui::FontId::proportional(12.0),
                ui.visuals().weak_text_color(),
            );
        }
        // each action's number and parameters on the map (on a backing so they read over the
        // mat), and the popup of a freshly placed one
        let (label_color, label_bg) = if dark {
            (
                egui::Color32::from_rgb(255, 210, 120),
                egui::Color32::from_rgba_unmultiplied(0, 0, 0, 150),
            )
        } else {
            (
                egui::Color32::from_rgb(140, 70, 0),
                egui::Color32::from_rgba_unmultiplied(255, 255, 255, 180),
            )
        };
        let labels = if editing {
            self.simulate.prop_labels(&cam, w, h)
        } else {
            self.simulate.route_labels(&cam, w, h)
        };
        for (p, text) in labels {
            let galley = ui.painter().layout_no_wrap(text, egui::FontId::proportional(13.0), label_color);
            let anchor = rect.min + egui::vec2(p.x, p.y);
            let r = egui::Rect::from_min_size(anchor - egui::vec2(0.0, galley.size().y), galley.size()).expand(2.0);
            ui.painter().rect_filled(r, 3.0, label_bg);
            ui.painter().galley(r.min + egui::vec2(2.0, 2.0), galley, label_color);
        }
        if self.simulate.click_marker().is_some() || self.simulate.placing.is_some() {
            // the click's ring fades out and the rubber band follows the pointer
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(40));
        }
        if !editing {
            self.simulate.draft_ui(ui.ctx());
        }
        ui.painter().text(
            response.rect.left_bottom() + egui::vec2(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            if editing { self.simulate.map_hint() } else { self.simulate.hint() },
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

/// A path's file name, for a list of builds.
fn file_name(p: &std::path::Path) -> String {
    p.file_name().unwrap_or(p.as_os_str()).to_string_lossy().to_string()
}

/// A build's name from its file name: what precedes the first dot
/// (`gate.assembly.json` is `gate`).
fn build_stem(name: &str) -> String {
    name.split('.').next().filter(|s| !s.is_empty()).unwrap_or(name).to_string()
}

impl App {
    /// One frame of the whole window, drawn with `gpu` (None shows a
    /// notice where the 3D views would be).
    pub fn frame_ui(&mut self, ui: &mut egui::Ui, gpu: Option<&Gpu>) {
        if self.tab != self.camera_tab {
            // each tab keeps its camera: stow the shown one, take out the new tab's
            let shown = std::mem::replace(&mut self.viewport.camera, self.cameras[self.tab.index()].clone());
            self.cameras[self.camera_tab.index()] = shown;
            self.camera_tab = self.tab;
        }
        egui::Panel::top("toolbar").show(ui, |ui| self.toolbar(ui));
        egui::Panel::bottom("status").show(ui, |ui| self.status_bar(ui));
        match self.tab {
            Tab::Simulate => {
                self.simulate_ui(ui, gpu);
            }
            Tab::Map => {
                self.map_ui(ui, gpu);
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
        self.poll_fetch(ui, gpu);
        self.autosave(std::time::Instant::now(), crate::drafts::now_ms());
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
    use crate::sim::testing::{fake_fetcher, fake_server};
    use crate::viewport::testing::{test_device, test_renderer};
    use egui::{Event, Key, Modifiers, PointerButton, Pos2};
    use egui_kittest::Harness;
    use egui_kittest::kittest::{By, NodeT, Queryable};

    #[test]
    fn colours_and_labels() {
        assert_ne!(cat_color("lego", false), cat_color("lego", true));
        assert_eq!(cat_color("mystery", false), cat_color("other", false));
        assert_eq!(rgb_color([0x5B, 0x7A, 0x9C]), cat_color("lego", false));
        assert_eq!(FileKind::Assembly.filter(), ("assembly", &["json"][..]));
        assert_eq!(FileKind::Stl.filter(), ("STL", &["stl"][..]));
        let answered = Dialogs::answering(Some(PathBuf::from("/x/y.json")));
        assert_eq!(answered.pick(FileKind::Stl), Some(PathBuf::from("/x/y.json")));
        assert_eq!(answered.save(FileKind::Assembly, "a.json", None), Some(PathBuf::from("/x/y.json")));
        assert_eq!(Dialogs::answering(None).pick(FileKind::Assembly), None);
        assert_ne!(rgb_color([200, 30, 30]), rgb_color([30, 30, 200]));
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
        let mut app = App::with_gpu(gpu, real_bundle(), None, python);
        // never the user's own drafts: the frames would keep them there
        app.drafts_in(std::env::temp_dir().join(format!("ob-harness-drafts-{}", std::process::id())));
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
    fn a_brick_dragged_into_its_neighbour_is_flushed_red_and_put_back() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        {
            let app = h.state_mut();
            let root = app.editor.doc.robot.root.clone();
            app.editor.doc.components.get_mut(&root).unwrap().children.clear();
            app.editor.doc.robot.roles.clear();
            app.editor.recompute();
            app.editor.magnet = false;
            let id = app.editor.ensure_ldraw_part("3001").unwrap();
            app.editor.add_instance(Some(id.clone()), None, [0.0; 3]);
            app.editor.add_instance(Some(id), None, [0.0; 3]);
            app.editor.fit_pending = true;
        }
        steps(&mut h, 3);
        let b = h.state().editor.selection[0].clone();
        assert_eq!(h.state().editor.selected_instances()[0].pos, [32.0, 0.0, 0.0], "beside the first");
        // nothing selected (so no gizmo under the pointer): press on the second brick's top face
        // and drag it one module into the first
        h.state_mut().editor.selection.clear();
        h.state_mut().viewport.camera.distance *= 1.6;
        steps(&mut h, 2);
        let from = on_screen(h.state(), Vec3::new(40.0, -4.0, 0.0));
        let to = on_screen(h.state(), Vec3::new(32.0, -4.0, 0.0));
        let rect = h.state().view_rect;
        assert!(rect.contains(from) && rect.contains(to), "{from:?} {to:?} in {rect:?}");
        press(&mut h, from, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, from + (to - from) * 0.5, Modifiers::NONE);
        drag_to(&mut h, to, Modifiers::NONE);
        steps(&mut h, 2);
        assert_eq!(h.state().editor.selection, vec![b.clone()]);
        assert_eq!(h.state().editor.selected_instances()[0].pos, [24.0, 0.0, 0.0]);
        assert!(!h.state().editor.overlapping.is_empty(), "{}", h.state().editor.status);
        let i = h.state().item_tops.iter().position(|t| *t == b).unwrap();
        let c = h.state().items[i].color;
        assert!(c[0] > 0.6 && c[0] > c[1] * 2.0, "flushed red while it would overlap: {c:?}");
        h.get_by_label_contains("would overlap");
        release(&mut h, to, PointerButton::Primary);
        steps(&mut h, 2);
        assert_eq!(h.state().editor.selected_instances()[0].pos, [32.0, 0.0, 0.0], "put back");
        assert!(
            h.state().editor.status.starts_with(&format!("{b} put back")),
            "{}",
            h.state().editor.status
        );
        assert!(h.state().editor.overlapping.is_empty());
        // Escape in the middle of such a drag abandons it: the brick goes back at once and the
        // release that follows does nothing
        let depth = h.state().editor.undo_depth();
        press(&mut h, from, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, from + (to - from) * 0.5, Modifiers::NONE);
        drag_to(&mut h, to, Modifiers::NONE);
        assert_eq!(h.state().editor.selected_instances()[0].pos, [24.0, 0.0, 0.0]);
        h.key_press(Key::Escape);
        h.step();
        assert_eq!(h.state().editor.selected_instances()[0].pos, [32.0, 0.0, 0.0], "abandoned");
        assert!(matches!(h.state().drag, Drag::None));
        release(&mut h, to, PointerButton::Primary);
        steps(&mut h, 2);
        assert_eq!(h.state().editor.selected_instances()[0].pos, [32.0, 0.0, 0.0]);
        assert_eq!(h.state().editor.undo_depth(), depth, "no undo point from an abandoned drag");
        // dragged the other way it is kept, and not red
        let away = on_screen(h.state(), Vec3::new(48.0, -4.0, 0.0));
        press(&mut h, from, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, from + (away - from) * 0.5, Modifiers::NONE);
        drag_to(&mut h, away, Modifiers::NONE);
        steps(&mut h, 2);
        assert!(h.state().editor.overlapping.is_empty());
        let i = h.state().item_tops.iter().position(|t| *t == b).unwrap();
        let c = h.state().items[i].color;
        assert!(c[0] < 0.6 || c[0] < c[1] * 2.0, "not red: {c:?}");
        release(&mut h, away, PointerButton::Primary);
        steps(&mut h, 2);
        assert_eq!(h.state().editor.selected_instances()[0].pos, [40.0, 0.0, 0.0]);
        assert_eq!(h.state().editor.undo_depth(), depth + 1);
    }

    /// Steps frames until `done` holds, a little while at most.
    fn wait_until(h: &mut Harness<'_, App>, done: impl Fn(&App) -> bool) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        // a frame first: a click just made takes effect in the next one
        h.step();
        while !done(h.state()) {
            assert!(std::time::Instant::now() < deadline, "waited in vain");
            std::thread::sleep(std::time::Duration::from_millis(10));
            h.step();
        }
    }

    #[test]
    fn a_part_the_library_lacks_is_fetched_by_number_and_joins_it() {
        let Some(gpu) = gpu() else { return };
        // the fetcher stand-in hands back the library's 2x4 brick under the number asked for
        let bundle = real_bundle();
        let mut rec = bundle.parts["3001"].clone();
        rec.name = "Brick  1 x  2 with Pin".into();
        rec.ldraw = "2458".into();
        rec.fetched = true;
        let one = crate::bundle::Bundle {
            format: bundle.format.clone(),
            source: "test".into(),
            parts: [("2458".to_string(), rec)].into_iter().collect(),
            missing: vec![],
            sets: Default::default(),
            colors: Default::default(),
        };
        let Some(fake) = fake_fetcher("app", &serde_json::to_string(&one).unwrap()) else {
            return;
        };
        let mut h = harness(&gpu, Some(fake.python.clone()));
        h.state_mut().fetching_with(fake.env.clone(), fake.dir.join("bricks"));
        assert!(!looks_like_part_number("wro") && !looks_like_part_number("1") && looks_like_part_number("3648b"));
        // a search for a number the library lacks says so and offers the fetch
        h.state_mut().search = "2458".into();
        steps(&mut h, 2);
        assert!(h.query_by_label("2458 is not in the library").is_some());
        h.get_by_label("Fetch 2458 from LDraw").click();
        steps(&mut h, 2);
        // under way (the stand-in takes a moment on purpose): its row says so
        assert!(h.state().fetch.is_some(), "{}", h.state().note_text());
        assert!(h.query_by_label_contains("fetching 2458").is_some());
        wait_until(&mut h, |a| a.fetch.is_none());
        assert!(h.state().fetch_note.is_none(), "{}", h.state().note_text());
        assert!(h.state().editor.bundle.parts.contains_key("2458"));
        assert!(
            h.state()
                .editor
                .status
                .starts_with("2458 Brick  1 x  2 with Pin joined the library (19 files, 15 colours)"),
            "{}",
            h.state().editor.status
        );
        assert!(fake.dir.join("bricks").join("2458.json").exists());
        steps(&mut h, 2);
        // the row is there now; + places it, with the record carried in the build
        assert!(h.query_by_label("2458 is not in the library").is_none());
        let n = h.state().editor.children().len();
        h.get_by_label("+").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.children().len(), n + 1);
        let part = h.state().editor.doc.parts["lego_2458"].clone();
        assert_eq!(part.ldraw.as_deref(), Some("2458"));
        assert!(
            part.extra.contains_key("mesh") && part.extra.contains_key("connectors"),
            "carried along"
        );
        // a library without the part still opens the build: its geometry comes from the record
        let doc = h.state().editor.doc.clone();
        let ed = Editor::new(real_bundle(), Some((std::path::PathBuf::from("x.assembly.json"), doc)));
        assert!(ed.errors.is_empty(), "{:?}", ed.errors);
        assert!(matches!(
            assembly::geometry_of(&ed.doc.parts["lego_2458"], &ed.bundle),
            assembly::Geometry::Imported { .. }
        ));
        assert!(ed.leaves.iter().any(|l| l.part_id == "lego_2458"));
        let leaf = ed.leaves.iter().find(|l| l.part_id == "lego_2458").unwrap();
        assert!(
            assembly::connectors_of_leaf(&ed.doc, &ed.bundle, leaf)
                .iter()
                .any(|c| c.kind == "stud_socket"),
            "it seats on studs there too"
        );
        // ldraw.org has no such part, and a fetcher that dies is said too — under that number's
        // line alone
        h.state_mut().search = "4040001".into();
        steps(&mut h, 2);
        h.get_by_label("Fetch 4040001 from LDraw").click();
        wait_until(&mut h, |a| a.fetch.is_none());
        assert_eq!(
            h.state().fetch_note,
            Some(("4040001".into(), "ldraw.org has no part 4040001".into()))
        );
        steps(&mut h, 2);
        assert!(h.query_by_label("ldraw.org has no part 4040001").is_some());
        h.state_mut().search = "4040002".into();
        steps(&mut h, 2);
        assert!(h.query_by_label("ldraw.org has no part 4040001").is_none(), "another number's line");
        assert!(h.query_by_label("Fetch 4040002 from LDraw").is_some());
        h.state_mut().search = "crash".into();
        steps(&mut h, 2);
        assert!(h.query_by_label_contains("is not in the library").is_none(), "not a number");
        h.state_mut().search = "99090001".into();
        steps(&mut h, 2);
        h.get_by_label("Fetch 99090001 from LDraw").click();
        wait_until(&mut h, |a| a.fetch.is_none());
        assert!(
            h.state().note_text().contains("without a word") || h.state().note_text().contains("would not read"),
            "{}",
            h.state().note_text()
        );
        // a fetched file that will not read is said under its number
        h.state_mut().search = "99040001".into();
        steps(&mut h, 2);
        h.get_by_label("Fetch 99040001 from LDraw").click();
        wait_until(&mut h, |a| a.fetch.is_none());
        assert!(
            h.state()
                .note_text()
                .starts_with("99040001 was fetched but its file would not read"),
            "{}",
            h.state().note_text()
        );
        // a part without colours says so in the status line
        h.state_mut().search = "99050001".into();
        steps(&mut h, 2);
        h.get_by_label("Fetch 99050001 from LDraw").click();
        wait_until(&mut h, |a| a.fetch.is_none());
        assert!(
            h.state()
                .editor
                .status
                .contains("(19 files, Rebrickable lists no colours for 99050001)"),
            "{}",
            h.state().editor.status
        );
        // one fetch at a time: while one runs, another number's line says so instead of offering
        // a button, and the running one can be cancelled
        h.state_mut().search = "99060001".into();
        steps(&mut h, 2);
        h.get_by_label("Fetch 99060001 from LDraw").click();
        steps(&mut h, 2);
        assert!(h.state().fetch.is_some(), "{}", h.state().note_text());
        h.state_mut().search = "99060002".into();
        steps(&mut h, 2);
        assert!(h.query_by_label_contains("fetching 99060001 first").is_some());
        assert!(h.query_by_label("Fetch 99060002 from LDraw").is_none());
        h.state_mut().start_fetch("99060002");
        assert_eq!(
            h.state().fetch.as_ref().map(|f| f.number.as_str()),
            Some("99060001"),
            "not replaced"
        );
        assert_eq!(h.state().note_text(), "99060001 is still being fetched");
        h.state_mut().search = "99060001".into();
        steps(&mut h, 2);
        assert!(h.query_by_label_contains("fetching 99060001").is_some());
        h.get_by_label("Cancel").click();
        steps(&mut h, 2);
        assert!(h.state().fetch.is_none());
        assert_eq!(h.state().editor.status, "The fetch of 99060001 was cancelled");
        assert!(h.query_by_label("Fetch 99060001 from LDraw").is_some(), "offered again");
        assert!(!h.state().editor.bundle.parts.contains_key("99060001"));
        let _ = std::fs::remove_dir_all(&fake.dir);
    }

    #[test]
    fn without_a_python_the_fetch_says_what_it_needs() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        h.state_mut().search = "2458".into();
        steps(&mut h, 2);
        h.get_by_label("Fetch 2458 from LDraw").click();
        steps(&mut h, 2);
        assert!(h.state().fetch.is_none());
        assert!(h.state().note_text().contains("openbricks sim"), "{}", h.state().note_text());
        // a Python that will not start is said too
        h.state_mut().python = Some("/no/such/python".into());
        h.state_mut().start_fetch("2458");
        assert!(h.state().fetch.is_none());
        assert!(
            h.state().note_text().contains("could not start /no/such/python"),
            "{}",
            h.state().note_text()
        );
        // what went wrong on the way in is listed with the build's problems, and stays there
        // when they are checked afresh
        h.state_mut()
            .editor
            .notes
            .push("fetched part file /x/bricks/2458.json: bundle JSON: EOF".into());
        h.state_mut().editor.selection.clear();
        steps(&mut h, 2);
        assert!(h.query_by_label_contains("fetched part file /x/bricks/2458.json").is_some());
        h.state_mut().editor.reset_to_example();
        h.state_mut().editor.forget_part("2458");
        steps(&mut h, 2);
        assert!(h.query_by_label_contains("fetched part file /x/bricks/2458.json").is_some());
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
        h.get_by_label("edges").click();
        h.step();
        assert!(!h.state().show_grid && !h.state().show_com && !h.state().editor.magnet);
        assert!(!h.state().viewport.edges, "the edges toggle turns the bricks' edges off");
        h.get_by_label("Snap").click();
        h.step();
        assert_eq!(h.state().editor.status, "Select an item to snap");
        h.get_by_label("Example").click();
        h.step();
        assert!(h.state().editor.dirty);
        assert_eq!(h.state().title_name(), "example");
        // the Simulate tab is a fixed plan: no view angles or editing controls, its own top-down camera
        h.get_by_label("Simulate").click();
        steps(&mut h, 2);
        assert_eq!(h.state().tab, Tab::Simulate);
        assert!(h.query_by_label("Iso").is_none() && h.query_by_label("Move").is_none() && h.query_by_label("ground").is_none());
        let cam = h.state().viewport.camera.clone();
        assert!(cam.ortho && cam.pitch == 90.0 && cam.yaw == -90.0, "{cam:?}");
        assert!(h.state().simulate.fit_is_pending(), "the plan fits the map as soon as it is there");
        assert!(
            h.query_by_label("Fit").is_none(),
            "nothing to fit by hand: the plan is the whole map"
        );
        // and back: the workbench's camera is as it was left
        h.get_by_label("Workbench").click();
        steps(&mut h, 2);
        assert_eq!(h.state().tab, Tab::Workbench);
        let cam = h.state().viewport.camera.clone();
        assert!(!cam.ortho && (cam.yaw, cam.pitch) == (-128.0, 28.0), "{cam:?}");
        assert!(h.query_by_label("Iso").is_some());
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
        // the WRO sets: a set's number lists its bricks, each row saying how many the set holds;
        // the sets' names find them too, and so does an inventory's own number for a part
        let (in_45819, in_any, beam13) = {
            let bundle = &h.state().editor.bundle;
            assert_eq!(bundle.sets["45811"].pieces, 724);
            let rec = &bundle.parts["41239"];
            let dens = rec.mass_g / (rec.volume_mm3 / 1000.0);
            (
                bundle.parts.values().filter(|r| r.sets.contains_key("45819")).count(),
                bundle.parts.values().filter(|r| !r.sets.is_empty()).count(),
                format!("41239 · {} g · {dens:.2} g/cm³ · 45819 ×8", assembly::fmt(rec.mass_g)),
            )
        };
        assert!(in_45819 >= 60 && in_any > in_45819, "{in_45819} / {in_any}");
        h.state_mut().search = "45819".into();
        h.step();
        assert_eq!(h.get_all_by_label("+").count(), in_45819);
        assert!(h.query_by_label(&beam13).is_some(), "{beam13}");
        h.state_mut().search = "olympiad".into();
        h.step();
        assert_eq!(h.get_all_by_label("+").count(), in_any);
        h.state_mut().search = "78c18".into();
        h.step();
        assert_eq!(h.get_all_by_label("+").count(), 1);
        assert!(
            h.query_by_label("Technic Ribbed Hose 18L").is_some(),
            "the inventory's number finds LDraw's part"
        );
        h.state_mut().search.clear();
        h.step();
    }

    #[test]
    fn the_library_starts_a_new_component_and_opens_it_to_build() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        // a taken name is refused, and stays to be corrected
        h.state_mut().new_component_name = "drive_unit".into();
        h.step();
        h.get_by_label("New").click();
        steps(&mut h, 3);
        assert!(h.state().editor.is_root());
        assert!(h.state().editor.status.contains("already exists"), "{}", h.state().editor.status);
        assert_eq!(h.state().new_component_name, "drive_unit");
        // a name of its own: in the library, empty, and open
        h.state_mut().new_component_name = "Sensor Mast".into();
        h.step();
        h.get_by_label("New").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.editing, "sensor_mast");
        assert!(h.state().editor.children().is_empty());
        assert!(h.state().new_component_name.is_empty());
        assert!(h.query_by_label("Component sensor_mast").is_some(), "the inspector shows its page");
        assert!(h.query_by_label("Contents of sensor_mast").is_some());
        assert!(h.query_by_label("Nothing here yet: add bricks from the library").is_some());
        h.state_mut().search = "sensor_mast".into();
        steps(&mut h, 2);
        assert!(h.query_by_label("0 g Σ · used ×0").is_some(), "listed in the library");
        assert!(h.query_by_label("+ add").is_none(), "not into itself");
        h.state_mut().search.clear();
        steps(&mut h, 2);
        // a brick from the library goes into it
        h.get_all_by_label("+").next().unwrap().click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.children().len(), 1);
        assert!(h.query_by_label("Nothing here yet: add bricks from the library").is_none());
        // the brick just added is selected: its page shows; with nothing selected, the component's
        h.state_mut().editor.selection.clear();
        steps(&mut h, 2);
        // back at the robot, it is added like any other component
        h.get_by_label("Back to the robot").click();
        steps(&mut h, 3);
        assert!(h.state().editor.is_root());
        let n = h.state().editor.children().len();
        h.state_mut().search = "sensor_mast".into();
        steps(&mut h, 2);
        h.get_by_label("+ add").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.children().len(), n + 1);
        assert_eq!(h.state().editor.selected_instances()[0].component.as_deref(), Some("sensor_mast"));
    }

    #[test]
    fn the_inspector_edits_the_selection_and_makes_components() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        // the IMU board: free to turn on the frame (the frame itself is held by its pins)
        let brick = "imu".to_string();
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
        // the IMU board: free to turn and slide on the frame (the frame itself is held by its pins)
        let brick = "imu".to_string();
        h.state_mut().editor.selection = vec![brick.clone()];
        h.step();
        let start = h.state().editor.selected_instances()[0].clone();
        h.key_press(Key::R);
        h.step();
        assert_eq!(
            h.state().editor.selected_instances()[0].rot[2],
            crate::editor::wrap_deg(start.rot[2] + 90.0)
        );
        // along the frame only: a step towards either rail would put the board into it
        h.key_press(Key::ArrowUp);
        h.key_press(Key::ArrowUp);
        h.step();
        let p = h.state().editor.selected_instances()[0].pos;
        assert_eq!(
            (p[0], p[1]),
            (start.pos[0] + 8.0, start.pos[1]),
            "two half-stud nudges: {}",
            h.state().editor.status
        );
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
        assert_eq!(h.state().editor.status, "Select an item to snap");
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
        // ...and so does a shift-drag, for a pointer with no right button
        let target1 = h.state().viewport.camera.target;
        press(&mut h, corner, PointerButton::Primary, Modifiers::SHIFT);
        drag_to(&mut h, corner + egui::vec2(10.0, 10.0), Modifiers::SHIFT);
        assert!(matches!(h.state().drag, Drag::Pan), "a shift-drag on empty space pans");
        drag_to(&mut h, corner + egui::vec2(50.0, 40.0), Modifiers::SHIFT);
        release(&mut h, corner + egui::vec2(50.0, 40.0), PointerButton::Primary);
        assert_ne!(h.state().viewport.camera.target, target1);
        assert!(h.state().editor.selection.is_empty());
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
        // two bricks of their own, well apart, so the brick dragged, lifted, pulled and turned
        // below meets nothing (the example's boards are hemmed in by their neighbours)
        {
            let app = h.state_mut();
            let root = app.editor.doc.robot.root.clone();
            app.editor.doc.components.get_mut(&root).unwrap().children.clear();
            app.editor.doc.robot.roles.clear();
            app.editor.recompute();
            let id = app.editor.ensure_ldraw_part("3001").unwrap();
            app.editor.add_instance(Some(id.clone()), None, [0.0; 3]);
            app.editor.add_instance(Some(id), None, [64.0, 0.0, 0.0]);
            app.editor.selection.clear();
            app.editor.fit_pending = true;
        }
        steps(&mut h, 3);
        // stood back so that a 90-pixel drag is more than half a module in each direction
        h.state_mut().viewport.camera.distance *= 2.5;
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
        assert!(
            (moved[0] / 4.0).fract() == 0.0 && (moved[1] / 4.0).fract() == 0.0,
            "on the half-stud grid: {moved:?}"
        );
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
        h.state_mut().editor.reset_to_example();
        steps(&mut h, 3);
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
        // the sim view is a plan that neither pans nor zooms: a drag on the empty map, a right
        // drag and the wheel leave the camera where the fit put it
        let rect = h.state().view_rect;
        let cam0 = h.state().viewport.camera.clone();
        assert!(cam0.ortho && cam0.pitch == 90.0, "{cam0:?}");
        // away from the chassis, which the run parked near the mat's centre
        let at = rect.center() + egui::vec2(300.0, 0.0);
        press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(10.0, 0.0), Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::None), "empty map: nothing to drag");
        drag_to(&mut h, at + egui::vec2(50.0, 0.0), Modifiers::NONE);
        release(&mut h, at + egui::vec2(50.0, 0.0), PointerButton::Primary);
        press(&mut h, at, PointerButton::Secondary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(10.0, 10.0), Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(40.0, 40.0), Modifiers::NONE);
        release(&mut h, at + egui::vec2(40.0, 40.0), PointerButton::Secondary);
        h.input_mut().events.push(egui::Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point,
            delta: egui::vec2(0.0, 40.0),
            phase: egui::TouchPhase::Move,
            modifiers: Modifiers::NONE,
        });
        steps(&mut h, 2);
        let cam = h.state().viewport.camera.clone();
        assert_eq!(
            (cam.target, cam.distance, cam.yaw, cam.pitch, cam.ortho),
            (cam0.target, cam0.distance, cam0.yaw, cam0.pitch, true)
        );
        // F asks for the fit again, which lands where it was
        h.key_press(Key::F);
        steps(&mut h, 2);
        let fitted = h.state().viewport.camera.clone();
        assert!((fitted.target - cam0.target).length() < 1.0, "{:?}", fitted.target);
        assert!(h.query_by_label("follow the robot").is_none(), "nothing moves the plan view");
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
    fn the_map_tab_moves_duplicates_removes_and_saves_props() {
        let Some(gpu) = gpu() else { return };
        let Some(fake) = fake_server("map") else { return };
        let mut h = harness(&gpu, None);
        h.state_mut().simulate = SimulateTab::new_with_env(Some(fake.python.clone()), fake.env.clone());
        let mdir = std::env::temp_dir().join(format!("ob-harness-map-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&mdir);
        h.state_mut().simulate.markers_dir = mdir.clone();
        h.get_by_label("Map").click();
        let wait_for = |h: &mut Harness<'_, App>, f: &dyn Fn(&App) -> bool| {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
            while !f(h.state()) && std::time::Instant::now() < deadline {
                h.step();
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            let a = h.state();
            let sent: Vec<String> = a.simulate.sent.iter().rev().take(4).map(|c| c["cmd"].to_string()).collect();
            assert!(
                f(a),
                "waited in vain: {} / world {} / props {} / selected {:?} / sent {sent:?} / search {:?} / {:?}",
                a.simulate.status_line(),
                a.simulate.world(),
                a.simulate.scene_props(),
                a.simulate.selected_prop,
                a.map_search,
                a.simulate.log
            );
        };
        wait_for(&mut h, &|a| a.simulate.scene_loaded());
        steps(&mut h, 3);
        assert_eq!(h.state().tab, Tab::Map);
        assert!(h.query_by_label("Props").is_some() && h.query_by_label("Route").is_none());
        let cam = h.state().viewport.camera.clone();
        assert!(!cam.ortho && cam.pitch < 89.0, "the map editor is a 3D view: {cam:?}");
        assert!(h.query_by_label("Fit").is_some() && h.query_by_label("Iso").is_some() && h.query_by_label("Move").is_none());
        // the toolbar is the map's: the assembly's file buttons and its component path stay on
        // the Workbench, the map's name shows instead
        let file_buttons = ["Open…", "Save as…", "Example"].iter().any(|l| h.query_by_label(l).is_some());
        assert!(!file_buttons, "the assembly's file buttons belong to the Workbench");
        let map_label = format!("map: {}", h.state().simulate.world());
        assert!(h.query_by_label(&map_label).is_some(), "{map_label}");
        // the view is the user's: panned aside (the right button), it stays there through the
        // edits below, each of which rebuilds the map
        let rect = h.state().view_rect;
        let empty = rect.left_top() + egui::vec2(30.0, 30.0);
        press(&mut h, empty, PointerButton::Secondary, Modifiers::NONE);
        drag_to(&mut h, empty + egui::vec2(10.0, 10.0), Modifiers::NONE);
        drag_to(&mut h, empty + egui::vec2(40.0, 30.0), Modifiers::NONE);
        release(&mut h, empty + egui::vec2(40.0, 30.0), PointerButton::Secondary);
        steps(&mut h, 2);
        let panned = h.state().viewport.camera.target;
        assert_ne!(panned, cam.target, "the pan moved the view's centre");
        // a point on a prop's top face, on the screen
        let top = |h: &Harness<'_, App>, x: f64, y: f64| on_screen(h.state(), Vec3::new(x as f32, y as f32, 20.0));
        // the stand-in's prop stands at (300, 200) mm, 20 mm tall: under the pointer it lights, a
        // drag moves it and the server hears the pose, the point taken hold of staying under the
        // pointer
        let at = top(&h, 300.0, 200.0);
        h.input_mut().events.push(Event::PointerMoved(at));
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.hover_prop, Some(0));
        press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(10.0, 0.0), Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::Prop { i: 0, .. }), "a prop drag");
        drag_to(&mut h, at + egui::vec2(60.0, 0.0), Modifiers::NONE);
        release(&mut h, at + egui::vec2(60.0, 0.0), PointerButton::Primary);
        steps(&mut h, 2);
        let moved = h
            .state()
            .simulate
            .sent
            .iter()
            .rfind(|c| c["cmd"] == "move")
            .cloned()
            .expect("a move");
        assert_eq!(moved["name"], "clef");
        let dx = moved["x_mm"].as_f64().unwrap() - 300.0;
        let dy = moved["y_mm"].as_f64().unwrap() - 200.0;
        let far = (dx * dx + dy * dy).sqrt();
        assert!(far > 20.0 && far < 1000.0, "moved ({dx}, {dy}) mm for 60 px");
        assert_eq!(h.state().simulate.selected_prop.as_deref(), Some("clef"));
        wait_for(&mut h, &|a| {
            a.simulate.prop_pose(0).map(|p| (p.x_mm - 300.0 - dx).abs() < 1.0).unwrap_or(false)
        });
        // shift-drag turns it
        let at = top(&h, 300.0 + dx, 200.0 + dy);
        press(&mut h, at, PointerButton::Primary, Modifiers::SHIFT);
        drag_to(&mut h, at + egui::vec2(10.0, 0.0), Modifiers::SHIFT);
        drag_to(&mut h, at + egui::vec2(100.0, 0.0), Modifiers::SHIFT);
        release(&mut h, at + egui::vec2(100.0, 0.0), PointerButton::Primary);
        steps(&mut h, 2);
        let turned = h.state().simulate.sent.iter().rfind(|c| c["cmd"] == "move").cloned().unwrap();
        assert!(turned["yaw_deg"].as_f64().unwrap().abs() > 5.0, "{turned}");
        // R turns it 90° more, the button another 90° — each from the pose the server reports,
        // so each waits for the server to have heard the last turn
        let y0 = turned["yaw_deg"].as_f64().unwrap();
        let last_yaw = |h: &Harness<'_, App>| {
            h.state().simulate.sent.iter().rfind(|c| c["cmd"] == "move").unwrap()["yaw_deg"]
                .as_f64()
                .unwrap()
        };
        let heard = |yaw: f64| {
            move |a: &App| {
                a.simulate
                    .prop_pose(0)
                    .map(|p| crate::route::wrap_deg(p.yaw_deg - yaw).abs() < 0.2)
                    .unwrap_or(false)
            }
        };
        wait_for(&mut h, &heard(y0));
        h.key_press(Key::R);
        steps(&mut h, 2);
        let y_r = last_yaw(&h);
        assert!(crate::route::wrap_deg(y_r - y0 - 90.0).abs() < 0.2, "R: {y_r} from {y0}");
        wait_for(&mut h, &heard(y_r));
        h.get_by_label("Turn 90°").click();
        steps(&mut h, 2);
        let y_b = last_yaw(&h);
        assert!(crate::route::wrap_deg(y_b - y0 - 180.0).abs() < 0.2, "the button: {y_b} from {y0}");
        // ⌘D duplicates (the newest prop selected once built), Del removes it
        h.key_press_modifiers(Modifiers::COMMAND, Key::D);
        wait_for(&mut h, &|a| a.simulate.prop_name(1).is_some());
        assert_eq!(h.state().simulate.selected_prop.as_deref(), Some("clef_2"));
        h.key_press(Key::Delete);
        wait_for(&mut h, &|a| a.simulate.prop_name(1).is_none());
        assert!(h.state().simulate.selected_prop.is_none());
        steps(&mut h, 2);
        let target = h.state().viewport.camera.target;
        assert_eq!(target, panned, "the edits' reloads left the view");
        // the panel: a row selects, the buttons duplicate and remove, Save writes a map of the
        // user's own that the tab then shows, listed as theirs, its markers carried over
        h.get_by_label("clef · clef").click();
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.selected_prop.as_deref(), Some("clef"));
        h.get_by_label("Duplicate").click();
        wait_for(&mut h, &|a| a.simulate.prop_name(1).is_some());
        h.get_by_label("Remove").click();
        wait_for(&mut h, &|a| a.simulate.prop_name(1).is_none());
        h.state_mut().simulate.markers.markers.push(crate::markers::Marker {
            name: "corner".into(),
            at: [1.0, 2.0],
        });
        h.state_mut().simulate.save_name = "Harness Map".into();
        steps(&mut h, 2);
        h.get_by_label("Save map").click();
        wait_for(&mut h, &|a| a.simulate.world() == "harness-map" && a.simulate.scene_loaded());
        assert!(crate::markers::Markers::file(&mdir, "harness-map").exists());
        steps(&mut h, 2);
        assert_eq!(h.state().viewport.camera.target, panned, "a save keeps the view");
        assert!(h.query_by_label("map: harness-map").is_some(), "the toolbar names the map saved");
        assert!(h.state().simulate.worlds().iter().any(|w| w.alias == "harness-map" && w.user));
        // a brick from the library: found by number, put on the map as a document, drawn as its
        // mesh (the server sends the prop's bricks), selected once built
        let num = h.state().editor.bundle.parts.keys().next().unwrap().clone();
        let before = h.state().simulate.scene_props();
        h.state_mut().map_search = num.clone();
        steps(&mut h, 2);
        h.get_by_label("+ to map").click();
        wait_for(&mut h, &|a| a.simulate.scene_props() == before + 1);
        steps(&mut h, 2);
        let added = h.state().simulate.selected_prop.clone().expect("the new prop is selected");
        let i = h.state().simulate.selected_prop_index().unwrap();
        assert!(
            !h.state().simulate.prop_bricks(i).is_empty(),
            "a document's prop carries its bricks"
        );
        let body = h.state().simulate.prop_body(i).unwrap();
        assert!(h.state().simulate.item_bodies.contains(&body), "its brick is drawn");
        // stuck to the map: the checkbox asks the server, the row says so
        h.get_by_label("stuck to the map").click();
        wait_for(&mut h, &|a| {
            a.simulate
                .selected_prop_index()
                .map(|i| a.simulate.prop_is_fixed(i))
                .unwrap_or(false)
        });
        steps(&mut h, 2);
        let kind = h.state().simulate.prop_kind(i).unwrap();
        assert!(h.query_by_label(&format!("{added} · {kind} · stuck")).is_some());
        // a component of the Workbench's build, as a document of its own
        let root = h.state().editor.doc.robot.root.clone();
        assert!(h.query_by_label("Add to the map").is_some());
        let sub = assembly::subset(&h.state().editor.doc, &root).unwrap();
        h.state_mut().simulate.add_model(&root, &sub);
        wait_for(&mut h, &|a| a.simulate.scene_props() == before + 2);
        // any saved build: opened for the map, it is the build listed, its components offered
        // (the whole build first) and one goes on the map, named after the file
        let gate = fake.dir.join("gate.assembly.json");
        let brick = assembly::brick_document(&h.state().editor.bundle, &num).unwrap();
        std::fs::write(&gate, serde_json::to_string(&brick).unwrap()).unwrap();
        h.state_mut().open_build(gate.clone()).expect("a saved build opens");
        steps(&mut h, 2);
        assert_eq!(h.state().map_build, 1, "the build just opened is the one listed");
        assert!(h.query_by_label("the whole build · 1 brick").is_some());
        h.get_by_label("+ brick").click();
        wait_for(&mut h, &|a| a.simulate.scene_props() == before + 3);
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.selected_prop.as_deref(), Some("gate"));
        assert!(h.state_mut().open_build(fake.dir.join("nowhere.assembly.json")).is_err());
        std::fs::write(&gate, "{}").unwrap();
        let again = h.state_mut().open_build(gate.clone());
        assert!(again.is_err(), "not an assembly: {again:?}");
        assert_eq!(h.state().map_builds.len(), 1, "a refused file is not listed");
        // the rows choose the build listed: the Workbench's, then the saved one
        let open_row = format!("{} · open in the Workbench", h.state().title_name());
        h.get_by_label(&open_row).click();
        steps(&mut h, 2);
        assert_eq!(h.state().map_build, 0);
        assert!(h.query_by_label(&format!("+ {root}")).is_some(), "the Workbench's build is listed");
        h.get_by_label("gate.assembly.json").click();
        steps(&mut h, 2);
        assert_eq!(h.state().map_build, 1);
        // the same file opened again replaces its entry, with what it holds now
        std::fs::write(&gate, serde_json::to_string(&sub).unwrap()).unwrap();
        h.state_mut().open_build(gate.clone()).expect("the same file opens again");
        steps(&mut h, 2);
        assert_eq!((h.state().map_builds.len(), h.state().map_build), (1, 1));
        assert!(h.query_by_label(&format!("+ {root}")).is_some(), "its components are the new ones");
        // a file of another format is refused, and the panel says so; a build with no bricks
        // says so too
        let mut odd = brick.clone();
        odd.format = "nope".into();
        let odd_path = fake.dir.join("odd.assembly.json");
        std::fs::write(&odd_path, serde_json::to_string(&odd).unwrap()).unwrap();
        h.state_mut().open_build_noted(odd_path);
        steps(&mut h, 2);
        let note = h.state().map_note.clone();
        assert!(note.contains("not an assembly file"), "{note}");
        assert!(h.query_by_label(&note).is_some(), "the refusal shows in the panel");
        let mut bare = brick.clone();
        bare.components.get_mut("brick").unwrap().children.clear();
        let bare_path = fake.dir.join("bare.assembly.json");
        std::fs::write(&bare_path, serde_json::to_string(&bare).unwrap()).unwrap();
        h.state_mut().open_build(bare_path).expect("a build with no bricks opens");
        steps(&mut h, 2);
        assert!(h.state().map_note.is_empty(), "a good open clears the note");
        assert!(h.query_by_label("this build has no bricks yet").is_some());
        // the heading field turns the selected prop: a drag on it
        h.get_by_label("clef · clef").click();
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.selected_prop.as_deref(), Some("clef"));
        let y1 = last_yaw(&h);
        let field = h.get_by_role(egui::accesskit::Role::SpinButton).rect().center();
        press(&mut h, field, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, field + egui::vec2(10.0, 0.0), Modifiers::NONE);
        drag_to(&mut h, field + egui::vec2(60.0, 0.0), Modifiers::NONE);
        release(&mut h, field + egui::vec2(60.0, 0.0), PointerButton::Primary);
        steps(&mut h, 2);
        let y2 = last_yaw(&h);
        assert!((y2 - y1).abs() > 5.0, "the heading field turned it: {y2} from {y1}");
        // the 3D view: a drag on the empty map orbits, a right drag pans, the wheel zooms, Fit
        // frames the map again
        h.key_press(Key::Escape);
        steps(&mut h, 2);
        assert!(h.state().simulate.selected_prop.is_none());
        let rect = h.state().view_rect;
        let cam0 = h.state().viewport.camera.clone();
        let empty = rect.left_top() + egui::vec2(30.0, 30.0);
        press(&mut h, empty, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, empty + egui::vec2(20.0, 0.0), Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::Orbit), "the empty map orbits");
        drag_to(&mut h, empty + egui::vec2(60.0, 0.0), Modifiers::NONE);
        release(&mut h, empty + egui::vec2(60.0, 0.0), PointerButton::Primary);
        assert!((h.state().viewport.camera.yaw - cam0.yaw).abs() > 5.0);
        press(&mut h, empty, PointerButton::Secondary, Modifiers::NONE);
        drag_to(&mut h, empty + egui::vec2(20.0, 20.0), Modifiers::NONE);
        drag_to(&mut h, empty + egui::vec2(60.0, 60.0), Modifiers::NONE);
        release(&mut h, empty + egui::vec2(60.0, 60.0), PointerButton::Secondary);
        assert_ne!(h.state().viewport.camera.target, cam0.target);
        // a shift-drag pans too (a trackpad has no right button), and a pinch zooms
        let t1 = h.state().viewport.camera.target;
        press(&mut h, empty, PointerButton::Primary, Modifiers::SHIFT);
        drag_to(&mut h, empty + egui::vec2(20.0, 20.0), Modifiers::SHIFT);
        assert!(matches!(h.state().drag, Drag::Pan), "a shift-drag on the map pans");
        drag_to(&mut h, empty + egui::vec2(60.0, 60.0), Modifiers::SHIFT);
        release(&mut h, empty + egui::vec2(60.0, 60.0), PointerButton::Primary);
        assert_ne!(h.state().viewport.camera.target, t1);
        let d1 = h.state().viewport.camera.distance;
        h.input_mut().events.push(Event::PointerMoved(rect.center()));
        h.input_mut().events.push(Event::Zoom(1.25));
        steps(&mut h, 2);
        assert!(h.state().viewport.camera.distance < d1, "a pinch zooms in");
        h.input_mut().events.push(Event::PointerMoved(rect.center()));
        h.input_mut().events.push(Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point,
            delta: egui::vec2(0.0, 40.0),
            phase: egui::TouchPhase::Move,
            modifiers: Modifiers::NONE,
        });
        steps(&mut h, 2);
        assert!(h.state().viewport.camera.distance < cam0.distance, "the wheel zooms in");
        h.get_by_label("Fit").click();
        steps(&mut h, 3);
        assert!(
            (h.state().viewport.camera.target.x).abs() < 1.0,
            "{:?}",
            h.state().viewport.camera.target
        );
        // the Simulate tab keeps its own, fixed plan camera
        h.get_by_label("Simulate").click();
        steps(&mut h, 2);
        assert!(h.state().viewport.camera.ortho);
        h.get_by_label("Map").click();
        steps(&mut h, 2);
        assert!(!h.state().viewport.camera.ortho);
        let _ = std::fs::remove_dir_all(&mdir);
    }

    #[test]
    fn the_route_panel_places_actions_by_clicks_and_edits_them_on_the_map() {
        let Some(gpu) = gpu() else { return };
        let Some(fake) = fake_server("route") else { return };
        let mut h = harness(&gpu, None);
        h.state_mut().simulate = SimulateTab::new_with_env(Some(fake.python.clone()), fake.env.clone());
        let mdir = std::env::temp_dir().join(format!("ob-harness-markers-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&mdir);
        h.state_mut().simulate.markers_dir = mdir.clone();
        h.get_by_label("Simulate").click();
        let wait_for = |h: &mut Harness<'_, App>, f: &dyn Fn(&App) -> bool| {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
            while !f(h.state()) && std::time::Instant::now() < deadline {
                h.step();
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            assert!(
                f(h.state()),
                "waited in vain: {} / {:?}",
                h.state().simulate.status(),
                h.state().simulate.log
            );
        };
        let click = |h: &mut Harness<'_, App>, at: Pos2| {
            press(h, at, PointerButton::Primary, Modifiers::NONE);
            release(h, at, PointerButton::Primary);
            steps(h, 2);
        };
        wait_for(&mut h, &|a| a.simulate.scene_loaded());
        steps(&mut h, 3);
        assert!(h.query_by_label("Route").is_some());
        // the chassis stands at its spawn; a drag on it puts its axle centre under the pointer, and the server hears
        wait_for(&mut h, &|a| a.simulate.chassis_pose().map(|p| p.x_mm < -500.0).unwrap_or(false));
        let p0 = h.state().simulate.chassis_pose().unwrap();
        assert!((p0.x_mm + 547.0).abs() < 1e-3 && (p0.yaw_deg - 90.0).abs() < 1e-3, "{p0:?}");
        let rect = h.state().view_rect;
        let at = on_screen(h.state(), Vec3::new(p0.x_mm as f32, p0.y_mm as f32, 50.0));
        assert!(rect.contains(at), "{at:?} in {rect:?}");
        press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, at + egui::vec2(8.0, 0.0), Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::Chassis { .. }), "the chassis was grabbed");
        let target = at + egui::vec2(120.0, 0.0);
        drag_to(&mut h, target, Modifiers::NONE);
        let ghost = h.state().simulate.ghost.expect("a ghost follows the pointer");
        assert!(ghost.x_mm > p0.x_mm + 50.0, "{ghost:?}");
        release(&mut h, target, PointerButton::Primary);
        let placed = h.state().simulate.route.start;
        assert!(placed.x_mm > p0.x_mm + 50.0, "{placed:?}");
        assert!(h.state().simulate.ghost.is_none());
        assert_eq!(h.state().simulate.sent.last().unwrap()["cmd"], "place");
        wait_for(&mut h, &|a| {
            a.simulate
                .chassis_pose()
                .map(|p| (p.x_mm - placed.x_mm).abs() < 1e-3)
                .unwrap_or(false)
        });
        steps(&mut h, 2);
        let at = on_screen(h.state(), Vec3::new(placed.x_mm as f32, placed.y_mm as f32, 50.0));
        press(&mut h, at, PointerButton::Primary, Modifiers::SHIFT);
        drag_to(&mut h, at + egui::vec2(8.0, 0.0), Modifiers::SHIFT);
        drag_to(&mut h, at + egui::vec2(100.0, 0.0), Modifiers::SHIFT);
        release(&mut h, at + egui::vec2(100.0, 0.0), PointerButton::Primary);
        let start = h.state().simulate.route.start;
        assert!(
            (start.x_mm - placed.x_mm).abs() < 1e-6 && start.yaw_deg < placed.yaw_deg,
            "{start:?} from {placed:?}"
        );
        wait_for(&mut h, &|a| {
            a.simulate
                .chassis_pose()
                .map(|p| (p.yaw_deg - start.yaw_deg).abs() < 1e-3)
                .unwrap_or(false)
        });
        let (hx, hy) = start.heading();
        let map = |h: &Harness<'_, App>, p: [f64; 2]| on_screen(h.state(), Vec3::new(p[0] as f32, p[1] as f32, 0.0));
        // the straight tool: a click near the chassis snaps to its start, a click 250 mm ahead opens the popup
        h.get_by_label("→ Straight").click();
        steps(&mut h, 2);
        assert!(h.state().simulate.placing.as_ref().is_some_and(|p| p.kind == "straight"));
        let at_ = map(&h, [start.x_mm + 5.0, start.y_mm - 5.0]);
        click(&mut h, at_);
        assert_eq!(
            h.state().simulate.placing.as_ref().unwrap().points,
            vec![start.point()],
            "snapped to the start"
        );
        let ahead = [start.x_mm + hx * 250.0, start.y_mm + hy * 250.0];
        // moving the pointer stretches the line from the start to the pointer, its length beside it
        let midway = map(&h, [start.x_mm + hx * 120.0, start.y_mm + hy * 120.0]);
        h.input_mut().events.push(Event::PointerMoved(midway));
        steps(&mut h, 2);
        let hover = h.state().simulate.hover.expect("the pointer is on the map");
        assert!(
            (hover[0] - start.x_mm - hx * 120.0).abs() < 4.0 && (hover[1] - start.y_mm - hy * 120.0).abs() < 4.0,
            "{hover:?}"
        );
        let labels: Vec<String> = {
            let app = h.state();
            let r = app.view_rect;
            app.simulate
                .route_labels(&app.viewport.camera, r.width(), r.height())
                .into_iter()
                .map(|l| l.1)
                .collect()
        };
        assert!(
            labels.iter().any(|l| l
                .strip_suffix(" mm")
                .and_then(|v| v.parse::<f64>().ok())
                .is_some_and(|v| (v - 120.0).abs() < 5.0)),
            "{labels:?}"
        );
        let at_ = map(&h, ahead);
        click(&mut h, at_);
        assert!(h.state().simulate.draft.is_some(), "the popup is up");
        assert!(h.state().simulate.click_marker().is_some(), "the click is marked");
        steps(&mut h, 2);
        assert!(h.query_by_label("New straight").is_some());
        assert!(h.state().simulate.route.actions.is_empty());
        h.get_by_label("Add").click();
        steps(&mut h, 2);
        let actions = h.state().simulate.route.actions.clone();
        assert_eq!(actions.len(), 1, "{actions:?}");
        let end0 = actions[0].action.end();
        assert!(
            (end0[0] - ahead[0]).abs() < 4.0 && (end0[1] - ahead[1]).abs() < 4.0,
            "{end0:?} vs {ahead:?}"
        );
        assert_eq!(h.state().simulate.selected, Some(0));
        let labels: Vec<String> = {
            let app = h.state();
            let r = app.view_rect;
            app.simulate
                .route_labels(&app.viewport.camera, r.width(), r.height())
                .into_iter()
                .map(|l| l.1)
                .collect()
        };
        assert_eq!(labels[0], "1");
        assert!(labels[1].ends_with(" mm"), "the placed line carries its length: {labels:?}");
        // a curve from that end (snapped) to a point ahead and to the right, then a stop where it ends
        h.get_by_label("⌒ Curve").click();
        steps(&mut h, 2);
        let at_ = map(&h, [end0[0] + 4.0, end0[1] + 4.0]);
        click(&mut h, at_);
        let aside = [end0[0] + hx * 150.0 + hy * 150.0, end0[1] + hy * 150.0 - hx * 150.0];
        let at_ = map(&h, aside);
        click(&mut h, at_);
        steps(&mut h, 2);
        // two clicks: the one arc tangent to the straight through that point, a quarter circle
        assert!(h.query_by_label("New curve").is_some(), "a chained curve takes two clicks");
        h.get_by_label("Add").click();
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.route.actions.len(), 2);
        assert_eq!(
            h.state().simulate.route.actions[1].action.start(),
            end0,
            "the curve starts where the straight ends"
        );
        let end1 = h.state().simulate.route.actions[1].action.end();
        h.get_by_label("■ Stop").click();
        steps(&mut h, 2);
        let at_ = map(&h, [end1[0] + 3.0, end1[1] - 3.0]);
        click(&mut h, at_);
        steps(&mut h, 2);
        h.get_by_label("Add").click();
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.route.actions.len(), 3);
        assert_eq!(h.state().simulate.route.actions[2].action.start(), end1);
        // a click on the straight's path selects it; one out on the mat clears the selection; a click
        // on the stop's marker selects the stop rather than the curve ending there
        let mid = [(start.x_mm + end0[0]) / 2.0, (start.y_mm + end0[1]) / 2.0];
        let at_ = map(&h, mid);
        click(&mut h, at_);
        assert_eq!(h.state().simulate.selected, Some(0));
        let at_ = map(&h, [start.x_mm - hy * 400.0, start.y_mm + hx * 400.0]);
        click(&mut h, at_);
        assert_eq!(h.state().simulate.selected, None);
        let at_ = map(&h, end1);
        click(&mut h, at_);
        assert_eq!(h.state().simulate.selected, Some(2));
        // ⌘C / ⌘V: the copy lands beside the original, after it, selected
        h.input_mut().events.push(Event::Copy);
        h.step();
        let text = h.state().simulate.copy_selected().unwrap();
        h.input_mut().events.push(Event::Paste(text));
        h.step();
        assert_eq!(h.state().simulate.route.actions.len(), 4);
        assert_eq!(h.state().simulate.selected, Some(3));
        let copy_at = h.state().simulate.route.actions[3].action.start();
        assert!(
            (copy_at[0] - end1[0] - 40.0).abs() < 0.1 && (copy_at[1] - end1[1] - 40.0).abs() < 0.1,
            "{copy_at:?}"
        );
        // ⌘L locks it: its handle will not drag; ⌘⇧L unlocks
        h.key_press_modifiers(Modifiers::COMMAND, Key::L);
        h.step();
        assert!(h.state().simulate.is_locked(3));
        let hs = map(&h, copy_at);
        press(&mut h, hs, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, hs + egui::vec2(12.0, 0.0), Modifiers::NONE);
        assert_ne!(
            h.state().simulate.dragging_index(),
            Some(3),
            "a locked action's handle is not grabbed"
        );
        release(&mut h, hs + egui::vec2(12.0, 0.0), PointerButton::Primary);
        assert_eq!(h.state().simulate.route.actions[3].action.start(), copy_at);
        assert!(h.state().simulate.is_locked(3));
        h.state_mut().simulate.selected = Some(3);
        h.key_press_modifiers(Modifiers::COMMAND | Modifiers::SHIFT, Key::L);
        h.step();
        assert!(!h.state().simulate.is_locked(3));
        // its handle drags 100 mm along the heading, the ghost showing where it ends; ⌘Z undoes it
        let hs = map(&h, copy_at);
        let further = map(&h, [copy_at[0] + hx * 100.0, copy_at[1] + hy * 100.0]);
        press(&mut h, hs, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, hs + (further - hs).normalized() * 12.0, Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::RouteHandle), "the handle was grabbed");
        assert_eq!(h.state().simulate.dragging_index(), Some(3));
        assert!(h.state().simulate.ghost.is_some(), "the ghost shows where the action ends");
        drag_to(&mut h, further, Modifiers::NONE);
        release(&mut h, further, PointerButton::Primary);
        let moved = h.state().simulate.route.actions[3].action.start();
        assert!(
            (moved[0] - copy_at[0] - hx * 100.0).abs() < 4.0 && (moved[1] - copy_at[1] - hy * 100.0).abs() < 4.0,
            "{moved:?} from {copy_at:?}"
        );
        assert!(h.state().simulate.ghost.is_none());
        h.key_press_modifiers(Modifiers::COMMAND, Key::Z);
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.route.actions[3].action.start(), copy_at, "undone");
        // Delete removes the selected copy; Escape drops an armed tool
        h.state_mut().simulate.selected = Some(3);
        h.key_press(Key::Delete);
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.route.actions.len(), 3);
        h.get_by_label("↻ Turn").click();
        h.step();
        assert!(h.state().simulate.placing.is_some());
        h.key_press(Key::Escape);
        steps(&mut h, 2);
        assert!(h.state().simulate.placing.is_none());
        // a marker: the tool, a click on the mat, a name in the popup; it is kept with the map,
        // route clicks snap to it, and Delete removes it
        h.get_by_label("◉ Marker").click();
        steps(&mut h, 2);
        let spot = [start.x_mm - hy * 300.0, start.y_mm + hx * 300.0];
        let at_ = map(&h, spot);
        click(&mut h, at_);
        assert!(h.state().simulate.marker_draft.is_some(), "the name popup is up");
        steps(&mut h, 2);
        assert!(h.query_by_label("New marker").is_some());
        h.get_by_label("Add").click();
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.markers.markers.len(), 1);
        let placed_at = h.state().simulate.markers.markers[0].at;
        assert!(
            (placed_at[0] - spot[0]).abs() < 4.0 && (placed_at[1] - spot[1]).abs() < 4.0,
            "{placed_at:?}"
        );
        assert_eq!(h.state().simulate.markers.markers[0].name, "M1");
        assert_eq!(h.state().simulate.selected_marker, Some(0));
        assert!(crate::markers::Markers::file(&mdir, "practice-line").exists());
        // dragging the flag moves the marker, and the file follows
        let hs = map(&h, placed_at);
        let there = map(&h, [placed_at[0] + hx * 80.0, placed_at[1] + hy * 80.0]);
        press(&mut h, hs, PointerButton::Primary, Modifiers::NONE);
        drag_to(&mut h, hs + (there - hs).normalized() * 12.0, Modifiers::NONE);
        assert!(matches!(h.state().drag, Drag::Marker), "the marker was grabbed");
        drag_to(&mut h, there, Modifiers::NONE);
        release(&mut h, there, PointerButton::Primary);
        let moved = h.state().simulate.markers.markers[0].at;
        assert!(
            (moved[0] - placed_at[0] - hx * 80.0).abs() < 4.0 && (moved[1] - placed_at[1] - hy * 80.0).abs() < 4.0,
            "{moved:?}"
        );
        assert_eq!(crate::markers::Markers::load(&mdir, "practice-line").unwrap().markers[0].at, moved);
        let placed_at = moved;
        // a second marker's popup, cancelled; a third, dropped with Escape
        h.get_by_label("◉ Marker").click();
        steps(&mut h, 2);
        let at_ = map(&h, [placed_at[0] + hx * 200.0, placed_at[1] + hy * 200.0]);
        click(&mut h, at_);
        steps(&mut h, 2);
        h.get_by_label("Cancel").click();
        steps(&mut h, 2);
        assert!(h.state().simulate.marker_draft.is_none());
        assert_eq!(h.state().simulate.markers.markers.len(), 1);
        h.get_by_label("◉ Marker").click();
        steps(&mut h, 2);
        click(&mut h, at_);
        assert!(h.state().simulate.marker_draft.is_some());
        h.key_press(Key::Escape);
        steps(&mut h, 2);
        assert!(h.state().simulate.marker_draft.is_none());
        assert_eq!(h.state().simulate.markers.markers.len(), 1);
        h.get_by_label("■ Stop").click();
        steps(&mut h, 2);
        let at_ = map(&h, [placed_at[0] + 6.0, placed_at[1] - 6.0]);
        click(&mut h, at_);
        assert_eq!(
            h.state().simulate.draft.as_ref().map(|d| d.action.start()),
            Some(placed_at),
            "a route click near the marker snaps to it"
        );
        h.key_press(Key::Escape);
        steps(&mut h, 2);
        assert!(h.state().simulate.draft.is_none());
        // an action's popup can be cancelled too
        h.get_by_label("■ Stop").click();
        steps(&mut h, 2);
        let at_ = map(&h, [placed_at[0] - hx * 150.0, placed_at[1] - hy * 150.0]);
        click(&mut h, at_);
        steps(&mut h, 2);
        assert!(h.query_by_label("New stop").is_some());
        h.get_by_label("Cancel").click();
        steps(&mut h, 2);
        assert!(h.state().simulate.draft.is_none());
        assert_eq!(h.state().simulate.route.actions.len(), 3);
        h.state_mut().simulate.selected_marker = Some(0);
        h.key_press(Key::Delete);
        steps(&mut h, 2);
        assert!(h.state().simulate.markers.markers.is_empty());
        let _ = std::fs::remove_dir_all(&mdir);
        // a path's colour: chosen, then back to the kind's default with one click
        h.state_mut().simulate.selected = Some(0);
        h.state_mut().simulate.route.actions[0].color = Some([255, 0, 0]);
        steps(&mut h, 2);
        h.get_by_label("default").click();
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.route.actions[0].color, None);
        // definitions and the program
        h.get_by_label("Definitions (what custom actions call)").click();
        steps(&mut h, 2);
        h.state_mut().simulate.route.prelude = "def line_follow():\n    pass\n".into();
        h.get_by_label("show the program").click();
        steps(&mut h, 2);
        assert!(h.state().simulate.route_program().unwrap().contains("def line_follow():"));
        // the run
        h.get_by_label("▶ Run route").click();
        wait_for(&mut h, &|a| a.simulate.status() == "running");
        assert!(
            h.state().simulate.log.iter().any(|(_, t)| t.contains("openbricks-route-")),
            "{:?}",
            h.state().simulate.log
        );
        h.get_by_label("⏹ Stop").click();
        wait_for(&mut h, &|a| a.simulate.status() == "stopped");
        h.get_by_label("Clear").click();
        steps(&mut h, 2);
        assert!(h.state().simulate.route.actions.is_empty());
        h.get_by_label("Undo").click();
        steps(&mut h, 2);
        assert_eq!(h.state().simulate.route.actions.len(), 3);
        h.state_mut().simulate.shutdown();
        let _ = std::fs::remove_dir_all(&fake.dir);
    }

    #[test]
    fn shift_frees_a_straight_from_the_heading_it_nears() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        h.get_by_label("Simulate").click();
        steps(&mut h, 2);
        assert!(!h.state().simulate.free);
        h.input_mut().modifiers = Modifiers::SHIFT;
        steps(&mut h, 2);
        assert!(h.state().simulate.free, "shift held: any angle");
        h.input_mut().modifiers = Modifiers::NONE;
        steps(&mut h, 2);
        assert!(!h.state().simulate.free);
    }

    #[test]
    fn drafts_of_the_build_and_the_route_are_kept_and_come_back() {
        use std::time::{Duration, Instant};
        let Some(gpu) = gpu() else { return };
        let dir = std::env::temp_dir().join(format!("ob-app-drafts-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut app = App::with_gpu(&gpu, real_bundle(), None, None);
        app.drafts_in(dir.clone());
        let t0 = Instant::now();
        app.autosave(t0 + Duration::from_secs(5), 5_000);
        assert!(crate::drafts::take(&dir, crate::drafts::BUILD).is_none() && crate::drafts::take(&dir, crate::drafts::ROUTE).is_none());
        // a change to each (the IMU slides along the frame); a moment later both drafts are there
        app.editor.selection = vec!["imu".into()];
        app.editor.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(app.editor.selected_instances()[0].pos[0], 38.0, "{}", app.editor.status);
        app.simulate.place_chassis(crate::route::Pose2::at([123.0, 45.0], 90.0));
        app.autosave(t0 + Duration::from_secs(6), 6_000);
        app.autosave(t0 + Duration::from_secs(9), 9_000);
        assert!(crate::drafts::take(&dir, crate::drafts::BUILD).is_some());
        let (route_text, note) = crate::drafts::take(&dir, crate::drafts::ROUTE).expect("the route's draft");
        assert!(route_text.contains("123") && note.kept_ms == 9_000);
        // a fresh app restores both, and says so
        let mut next = App::with_gpu(&gpu, real_bundle(), None, None);
        next.drafts_in(dir.clone());
        next.restore_drafts(9_000 + 3_600_000);
        assert!(next.editor.dirty);
        assert!(
            next.editor.status.starts_with("Restored the unsaved draft kept 1 hour ago"),
            "{}",
            next.editor.status
        );
        assert_eq!(next.simulate.route.start, crate::route::Pose2::at([123.0, 45.0], 90.0));
        assert_eq!(next.simulate.message, "restored the unsaved route draft kept 1 hour ago");
        // saving the route drops its draft; an unreadable route draft is said
        let file = dir.join("r.route.json");
        next.simulate.save_route(file.clone());
        assert!(crate::drafts::take(&dir, crate::drafts::ROUTE).is_none());
        crate::drafts::keep(&dir, crate::drafts::ROUTE, "{", &crate::drafts::Note::default()).unwrap();
        assert!(!next.simulate.restore_draft(0));
        assert!(
            next.simulate.message.starts_with("ignored an unreadable route draft"),
            "{}",
            next.simulate.message
        );
        next.simulate.load_route(file.clone());
        assert!(crate::drafts::take(&dir, crate::drafts::ROUTE).is_none(), "a file took its place");
        // a route that belongs to a file: the draft's note names it and the restore says so
        next.simulate.place_chassis(crate::route::Pose2::at([1.0, 2.0], 0.0));
        next.simulate.autosave(t0 + Duration::from_secs(20), 20_000);
        next.simulate.autosave(t0 + Duration::from_secs(23), 23_000);
        assert_eq!(
            crate::drafts::take(&dir, crate::drafts::ROUTE).unwrap().1.path.as_deref(),
            Some(file.as_path())
        );
        let mut third = App::with_gpu(&gpu, real_bundle(), None, None);
        third.drafts_in(dir.clone());
        assert!(third.simulate.restore_draft(23_000));
        assert_eq!(
            third.simulate.message,
            format!("restored the unsaved route draft kept moments ago of {}", file.display())
        );
        // keeping into a place that cannot be written is said
        std::fs::write(dir.join("blocker"), "").unwrap();
        third.simulate.draft_dir = dir.join("blocker");
        third.simulate.keep_draft(0);
        assert!(
            third.simulate.message.starts_with("could not keep a route draft"),
            "{}",
            third.simulate.message
        );
        let _ = std::fs::remove_dir_all(&dir);
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
    fn bricks_are_placed_in_a_lego_colour_from_the_library_and_recoloured_in_the_inspector() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        let (grey_elements, red_element, grey) = {
            let b = &h.state().editor.bundle;
            let rec = &b.parts["32278"];
            (
                rec.colors[&72].clone(),
                rec.colors[&4][0].clone(),
                rgb_color(b.color_rgb(72).unwrap()),
            )
        };
        // a LEGO element number typed in the search finds its part and picks its colour
        h.state_mut().search = red_element.clone();
        steps(&mut h, 2);
        assert_eq!(h.get_all_by_label("+").count(), 1, "one part answers to element {red_element}");
        assert_eq!(h.state().pick_color.get("32278"), Some(&4));
        // the colour picked in the library is what + places the brick in
        h.state_mut().pick_color.insert("32278".into(), 72);
        h.state_mut().search = "32278".into();
        steps(&mut h, 2);
        let n = h.state().editor.children().len();
        h.get_all_by_label("+").next().unwrap().click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.children().len(), n + 1);
        let inst = h.state().editor.selected_instances()[0].clone();
        assert_eq!(inst.color, Some(72));
        // its page offers the part's colours and names the LEGO element(s) of the part in that colour
        assert!(h.query_by_label("colour").is_some());
        assert!(h.query_by_label(&format!("LEGO element {}", grey_elements.join(", "))).is_some());
        // drawn in dark bluish gray once it is not tinted as the selection
        h.state_mut().editor.selection.clear();
        steps(&mut h, 2);
        assert!(h.state().items.iter().any(|i| i.color == grey), "drawn in the colour");
        // back to the category colour: nothing drawn in it, nothing in the file
        h.state_mut().editor.selection = vec![inst.name.clone()];
        h.step();
        h.state_mut().editor.set_selection_color(None);
        h.state_mut().editor.selection.clear();
        steps(&mut h, 2);
        assert!(h.state().items.iter().all(|i| i.color != grey));
        assert!(!serde_json::to_string(&h.state().editor.doc).unwrap().contains("\"color\""));
        // the combos themselves: the library row's picks the colour + places, the page's recolours.
        // A combo's selected text is its accessible value; the list's entries are labels, and the
        // list scrolls, so entries near its top are the ones a click can reach.
        {
            let rec = &h.state().editor.bundle.parts["32278"];
            assert!(
                rec.colors.contains_key(&0) && rec.colors.contains_key(&1),
                "black and blue beams exist"
            );
        }
        h.state_mut().pick_color.insert("32278".into(), 4);
        h.state_mut().editor.selection.clear();
        steps(&mut h, 2);
        h.get(By::new().value("Red")).click();
        steps(&mut h, 2);
        let at = h.get_by_label("Blue").rect().center();
        press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
        release(&mut h, at, PointerButton::Primary);
        steps(&mut h, 2);
        assert_eq!(h.state().pick_color.get("32278"), Some(&1), "picked in the list");
        h.get_all_by_label("+").next().unwrap().click();
        steps(&mut h, 3);
        let placed = h.state().editor.selected_instances()[0].clone();
        assert_eq!(placed.color, Some(1));
        h.state_mut().pick_color.insert("32278".into(), 4);
        steps(&mut h, 2);
        h.get(By::new().value("Blue")).click();
        steps(&mut h, 2);
        let at = h.get_by_label("Black").rect().center();
        press(&mut h, at, PointerButton::Primary, Modifiers::NONE);
        release(&mut h, at, PointerButton::Primary);
        steps(&mut h, 2);
        assert_eq!(h.state().editor.selected_instances()[0].color, Some(0), "recoloured from its page");
        assert!(h.state().editor.status.ends_with("in Black"), "{}", h.state().editor.status);
        h.state_mut().editor.remove_selection();
        h.state_mut().editor.selection.clear();
        steps(&mut h, 2);
        // several bricks at once are offered the colours they all come in
        let two: Vec<String> = h
            .state()
            .editor
            .children()
            .iter()
            .filter(|c| c.part.as_deref().is_some_and(|p| p.starts_with("lego_")))
            .map(|c| c.name.clone())
            .take(2)
            .collect();
        h.state_mut().editor.selection = two;
        steps(&mut h, 2);
        assert!(h.query_by_label("2 selected").is_some());
        assert!(h.query_by_label("colour").is_some(), "the selection's colour combo");
        h.state_mut().search.clear();
        h.step();
    }

    #[test]
    fn a_component_can_be_saved_as_a_build_and_a_build_imported() {
        let Some(gpu) = gpu() else { return };
        let mut h = harness(&gpu, None);
        steps(&mut h, 2);
        // the Workbench's toolbar imports; the library's component rows save (not the robot's)
        assert!(h.query_by_label("Import…").is_some());
        let non_root = h.state().editor.doc.components.len() - 1;
        assert_eq!(h.get_all_by_label("save").count(), non_root);
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
        steps(&mut h, 2);
        assert!(h.query_by_label("Save as build…").is_some(), "a component's page saves it");
        // the other tabs carry neither
        h.get_by_label("Map").click();
        steps(&mut h, 2);
        assert!(h.query_by_label("Import…").is_none() && h.query_by_label("save").is_none());
        h.get_by_label("Workbench").click();
        steps(&mut h, 2);
        // the buttons at work, the dialogs answered: the page's button writes the component as
        // a build; Import… of that file finds nothing new; Open… opens it; Save as… writes it
        let dir = std::env::temp_dir().join(format!("ob-app-files-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("comp.assembly.json");
        h.state_mut().dialogs = Dialogs::answering(Some(file.clone()));
        h.get_by_label("Save as build…").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.status, format!("Saved {cid} as {}", file.display()));
        assert!(
            !h.state().editor.dirty || h.state().editor.path.is_none(),
            "the build here is untouched"
        );
        h.get_by_label("Import…").click();
        steps(&mut h, 3);
        assert!(h.state().editor.status.starts_with("Nothing new in"), "{}", h.state().editor.status);
        h.get_by_label("Open…").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.doc.robot.root, cid);
        assert_eq!(h.state().editor.path.as_deref(), Some(file.as_path()));
        let other = dir.join("again.assembly.json");
        h.state_mut().dialogs = Dialogs::answering(Some(other.clone()));
        h.get_by_label("Save as…").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.path.as_deref(), Some(other.as_path()));
        // a library row's save button, and Save with a file already: no dialog needed
        h.state_mut().editor.selection.clear();
        h.state_mut().search = cid.clone();
        steps(&mut h, 2);
        h.state_mut().dialogs = Dialogs::answering(Some(dir.join("row.assembly.json")));
        assert!(h.query_by_label("save").is_none(), "the root has no save button");
        h.state_mut().search.clear();
        h.state_mut().editor.reset_to_example();
        steps(&mut h, 2);
        h.get_all_by_label("save").next().unwrap().click();
        steps(&mut h, 3);
        assert!(dir.join("row.assembly.json").exists(), "{}", h.state().editor.status);
        // an STL through the same dialogs; a declined dialog does nothing
        let stl = dir.join("tri.stl");
        std::fs::write(
            &stl,
            "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 10 0 0\nvertex 0 10 0\nendloop\nendfacet\nendsolid t\n",
        )
        .unwrap();
        h.state_mut().dialogs = Dialogs::answering(Some(stl));
        // the button sits below the library's long list: a search that hides the list brings it up
        h.state_mut().search = "zzzz-nothing".into();
        steps(&mut h, 2);
        h.get_by_label("Import STL…").click();
        steps(&mut h, 2);
        assert!(
            h.state().stl.is_some(),
            "the import form opens on the file: {}",
            h.state().editor.status
        );
        h.state_mut().stl = None;
        h.state_mut().stl_prepared = None;
        h.state_mut().dialogs = Dialogs::answering(None);
        let status = h.state().editor.status.clone();
        h.get_by_label("Import…").click();
        h.get_by_label("Import STL…").click();
        h.get_by_label("Save as…").click();
        steps(&mut h, 3);
        assert_eq!(h.state().editor.status, status, "declined: nothing happened");
        assert!(h.state().stl.is_none());
        let _ = std::fs::remove_dir_all(&dir);
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
