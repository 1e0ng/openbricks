//! The Simulate tab: a map, the chassis assembled in the workbench and
//! the program; run, pause, stop; the run shown live from the poses
//! the run server streams.

use crate::assembly::Document;
use crate::bundle::Bundle;
use crate::geometry;
use crate::sim::{Event, Pose, Scene, SimProcess, WorldEntry};
use crate::viewport::{DrawItem, Line, Viewport};
use eframe::egui;
use eframe::egui_wgpu::wgpu;
use glam::{Mat4, Quat, Vec3};
use std::collections::HashMap;
use std::path::PathBuf;

const M_TO_MM: f32 = 1000.0;

pub struct SimulateTab {
    python: Option<String>,
    env: Vec<(String, String)>,
    process: Option<SimProcess>,
    connected: bool,
    worlds: Vec<WorldEntry>,
    world: String,
    chassis: Option<PathBuf>,
    chassis_doc: Option<Document>,
    script: Option<PathBuf>,
    scene: Option<Scene>,
    poses: Vec<Pose>,
    status: String,
    t_ms: u64,
    speed: f64,
    speed_sent: f64,
    error: Option<String>,
    pub log: Vec<(String, String)>,
    message: String,
    textures_loaded: HashMap<String, bool>,
    fit_pending: bool,
    follow: bool,
    pending_load: bool,
    scene_gen: u32,
    /// The default map is loaded once, when the tab first shows.
    auto_loaded: bool,
    /// Every command sent, for tests.
    #[cfg(test)]
    pub sent: Vec<serde_json::Value>,
}

impl SimulateTab {
    pub fn new(python: Option<String>) -> Self {
        Self::new_with_env(python, vec![])
    }

    /// `env` is added to the run server's environment.
    pub fn new_with_env(python: Option<String>, env: Vec<(String, String)>) -> Self {
        SimulateTab {
            python,
            env,
            process: None,
            connected: false,
            worlds: vec![],
            world: "practice-line".into(),
            chassis: None,
            chassis_doc: None,
            script: None,
            scene: None,
            poses: vec![],
            status: "no run server".into(),
            t_ms: 0,
            speed: 1.0,
            speed_sent: 1.0,
            error: None,
            log: vec![],
            message: String::new(),
            textures_loaded: HashMap::new(),
            fit_pending: false,
            follow: false,
            pending_load: false,
            scene_gen: 0,
            auto_loaded: false,
            #[cfg(test)]
            sent: vec![],
        }
    }

    pub fn follows(&self) -> bool {
        self.follow
    }

    /// A chassis chosen: the map is rebuilt with it.
    pub fn set_chassis(&mut self, path: PathBuf, doc: Document) {
        self.chassis = Some(path);
        self.chassis_doc = Some(doc);
        self.reload();
    }

    /// Back to the default chassis.
    pub fn clear_chassis(&mut self) {
        self.chassis = None;
        self.chassis_doc = None;
        self.reload();
    }

    /// The tab is showing: load the default map once so the view is
    /// never empty. Failures (no Python) are reported once.
    pub fn ensure_loaded(&mut self) {
        if self.auto_loaded {
            return;
        }
        self.auto_loaded = true;
        self.load();
    }

    /// A different map or chassis was chosen: show it. A run in
    /// progress is stopped first; the load follows its `stopped` state.
    pub fn reload(&mut self) {
        if matches!(self.status.as_str(), "running" | "paused") {
            self.pending_load = true;
            self.send(serde_json::json!({"cmd": "stop"}));
        } else {
            self.load();
        }
    }

    // --------------------------------------------------------- process

    fn ensure_process(&mut self) -> bool {
        if self.process.is_some() {
            return true;
        }
        let Some(python) = self.python.clone() else {
            self.message = "no Python interpreter: start the sim with `openbricks sim` (or pass --python)".into();
            return false;
        };
        match SimProcess::spawn_with_env(&python, &self.env) {
            Ok(p) => {
                self.process = Some(p);
                self.status = "starting the run server".into();
                self.send(serde_json::json!({"cmd": "worlds"}));
                true
            }
            Err(e) => {
                self.message = e;
                false
            }
        }
    }

    fn send(&mut self, cmd: serde_json::Value) {
        #[cfg(test)]
        self.sent.push(cmd.clone());
        if let Some(p) = self.process.as_mut()
            && let Err(e) = p.send(&cmd)
        {
            self.message = e;
            self.process = None;
            self.connected = false;
        }
    }

    /// Drain the server's events; returns true when something changed.
    pub fn pump(&mut self) -> bool {
        let mut changed = false;
        while let Some(ev) = self.process.as_mut().and_then(|p| p.try_recv()) {
            changed = true;
            self.apply(ev);
        }
        changed
    }

    /// One event from the run server applied to the tab.
    pub fn apply(&mut self, ev: Event) {
        {
            match ev {
                Event::Hello { version } => {
                    self.connected = true;
                    self.status = format!("run server {version} ready");
                }
                Event::Worlds(w) => {
                    if !w.iter().any(|e| e.alias == self.world)
                        && let Some(first) = w.iter().find(|e| e.alias != "empty")
                    {
                        self.world = first.alias.clone();
                    }
                    self.worlds = w;
                    if self.pending_load {
                        self.pending_load = false;
                        self.load();
                    }
                }
                Event::Scene(s) => {
                    self.textures_loaded.clear();
                    self.scene_gen += 1;
                    self.poses = vec![
                        Pose {
                            pos: [0.0; 3],
                            quat: [1.0, 0.0, 0.0, 0.0]
                        };
                        s.bodies.len()
                    ];
                    self.scene = Some(*s);
                    self.fit_pending = true;
                }
                Event::Frame { t_ms, poses } => {
                    self.t_ms = t_ms;
                    if poses.len() == self.poses.len() {
                        self.poses = poses;
                    }
                }
                Event::Log { stream, text } => {
                    self.log.push((stream, text));
                    if self.log.len() > 2000 {
                        self.log.drain(0..500);
                    }
                }
                Event::State {
                    status,
                    t_ms,
                    speed,
                    error,
                } => {
                    self.status = status;
                    self.t_ms = t_ms;
                    self.speed_sent = speed;
                    self.error = error;
                    if self.pending_load && !matches!(self.status.as_str(), "running" | "paused") {
                        self.pending_load = false;
                        self.load();
                    }
                }
                Event::Error(text) => {
                    self.message = text.clone();
                    self.log.push(("server".into(), text));
                }
                Event::Bye => self.server_gone(None),
                Event::Exited(why) => self.server_gone(Some(why)),
            }
        }
    }

    fn server_gone(&mut self, why: Option<String>) {
        if let Some(w) = why {
            self.message = w;
        }
        self.process = None;
        self.connected = false;
        self.status = "run server stopped".into();
    }

    pub fn is_live(&self) -> bool {
        self.process.is_some()
            && matches!(
                self.status.as_str(),
                "running" | "paused" | "loaded" | "finished" | "stopped" | "error"
            )
    }

    fn load(&mut self) {
        if !self.ensure_process() {
            return;
        }
        if !self.connected || self.worlds.is_empty() {
            self.pending_load = true;
            return;
        }
        let mut cmd = serde_json::json!({"cmd": "load", "world": self.world});
        if let Some(c) = &self.chassis {
            cmd["assembly"] = serde_json::Value::String(c.to_string_lossy().to_string());
        }
        self.message.clear();
        self.error = None;
        self.log.clear();
        self.send(cmd);
    }

    fn run(&mut self) {
        let Some(script) = self.script.clone() else {
            self.message = "choose a program first".into();
            return;
        };
        if self.scene.is_none() {
            self.load();
        }
        self.message.clear();
        self.error = None;
        self.send(serde_json::json!({"cmd": "run", "script": script.to_string_lossy()}));
    }

    pub fn shutdown(&mut self) {
        if let Some(mut p) = self.process.take() {
            p.kill();
        }
    }

    // --------------------------------------------------------- drawing

    /// Draw items for the scene as last posed. Meshes are built on the
    /// viewport on first use (bricks from the bundle, geoms tessellated).
    pub fn draw_items(
        &mut self,
        viewport: &mut Viewport,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bundle: &Bundle,
        dark: bool,
    ) -> (Vec<DrawItem>, Vec<Line>) {
        let Some(scene) = self.scene.as_ref() else {
            return (vec![], vec![]);
        };
        let mut items = Vec::new();
        let body_pose = |id: usize| -> Mat4 {
            let p = self.poses.get(id).copied().unwrap_or(Pose {
                pos: [0.0; 3],
                quat: [1.0, 0.0, 0.0, 0.0],
            });
            let q = Quat::from_xyzw(p.quat[1] as f32, p.quat[2] as f32, p.quat[3] as f32, p.quat[0] as f32).normalize();
            Mat4::from_rotation_translation(q, Vec3::new(p.pos[0] as f32, p.pos[1] as f32, p.pos[2] as f32) * M_TO_MM)
        };
        // textures the materials name
        for (tex_name, path) in &scene.textures {
            if self.textures_loaded.contains_key(tex_name) {
                continue;
            }
            let ok = match image::open(path) {
                Ok(img) => {
                    let rgba = img.to_rgba8();
                    let (w, h) = rgba.dimensions();
                    viewport.add_texture(device, queue, &format!("tex:{tex_name}"), w, h, &rgba);
                    true
                }
                Err(e) => {
                    self.log.push(("server".into(), format!("texture {path}: {e}")));
                    false
                }
            };
            self.textures_loaded.insert(tex_name.clone(), ok);
        }
        let chassis_id = scene.body_id("chassis");
        for (gi, g) in scene.geoms.iter().enumerate() {
            if g.name.starts_with("chassis_brick:") {
                continue; // the exact brick is drawn below
            }
            if g.group >= 3 && !g.name.starts_with("chassis_brick:") {
                continue; // MuJoCo's convention: groups 3+ are helper geometry
            }
            let key = format!("geom:{}:{gi}", self.scene_gen);
            let (mesh, texture, mut color) = match g.kind.as_str() {
                "plane" => {
                    let (hx, hy) = if g.size[0] > 0.0 && g.size[1] > 0.0 {
                        (g.size[0] as f32, g.size[1] as f32)
                    } else {
                        (5.0, 5.0)
                    };
                    let mat = g.material.as_ref().and_then(|m| scene.materials.get(m));
                    let rep = mat.map(|m| [m.texrepeat[0] as f32, m.texrepeat[1] as f32]).unwrap_or([1.0, 1.0]);
                    let tex = mat
                        .and_then(|m| m.texture.clone())
                        .filter(|t| self.textures_loaded.get(t).copied().unwrap_or(false))
                        .map(|t| format!("tex:{t}"));
                    (geometry::plane_mesh(hx * M_TO_MM, hy * M_TO_MM, rep), tex, [1.0, 1.0, 1.0, 1.0])
                }
                "box" => (
                    geometry::box_mesh(
                        [
                            2.0 * g.size[0] as f32 * M_TO_MM,
                            2.0 * g.size[1] as f32 * M_TO_MM,
                            2.0 * g.size[2] as f32 * M_TO_MM,
                        ],
                        [0.0; 3],
                    ),
                    None,
                    [1.0; 4],
                ),
                "sphere" => (geometry::sphere_mesh(g.size[0] as f32 * M_TO_MM, [0.0; 3], 24, 16), None, [1.0; 4]),
                "cylinder" => (
                    geometry::cylinder_mesh(g.size[0] as f32 * M_TO_MM, 2.0 * g.size[1] as f32 * M_TO_MM, "z", [0.0; 3], 32),
                    None,
                    [1.0; 4],
                ),
                "capsule" => (
                    geometry::capsule_mesh(g.size[0] as f32 * M_TO_MM, g.size[1] as f32 * M_TO_MM, 24),
                    None,
                    [1.0; 4],
                ),
                "ellipsoid" => {
                    let mut m = geometry::sphere_mesh(1.0, [0.0; 3], 24, 16);
                    for p in m.positions.iter_mut() {
                        p[0] *= g.size[0] as f32 * M_TO_MM;
                        p[1] *= g.size[1] as f32 * M_TO_MM;
                        p[2] *= g.size[2] as f32 * M_TO_MM;
                    }
                    (m, None, [1.0; 4])
                }
                _ => continue, // meshes from files: not drawn yet
            };
            if !viewport.has_mesh(&key) {
                viewport.add_mesh(device, &key, &mesh);
            }
            let rgba = match g.material.as_ref().and_then(|m| scene.materials.get(m)) {
                Some(m) if g.rgba == [0.5, 0.5, 0.5, 1.0] => m.rgba,
                _ => g.rgba,
            };
            if texture.is_none() {
                color = [
                    srgb_to_linear(rgba[0] as f32),
                    srgb_to_linear(rgba[1] as f32),
                    srgb_to_linear(rgba[2] as f32),
                    rgba[3] as f32,
                ];
            }
            let local = Mat4::from_rotation_translation(
                Quat::from_xyzw(g.quat[1] as f32, g.quat[2] as f32, g.quat[3] as f32, g.quat[0] as f32).normalize(),
                Vec3::new(g.pos[0] as f32, g.pos[1] as f32, g.pos[2] as f32) * M_TO_MM,
            );
            items.push(DrawItem {
                mesh: key,
                model: body_pose(g.body) * local,
                color,
                texture,
            });
        }
        // the assembled chassis: exact bricks at the chassis body's pose
        if let (Some(cid), Some(doc)) = (chassis_id, self.chassis_doc.as_ref()) {
            let body = body_pose(cid);
            for b in &scene.bricks {
                let part = doc.parts.get(&b.part);
                let (key, mesh, category) = match part {
                    Some(p) => match crate::assembly::geometry_of(p, bundle) {
                        crate::assembly::Geometry::Record(rec) => (
                            format!("ld:{}", rec.ldraw),
                            if viewport.has_mesh(&format!("ld:{}", rec.ldraw)) {
                                None
                            } else {
                                rec.mesh.decode().ok()
                            },
                            p.category.clone(),
                        ),
                        crate::assembly::Geometry::Imported { .. } => {
                            let key = format!("part:{}", b.part);
                            let m = if viewport.has_mesh(&key) {
                                None
                            } else {
                                part.and_then(|p| p.extra.get("mesh"))
                                    .and_then(|v| serde_json::from_value::<crate::bundle::MeshRecord>(v.clone()).ok())
                                    .and_then(|m| m.decode().ok())
                            };
                            (key, m, p.category.clone())
                        }
                        crate::assembly::Geometry::Shapes => {
                            let key = format!("part:{}", b.part);
                            let m = if viewport.has_mesh(&key) {
                                None
                            } else {
                                let mut out = crate::bundle::MeshData::default();
                                for s in &p.shapes {
                                    let sm = geometry::shape_mesh(s);
                                    let base = out.positions.len() as u32;
                                    out.positions.extend(sm.positions);
                                    out.normals.extend(sm.normals);
                                    out.indices.extend(sm.indices.iter().map(|i| i + base));
                                }
                                Some(out)
                            };
                            (key, m, p.category.clone())
                        }
                        crate::assembly::Geometry::None => (String::new(), None, String::new()),
                    },
                    None => (String::new(), None, String::new()),
                };
                if key.is_empty() {
                    continue;
                }
                if let Some(m) = mesh {
                    viewport.add_mesh(device, &key, &m);
                }
                // b.pos_m / quat place the brick's bounding-box centre; the mesh is in the brick's own frame
                let q = Quat::from_xyzw(b.quat[1] as f32, b.quat[2] as f32, b.quat[3] as f32, b.quat[0] as f32).normalize();
                let centre = match part.map(|p| crate::assembly::part_props(p, bundle)) {
                    Some(pr) => pr.bbox.map(|bb| ((bb.min + bb.max) * 0.5).as_vec3()).unwrap_or(Vec3::ZERO),
                    None => Vec3::ZERO,
                };
                let pos = Vec3::new(b.pos_m[0] as f32, b.pos_m[1] as f32, b.pos_m[2] as f32) * M_TO_MM - q * centre;
                items.push(DrawItem {
                    mesh: key,
                    model: body * Mat4::from_rotation_translation(q, pos),
                    color: crate::app::cat_color(&category, dark),
                    texture: None,
                });
            }
        }
        let lines = vec![
            Line {
                a: Vec3::ZERO,
                b: Vec3::X * 100.0,
                color: [0.6, 0.2, 0.1, 1.0],
            },
            Line {
                a: Vec3::ZERO,
                b: Vec3::Y * 100.0,
                color: [0.15, 0.45, 0.25, 1.0],
            },
            Line {
                a: Vec3::ZERO,
                b: Vec3::Z * 100.0,
                color: [0.1, 0.35, 0.65, 1.0],
            },
        ];
        (items, lines)
    }

    /// Where the camera should look: the mat's extent on first load, the
    /// chassis while following.
    pub fn frame_target(&mut self) -> Option<(Vec3, Vec3)> {
        let scene = self.scene.as_ref()?;
        if self.follow
            && let Some(cid) = scene.body_id("chassis")
        {
            let p = self.poses.get(cid)?;
            let c = Vec3::new(p.pos[0] as f32, p.pos[1] as f32, p.pos[2] as f32) * M_TO_MM;
            return Some((c - Vec3::splat(250.0), c + Vec3::splat(250.0)));
        }
        if !self.fit_pending {
            return None;
        }
        self.fit_pending = false;
        let plane = scene.geoms.iter().find(|g| g.kind == "plane" && g.size[0] > 0.0);
        Some(match plane {
            Some(g) => (
                Vec3::new(-g.size[0] as f32, -g.size[1] as f32, 0.0) * M_TO_MM,
                Vec3::new(g.size[0] as f32, g.size[1] as f32, 0.2) * M_TO_MM,
            ),
            None => (Vec3::splat(-500.0), Vec3::new(500.0, 500.0, 200.0)),
        })
    }

    // -------------------------------------------------------------- ui

    pub fn controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label("map");
            let aliases: Vec<String> = if self.worlds.is_empty() {
                vec![
                    "practice-line".into(),
                    "practice-zones".into(),
                    "practice-walls".into(),
                    "wro-2026-elementary".into(),
                    "wro-2026-junior".into(),
                    "wro-2026-senior".into(),
                    "empty".into(),
                ]
            } else {
                self.worlds.iter().map(|w| w.alias.clone()).collect()
            };
            let before = self.world.clone();
            egui::ComboBox::from_id_salt("world")
                .selected_text(self.world.clone())
                .show_ui(ui, |ui| {
                    for a in aliases {
                        ui.selectable_value(&mut self.world, a.clone(), a);
                    }
                });
            if self.world != before {
                self.reload();
            }
            ui.label("chassis");
            let chassis_label = self
                .chassis
                .as_ref()
                .and_then(|p| p.file_name())
                .map(|f| f.to_string_lossy().to_string())
                .unwrap_or_else(|| "the default box chassis".into());
            if ui
                .button(chassis_label)
                .on_hover_text("a robot.assembly.json from the workbench")
                .clicked()
                && let Some(p) = rfd::FileDialog::new().add_filter("assembly", &["json"]).pick_file()
            {
                match std::fs::read_to_string(&p)
                    .map_err(|e| e.to_string())
                    .and_then(|t| serde_json::from_str::<Document>(&t).map_err(|e| e.to_string()))
                {
                    Ok(doc) => self.set_chassis(p, doc),
                    Err(e) => self.message = format!("not an assembly file: {e}"),
                }
            }
            if self.chassis.is_some() && ui.small_button("×").on_hover_text("use the default chassis").clicked() {
                self.clear_chassis();
            }
            ui.label("program");
            let script_label = self
                .script
                .as_ref()
                .and_then(|p| p.file_name())
                .map(|f| f.to_string_lossy().to_string())
                .unwrap_or_else(|| "choose main.py".into());
            if ui.button(script_label).clicked()
                && let Some(p) = rfd::FileDialog::new().add_filter("python", &["py"]).pick_file()
            {
                self.script = Some(p);
            }
            ui.separator();
            if ui.button("Load").on_hover_text("build the world with this chassis").clicked() {
                self.load();
            }
            let running = self.status == "running";
            let paused = self.status == "paused";
            if !running && !paused && ui.add_enabled(self.script.is_some(), egui::Button::new("▶ Run")).clicked() {
                self.run();
            }
            if running && ui.button("⏸ Pause").clicked() {
                self.send(serde_json::json!({"cmd": "pause"}));
            }
            if paused && ui.button("▶ Resume").clicked() {
                self.send(serde_json::json!({"cmd": "resume"}));
            }
            if (running || paused) && ui.button("⏹ Stop").clicked() {
                self.send(serde_json::json!({"cmd": "stop"}));
            }
            ui.label("speed");
            let slider = ui.add(
                egui::Slider::new(&mut self.speed, 0.1..=8.0)
                    .logarithmic(true)
                    .fixed_decimals(1)
                    .suffix("×"),
            );
            if (slider.drag_stopped() || (slider.changed() && !slider.dragged())) && (self.speed - self.speed_sent).abs() > 1e-6 {
                self.speed_sent = self.speed;
                self.send(serde_json::json!({"cmd": "speed", "factor": self.speed}));
            }
            ui.checkbox(&mut self.follow, "follow the robot");
            ui.separator();
            ui.monospace(format!("{} · t = {:.2} s", self.status, self.t_ms as f64 / 1000.0));
            if let Some(e) = &self.error {
                ui.colored_label(egui::Color32::from_rgb(196, 68, 42), e.lines().last().unwrap_or(e));
            }
            if !self.message.is_empty() {
                ui.colored_label(egui::Color32::from_rgb(196, 68, 42), &self.message);
            }
        });
    }

    pub fn log_ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.strong("Program output");
            if ui.small_button("clear").clicked() {
                self.log.clear();
            }
        });
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for (stream, text) in &self.log {
                    match stream.as_str() {
                        "stderr" | "server" => ui.colored_label(egui::Color32::from_rgb(196, 68, 42), text.to_string()),
                        _ => ui.monospace(text),
                    };
                }
            });
    }
}

fn srgb_to_linear(c: f32) -> f32 {
    c.clamp(0.0, 1.0).powf(2.2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assembly::{self, Part, Shape};
    use crate::sim::testing::fake_server;
    use crate::viewport::testing::{test_device, test_renderer};
    use std::time::{Duration, Instant};

    const SCENE: &str = r#"{"bodies":["world","chassis"],"geoms":[{"name":"floor","type":"plane","body":0,"size":[1.2,0.9,0.1],"pos":[0,0,0],"quat":[1,0,0,0],"rgba":[1,1,1,1],"material":null,"group":0,"mesh":null}],"materials":{},"textures":{},"bricks":[],"timestep_ms":1}"#;

    fn pump_until(tab: &mut SimulateTab, secs: u64, f: impl Fn(&SimulateTab) -> bool) -> bool {
        let deadline = Instant::now() + Duration::from_secs(secs);
        while Instant::now() < deadline {
            tab.pump();
            if f(tab) {
                return true;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        false
    }

    fn state(status: &str) -> Event {
        Event::State {
            status: status.into(),
            t_ms: 0,
            speed: 1.0,
            error: None,
        }
    }

    #[test]
    fn events_drive_the_state_machine() {
        let mut t = SimulateTab::new(None);
        t.apply(Event::Hello { version: "9".into() });
        assert!(t.connected && t.status == "run server 9 ready");
        t.world = "nope".into();
        t.apply(Event::Worlds(vec![
            WorldEntry {
                alias: "empty".into(),
                ..Default::default()
            },
            WorldEntry {
                alias: "practice-line".into(),
                ..Default::default()
            },
        ]));
        assert_eq!(t.world, "practice-line");
        t.apply(Event::Scene(Box::new(serde_json::from_str(SCENE).unwrap())));
        assert_eq!(t.poses.len(), 2);
        assert_eq!(t.scene_gen, 1);
        assert!(t.fit_pending);
        t.apply(Event::Frame {
            t_ms: 5,
            poses: vec![Pose::default()],
        });
        assert_eq!(t.t_ms, 5);
        assert_eq!(t.poses[1].quat, [1.0, 0.0, 0.0, 0.0], "a frame of the wrong size is ignored");
        t.apply(Event::Frame {
            t_ms: 6,
            poses: vec![
                Pose::default(),
                Pose {
                    pos: [1.0, 2.0, 3.0],
                    quat: [1.0, 0.0, 0.0, 0.0],
                },
            ],
        });
        assert_eq!(t.poses[1].pos, [1.0, 2.0, 3.0]);
        for i in 0..2001 {
            t.apply(Event::Log {
                stream: "stdout".into(),
                text: i.to_string(),
            });
        }
        assert_eq!(t.log.len(), 1501);
        t.apply(Event::State {
            status: "running".into(),
            t_ms: 7,
            speed: 2.0,
            error: Some("x".into()),
        });
        assert_eq!(
            (t.status.as_str(), t.t_ms, t.speed_sent, t.error.as_deref()),
            ("running", 7, 2.0, Some("x"))
        );
        assert!(!t.is_live(), "no process: not live");
        t.apply(Event::Error("bad".into()));
        assert_eq!(t.message, "bad");
        assert_eq!(t.log.last().unwrap().0, "server");
        t.apply(Event::Exited("gone".into()));
        assert!(!t.connected && t.status == "run server stopped" && t.message == "gone");
        t.status = "loaded".into();
        t.apply(Event::Bye);
        assert_eq!(t.status, "run server stopped");
    }

    #[test]
    fn a_map_chosen_mid_run_loads_once_the_run_has_stopped() {
        let mut t = SimulateTab::new(None);
        t.status = "running".into();
        t.reload();
        assert!(t.pending_load);
        assert_eq!(t.sent.last().unwrap()["cmd"], "stop");
        t.apply(state("stopped"));
        assert!(!t.pending_load);
        assert!(t.message.contains("no Python interpreter"), "{}", t.message);
        // the tab showing loads the default map once
        t.message.clear();
        t.ensure_loaded();
        assert!(t.message.contains("no Python interpreter"));
        t.message.clear();
        t.ensure_loaded();
        assert!(t.message.is_empty());
        // a run needs a program
        t.run();
        assert_eq!(t.message, "choose a program first");
    }

    #[test]
    fn choosing_a_map_shows_it_and_the_controls_reach_the_server() {
        let Some(fake) = fake_server("simulate") else {
            return;
        };
        let (dir, mut t) = (fake.dir, SimulateTab::new_with_env(Some(fake.python), fake.env));
        t.ensure_loaded();
        assert!(
            pump_until(&mut t, 30, |t| t.scene.is_some() && t.status == "loaded"),
            "{} / {}",
            t.status,
            t.message
        );
        assert!(t.log.iter().any(|(_, x)| x == "loaded practice-line with None"), "{:?}", t.log);
        assert_eq!(t.poses.len(), 2);
        assert!(t.worlds.iter().any(|w| w.alias == "wro-2026-senior"));
        // another map: loaded as soon as it is chosen
        t.world = "wro-2026-senior".into();
        t.reload();
        assert!(
            pump_until(&mut t, 30, |t| t.log.iter().any(|(_, x)| x == "loaded wro-2026-senior with None")),
            "{:?}",
            t.log
        );
        // a run: its print and its frame arrive
        let script = dir.join("main.py");
        std::fs::write(&script, "print('hi')\n").unwrap();
        t.script = Some(script);
        t.run();
        assert!(pump_until(&mut t, 30, |t| t.status == "running"), "{}", t.status);
        assert!(t.log.iter().any(|(s, x)| s == "stdout" && x.ends_with("main.py")), "{:?}", t.log);
        assert!(t.t_ms == 10 && (t.poses[1].pos[0] - 0.01).abs() < 1e-9);
        assert!(t.is_live());
        // a map chosen mid-run: stopped first, then loaded
        t.world = "practice-line".into();
        t.reload();
        assert!(t.pending_load);
        assert!(
            pump_until(&mut t, 30, |t| !t.pending_load
                && t.status == "loaded"
                && t.log.iter().any(|(_, x)| x == "loaded practice-line with None")),
            "{} {:?}",
            t.status,
            t.log
        );
        // pause, resume, stop, speed, and an error the server reports
        t.run();
        assert!(pump_until(&mut t, 30, |t| t.status == "running"));
        t.send(serde_json::json!({"cmd": "pause"}));
        assert!(pump_until(&mut t, 30, |t| t.status == "paused"));
        t.send(serde_json::json!({"cmd": "resume"}));
        assert!(pump_until(&mut t, 30, |t| t.status == "running"));
        t.send(serde_json::json!({"cmd": "stop"}));
        assert!(pump_until(&mut t, 30, |t| t.status == "stopped"));
        t.send(serde_json::json!({"cmd": "speed", "factor": 2.5}));
        assert!(pump_until(&mut t, 30, |t| t.speed_sent == 2.5));
        t.send(serde_json::json!({"cmd": "bogus"}));
        assert!(pump_until(&mut t, 30, |t| t.message == "no such command bogus"), "{}", t.message);
        // a chassis: the map is rebuilt with it, and without it again
        let path = dir.join("robot.assembly.json");
        std::fs::write(&path, assembly::EXAMPLE).unwrap();
        t.set_chassis(path.clone(), assembly::example());
        assert!(
            pump_until(&mut t, 30, |t| t.log.iter().any(|(_, x)| x.ends_with("robot.assembly.json"))),
            "{:?}",
            t.log
        );
        t.clear_chassis();
        assert!(pump_until(&mut t, 30, |t| t
            .log
            .iter()
            .any(|(_, x)| x == "loaded practice-line with None")));
        t.shutdown();
        assert!(t.process.is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The real runtime: `OPENBRICKS_SIM_PYTHON` names an interpreter
    /// with `openbricks_sim` and MuJoCo installed (CI's Linux leg sets
    /// it); unset, the test is skipped.
    #[test]
    fn the_real_run_server_end_to_end() {
        let Some(python) = std::env::var("OPENBRICKS_SIM_PYTHON").ok().filter(|p| !p.is_empty()) else {
            eprintln!("OPENBRICKS_SIM_PYTHON is unset: skipping the end-to-end test");
            return;
        };
        let dir = std::env::temp_dir().join(format!("ob-e2e-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let assembly = dir.join("robot.assembly.json");
        std::fs::write(&assembly, assembly::EXAMPLE).unwrap();
        let script = dir.join("main.py");
        std::fs::write(&script, "print('hello from the program')\nrobot.run_for(0.3)\nprint('done')\n").unwrap();
        let long = dir.join("long.py");
        std::fs::write(&long, "robot.run_for(30.0)\n").unwrap();
        let mut t = SimulateTab::new(Some(python));
        t.world = "practice-line".into();
        t.set_chassis(assembly, assembly::example());
        assert!(
            pump_until(&mut t, 180, |t| t.scene.is_some() && t.status == "loaded"),
            "{} / {} / {:?}",
            t.status,
            t.message,
            t.log
        );
        let scene = t.scene.as_ref().unwrap();
        assert!(scene.body_id("chassis").is_some(), "{:?}", scene.bodies);
        assert!(!scene.bricks.is_empty(), "the assembled chassis carries its bricks");
        assert!(scene.geoms.iter().any(|g| g.kind == "plane"));
        assert_eq!(t.poses.len(), scene.bodies.len());
        t.script = Some(script);
        t.run();
        assert!(
            pump_until(&mut t, 180, |t| t.status == "finished"),
            "{} / {} / {:?}",
            t.status,
            t.message,
            t.log
        );
        assert!(
            t.log.iter().any(|(s, x)| s == "stdout" && x == "hello from the program"),
            "{:?}",
            t.log
        );
        assert!(t.log.iter().any(|(s, x)| s == "stdout" && x == "done"), "{:?}", t.log);
        assert!(t.t_ms >= 300, "{}", t.t_ms);
        // pause, resume and stop a long program
        t.script = Some(long);
        t.run();
        assert!(pump_until(&mut t, 60, |t| t.status == "running"), "{} / {}", t.status, t.message);
        t.send(serde_json::json!({"cmd": "pause"}));
        assert!(pump_until(&mut t, 60, |t| t.status == "paused"), "{}", t.status);
        t.send(serde_json::json!({"cmd": "resume"}));
        assert!(pump_until(&mut t, 60, |t| t.status == "running"), "{}", t.status);
        t.send(serde_json::json!({"cmd": "stop"}));
        assert!(pump_until(&mut t, 60, |t| t.status == "stopped"), "{}", t.status);
        assert!(t.error.is_none(), "{:?}", t.error);
        t.shutdown();
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn draw_items_builds_meshes_for_every_geom_kind_and_the_chassis_bricks() {
        let Some((device, queue)) = test_device() else { return };
        let mut vp = Viewport::new(&device, &queue);
        let bundle_path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../openbricks/openbricks_sim/bricks/technic_bundle.json.zlib");
        let bundle = crate::bundle::load_bundle(&bundle_path).expect("the shipped brick bundle");
        let dir = std::env::temp_dir().join(format!("ob-draw-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let png = dir.join("mat.png");
        image::save_buffer(
            &png,
            &[255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255],
            2,
            2,
            image::ColorType::Rgba8,
        )
        .unwrap();
        let geom = |name: &str, kind: &str, body: usize, group: i64, mesh: Option<&str>, material: Option<&str>| {
            format!(
                r#"{{"name":"{name}","type":"{kind}","body":{body},"size":[0.1,0.05,0.02],"pos":[0.1,0,0],"quat":[1,0,0,0],"rgba":[0.5,0.5,0.5,1],"material":{},"group":{group},"mesh":{}}}"#,
                material.map(|m| format!("\"{m}\"")).unwrap_or("null".into()),
                mesh.map(|m| format!("\"{m}\"")).unwrap_or("null".into())
            )
        };
        let geoms = [
            geom("floor", "plane", 0, 0, None, Some("mat")),
            geom("b", "box", 0, 0, None, Some("plain")),
            geom("s", "sphere", 0, 0, None, None),
            geom("c", "cylinder", 1, 0, None, None),
            geom("k", "capsule", 1, 0, None, None),
            geom("e", "ellipsoid", 1, 0, None, None),
            geom("m", "mesh", 0, 0, Some("frame"), None),
            geom("helper", "box", 0, 3, None, None),
            geom("chassis_brick:beam", "box", 1, 3, None, None),
        ]
        .join(",");
        let scene_json = format!(
            r#"{{"bodies":["world","chassis"],"geoms":[{geoms}],"materials":{{"mat":{{"rgba":[0.2,0.3,0.4,1],"texture":"tex","texrepeat":[2,2]}},"plain":{{"rgba":[0.9,0.1,0.1,1],"texture":null,"texrepeat":[1,1]}}}},"textures":{{"tex":"{}"}},"bricks":[{{"path":"beam","part":"lego_32278","ldraw":"32278","pos_m":[0.01,0,0.02],"quat":[1,0,0,0],"half_m":[0.06,0.004,0.004],"category":"lego"}},{{"path":"servo","part":"servo","ldraw":null,"pos_m":[0,0,0],"quat":[1,0,0,0],"half_m":[0.01,0.01,0.01],"category":"servo"}},{{"path":"ghost","part":"missing","ldraw":null,"pos_m":[0,0,0],"quat":[1,0,0,0],"half_m":[0.01,0.01,0.01],"category":""}},{{"path":"bare","part":"bare","ldraw":null,"pos_m":[0,0,0],"quat":[1,0,0,0],"half_m":[0.01,0.01,0.01],"category":""}}],"timestep_ms":1}}"#,
            png.to_string_lossy().replace('\\', "/")
        );
        let mut t = SimulateTab::new(None);
        t.apply(Event::Scene(Box::new(serde_json::from_str(&scene_json).unwrap())));
        let mut doc = assembly::example();
        doc.parts.insert(
            "lego_32278".into(),
            Part {
                name: "beam".into(),
                category: "lego".into(),
                mass_g: 1.0,
                source: String::new(),
                source_note: String::new(),
                ldraw: Some("32278".into()),
                shapes: vec![],
                extra: Default::default(),
            },
        );
        doc.parts.insert(
            "servo".into(),
            Part {
                name: "servo".into(),
                category: "servo".into(),
                mass_g: 20.0,
                source: String::new(),
                source_note: String::new(),
                ldraw: None,
                shapes: vec![Shape::Box {
                    size: [20.0, 20.0, 20.0],
                    pos: [0.0; 3],
                }],
                extra: Default::default(),
            },
        );
        doc.parts.insert(
            "bare".into(),
            Part {
                name: "bare".into(),
                category: String::new(),
                mass_g: 1.0,
                source: String::new(),
                source_note: String::new(),
                ldraw: None,
                shapes: vec![],
                extra: Default::default(),
            },
        );
        t.chassis_doc = Some(doc);
        let (items, lines) = t.draw_items(&mut vp, &device, &queue, &bundle, false);
        // six geoms drawn; the file mesh, the helper and the brick stand-in skipped; two of four bricks drawable
        assert_eq!(items.len(), 8, "{:?}", items.iter().map(|i| i.mesh.clone()).collect::<Vec<_>>());
        assert_eq!(lines.len(), 3);
        assert_eq!(
            items[0].texture.as_deref(),
            Some("tex:tex"),
            "the plane takes its material's texture"
        );
        assert_eq!(t.textures_loaded.get("tex"), Some(&true));
        assert!(items[1].texture.is_none());
        assert!(
            items[1].color[0] > items[1].color[2],
            "the default grey takes the material colour (red)"
        );
        assert!(vp.has_mesh("ld:32278") && vp.has_mesh("part:servo"));
        assert!(!vp.has_mesh("part:bare") && !vp.has_mesh("part:missing"));
        let mut renderer = test_renderer(&device);
        vp.render(
            &device,
            &queue,
            &mut renderer,
            (64, 48),
            &crate::viewport::Scene {
                items: &items,
                lines: &lines,
                overlay: &[],
                background: [0.0; 4],
            },
        );
        // a second pass reuses every mesh; a texture that cannot be read is reported once
        t.textures_loaded.clear();
        t.scene
            .as_mut()
            .unwrap()
            .textures
            .insert("tex".into(), dir.join("missing.png").to_string_lossy().to_string());
        let (again, _) = t.draw_items(&mut vp, &device, &queue, &bundle, true);
        assert_eq!(again.len(), 8);
        assert!(again[0].texture.is_none(), "no texture when the file is unreadable");
        assert!(t.log.iter().any(|(s, x)| s == "server" && x.starts_with("texture ")), "{:?}", t.log);
        // the camera: the mat once, then the chassis while following
        assert!(t.frame_target().is_some());
        assert!(t.frame_target().is_none());
        t.follow = true;
        assert!(t.frame_target().is_some());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn frame_target_fits_the_mat_once() {
        let mut t = SimulateTab::new(None);
        assert!(t.frame_target().is_none());
        let scene: Scene = serde_json::from_str(r#"{"bodies":["world","chassis"],"geoms":[{"name":"floor","type":"plane","body":0,"size":[1.2,0.9,0.1],"pos":[0,0,0],"quat":[1,0,0,0],"rgba":[1,1,1,1],"material":null,"group":0,"mesh":null}],"materials":{},"textures":{},"bricks":[],"timestep_ms":1}"#).unwrap();
        t.poses = vec![
            Pose {
                pos: [0.0; 3],
                quat: [1.0, 0.0, 0.0, 0.0]
            };
            2
        ];
        t.scene = Some(scene);
        t.fit_pending = true;
        let (lo, hi) = t.frame_target().unwrap();
        assert_eq!(lo, Vec3::new(-1200.0, -900.0, 0.0));
        assert_eq!(hi.x, 1200.0);
        assert!(t.frame_target().is_none());
        t.follow = true;
        t.poses[1].pos = [0.5, 0.25, 0.05];
        let (lo, hi) = t.frame_target().unwrap();
        assert_eq!((lo + hi) * 0.5, Vec3::new(500.0, 250.0, 50.0));
    }

    #[test]
    fn pump_without_a_process_is_quiet() {
        let mut t = SimulateTab::new(None);
        assert!(!t.pump());
        assert!(!t.is_live());
        t.load();
        assert!(t.message.contains("Python"));
    }

    #[test]
    fn srgb_conversion() {
        assert_eq!(srgb_to_linear(0.0), 0.0);
        assert_eq!(srgb_to_linear(1.0), 1.0);
        assert!(srgb_to_linear(0.5) < 0.5);
        assert!(crate::sim::parse_event("{\"ev\":\"bye\"}").is_ok());
    }
}
