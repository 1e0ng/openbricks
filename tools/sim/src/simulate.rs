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
}

impl SimulateTab {
    pub fn new(python: Option<String>) -> Self {
        SimulateTab {
            python,
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
        }
    }

    pub fn follows(&self) -> bool {
        self.follow
    }

    pub fn set_chassis(&mut self, path: PathBuf, doc: Document) {
        self.chassis = Some(path);
        self.chassis_doc = Some(doc);
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
        match SimProcess::spawn(&python) {
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
        let mut dead = false;
        while let Some(ev) = self.process.as_mut().and_then(|p| p.try_recv()) {
            changed = true;
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
                }
                Event::Error(text) => {
                    self.message = text.clone();
                    self.log.push(("server".into(), text));
                }
                Event::Bye => {
                    dead = true;
                }
                Event::Exited(why) => {
                    self.message = why;
                    dead = true;
                }
            }
        }
        if dead {
            self.process = None;
            self.connected = false;
            self.status = "run server stopped".into();
        }
        changed
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
            egui::ComboBox::from_id_salt("world")
                .selected_text(self.world.clone())
                .show_ui(ui, |ui| {
                    for a in aliases {
                        ui.selectable_value(&mut self.world, a.clone(), a);
                    }
                });
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
                self.chassis = None;
                self.chassis_doc = None;
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
