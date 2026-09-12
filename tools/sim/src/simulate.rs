//! The Simulate tab: a map, the chassis assembled in the workbench and
//! the program; run, pause, stop; the run shown live from the poses
//! the run server streams.

use crate::assembly::Document;
use crate::bundle::Bundle;
use crate::geometry;
use crate::route::{self, Action, End, KINDS, Pose2, Route};
use crate::sim::{ChassisInfo, Event, Pose, Scene, SimProcess, WorldEntry};
use crate::viewport::{DrawItem, Line, Viewport};
use eframe::egui;
use eframe::egui_wgpu::wgpu;
use glam::{Mat4, Quat, Vec3};
use std::collections::HashMap;
use std::path::PathBuf;

const M_TO_MM: f32 = 1000.0;
/// How close (screen points) a route handle is grabbed from.
const HANDLE_PX: f32 = 10.0;
/// The ghost chassis: 70 % transparent.
const GHOST_ALPHA: f32 = 0.3;

/// The map tools: the next click reaches that point in a straight line
/// (a turn, then the distance) or on the tangent arc.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Tool {
    StraightTo,
    CurveTo,
}

impl Tool {
    pub fn label(self) -> &'static str {
        match self {
            Tool::StraightTo => "straight line",
            Tool::CurveTo => "curve",
        }
    }
}

/// What one frame of the Simulate view draws.
#[derive(Default)]
pub struct SimDraw {
    pub items: Vec<DrawItem>,
    pub lines: Vec<Line>,
    /// The chassis where a drag would put it, to blend translucently.
    pub ghost: Vec<DrawItem>,
}

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
    /// The route being planned on this map.
    pub route: Route,
    route_path: Option<PathBuf>,
    /// Armed: the next click on the map reaches that point with this tool.
    pub pick: Option<Tool>,
    /// Where the chassis was put (dragged or typed); applied after every load.
    placed: Option<Pose2>,
    /// A translucent chassis where a drag would put it: under the
    /// pointer, or at a dragged action's end.
    pub ghost: Option<Pose2>,
    /// The action whose handle is being dragged.
    dragging: Option<usize>,
    /// The action lit on the map and in the list.
    pub selected: Option<usize>,
    show_program: bool,
    show_definitions: bool,
    /// Indices in the last `draw_items` result that draw the chassis.
    pub chassis_items: Vec<usize>,
    /// The chassis's items in its own frame: mesh, placement, colour.
    chassis_locals: Vec<(String, Mat4, [f32; 4])>,
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
            route: Route::default(),
            route_path: None,
            pick: None,
            placed: None,
            ghost: None,
            dragging: None,
            selected: None,
            show_program: false,
            show_definitions: false,
            chassis_items: vec![],
            chassis_locals: vec![],
            #[cfg(test)]
            sent: vec![],
        }
    }

    #[cfg(test)]
    pub fn follows(&self) -> bool {
        self.follow
    }

    #[cfg(test)]
    pub fn status(&self) -> &str {
        &self.status
    }

    #[cfg(test)]
    pub fn scene_loaded(&self) -> bool {
        self.scene.is_some()
    }

    #[cfg(test)]
    pub fn set_script(&mut self, path: PathBuf) {
        self.script = Some(path);
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
                    self.route.world = self.world.clone();
                    match (self.placed, s.chassis.as_ref()) {
                        (Some(p), _) => {
                            // the server put the robot at the assembly's spawn; the user's place wins
                            self.route.start = p;
                            self.send(serde_json::json!({"cmd": "place", "x_mm": p.x_mm, "y_mm": p.y_mm, "yaw_deg": p.yaw_deg}));
                        }
                        (None, Some(c)) => self.route.start = c.spawn,
                        (None, None) => {}
                    }
                    self.scene = Some(*s);
                    self.fit_pending = true;
                }
                Event::Frame { t_ms, poses } => {
                    self.t_ms = t_ms;
                    if poses.len() == self.poses.len() {
                        // a body the physics has not placed yet (all zeros, or worse) stands at
                        // the origin upright rather than vanishing into NaN
                        self.poses = poses.into_iter().map(sane_pose).collect();
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

    // ------------------------------------------------------------ route

    pub fn chassis(&self) -> Option<&ChassisInfo> {
        self.scene.as_ref()?.chassis.as_ref()
    }

    fn busy(&self) -> bool {
        matches!(self.status.as_str(), "running" | "paused")
    }

    /// The chassis as the last frame has it.
    pub fn chassis_pose(&self) -> Option<Pose2> {
        let scene = self.scene.as_ref()?;
        let p = self.poses.get(scene.body_id("chassis")?)?;
        let [w, x, y, z] = p.quat;
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)).to_degrees();
        Some(Pose2 {
            x_mm: p.pos[0] * 1000.0,
            y_mm: p.pos[1] * 1000.0,
            yaw_deg: route::wrap_deg(yaw),
        })
    }

    /// Put the chassis at `pose`: the route starts there, the server
    /// teleports the robot, and every later load puts it back.
    pub fn place_chassis(&mut self, pose: Pose2) {
        if self.busy() {
            self.message = "stop the program before moving the chassis".into();
            return;
        }
        let pose = Pose2 {
            x_mm: (pose.x_mm * 10.0).round() / 10.0,
            y_mm: (pose.y_mm * 10.0).round() / 10.0,
            yaw_deg: route::wrap_deg((pose.yaw_deg * 10.0).round() / 10.0),
        };
        self.route.start = pose;
        self.placed = Some(pose);
        if self.scene.is_some() {
            self.send(serde_json::json!({"cmd": "place", "x_mm": pose.x_mm, "y_mm": pose.y_mm, "yaw_deg": pose.yaw_deg}));
        }
    }

    /// A drag of the chassis on the map: a ghost follows the pointer
    /// with its axle centre under it; letting go places it.
    pub fn begin_chassis_drag(&mut self) -> Option<Pose2> {
        if self.busy() {
            self.message = "stop the program before moving the chassis".into();
            return None;
        }
        let p = self.chassis_pose()?;
        self.ghost = Some(p);
        Some(p)
    }

    pub fn drag_chassis(&mut self, pose: Pose2) {
        if self.ghost.is_some() && self.dragging.is_none() {
            self.ghost = Some(pose);
        }
    }

    pub fn end_chassis_drag(&mut self) {
        if self.dragging.is_none()
            && let Some(p) = self.ghost.take()
        {
            self.place_chassis(p);
        }
    }

    /// The route worked out from its start.
    pub fn steps(&self) -> Vec<route::Step> {
        route::plan(&self.route)
    }

    fn end_pose(&self) -> Pose2 {
        self.steps().last().map(|s| s.end).unwrap_or(self.route.start)
    }

    /// A click on the map at `(x_mm, y_mm)` with a tool armed: the
    /// actions that reach the point join the route. Returns whether the
    /// click was used.
    pub fn map_click(&mut self, x_mm: f64, y_mm: f64) -> bool {
        let Some(tool) = self.pick.take() else { return false };
        let to = [(x_mm * 10.0).round() / 10.0, (y_mm * 10.0).round() / 10.0];
        let from = self.end_pose();
        let (actions, note) = match tool {
            Tool::StraightTo => (route::straight_to(from, to), None),
            Tool::CurveTo => route::curve_to(from, to),
        };
        self.message = note.unwrap_or_default();
        if actions.is_empty() {
            return true;
        }
        self.route.actions.extend(actions);
        self.selected = Some(self.route.actions.len() - 1);
        true
    }

    /// Append an action of a kind with its default parameters.
    pub fn add_action(&mut self, kind: &str) {
        self.route.actions.push(Action::default_of(kind));
        self.selected = Some(self.route.actions.len() - 1);
    }

    pub fn remove_action(&mut self, i: usize) {
        if i < self.route.actions.len() {
            self.route.actions.remove(i);
            self.selected = None;
        }
    }

    /// Move the action at `from` so it sits at `to` (an insertion index
    /// in the list as it was).
    pub fn move_action(&mut self, from: usize, to: usize) {
        let n = self.route.actions.len();
        if from >= n || to > n {
            return;
        }
        let a = self.route.actions.remove(from);
        let at = if to > from { to - 1 } else { to };
        self.route.actions.insert(at, a);
        self.selected = Some(at);
    }

    pub fn undo_segment(&mut self) {
        self.route.actions.pop();
        self.selected = None;
    }

    pub fn clear_route(&mut self) {
        self.route.actions.clear();
        self.pick = None;
        self.selected = None;
    }

    /// The action whose handle is under a screen point.
    pub fn route_handle_at(&self, cam: &crate::viewport::Camera, px: f32, py: f32, w: f32, h: f32) -> Option<usize> {
        self.scene.as_ref()?;
        let mut best: Option<(f32, usize)> = None;
        for (i, (step, action)) in self.steps().iter().zip(&self.route.actions).enumerate() {
            let Some(hp) = route::handle(step, action) else { continue };
            let Some(sp) = cam.project(Vec3::new(hp[0] as f32, hp[1] as f32, 3.0), w, h) else {
                continue;
            };
            let d = (sp - glam::Vec2::new(px, py)).length();
            if d <= HANDLE_PX && best.map(|b| d < b.0).unwrap_or(true) {
                best = Some((d, i));
            }
        }
        best.map(|b| b.1)
    }

    /// Start dragging an action's handle: the ghost shows where the
    /// robot ends up after it.
    pub fn begin_handle_drag(&mut self, i: usize) -> bool {
        if i >= self.route.actions.len() {
            return false;
        }
        self.dragging = Some(i);
        self.selected = Some(i);
        self.ghost = self.steps().get(i).map(|s| s.end);
        true
    }

    /// The dragged handle at a map point: the action takes the
    /// parameters that reach it, and the ghost moves to its new end.
    pub fn drag_handle(&mut self, x_mm: f64, y_mm: f64) {
        let Some(i) = self.dragging else { return };
        let steps = self.steps();
        let Some(step) = steps.get(i) else { return };
        let action = route::dragged(&self.route.actions[i], step.start, [x_mm, y_mm]);
        self.route.actions[i] = action;
        self.ghost = self.steps().get(i).map(|s| s.end);
    }

    pub fn end_handle_drag(&mut self) {
        self.dragging = None;
        self.ghost = None;
    }

    /// The route's program for the loaded chassis.
    pub fn route_program(&self) -> Result<String, String> {
        let c = self.chassis().ok_or("load a map with a chassis first")?;
        if self.route.actions.is_empty() {
            return Err("add an action first".into());
        }
        Ok(route::program(&self.route, c.wheel_diameter_mm, c.axle_track_mm))
    }

    /// Write the route's program and run it.
    pub fn run_route(&mut self) {
        let text = match self.route_program() {
            Ok(t) => t,
            Err(e) => {
                self.message = e;
                return;
            }
        };
        let path = std::env::temp_dir().join(format!("openbricks-route-{}.py", std::process::id()));
        if let Err(e) = std::fs::write(&path, text) {
            self.message = format!("could not write the route's program: {e}");
            return;
        }
        self.script = Some(path);
        self.run();
    }

    pub fn save_route(&mut self, path: PathBuf) {
        match self.route.save(&path) {
            Ok(()) => {
                self.message = format!("saved the route to {}", path.display());
                self.route_path = Some(path);
            }
            Err(e) => self.message = format!("could not save the route: {e}"),
        }
    }

    /// Load a route: its map loads if it is another one, and the
    /// chassis goes to its start.
    pub fn load_route(&mut self, path: PathBuf) {
        match Route::load(&path) {
            Ok(r) => {
                self.pick = None;
                self.selected = None;
                self.placed = Some(r.start);
                let other_map = !r.world.is_empty() && r.world != self.world;
                self.route = r;
                self.route_path = Some(path);
                if other_map {
                    self.world = self.route.world.clone();
                    self.reload();
                } else if self.scene.is_some() {
                    let p = self.route.start;
                    self.place_chassis(p);
                }
            }
            Err(e) => self.message = format!("could not load the route: {e}"),
        }
    }

    /// The route drawn on the map: the start's heading, each action's
    /// path, a handle at every end, and the lit one bigger.
    pub fn route_lines(&self, dark: bool) -> Vec<Line> {
        let mut out = Vec::new();
        if self.scene.is_none() {
            return out;
        }
        let z = 3.0;
        let (straight, curve, mark, lit) = if dark {
            (
                [0.35, 0.65, 1.0, 1.0],
                [0.4, 0.9, 0.5, 1.0],
                [1.0, 0.75, 0.25, 1.0],
                [1.0, 1.0, 1.0, 1.0],
            )
        } else {
            (
                [0.1, 0.35, 0.8, 1.0],
                [0.1, 0.55, 0.25, 1.0],
                [0.85, 0.45, 0.0, 1.0],
                [0.9, 0.1, 0.1, 1.0],
            )
        };
        let steps = self.steps();
        for (i, (step, action)) in steps.iter().zip(&self.route.actions).enumerate() {
            let on = self.selected == Some(i);
            let color = match action {
                Action::Straight { .. } => straight,
                Action::Curve { .. } => curve,
                _ => mark,
            };
            let color = if on { lit } else { color };
            for w in step.points.windows(2) {
                out.push(Line {
                    a: Vec3::new(w[0][0] as f32, w[0][1] as f32, z),
                    b: Vec3::new(w[1][0] as f32, w[1][1] as f32, z),
                    color,
                });
            }
            if let Action::Turn { .. } = action {
                // the new heading, from the spot
                let (hx, hy) = (
                    step.end.yaw_deg.to_radians().cos() as f32,
                    step.end.yaw_deg.to_radians().sin() as f32,
                );
                let o = Vec3::new(step.end.x_mm as f32, step.end.y_mm as f32, z);
                out.push(Line {
                    a: o,
                    b: o + Vec3::new(hx, hy, 0.0) * route::TURN_HANDLE_MM as f32,
                    color,
                });
            }
            if let Some(hp) = route::handle(step, action) {
                let (x, y) = (hp[0] as f32, hp[1] as f32);
                let r = if on { 14.0 } else { 9.0 };
                let corners = [(-r, -r), (r, -r), (r, r), (-r, r)];
                for k in 0..4 {
                    let (ax, ay) = corners[k];
                    let (bx, by) = corners[(k + 1) % 4];
                    out.push(Line {
                        a: Vec3::new(x + ax, y + ay, z),
                        b: Vec3::new(x + bx, y + by, z),
                        color: if on { lit } else { mark },
                    });
                }
            } else {
                // a stop or a call: a small diamond where it happens
                let (x, y) = (step.end.x_mm as f32, step.end.y_mm as f32);
                let d = 8.0;
                let pts = [(0.0, -d), (d, 0.0), (0.0, d), (-d, 0.0)];
                for k in 0..4 {
                    let (ax, ay) = pts[k];
                    let (bx, by) = pts[(k + 1) % 4];
                    out.push(Line {
                        a: Vec3::new(x + ax, y + ay, z + 0.5),
                        b: Vec3::new(x + bx, y + by, z + 0.5),
                        color: if on { lit } else { mark },
                    });
                }
            }
        }
        // the start: an arrow along the heading
        let s = self.route.start;
        let (c, sn) = (s.yaw_deg.to_radians().cos() as f32, s.yaw_deg.to_radians().sin() as f32);
        let o = Vec3::new(s.x_mm as f32, s.y_mm as f32, z);
        let tip = o + Vec3::new(c, sn, 0.0) * 60.0;
        let side = Vec3::new(-sn, c, 0.0) * 14.0;
        let back = o + Vec3::new(c, sn, 0.0) * 40.0;
        for (a, b) in [(o, tip), (tip, back + side), (tip, back - side)] {
            out.push(Line { a, b, color: mark });
        }
        out
    }

    /// The route panel: the start, the tools, the actions with their
    /// parameters to type or drag, and what to do with the route.
    pub fn route_ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.heading("Route");
            if ui.button("Save…").clicked()
                && let Some(p) = rfd::FileDialog::new()
                    .add_filter("route", &["json"])
                    .set_file_name("plan.route.json")
                    .save_file()
            {
                self.save_route(p);
            }
            if ui.button("Load…").clicked()
                && let Some(p) = rfd::FileDialog::new().add_filter("route", &["json"]).pick_file()
            {
                self.load_route(p);
            }
        });
        if let Some(p) = &self.route_path {
            ui.weak(p.file_name().map(|f| f.to_string_lossy().to_string()).unwrap_or_default());
        }
        let busy = self.busy();
        ui.add_space(4.0);
        ui.strong("Start  (drag the chassis on the map, shift turns it)");
        let mut start = self.route.start;
        let mut changed = false;
        ui.add_enabled_ui(!busy, |ui| {
            ui.horizontal(|ui| {
                ui.weak("x");
                changed |= ui.add(egui::DragValue::new(&mut start.x_mm).speed(1.0).suffix(" mm")).changed();
                ui.weak("y");
                changed |= ui.add(egui::DragValue::new(&mut start.y_mm).speed(1.0).suffix(" mm")).changed();
                ui.weak("heading");
                changed |= ui.add(egui::DragValue::new(&mut start.yaw_deg).speed(1.0).suffix("°")).changed();
            });
        });
        if changed {
            self.place_chassis(start);
        }
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            ui.weak("to a point on the map:");
            for (tool, label) in [(Tool::StraightTo, "→ point"), (Tool::CurveTo, "⌒ point")] {
                if ui
                    .selectable_label(self.pick == Some(tool), label)
                    .on_hover_text(match tool {
                        Tool::StraightTo => "a turn to face the point, then the distance",
                        Tool::CurveTo => "the arc tangent to the heading through the point",
                    })
                    .clicked()
                {
                    self.pick = if self.pick == Some(tool) { None } else { Some(tool) };
                }
            }
        });
        if let Some(t) = self.pick {
            ui.colored_label(
                egui::Color32::from_rgb(217, 145, 15),
                format!("click the map where the {} ends", t.label()),
            );
        }
        ui.horizontal_wrapped(|ui| {
            ui.weak("add:");
            for kind in KINDS {
                let label = format!("+ {}{}", &kind[..1].to_uppercase(), &kind[1..]);
                if ui.button(label).clicked() {
                    self.add_action(kind);
                }
            }
        });
        let steps = self.steps();
        let functions = self.route.functions();
        let mut remove = None;
        let mut nudge: Option<(usize, i32)> = None;
        let mut from_to: Option<(usize, usize)> = None;
        let mut select = None;
        let selected = self.selected;
        egui::ScrollArea::vertical().max_height(420.0).show(ui, |ui| {
            let frame = egui::Frame::default().inner_margin(2.0);
            let (_, _dropped) = ui.dnd_drop_zone::<usize, ()>(frame, |ui| {
                for (i, action) in self.route.actions.iter_mut().enumerate() {
                    let step = &steps[i];
                    let on = selected == Some(i);
                    let row = ui.horizontal(|ui| {
                        ui.dnd_drag_source(egui::Id::new(("route-action", i)), i, |ui| {
                            ui.label("≡").on_hover_text("drag to reorder");
                        });
                        if ui.selectable_label(on, format!("{}.", i + 1)).clicked() {
                            select = Some(i);
                        }
                        ui.strong(action.kind());
                        match action {
                            Action::Straight { mm, then } => {
                                ui.add(egui::DragValue::new(mm).speed(1.0).suffix(" mm"));
                                end_combo(ui, ("then", i), then, &End::MOVES);
                            }
                            Action::Turn { deg } => {
                                ui.add(egui::DragValue::new(deg).speed(1.0).suffix("°"));
                                ui.weak(if *deg >= 0.0 { "right" } else { "left" });
                            }
                            Action::Curve { radius_mm, deg, then } => {
                                ui.weak("r");
                                ui.add(egui::DragValue::new(radius_mm).speed(1.0).suffix(" mm"));
                                ui.add(egui::DragValue::new(deg).speed(1.0).suffix("°"));
                                end_combo(ui, ("then", i), then, &End::MOVES);
                            }
                            Action::Stop { then, wait_ms } => {
                                end_combo(ui, ("then", i), then, &End::STOPS);
                                ui.weak("wait");
                                ui.add(egui::DragValue::new(wait_ms).speed(10.0).range(0.0..=600000.0).suffix(" ms"));
                            }
                            Action::Custom { code } => {
                                if !functions.is_empty() {
                                    egui::ComboBox::from_id_salt(("call", i)).selected_text("call…").show_ui(ui, |ui| {
                                        for f in &functions {
                                            if ui.selectable_label(false, f).clicked() {
                                                *code = format!("{f}()");
                                            }
                                        }
                                    });
                                }
                                ui.add(egui::TextEdit::singleline(code).hint_text("line_follow()").desired_width(180.0));
                            }
                        }
                        if ui.small_button("↑").on_hover_text("earlier").clicked() {
                            nudge = Some((i, -1));
                        }
                        if ui.small_button("↓").on_hover_text("later").clicked() {
                            nudge = Some((i, 1));
                        }
                        if ui.small_button("×").on_hover_text("remove this action").clicked() {
                            remove = Some(i);
                        }
                    });
                    // a dragged row: a line where it would land, and the drop
                    if let (Some(pointer), Some(hovered)) =
                        (ui.input(|i| i.pointer.interact_pos()), row.response.dnd_hover_payload::<usize>())
                    {
                        let rect = row.response.rect;
                        let insert = if *hovered == i || pointer.y < rect.center().y { i } else { i + 1 };
                        let y = if insert <= i { rect.top() } else { rect.bottom() };
                        ui.painter()
                            .hline(rect.x_range(), y, egui::Stroke::new(2.0, egui::Color32::from_rgb(217, 145, 15)));
                        if let Some(dragged) = row.response.dnd_release_payload::<usize>() {
                            from_to = Some((*dragged, insert));
                        }
                    }
                    let end = step.end;
                    let where_to = if matches!(action, Action::Straight { .. } | Action::Curve { .. } | Action::Turn { .. }) {
                        format!(" → ({}, {}) {}°", end.x_mm.round(), end.y_mm.round(), end.yaw_deg.round())
                    } else {
                        String::new()
                    };
                    ui.weak(format!("     {}{}", action.text(), where_to));
                }
            });
        });
        if let Some(i) = select {
            self.selected = Some(i);
        }
        if let Some((from, to)) = from_to {
            self.move_action(from, to);
        } else if let Some((i, d)) = nudge {
            let to = if d < 0 {
                i.saturating_sub(1)
            } else {
                (i + 2).min(self.route.actions.len())
            };
            if (d < 0 && i > 0) || (d > 0 && i + 1 < self.route.actions.len()) {
                self.move_action(i, to);
            }
        } else if let Some(i) = remove {
            self.remove_action(i);
        }
        if !self.route.actions.is_empty() {
            let end = self.end_pose();
            ui.weak(format!(
                "{} mm in all · ends at ({}, {}) heading {}°",
                route::length_mm(&self.route).round(),
                end.x_mm.round(),
                end.y_mm.round(),
                end.yaw_deg.round()
            ));
        }
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    !busy && !self.route.actions.is_empty() && self.chassis().is_some(),
                    egui::Button::new("▶ Run route"),
                )
                .on_hover_text("writes the route's program and runs it")
                .clicked()
            {
                self.run_route();
            }
            if ui
                .add_enabled(!self.route.actions.is_empty(), egui::Button::new("Undo last"))
                .clicked()
            {
                self.undo_segment();
            }
            if ui.add_enabled(!self.route.actions.is_empty(), egui::Button::new("Clear")).clicked() {
                self.clear_route();
            }
        });
        ui.checkbox(&mut self.show_definitions, "Definitions (what custom actions call)");
        if self.show_definitions {
            ui.add(
                egui::TextEdit::multiline(&mut self.route.prelude)
                    .code_editor()
                    .desired_rows(6)
                    .desired_width(f32::INFINITY)
                    .hint_text("def line_follow():\n    ...\n\ndef run_until_all_black():\n    ..."),
            );
        }
        ui.checkbox(&mut self.show_program, "show the program");
        if self.show_program {
            match self.route_program() {
                Ok(text) => {
                    let mut t = text;
                    ui.add(
                        egui::TextEdit::multiline(&mut t)
                            .code_editor()
                            .desired_rows(8)
                            .desired_width(f32::INFINITY),
                    );
                }
                Err(e) => {
                    ui.weak(e);
                }
            }
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
    ) -> SimDraw {
        let Some(scene) = self.scene.as_ref() else {
            return SimDraw::default();
        };
        let mut items = Vec::new();
        let mut chassis_items = Vec::new();
        let mut chassis_locals: Vec<(String, Mat4, [f32; 4])> = Vec::new();
        let chassis_id = scene.body_id("chassis");
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
        for (gi, g) in scene.geoms.iter().enumerate() {
            if g.name.starts_with("chassis_brick:") {
                continue; // the exact brick is drawn below
            }
            if g.group >= 3 && !g.name.starts_with("chassis_brick:") {
                continue; // MuJoCo's convention: groups 3+ are helper geometry
            }
            let key = format!("geom:{}:{gi}", self.scene_gen);
            let mat = g.material.as_ref().and_then(|m| scene.materials.get(m));
            // a plane takes its material's texture; everything else a flat colour
            let texture = if g.kind == "plane" {
                mat.and_then(|m| m.texture.clone())
                    .filter(|t| self.textures_loaded.get(t).copied().unwrap_or(false))
                    .map(|t| format!("tex:{t}"))
            } else {
                None
            };
            if !viewport.has_mesh(&key) {
                let size = |k: usize| g.size[k] as f32 * M_TO_MM;
                let mesh = match g.kind.as_str() {
                    "plane" => {
                        let (hx, hy) = if g.size[0] > 0.0 && g.size[1] > 0.0 {
                            (size(0), size(1))
                        } else {
                            (5.0 * M_TO_MM, 5.0 * M_TO_MM)
                        };
                        let rep = mat.map(|m| [m.texrepeat[0] as f32, m.texrepeat[1] as f32]).unwrap_or([1.0, 1.0]);
                        geometry::plane_mesh(hx, hy, rep)
                    }
                    "box" => geometry::box_mesh([2.0 * size(0), 2.0 * size(1), 2.0 * size(2)], [0.0; 3]),
                    "sphere" => geometry::sphere_mesh(size(0), [0.0; 3], 24, 16),
                    "cylinder" => geometry::cylinder_mesh(size(0), 2.0 * size(1), "z", [0.0; 3], 32),
                    "capsule" => geometry::capsule_mesh(size(0), size(1), 24),
                    "ellipsoid" => {
                        let mut m = geometry::sphere_mesh(1.0, [0.0; 3], 24, 16);
                        for p in m.positions.iter_mut() {
                            p[0] *= size(0);
                            p[1] *= size(1);
                            p[2] *= size(2);
                        }
                        m
                    }
                    "mesh" => {
                        // the asset as the server packed it, in mm, already posed like MuJoCo draws it
                        let decoded = g.mesh.as_ref().and_then(|name| scene.meshes.get(name)).map(|rec| rec.decode());
                        match decoded {
                            Some(Ok(m)) => m,
                            Some(Err(e)) => {
                                self.log
                                    .push(("server".into(), format!("mesh {}: {e}", g.mesh.clone().unwrap_or_default())));
                                continue;
                            }
                            None => continue,
                        }
                    }
                    _ => continue,
                };
                viewport.add_mesh(device, &key, &mesh);
            }
            let rgba = match mat {
                Some(m) if g.rgba == [0.5, 0.5, 0.5, 1.0] => m.rgba,
                _ => g.rgba,
            };
            let color = if texture.is_some() {
                [1.0, 1.0, 1.0, 1.0]
            } else {
                [
                    srgb_to_linear(rgba[0] as f32),
                    srgb_to_linear(rgba[1] as f32),
                    srgb_to_linear(rgba[2] as f32),
                    rgba[3] as f32,
                ]
            };
            let local = Mat4::from_rotation_translation(
                Quat::from_xyzw(g.quat[1] as f32, g.quat[2] as f32, g.quat[3] as f32, g.quat[0] as f32).normalize(),
                Vec3::new(g.pos[0] as f32, g.pos[1] as f32, g.pos[2] as f32) * M_TO_MM,
            );
            if Some(g.body) == chassis_id {
                chassis_items.push(items.len());
                chassis_locals.push((key.clone(), local, color));
            }
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
                let local = Mat4::from_rotation_translation(q, pos);
                let color = crate::app::cat_color(&category, dark);
                chassis_items.push(items.len());
                chassis_locals.push((key.clone(), local, color));
                items.push(DrawItem {
                    mesh: key,
                    model: body * local,
                    color,
                    texture: None,
                });
            }
        }
        // the ghost: the chassis, 70 % transparent, where a drag would put it
        let mut ghost = Vec::new();
        if let (Some(g), Some(cid)) = (self.ghost, chassis_id) {
            let z = self.poses.get(cid).map(|p| p.pos[2] as f32 * M_TO_MM).unwrap_or(0.0);
            let body = Mat4::from_rotation_translation(
                Quat::from_rotation_z(g.yaw_deg.to_radians() as f32),
                Vec3::new(g.x_mm as f32, g.y_mm as f32, z),
            );
            for (mesh, local, color) in &chassis_locals {
                ghost.push(DrawItem {
                    mesh: mesh.clone(),
                    model: body * *local,
                    color: [color[0], color[1], color[2], GHOST_ALPHA],
                    texture: None,
                });
            }
        }
        self.chassis_items = chassis_items;
        self.chassis_locals = chassis_locals;
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
        SimDraw { items, lines, ghost }
    }

    /// Frame the whole map again at the next frame.
    pub fn refit(&mut self) {
        self.fit_pending = true;
    }

    #[cfg(test)]
    pub fn fit_is_pending(&self) -> bool {
        self.fit_pending
    }

    /// One line on what the view is waiting for: the status, and the
    /// last message when there is one.
    pub fn status_line(&self) -> String {
        if self.message.is_empty() {
            self.status.clone()
        } else {
            format!("{} — {}", self.status, self.message)
        }
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

/// A pose the renderer can use: a zero or non-finite quaternion becomes
/// identity, a non-finite position the origin.
fn sane_pose(p: Pose) -> Pose {
    let finite = |v: &[f64]| v.iter().all(|x| x.is_finite());
    let len2: f64 = p.quat.iter().map(|x| x * x).sum();
    Pose {
        pos: if finite(&p.pos) { p.pos } else { [0.0; 3] },
        quat: if finite(&p.quat) && len2 > 1e-12 {
            p.quat
        } else {
            [1.0, 0.0, 0.0, 0.0]
        },
    }
}

/// A small combo for a move's or a stop's end state.
fn end_combo(ui: &mut egui::Ui, salt: (&str, usize), then: &mut End, choices: &[End]) {
    egui::ComboBox::from_id_salt(salt)
        .selected_text(then.label())
        .width(84.0)
        .show_ui(ui, |ui| {
            for e in choices {
                ui.selectable_value(then, *e, e.label());
            }
        });
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

    const SCENE_WITH_CHASSIS: &str = r#"{"bodies":["world","chassis"],"geoms":[{"name":"floor","type":"plane","body":0,"size":[1.2,0.9,0.1],"pos":[0,0,0],"quat":[1,0,0,0],"rgba":[1,1,1,1],"material":null,"group":0,"mesh":null}],"materials":{},"textures":{},"bricks":[],"chassis":{"wheel_diameter_mm":86.4,"axle_track_mm":135.0,"spawn":{"x_mm":-547.0,"y_mm":-150.0,"yaw_deg":90.0}},"timestep_ms":1}"#;

    fn frame_with_chassis(x_m: f64, y_m: f64, yaw_deg: f64) -> Event {
        let h = yaw_deg.to_radians() / 2.0;
        Event::Frame {
            t_ms: 1,
            poses: vec![
                Pose::default(),
                Pose {
                    pos: [x_m, y_m, 0.05],
                    quat: [h.cos(), 0.0, 0.0, h.sin()],
                },
            ],
        }
    }

    #[test]
    fn routes_are_planned_placed_drawn_and_turned_into_a_program() {
        let mut t = SimulateTab::new(None);
        assert!(!t.map_click(10.0, 20.0), "nothing armed: the click is not used");
        assert!(t.route_lines(false).is_empty(), "no map: nothing to draw");
        assert_eq!(t.route_program().unwrap_err(), "load a map with a chassis first");
        // the map arrives: the route starts at the chassis's spawn
        t.apply(Event::Scene(Box::new(serde_json::from_str(SCENE_WITH_CHASSIS).unwrap())));
        assert_eq!(
            t.route.start,
            Pose2 {
                x_mm: -547.0,
                y_mm: -150.0,
                yaw_deg: 90.0
            }
        );
        assert_eq!(t.route.world, "practice-line");
        assert_eq!(t.route_program().unwrap_err(), "add an action first");
        assert!(t.route_lines(true).len() >= 3, "the start's arrow");
        // the point tools: a click dead ahead is one straight, a click off to the side an arc
        t.pick = Some(Tool::StraightTo);
        assert!(t.map_click(-547.04, 100.02));
        assert!(t.pick.is_none(), "one point per arming");
        assert_eq!(t.selected, Some(0));
        t.pick = Some(Tool::CurveTo);
        assert!(t.map_click(-447.0, 200.0));
        assert_eq!(
            t.route.actions,
            vec![
                Action::Straight {
                    mm: 250.0,
                    then: End::Coast
                },
                Action::Curve {
                    radius_mm: 100.0,
                    deg: 90.0,
                    then: End::Coast
                }
            ],
            "rounded to 0.1 mm"
        );
        // a curve tool click straight ahead is still a line, and says so
        t.pick = Some(Tool::CurveTo);
        assert!(t.map_click(-300.0, 200.0));
        assert_eq!(t.message, "straight ahead: a line, not an arc");
        assert_eq!(t.route.actions.len(), 3);
        // a click where the robot already is adds nothing
        t.pick = Some(Tool::StraightTo);
        assert!(t.map_click(-300.0, 200.0));
        assert_eq!(t.route.actions.len(), 3);
        // typed actions with their defaults; a stop; a custom call
        t.add_action("turn");
        t.add_action("stop");
        t.add_action("custom");
        assert_eq!(t.selected, Some(5));
        assert_eq!(t.route.actions[3], Action::Turn { deg: 90.0 });
        if let Action::Custom { code } = &mut t.route.actions[5] {
            *code = "line_follow()".into();
        }
        t.route.prelude = "def line_follow():\n    pass\n".into();
        let text = t.route_program().unwrap();
        assert!(text.contains("db.straight(250)") && text.contains("db.curve(100, 90)"), "{text}");
        assert!(
            text.contains("db.turn(90)") && text.contains("db.stop()") && text.contains("line_follow()"),
            "{text}"
        );
        assert!(text.contains("wheel_diameter_mm=86.4, axle_track_mm=135"), "{text}");
        assert!(
            text.find("def line_follow").unwrap() > text.find("DriveBase(").unwrap(),
            "definitions after the setup"
        );
        assert!(
            t.route_lines(false).len() > 30,
            "lines, arcs, handles, the turn's heading and the marks"
        );
        // running writes the program and asks the server to run it
        t.run_route();
        let last = t.sent.last().unwrap().clone();
        assert_eq!(last["cmd"], "run");
        let script = last["script"].as_str().unwrap().to_string();
        assert!(std::fs::read_to_string(&script).unwrap().contains("db.curve(100, 90)"));
        // the list: move, nudge past the ends, remove, undo, clear
        t.move_action(5, 0);
        assert!(matches!(t.route.actions[0], Action::Custom { .. }) && t.selected == Some(0));
        t.move_action(0, 6);
        assert!(matches!(t.route.actions[5], Action::Custom { .. }) && t.selected == Some(5));
        t.move_action(9, 0);
        t.move_action(0, 9);
        assert_eq!(t.route.actions.len(), 6, "out of range: nothing happens");
        t.remove_action(3);
        assert_eq!(t.route.actions.len(), 5);
        assert!(!matches!(t.route.actions[3], Action::Turn { .. }));
        t.remove_action(42);
        assert_eq!(t.route.actions.len(), 5);
        t.undo_segment();
        assert_eq!(t.route.actions.len(), 4);
        t.pick = Some(Tool::CurveTo);
        t.selected = Some(1);
        t.clear_route();
        assert!(t.route.actions.is_empty() && t.pick.is_none() && t.selected.is_none());
        t.run_route();
        assert_eq!(t.message, "add an action first");
        // the chassis as the frames report it
        t.apply(frame_with_chassis(0.1, 0.2, -45.0));
        let p = t.chassis_pose().unwrap();
        assert!(
            (p.x_mm - 100.0).abs() < 1e-6 && (p.y_mm - 200.0).abs() < 1e-6 && (p.yaw_deg + 45.0).abs() < 1e-6,
            "{p:?}"
        );
        // placing it: the route starts there, the server is told, and every later load repeats it
        t.place_chassis(Pose2 {
            x_mm: 10.04,
            y_mm: -20.0,
            yaw_deg: 370.0,
        });
        let placed = Pose2 {
            x_mm: 10.0,
            y_mm: -20.0,
            yaw_deg: 10.0,
        };
        assert_eq!(t.route.start, placed);
        let last = t.sent.last().unwrap().clone();
        assert_eq!(
            (last["cmd"].as_str(), last["x_mm"].as_f64(), last["yaw_deg"].as_f64()),
            (Some("place"), Some(10.0), Some(10.0))
        );
        let n = t.sent.len();
        t.apply(Event::Scene(Box::new(serde_json::from_str(SCENE_WITH_CHASSIS).unwrap())));
        assert_eq!(t.route.start, placed, "a reload keeps the user's placement");
        assert_eq!(t.sent.len(), n + 1);
        assert_eq!(t.sent.last().unwrap()["cmd"], "place");
        // a drag: a ghost follows the pointer while the chassis stays; letting go places it
        let p0 = t.begin_chassis_drag().unwrap();
        assert_eq!(t.ghost, Some(p0));
        t.drag_chassis(Pose2 {
            x_mm: p0.x_mm + 50.0,
            y_mm: p0.y_mm,
            yaw_deg: p0.yaw_deg,
        });
        assert_eq!(t.ghost.unwrap().x_mm, p0.x_mm + 50.0, "the ghost is where the pointer is");
        assert_eq!(t.chassis_pose().unwrap().x_mm, p0.x_mm, "the chassis waits for the drop");
        t.end_chassis_drag();
        assert_eq!(t.route.start.x_mm, p0.x_mm + 50.0);
        assert!(t.ghost.is_none());
        assert_eq!(t.sent.last().unwrap()["cmd"], "place");
        assert!(t.chassis_pose().is_some());
        // not while a program runs
        t.apply(state("running"));
        t.place_chassis(placed);
        assert!(t.message.contains("stop the program"), "{}", t.message);
        assert!(t.begin_chassis_drag().is_none());
        t.apply(state("stopped"));
        // an action's handle dragged on the map: the straight follows along its line, the ghost shows its end
        let start = t.route.start;
        t.add_action("straight");
        t.add_action("stop");
        assert!(!t.begin_handle_drag(7), "no such action");
        assert!(t.begin_handle_drag(0));
        assert_eq!(t.selected, Some(0));
        let (hx, hy) = start.heading();
        assert_eq!(
            t.ghost
                .map(|g| (g.x_mm - start.x_mm - hx * 200.0).abs() < 1e-6 && (g.y_mm - start.y_mm - hy * 200.0).abs() < 1e-6),
            Some(true),
            "{:?}",
            t.ghost
        );
        t.drag_chassis(Pose2 {
            x_mm: 999.0,
            y_mm: 999.0,
            yaw_deg: 0.0,
        });
        assert_ne!(t.ghost.unwrap().x_mm, 999.0, "a handle drag is not a chassis drag");
        t.drag_handle(start.x_mm + hx * 320.0 + hy * 40.0, start.y_mm + hy * 320.0 - hx * 40.0);
        assert_eq!(
            t.route.actions[0],
            Action::Straight {
                mm: 320.0,
                then: End::Coast
            },
            "the distance along the line, the side offset ignored"
        );
        assert!((t.ghost.unwrap().x_mm - start.x_mm - hx * 320.0).abs() < 1e-6);
        t.end_chassis_drag();
        assert!(t.ghost.is_some(), "the chassis drop does not end a handle drag");
        t.end_handle_drag();
        assert!(t.ghost.is_none());
        t.drag_handle(0.0, 0.0);
        assert!(
            matches!(t.route.actions[0], Action::Straight { mm: 320.0, .. }),
            "nothing dragged: nothing changes"
        );
        // the handles on screen: a camera over the map finds the straight's end and misses the stop
        let cam = crate::viewport::Camera {
            target: Vec3::new(start.x_mm as f32, start.y_mm as f32, 0.0),
            pitch: 89.0,
            distance: 1500.0,
            ..Default::default()
        };
        let steps = t.steps();
        let end = route::handle(&steps[0], &t.route.actions[0]).unwrap();
        let sp = cam.project(Vec3::new(end[0] as f32, end[1] as f32, 3.0), 800.0, 600.0).unwrap();
        assert_eq!(t.route_handle_at(&cam, sp.x, sp.y, 800.0, 600.0), Some(0));
        assert_eq!(
            t.route_handle_at(&cam, sp.x + 4.0, sp.y - 3.0, 800.0, 600.0),
            Some(0),
            "within the grab distance"
        );
        assert_eq!(t.route_handle_at(&cam, sp.x + 40.0, sp.y, 800.0, 600.0), None);
        let sp0 = cam
            .project(Vec3::new(start.x_mm as f32, start.y_mm as f32, 3.0), 800.0, 600.0)
            .unwrap();
        assert_eq!(t.route_handle_at(&cam, sp0.x, sp0.y, 800.0, 600.0), None, "the start is no handle");
        t.clear_route();
        // save and load, with a route for another map
        let dir = std::env::temp_dir().join(format!("ob-tab-route-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        t.pick = Some(Tool::StraightTo);
        t.map_click(0.0, 0.0);
        t.save_route(dir.join("a.route.json"));
        assert!(t.message.starts_with("saved the route"), "{}", t.message);
        let mut other = SimulateTab::new(None);
        other.apply(Event::Scene(Box::new(serde_json::from_str(SCENE_WITH_CHASSIS).unwrap())));
        other.world = "wro-2026-senior".into();
        other.load_route(dir.join("a.route.json"));
        assert_eq!(other.route.actions.len(), 2, "a turn and a straight");
        assert_eq!(other.world, "practice-line", "the route's map is chosen");
        assert!(
            other.pending_load || other.message.contains("no Python interpreter"),
            "{}",
            other.message
        );
        other.load_route(dir.join("missing.json"));
        assert!(other.message.starts_with("could not load the route"), "{}", other.message);
        t.save_route(dir.join("no-such-dir").join("x.json"));
        assert!(t.message.starts_with("could not save the route"), "{}", t.message);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The run server's first frame after a load reports every body as
    /// zeros until the physics has run a forward pass; such a frame
    /// must not blank the view.
    #[test]
    fn a_frame_of_zero_quaternions_stands_every_body_upright_at_the_origin() {
        let mut t = SimulateTab::new(None);
        t.apply(Event::Scene(Box::new(serde_json::from_str(SCENE_WITH_CHASSIS).unwrap())));
        t.apply(Event::Frame {
            t_ms: 0,
            poses: vec![
                Pose {
                    pos: [0.0; 3],
                    quat: [0.0; 4],
                },
                Pose {
                    pos: [f64::NAN, 0.0, 0.0],
                    quat: [f64::NAN, 0.0, 0.0, 0.0],
                },
            ],
        });
        assert_eq!(t.poses[0].quat, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(t.poses[1], Pose::default());
        let p = t.chassis_pose().unwrap();
        assert!(p.x_mm == 0.0 && p.yaw_deg == 0.0, "{p:?}");
        // a real pose passes through untouched
        let real = Pose {
            pos: [0.1, 0.2, 0.05],
            quat: [std::f64::consts::FRAC_1_SQRT_2, 0.0, 0.0, std::f64::consts::FRAC_1_SQRT_2],
        };
        assert_eq!(sane_pose(real), real);
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
        // placing the chassis: the server moves it and the next frame shows it there
        t.place_chassis(Pose2 {
            x_mm: 250.0,
            y_mm: -80.0,
            yaw_deg: 30.0,
        });
        assert!(
            pump_until(&mut t, 30, |t| t
                .chassis_pose()
                .map(|p| (p.x_mm - 250.0).abs() < 1e-3 && (p.yaw_deg - 30.0).abs() < 1e-3)
                .unwrap_or(false)),
            "{:?} {:?}",
            t.chassis_pose(),
            t.log
        );
        // a route runs as a program the server receives
        t.pick = Some(Tool::StraightTo);
        t.map_click(250.0, 120.0);
        t.run_route();
        assert!(pump_until(&mut t, 30, |t| t.status == "running"), "{} {}", t.status, t.message);
        assert!(
            t.log.iter().any(|(s, x)| s == "stdout" && x.contains("openbricks-route-")),
            "{:?}",
            t.log
        );
        t.send(serde_json::json!({"cmd": "stop"}));
        assert!(pump_until(&mut t, 30, |t| t.status == "stopped"));
        t.shutdown();
        assert!(t.process.is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Every world the run server ships loads into the tab and draws:
    /// the scene parses, the mat is found, every geom becomes an item,
    /// and the view frames the mat. Needs the real runtime like the
    /// end-to-end test.
    #[test]
    fn every_shipped_world_loads_and_draws() {
        let Some(python) = std::env::var("OPENBRICKS_SIM_PYTHON").ok().filter(|p| !p.is_empty()) else {
            eprintln!("OPENBRICKS_SIM_PYTHON is unset: skipping the shipped-worlds test");
            return;
        };
        let Some((device, queue)) = test_device() else { return };
        let mut vp = Viewport::new(&device, &queue);
        let bundle_path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../openbricks/openbricks_sim/bricks/technic_bundle.json.zlib");
        let bundle = crate::bundle::load_bundle(&bundle_path).expect("the shipped brick bundle");
        let mut t = SimulateTab::new(Some(python));
        t.world = "empty".into();
        t.ensure_loaded();
        assert!(
            pump_until(&mut t, 120, |t| !t.worlds.is_empty() && t.scene.is_some() && t.status == "loaded"),
            "{} / {} / {:?}",
            t.status,
            t.message,
            t.log
        );
        let aliases: Vec<String> = t.worlds.iter().map(|w| w.alias.clone()).filter(|a| a != "empty").collect();
        assert!(aliases.len() >= 6, "{aliases:?}");
        let mut renderer = test_renderer(&device);
        let (w, h) = (400u32, 300u32);
        let mut dark = Vec::new();
        for alias in aliases {
            let generation = t.scene_gen;
            t.world = alias.clone();
            t.reload();
            assert!(
                pump_until(&mut t, 180, |t| t.status == "loaded"
                    && t.scene_gen > generation
                    && t.route.world == alias),
                "{alias}: {} / {} / {:?}",
                t.status,
                t.message,
                t.log
            );
            let scene = t.scene.as_ref().unwrap();
            let n = scene.geoms.len();
            assert!(scene.geoms.iter().any(|g| g.kind == "plane"), "{alias}: a mat");
            let started = Instant::now();
            let draw = t.draw_items(&mut vp, &device, &queue, &bundle, false);
            eprintln!("{alias}: {n} geoms drawn as {} items in {:?}", draw.items.len(), started.elapsed());
            assert_eq!(draw.items.len(), n, "{alias}: every geom is drawn");
            let (lo, hi) = t.frame_target().expect("the view frames the mat");
            let complaints: Vec<&String> = t
                .log
                .iter()
                .filter(|(s, x)| s == "server" && (x.starts_with("texture ") || x.starts_with("mesh broken")))
                .map(|(_, x)| x)
                .collect();
            assert!(complaints.is_empty(), "{alias}: {complaints:?}");
            // seen from above as the Simulate tab shows it: the mat covers the middle of the view
            vp.camera = crate::viewport::Camera::top_down();
            vp.camera.fit_plan(lo, hi, w as f32 / h as f32);
            let lines = t.route_lines(false);
            vp.render(
                &device,
                &queue,
                &mut renderer,
                (w, h),
                &crate::viewport::Scene {
                    items: &draw.items,
                    lines: &lines,
                    ghost: &[],
                    overlay: &[],
                    background: [0.0, 0.0, 0.0, 1.0],
                },
            );
            let (_, _, px) = vp.read_pixels(&device, &queue).unwrap();
            let lit = (0..h)
                .flat_map(|y| (0..w).map(move |x| (x, y)))
                .filter(|&(x, y)| {
                    let i = ((y * w + x) * 4) as usize;
                    px[i] as u32 + px[i + 1] as u32 + px[i + 2] as u32 > 30
                })
                .count();
            eprintln!("{alias}: {lit} of {} pixels lit from above", w * h);
            if let Some(dir) = std::env::var_os("OPENBRICKS_SIM_RENDER_DIR") {
                let path = std::path::Path::new(&dir).join(format!("{alias}.png"));
                image::save_buffer(&path, &px, w, h, image::ColorType::Rgba8).unwrap();
            }
            dark.push((alias.clone(), lit));
        }
        t.shutdown();
        let failed: Vec<&(String, usize)> = dark.iter().filter(|(_, lit)| *lit <= (w * h / 3) as usize).collect();
        assert!(failed.is_empty(), "the mat should fill the view from above: {failed:?}");
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
        // a world with a mesh asset: the frame arrives packed and decodes
        t.world = "wro-2026-senior".into();
        t.reload();
        assert!(
            pump_until(&mut t, 180, |t| t.status == "loaded"
                && t.scene.as_ref().map(|s| !s.meshes.is_empty()).unwrap_or(false)),
            "{} / {} / {:?}",
            t.status,
            t.message,
            t.log
        );
        let scene = t.scene.as_ref().unwrap();
        let frame = scene.meshes.get("mosaic_frame").expect("the mosaic frame");
        assert!(frame.decode().unwrap().indices.len() > 300);
        assert!(
            scene
                .geoms
                .iter()
                .any(|g| g.kind == "mesh" && g.mesh.as_deref() == Some("mosaic_frame"))
        );
        // back on the practice map: place the chassis, then drive a planned route and land where it was planned
        t.world = "practice-line".into();
        t.reload();
        assert!(
            pump_until(&mut t, 180, |t| t.status == "loaded" && t.route.world == "practice-line"),
            "{} / {} / {:?} / {:?}",
            t.status,
            t.message,
            t.error,
            t.log
        );
        let start = Pose2 {
            x_mm: -400.0,
            y_mm: -150.0,
            yaw_deg: 90.0,
        };
        t.place_chassis(start);
        assert!(
            pump_until(&mut t, 60, |t| t
                .chassis_pose()
                .map(|p| (p.x_mm + 400.0).abs() < 2.0 && (p.yaw_deg - 90.0).abs() < 1.0)
                .unwrap_or(false)),
            "{:?}",
            t.chassis_pose()
        );
        t.pick = Some(Tool::StraightTo);
        t.map_click(-400.0, 100.0);
        t.pick = Some(Tool::CurveTo);
        t.map_click(-300.0, 200.0);
        // a continuous straight into a held stop, and a custom call, ride along
        t.add_action("straight");
        t.route.actions[2] = Action::Straight {
            mm: 100.0,
            then: End::Continue,
        };
        t.add_action("stop");
        t.route.actions[3] = Action::Stop {
            then: End::Hold,
            wait_ms: 200.0,
        };
        t.add_action("custom");
        t.route.actions[4] = Action::Custom { code: "say_hi()".into() };
        t.route.prelude = "def say_hi():
    print('hi from the route')
"
        .into();
        let planned = route::plan(&t.route);
        let end = planned.last().unwrap().end;
        assert!((end.x_mm + 200.0).abs() < 1e-6 && (end.yaw_deg).abs() < 1e-6, "{end:?}");
        t.run_route();
        assert!(
            pump_until(&mut t, 180, |t| matches!(t.status.as_str(), "finished" | "error")),
            "{} / {} / {:?}",
            t.status,
            t.message,
            t.log
        );
        assert_eq!(t.status, "finished", "{:?} / {:?}", t.error, t.log);
        assert!(t.log.iter().any(|(_, x)| x.contains("hi from the route")), "{:?}", t.log);
        let p = t.chassis_pose().unwrap();
        assert!(
            (p.x_mm - end.x_mm).abs() < 25.0 && (p.y_mm - end.y_mm).abs() < 25.0 && route::wrap_deg(p.yaw_deg - end.yaw_deg).abs() < 10.0,
            "drove to {p:?}, planned {end:?}"
        );
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
            geom("m2", "mesh", 0, 0, Some("unknown"), None),
            geom("m3", "mesh", 0, 0, Some("broken"), None),
            geom("helper", "box", 0, 3, None, None),
            geom("chassis_brick:beam", "box", 1, 3, None, None),
        ]
        .join(",");
        let frame = crate::stl::pack(&[[[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [0.0, 100.0, 0.0]]]);
        let meshes = format!(
            r#""frame":{},"broken":{{"verts":1,"tris":1,"pos":"AAA=","nrm":"AAAA","idx":"AAAAAAAA"}}"#,
            serde_json::to_string(&frame).unwrap()
        );
        let scene_json = format!(
            r#"{{"bodies":["world","chassis"],"geoms":[{geoms}],"materials":{{"mat":{{"rgba":[0.2,0.3,0.4,1],"texture":"tex","texrepeat":[2,2]}},"plain":{{"rgba":[0.9,0.1,0.1,1],"texture":null,"texrepeat":[1,1]}}}},"textures":{{"tex":"{}"}},"meshes":{{{meshes}}},"bricks":[{{"path":"beam","part":"lego_32278","ldraw":"32278","pos_m":[0.01,0,0.02],"quat":[1,0,0,0],"half_m":[0.06,0.004,0.004],"category":"lego"}},{{"path":"servo","part":"servo","ldraw":null,"pos_m":[0,0,0],"quat":[1,0,0,0],"half_m":[0.01,0.01,0.01],"category":"servo"}},{{"path":"ghost","part":"missing","ldraw":null,"pos_m":[0,0,0],"quat":[1,0,0,0],"half_m":[0.01,0.01,0.01],"category":""}},{{"path":"bare","part":"bare","ldraw":null,"pos_m":[0,0,0],"quat":[1,0,0,0],"half_m":[0.01,0.01,0.01],"category":""}}],"timestep_ms":1}}"#,
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
        let draw = t.draw_items(&mut vp, &device, &queue, &bundle, false);
        let (items, lines) = (draw.items, draw.lines);
        assert!(draw.ghost.is_empty(), "no drag: no ghost");
        // six primitives and the mesh asset drawn; the unknown and broken meshes, the helper and the brick stand-in skipped; two of four bricks drawable
        assert_eq!(items.len(), 9, "{:?}", items.iter().map(|i| i.mesh.clone()).collect::<Vec<_>>());
        assert!(
            t.log.iter().any(|(s, x)| s == "server" && x.starts_with("mesh broken:")),
            "{:?}",
            t.log
        );
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
        // the chassis body's geoms (cylinder, capsule, ellipsoid) and its two drawable bricks
        assert_eq!(t.chassis_items.len(), 5, "{:?}", t.chassis_items);
        assert!(t.chassis_items.iter().all(|i| *i < items.len()));
        // while dragged, a translucent copy of the chassis stands at the ghost pose; the chassis itself stays
        t.ghost = Some(Pose2 {
            x_mm: 123.0,
            y_mm: -45.0,
            yaw_deg: 0.0,
        });
        let dragged = t.draw_items(&mut vp, &device, &queue, &bundle, false);
        assert_eq!(dragged.ghost.len(), t.chassis_items.len(), "every chassis item has a ghost");
        let last = *t.chassis_items.last().unwrap();
        let brick = items[last].model.w_axis;
        let ghost = dragged.ghost.last().unwrap();
        assert_eq!(ghost.mesh, items[last].mesh);
        // the chassis body sits at the origin in this frame, so the ghost's brick is the same brick moved to (123, -45)
        assert!(
            (ghost.model.w_axis.x - brick.x - 123.0).abs() < 1e-3 && (ghost.model.w_axis.y - brick.y + 45.0).abs() < 1e-3,
            "{:?} vs {brick:?}",
            ghost.model.w_axis
        );
        assert!((ghost.color[3] - GHOST_ALPHA).abs() < 1e-6 && ghost.color[..3] == items[last].color[..3]);
        assert_eq!(dragged.items[last].model.w_axis, brick, "the chassis does not move with the ghost");
        t.ghost = None;
        let mut renderer = test_renderer(&device);
        vp.render(
            &device,
            &queue,
            &mut renderer,
            (64, 48),
            &crate::viewport::Scene {
                items: &items,
                lines: &lines,
                ghost: &dragged.ghost,
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
        let again = t.draw_items(&mut vp, &device, &queue, &bundle, true).items;
        assert_eq!(again.len(), 9);
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
