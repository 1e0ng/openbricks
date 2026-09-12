//! The Simulate tab: a map, the chassis assembled in the workbench and
//! the program; run, pause, stop; the run shown live from the poses
//! the run server streams.

use crate::assembly::Document;
use crate::bundle::Bundle;
use crate::geometry;
use crate::markers::{Marker, Markers};
use crate::route::{self, Action, End, Handle, KINDS, Point, Pose2, Route};
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

/// A tool armed on the map: the kind being placed and the clicks so
/// far; `for_end` names a custom call whose end point is the next click.
#[derive(Clone, Debug, PartialEq)]
pub struct Placing {
    pub kind: &'static str,
    pub points: Vec<Point>,
    pub for_end: Option<usize>,
}

impl Placing {
    /// What the next click does.
    pub fn hint(&self) -> String {
        if self.for_end.is_some() {
            return "click where the call leaves the robot · Esc keeps it in place".into();
        }
        let ask = KINDS.iter().find(|k| k.0 == self.kind).map(|k| k.2).unwrap_or("");
        let step = match (self.kind, self.points.len()) {
            ("marker", _) => "click where it goes; a name follows",
            ("straight" | "curve", 0) => "click where it starts",
            ("straight", _) | ("curve", 1) => "click where it ends",
            ("curve", _) => "click a point to face at the end",
            ("turn", 0) => "click where it turns",
            ("turn", _) => "click a point to face",
            _ => ask,
        };
        format!("{}: {step} · Esc cancels", self.kind)
    }
}

/// A freshly placed action awaiting its parameters in a popup.
#[derive(Clone, Debug, PartialEq)]
pub struct Draft {
    pub action: Action,
    /// A custom call that moves the robot: its end is asked for next.
    pub moves: bool,
    /// Where the popup opens (screen), when known.
    pub at: Option<egui::Pos2>,
    /// The path's colour, when not the kind's default.
    pub color: Option<[u8; 3]>,
}

/// The arrowhead at the end of a path: its length along the heading and
/// half its width, in mm on the map.
pub const HEAD_MM: f64 = 14.0;
pub const HEAD_HALF_MM: f64 = 8.0;

/// The kind's default path colour, as drawn (sRGB bytes).
pub fn default_color(kind: &str, dark: bool) -> [u8; 3] {
    match (kind, dark) {
        ("straight", true) => [89, 166, 255],
        ("straight", false) => [26, 89, 204],
        ("curve", true) => [102, 230, 128],
        ("curve", false) => [26, 140, 64],
        (_, true) => [255, 191, 64],
        (_, false) => [217, 115, 0],
    }
}

/// How many undo steps are kept.
const HISTORY: usize = 60;
/// How long a click's marker stays on the map.
const CLICK_MARKER_FOR: std::time::Duration = std::time::Duration::from_millis(600);

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
    /// An armed tool: the kind being placed and the map clicks so far.
    pub placing: Option<Placing>,
    /// A popup awaiting the parameters of a freshly placed action.
    pub draft: Option<Draft>,
    /// The pointer on the map, for the rubber band while placing.
    pub hover: Option<Point>,
    /// The last click on the map and when it landed: a marker shows there briefly.
    last_click: Option<(Point, std::time::Instant)>,
    /// The map's markers: named points kept per map on this machine.
    pub markers: Markers,
    /// Where they are kept; tests point this at a scratch directory.
    pub markers_dir: std::path::PathBuf,
    pub selected_marker: Option<usize>,
    /// A freshly placed marker awaiting its name: where, the name so far,
    /// and where its popup opens.
    pub marker_draft: Option<(Point, String, Option<egui::Pos2>)>,
    marker_dragging: Option<usize>,
    /// Where the chassis was put (dragged or typed); applied after every load.
    placed: Option<Pose2>,
    /// A translucent chassis where a drag would put it: under the
    /// pointer, or at a dragged action's end.
    pub ghost: Option<Pose2>,
    /// The action, handle and last map point of a drag in progress.
    dragging: Option<(usize, Handle, Point)>,
    /// The action lit on the map and in the list.
    pub selected: Option<usize>,
    /// Route snapshots for undo, oldest first.
    history: Vec<Route>,
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
            placing: None,
            draft: None,
            hover: None,
            last_click: None,
            markers: Markers::empty(""),
            markers_dir: crate::markers::data_dir(),
            selected_marker: None,
            marker_draft: None,
            marker_dragging: None,
            placed: None,
            ghost: None,
            dragging: None,
            selected: None,
            history: vec![],
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
                    if self.markers.world != self.world {
                        // the markers kept for this map on this machine come back with it
                        self.markers = match Markers::load(&self.markers_dir, &self.world) {
                            Ok(m) => m,
                            Err(e) => {
                                self.message = e;
                                Markers::empty(&self.world)
                            }
                        };
                        self.selected_marker = None;
                    }
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
    /// Keep each curve entering the way the robot arrives at its start —
    /// the previous action's end heading there, else the drive's direction
    /// — so it joins smoothly as the route is edited; a locked curve keeps
    /// its own.
    pub fn sync_curves(&mut self) {
        let mut pose = self.route.start;
        for item in &mut self.route.actions {
            if !item.locked
                && let Action::Curve { start, heading_deg, .. } = &mut item.action
            {
                let (dx, dy) = (start[0] - pose.x_mm, start[1] - pose.y_mm);
                let h = if (dx * dx + dy * dy).sqrt() < 0.5 {
                    pose.yaw_deg
                } else {
                    dy.atan2(dx).to_degrees()
                };
                *heading_deg = (route::wrap_deg(h) * 10.0).round() / 10.0;
            }
            let a = &item.action;
            let (_, _, from) = route::approach(pose, a.start(), a.start_heading());
            pose = Pose2::at(a.end(), a.end_heading(from.yaw_deg));
        }
    }

    /// The heading the robot arrives at `p` with: the last action's end
    /// heading when `p` is that end (the route start's before any action),
    /// else the way the plan's drive there faces.
    pub fn heading_at(&self, p: Point) -> f64 {
        let last = self.steps().last().map(|s| s.end).unwrap_or(self.route.start);
        let (dx, dy) = (p[0] - last.x_mm, p[1] - last.y_mm);
        if (dx * dx + dy * dy).sqrt() < 0.5 {
            last.yaw_deg
        } else {
            dy.atan2(dx).to_degrees()
        }
    }

    pub fn steps(&self) -> Vec<route::Step> {
        route::plan(&self.route)
    }

    /// Keep the route as it is, for undo.
    fn record(&mut self) {
        if self.history.last() != Some(&self.route) {
            self.history.push(self.route.clone());
            if self.history.len() > HISTORY {
                self.history.remove(0);
            }
        }
    }

    /// Back to the route before the last change.
    pub fn undo(&mut self) -> bool {
        let Some(r) = self.history.pop() else {
            self.message = "nothing to undo".into();
            return false;
        };
        self.route = r;
        self.placing = None;
        self.draft = None;
        self.dragging = None;
        self.ghost = None;
        if self.selected.is_some_and(|i| i >= self.route.actions.len()) {
            self.selected = None;
        }
        true
    }

    /// Arm a tool: the next map clicks place an action of this kind.
    pub fn arm(&mut self, kind: &'static str) {
        self.draft = None;
        self.placing = Some(Placing {
            kind,
            points: vec![],
            for_end: None,
        });
    }

    /// A click landed on the map at `p`: a marker shows there for a moment.
    pub fn note_click(&mut self, p: Point) {
        self.last_click = Some((p, std::time::Instant::now()));
    }

    /// Where the click marker is, if the last click is recent enough.
    pub fn click_marker(&self) -> Option<Point> {
        self.click_marker_at(std::time::Instant::now())
    }

    pub fn click_marker_at(&self, now: std::time::Instant) -> Option<Point> {
        let (p, at) = self.last_click?;
        (now.duration_since(at) < CLICK_MARKER_FOR).then_some(p)
    }

    /// Drop the tool and any popup.
    pub fn cancel(&mut self) {
        self.placing = None;
        self.draft = None;
        self.marker_draft = None;
    }

    /// Where a click snaps: the chassis's start, every action's end, and
    /// the map's markers.
    fn snap_targets(&self) -> Vec<Point> {
        let mut v = vec![self.route.start.point()];
        v.extend(self.route.actions.iter().map(|i| i.action.end()));
        v.extend(self.markers.points());
        v
    }

    // ---------------------------------------------------------- markers

    fn save_markers(&mut self) {
        if let Err(e) = self.markers.save(&self.markers_dir) {
            self.message = e;
        }
    }

    /// The named marker in the popup joins the map's markers, selected,
    /// and the map's file is updated.
    pub fn commit_marker_draft(&mut self) {
        let Some((at, name, _)) = self.marker_draft.take() else { return };
        let name = if name.trim().is_empty() {
            self.markers.next_name()
        } else {
            name.trim().to_string()
        };
        self.markers.markers.push(Marker { name, at });
        self.selected_marker = Some(self.markers.markers.len() - 1);
        self.selected = None;
        self.save_markers();
    }

    pub fn rename_marker(&mut self, i: usize, name: &str) {
        if let Some(m) = self.markers.markers.get_mut(i)
            && m.name != name
        {
            m.name = name.to_string();
            self.save_markers();
        }
    }

    pub fn remove_selected_marker(&mut self) {
        if let Some(i) = self.selected_marker.take()
            && i < self.markers.markers.len()
        {
            self.markers.markers.remove(i);
            self.save_markers();
        }
    }

    /// The marker under a screen point, within the handle distance.
    pub fn marker_at(&self, cam: &crate::viewport::Camera, px: f32, py: f32, w: f32, h: f32) -> Option<usize> {
        self.scene.as_ref()?;
        let at = glam::Vec2::new(px, py);
        self.markers
            .markers
            .iter()
            .enumerate()
            .filter_map(|(i, m)| {
                let sp = cam.project(Vec3::new(m.at[0] as f32, m.at[1] as f32, 3.0), w, h)?;
                let d = (sp - at).length();
                (d <= HANDLE_PX).then_some((d, i))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, i)| i)
    }

    pub fn begin_marker_drag(&mut self, i: usize) -> bool {
        if i >= self.markers.markers.len() {
            return false;
        }
        self.marker_dragging = Some(i);
        self.selected_marker = Some(i);
        self.selected = None;
        true
    }

    pub fn drag_marker(&mut self, x_mm: f64, y_mm: f64) {
        if let Some(i) = self.marker_dragging
            && let Some(m) = self.markers.markers.get_mut(i)
        {
            m.at = [(x_mm * 10.0).round() / 10.0, (y_mm * 10.0).round() / 10.0];
        }
    }

    pub fn end_marker_drag(&mut self) {
        if self.marker_dragging.take().is_some() {
            self.save_markers();
        }
    }

    /// A click on the map while a tool is armed: the point joins the
    /// placement (a first click snaps to where the previous action ends,
    /// within `tol_mm`), and once the kind has all its points its popup
    /// opens. Returns whether the click was used.
    pub fn map_click(&mut self, x_mm: f64, y_mm: f64, tol_mm: f64) -> bool {
        let Some(mut placing) = self.placing.take() else { return false };
        let p = [(x_mm * 10.0).round() / 10.0, (y_mm * 10.0).round() / 10.0];
        if placing.kind == "marker" {
            self.marker_draft = Some((p, self.markers.next_name(), None));
            return true;
        }
        if let Some(i) = placing.for_end {
            if i < self.route.actions.len() {
                self.record();
                self.route.actions[i].action.drag(Handle::End, p);
                self.selected = Some(i);
            }
            return true;
        }
        let p = if placing.points.is_empty() {
            route::snap(p, &self.snap_targets(), tol_mm)
        } else {
            p
        };
        placing.points.push(p);
        if placing.points.len() >= route::clicks_needed(placing.kind) {
            self.draft = Some(Draft {
                action: Action::placed(placing.kind, &placing.points, self.heading_at(placing.points[0])),
                moves: false,
                at: None,
                color: None,
            });
        } else {
            self.placing = Some(placing);
        }
        true
    }

    /// The popup's action joins the route, selected; a custom call that
    /// moves the robot then asks for its end point.
    pub fn commit_draft(&mut self) {
        let Some(draft) = self.draft.take() else { return };
        self.record();
        let mut item: route::Item = draft.action.into();
        item.color = draft.color;
        self.route.actions.push(item);
        let i = self.route.actions.len() - 1;
        self.selected = Some(i);
        if draft.moves {
            self.placing = Some(Placing {
                kind: "custom",
                points: vec![],
                for_end: Some(i),
            });
        }
    }

    /// Select the action whose path is nearest a map point, within
    /// `tol_mm`; nothing there clears the selection. Returns whether one
    /// was hit.
    pub fn select_at(&mut self, p: Point, tol_mm: f64) -> bool {
        let action = self
            .route
            .actions
            .iter()
            .enumerate()
            .map(|(i, item)| {
                let d = route::distance_to_path(p, &item.action.path()).min(
                    item.action
                        .handles()
                        .iter()
                        .map(|(_, h)| ((h[0] - p[0]).powi(2) + (h[1] - p[1]).powi(2)).sqrt())
                        .fold(f64::INFINITY, f64::min),
                );
                // a mark (stop, turn, call) wins over a path that ends on it: distances
                // within a tenth of a millimetre count as the same
                (d, (d * 10.0).round() as i64, item.action.path().len() > 1, i)
            })
            .filter(|(d, _, _, _)| *d <= tol_mm)
            .min_by(|a, b| a.1.cmp(&b.1).then(a.2.cmp(&b.2)))
            .map(|(d, _, _, i)| (d, i));
        let marker = self.markers.nearest(p, tol_mm);
        match (marker, action) {
            (Some((mi, dm)), a) if a.is_none_or(|(da, _)| dm <= da) => {
                self.selected_marker = Some(mi);
                self.selected = None;
                true
            }
            (_, Some((_, ai))) => {
                self.selected = Some(ai);
                self.selected_marker = None;
                true
            }
            _ => {
                self.selected = None;
                self.selected_marker = None;
                false
            }
        }
    }

    pub fn remove_action(&mut self, i: usize) {
        if i < self.route.actions.len() {
            if self.route.actions[i].locked {
                self.message = format!("action {} is locked: unlock it first", i + 1);
                return;
            }
            self.record();
            self.route.actions.remove(i);
            self.selected = None;
        }
    }

    /// Remove the selected action (a locked one stays).
    pub fn remove_selected(&mut self) {
        if let Some(i) = self.selected {
            self.remove_action(i);
        }
    }

    /// Move the action at `from` so it sits at `to` (an insertion index
    /// in the list as it was).
    pub fn move_action(&mut self, from: usize, to: usize) {
        let n = self.route.actions.len();
        if from >= n || to > n {
            return;
        }
        self.record();
        let a = self.route.actions.remove(from);
        let at = if to > from { to - 1 } else { to };
        self.route.actions.insert(at, a);
        self.selected = Some(at);
    }

    /// Lock or unlock the selected action against edits on the map.
    pub fn set_locked(&mut self, locked: bool) {
        if let Some(i) = self.selected
            && let Some(item) = self.route.actions.get_mut(i)
            && item.locked != locked
        {
            self.record();
            self.route.actions[i].locked = locked;
        }
    }

    pub fn is_locked(&self, i: usize) -> bool {
        self.route.actions.get(i).is_some_and(|a| a.locked)
    }

    /// The selected action as clipboard text.
    pub fn copy_selected(&self) -> Option<String> {
        let item = self.route.actions.get(self.selected?)?;
        let v = serde_json::json!({"format": route::CLIP_FORMAT, "actions": [item]});
        serde_json::to_string(&v).ok()
    }

    /// Actions from clipboard text join the route after the selection
    /// (or at the end), a little to the side and unlocked; the last one
    /// is selected. Returns how many were pasted.
    pub fn paste(&mut self, text: &str) -> usize {
        let Ok(v) = serde_json::from_str::<serde_json::Value>(text) else {
            return 0;
        };
        if v.get("format").and_then(|f| f.as_str()) != Some(route::CLIP_FORMAT) {
            return 0;
        }
        let Ok(items) = serde_json::from_value::<Vec<route::Item>>(v.get("actions").cloned().unwrap_or_default()) else {
            return 0;
        };
        if items.is_empty() {
            return 0;
        }
        self.record();
        let mut at = self
            .selected
            .map(|i| i + 1)
            .unwrap_or(self.route.actions.len())
            .min(self.route.actions.len());
        let n = items.len();
        for mut item in items {
            item.action.translate(route::PASTE_OFFSET_MM, route::PASTE_OFFSET_MM);
            item.locked = false;
            self.route.actions.insert(at, item);
            at += 1;
        }
        self.selected = Some(at - 1);
        n
    }

    pub fn clear_route(&mut self) {
        self.record();
        self.route.actions.clear();
        self.placing = None;
        self.draft = None;
        self.selected = None;
    }

    /// The handle under a screen point: the selected action's handles
    /// first, then any unlocked action's path (dragged as a whole).
    pub fn route_handle_at(&self, cam: &crate::viewport::Camera, px: f32, py: f32, w: f32, h: f32) -> Option<(usize, Handle)> {
        self.scene.as_ref()?;
        let at = glam::Vec2::new(px, py);
        let on_screen = |p: Point| cam.project(Vec3::new(p[0] as f32, p[1] as f32, 3.0), w, h);
        if let Some(i) = self.selected
            && let Some(item) = self.route.actions.get(i)
            && !item.locked
        {
            let mut best: Option<(f32, Handle)> = None;
            for (handle, hp) in item.action.handles() {
                let Some(sp) = on_screen(hp) else { continue };
                let d = (sp - at).length();
                if d <= HANDLE_PX && best.map(|b| d < b.0).unwrap_or(true) {
                    best = Some((d, handle));
                }
            }
            if let Some((_, handle)) = best {
                return Some((i, handle));
            }
        }
        let (o, d) = cam.ray(px, py, w, h);
        let hit = crate::viewport::ray_plane_z(o, d, 0.0)?;
        let p = [hit.x as f64, hit.y as f64];
        let tol = (HANDLE_PX * cam.units_per_px(h)) as f64;
        self.route
            .actions
            .iter()
            .enumerate()
            .filter(|(_, item)| !item.locked)
            .map(|(i, item)| {
                let d = route::distance_to_path(p, &item.action.path());
                (d, (d * 10.0).round() as i64, item.action.path().len() > 1, i)
            })
            .filter(|(d, _, _, _)| *d <= tol)
            .min_by(|a, b| a.1.cmp(&b.1).then(a.2.cmp(&b.2)))
            .map(|(_, _, _, i)| (i, Handle::Body))
    }

    /// Start dragging an action's handle from a map point: the ghost
    /// shows where the robot ends up after it. Locked actions refuse.
    pub fn begin_handle_drag(&mut self, i: usize, handle: Handle, at: Point) -> bool {
        if i >= self.route.actions.len() || self.route.actions[i].locked {
            return false;
        }
        self.record();
        self.dragging = Some((i, handle, at));
        self.selected = Some(i);
        self.ghost = self.steps().get(i).map(|s| s.end);
        true
    }

    /// The dragged handle at a map point: an end moves, a curve bends, a
    /// turn aims, a body shifts whole; the ghost moves to the new end.
    pub fn drag_handle(&mut self, x_mm: f64, y_mm: f64) {
        let Some((i, handle, last)) = self.dragging else { return };
        let Some(item) = self.route.actions.get_mut(i) else { return };
        let p = [x_mm, y_mm];
        if handle == Handle::Body {
            item.action.translate(p[0] - last[0], p[1] - last[1]);
            self.dragging = Some((i, handle, p));
        } else {
            item.action.drag(handle, p);
        }
        self.ghost = self.steps().get(i).map(|s| s.end);
    }

    pub fn end_handle_drag(&mut self) {
        self.dragging = None;
        self.ghost = None;
    }

    #[cfg(test)]
    pub fn dragging_index(&self) -> Option<usize> {
        self.dragging.map(|d| d.0)
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
                self.placing = None;
                self.draft = None;
                self.selected = None;
                self.history.clear();
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

    /// What the view says under the pointer.
    pub fn hint(&self) -> String {
        if let Some(p) = &self.placing {
            return p.hint();
        }
        if self.draft.is_some() {
            return "set the action's parameters in the popup".into();
        }
        if self.marker_draft.is_some() {
            return "name the marker in the popup".into();
        }
        if let Some(m) = self.selected_marker.and_then(|i| self.markers.markers.get(i)) {
            return format!("marker {}: drag it · Delete removes it · rename it in the panel", m.name);
        }
        match self.selected {
            Some(i) if self.is_locked(i) => format!("action {} is locked · ⌘⇧L unlocks", i + 1),
            Some(i) => format!(
                "action {}: drag its handles or the path · ⌘C copies · ⌘L locks · Delete removes · ⌘Z undoes",
                i + 1
            ),
            None => "pan: drag · zoom: wheel · F fits the map · click a path to select it · drag the chassis to place it (shift turns it)"
                .into(),
        }
    }

    /// The map's labels, in view pixels: each action's number at its
    /// start (with a lock when it is locked), its key parameters beside
    /// its path, and — while a placement is in progress — the parameters
    /// of what the next click would make, beside the rubber band.
    pub fn route_labels(&self, cam: &crate::viewport::Camera, w: f32, h: f32) -> Vec<(glam::Vec2, String)> {
        if self.scene.is_none() {
            return vec![];
        }
        let on_screen = |p: Point| cam.project(Vec3::new(p[0] as f32, p[1] as f32, 3.0), w, h);
        let mut out = Vec::new();
        for (i, item) in self.route.actions.iter().enumerate() {
            if let Some(sp) = on_screen(item.action.start()) {
                out.push((
                    sp + glam::Vec2::new(8.0, -8.0),
                    format!("{}{}", i + 1, if item.locked { " 🔒" } else { "" }),
                ));
            }
            if let Some(mp) = on_screen(item.action.label_point()) {
                out.push((mp + glam::Vec2::new(8.0, 16.0), item.action.brief()));
            }
        }
        for m in &self.markers.markers {
            if let Some(sp) = on_screen([m.at[0], m.at[1] + 30.0]) {
                out.push((sp + glam::Vec2::new(6.0, -2.0), m.name.clone()));
            }
        }
        if let (Some(p), Some(hover)) = (&self.placing, self.hover) {
            if let Some(i) = p.for_end {
                if let Some(item) = self.route.actions.get(i) {
                    let s = item.action.start();
                    let mid = [(s[0] + hover[0]) / 2.0, (s[1] + hover[1]) / 2.0];
                    let d = ((hover[0] - s[0]).powi(2) + (hover[1] - s[1]).powi(2)).sqrt();
                    if let Some(mp) = on_screen(mid) {
                        out.push((mp + glam::Vec2::new(8.0, 16.0), format!("moves {} mm", d.round())));
                    }
                }
            } else if let Some(first) = p.points.first() {
                let mut pts = p.points.clone();
                pts.push(hover);
                let preview = Action::placed(p.kind, &pts, self.heading_at(*first));
                if let Some(mp) = on_screen(preview.label_point()) {
                    out.push((mp + glam::Vec2::new(8.0, 16.0), preview.brief()));
                }
            }
        }
        out
    }

    /// The route drawn on the map: the start's heading, the drives that
    /// join actions (dashed), each action's path, marks at stops and
    /// calls, the selected one lit with its handles, and the rubber band
    /// of a placement in progress.
    pub fn route_lines(&self, dark: bool) -> Vec<Line> {
        let mut out = Vec::new();
        if self.scene.is_none() {
            return out;
        }
        let z = 3.0;
        let rgb = |c: [u8; 3]| [c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0, 1.0];
        let mark = rgb(default_color("stop", dark));
        let (lit, link_c, locked_c) = if dark {
            ([1.0, 1.0, 1.0, 1.0], [0.6, 0.6, 0.65, 1.0], [0.55, 0.55, 0.6, 1.0])
        } else {
            ([0.9, 0.1, 0.1, 1.0], [0.45, 0.45, 0.5, 1.0], [0.5, 0.5, 0.55, 1.0])
        };
        let seg = |out: &mut Vec<Line>, a: Point, b: Point, color: [f32; 4], z: f32| {
            out.push(Line {
                a: Vec3::new(a[0] as f32, a[1] as f32, z),
                b: Vec3::new(b[0] as f32, b[1] as f32, z),
                color,
            });
        };
        let dashed = |out: &mut Vec<Line>, a: Point, b: Point, color: [f32; 4]| {
            let d = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
            let n = ((d / 20.0).ceil() as usize).max(1);
            for k in (0..n).step_by(2) {
                let t0 = k as f64 / n as f64;
                let t1 = ((k + 1) as f64 / n as f64).min(1.0);
                seg(
                    out,
                    [a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0],
                    [a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1],
                    color,
                    z,
                );
            }
        };
        let square = |out: &mut Vec<Line>, p: Point, r: f64, color: [f32; 4]| {
            let c = [(-r, -r), (r, -r), (r, r), (-r, r)];
            for k in 0..4 {
                seg(
                    out,
                    [p[0] + c[k].0, p[1] + c[k].1],
                    [p[0] + c[(k + 1) % 4].0, p[1] + c[(k + 1) % 4].1],
                    color,
                    z + 0.5,
                );
            }
        };
        let diamond = |out: &mut Vec<Line>, p: Point, d: f64, color: [f32; 4]| {
            let c = [(0.0, -d), (d, 0.0), (0.0, d), (-d, 0.0)];
            for k in 0..4 {
                seg(
                    out,
                    [p[0] + c[k].0, p[1] + c[k].1],
                    [p[0] + c[(k + 1) % 4].0, p[1] + c[(k + 1) % 4].1],
                    color,
                    z + 0.5,
                );
            }
        };
        // an arrowhead: the robot's heading at a point, as a closed triangle with its tip there
        let head = |out: &mut Vec<Line>, tip: Point, heading_deg: f64, color: [f32; 4]| {
            let (c, s) = (heading_deg.to_radians().cos(), heading_deg.to_radians().sin());
            let back = [tip[0] - c * HEAD_MM, tip[1] - s * HEAD_MM];
            let l = [back[0] - s * HEAD_HALF_MM, back[1] + c * HEAD_HALF_MM];
            let r = [back[0] + s * HEAD_HALF_MM, back[1] - c * HEAD_HALF_MM];
            seg(out, tip, l, color, z + 0.5);
            seg(out, tip, r, color, z + 0.5);
            seg(out, l, r, color, z + 0.5);
        };
        let arrow = |out: &mut Vec<Line>, from: Point, heading_deg: f64, len: f64, color: [f32; 4]| {
            let (c, s) = (heading_deg.to_radians().cos(), heading_deg.to_radians().sin());
            let tip = [from[0] + c * len, from[1] + s * len];
            seg(out, from, tip, color, z);
            head(out, tip, heading_deg, color);
        };
        // the map's markers first: a flag on a pole, the selected one lit with a handle; the
        // route is drawn after them and so lies over them
        let marker_c = if dark { [0.85, 0.6, 1.0, 1.0] } else { [0.5, 0.2, 0.75, 1.0] };
        for (i, m) in self.markers.markers.iter().enumerate() {
            let on = self.selected_marker == Some(i);
            let c = if on { lit } else { marker_c };
            let p = m.at;
            seg(&mut out, p, [p[0], p[1] + 30.0], c, z + 0.5);
            seg(&mut out, [p[0], p[1] + 30.0], [p[0] + 18.0, p[1] + 24.0], c, z + 0.5);
            seg(&mut out, [p[0] + 18.0, p[1] + 24.0], [p[0], p[1] + 18.0], c, z + 0.5);
            diamond(&mut out, p, 5.0, c);
            if on {
                square(&mut out, p, 9.0, lit);
            }
        }
        let steps = self.steps();
        for (i, (step, item)) in steps.iter().zip(&self.route.actions).enumerate() {
            let on = self.selected == Some(i);
            let a = &item.action;
            let color = if on {
                lit
            } else if item.locked {
                locked_c
            } else {
                rgb(item.color.unwrap_or_else(|| default_color(a.kind(), dark)))
            };
            if step.link.len() == 2 {
                dashed(&mut out, step.link[0], step.link[1], link_c);
            }
            for w in step.points.windows(2) {
                seg(&mut out, w[0], w[1], color, z);
            }
            // where a path ends, the way the robot faces there
            if step.points.len() >= 2 {
                head(&mut out, step.end.point(), step.end.yaw_deg, color);
            }
            match a {
                Action::Turn { at, heading_deg, .. } => arrow(&mut out, *at, *heading_deg, route::TURN_HANDLE_MM, color),
                Action::Stop { at, .. } => diamond(&mut out, *at, 8.0, color),
                Action::Custom { at, .. } => {
                    square(&mut out, *at, 6.0, color);
                    diamond(&mut out, *at, 9.0, color);
                }
                _ => {}
            }
            if on {
                for (_, hp) in a.handles() {
                    square(&mut out, hp, 9.0, lit);
                }
            } else if !item.locked {
                square(&mut out, a.end(), 4.0, color);
            }
        }
        let circle = |out: &mut Vec<Line>, p: Point, r: f64, color: [f32; 4]| {
            for k in 0..8 {
                let (a0, a1) = (k as f64 * std::f64::consts::FRAC_PI_4, (k + 1) as f64 * std::f64::consts::FRAC_PI_4);
                seg(
                    out,
                    [p[0] + r * a0.cos(), p[1] + r * a0.sin()],
                    [p[0] + r * a1.cos(), p[1] + r * a1.sin()],
                    color,
                    z + 0.5,
                );
            }
        };
        let crosshair = |out: &mut Vec<Line>, p: Point, r: f64, color: [f32; 4]| {
            seg(out, [p[0] - r, p[1]], [p[0] + r, p[1]], color, z + 0.5);
            seg(out, [p[0], p[1] - r], [p[0], p[1] + r], color, z + 0.5);
        };
        // a placement in progress: a marker on every point clicked so far, and what the next
        // click would make, following the pointer
        if let Some(p) = &self.placing {
            for pt in &p.points {
                crosshair(&mut out, *pt, 14.0, mark);
                circle(&mut out, *pt, 8.0, mark);
            }
            if let Some(hover) = self.hover {
                if let Some(i) = p.for_end {
                    if let Some(item) = self.route.actions.get(i) {
                        dashed(&mut out, item.action.start(), hover, mark);
                        circle(&mut out, hover, 6.0, mark);
                    }
                } else if let Some(first) = p.points.first() {
                    let mut pts = p.points.clone();
                    pts.push(hover);
                    let preview = Action::placed(p.kind, &pts, self.heading_at(*first));
                    let path = preview.path();
                    for w in path.windows(2) {
                        seg(&mut out, w[0], w[1], mark, z);
                    }
                    if let Action::Turn { at, heading_deg, .. } = &preview {
                        arrow(&mut out, *at, *heading_deg, route::TURN_HANDLE_MM, mark);
                    } else if path.len() >= 2 {
                        head(&mut out, preview.end(), preview.end_heading(0.0), mark);
                    }
                    circle(&mut out, hover, 6.0, mark);
                } else {
                    square(&mut out, hover, 6.0, mark);
                }
            }
        }
        // the last click, wherever it landed
        if let Some(pt) = self.click_marker() {
            circle(&mut out, pt, 16.0, mark);
        }
        // the start: an arrow along the heading
        arrow(&mut out, self.route.start.point(), self.route.start.yaw_deg, 60.0, mark);
        out
    }

    /// The popup for a freshly placed marker: its name, then Add (Enter)
    /// or Cancel.
    pub fn marker_draft_ui(&mut self, ctx: &egui::Context) {
        let Some((at, mut name, pos)) = self.marker_draft.take() else {
            return;
        };
        let mut window = egui::Window::new("New marker").collapsible(false).resizable(false);
        window = match pos {
            Some(p) => window.default_pos(p),
            None => window.anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0)),
        };
        let mut done: Option<bool> = None;
        window.show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.weak("name");
                ui.add(egui::TextEdit::singleline(&mut name).desired_width(140.0)).request_focus();
            });
            ui.weak(format!("at ({}, {}) mm · kept with this map", at[0].round(), at[1].round()));
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui.button("Add").clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    done = Some(true);
                }
                if ui.button("Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        match done {
            Some(true) => {
                self.marker_draft = Some((at, name, pos));
                self.commit_marker_draft();
            }
            Some(false) => {}
            None => self.marker_draft = Some((at, name, pos)),
        }
    }

    /// The popup for a freshly placed action: its parameters, then Add
    /// (Enter) or Cancel.
    pub fn draft_ui(&mut self, ctx: &egui::Context) {
        self.marker_draft_ui(ctx);
        let Some(mut draft) = self.draft.take() else { return };
        let functions = self.route.functions();
        let wheel = self.chassis().map(|c| c.wheel_diameter_mm).unwrap_or(0.0);
        let title = format!("New {}", draft.action.kind());
        let mut window = egui::Window::new(title).collapsible(false).resizable(false);
        if let Some(at) = draft.at {
            window = window.default_pos(at);
        } else {
            window = window.anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0));
        }
        let mut done: Option<bool> = None;
        let dark = ctx.global_style().visuals.dark_mode;
        window.show(ctx, |ui| {
            action_fields(ui, &mut draft.action, &functions, wheel, "draft");
            if let Action::Custom { .. } = draft.action {
                ui.checkbox(&mut draft.moves, "moves the robot (click where it ends next)");
            }
            color_field(ui, &mut draft.color, default_color(draft.action.kind(), dark));
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui.button("Add").clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    done = Some(true);
                }
                if ui.button("Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        match done {
            Some(true) => {
                self.draft = Some(draft);
                self.commit_draft();
            }
            Some(false) => {}
            None => self.draft = Some(draft),
        }
    }

    /// The route panel: the tools, the start, the actions with their
    /// parameters, and what to do with the route.
    pub fn route_ui(&mut self, ui: &mut egui::Ui) {
        self.sync_curves();
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
        let has_map = self.scene.is_some();
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
        ui.strong("Add an action, then click the map");
        ui.horizontal_wrapped(|ui| {
            for (kind, label, ask) in KINDS {
                let armed = self.placing.as_ref().is_some_and(|p| p.kind == kind && p.for_end.is_none());
                if ui
                    .add_enabled(has_map, egui::Button::selectable(armed, label))
                    .on_hover_text(ask)
                    .clicked()
                {
                    if armed {
                        self.cancel();
                    } else {
                        self.arm(kind);
                    }
                }
            }
        });
        if let Some(p) = &self.placing {
            ui.colored_label(egui::Color32::from_rgb(217, 145, 15), p.hint());
        }
        let steps = self.steps();
        let mut remove = None;
        let mut nudge: Option<(usize, i32)> = None;
        let mut from_to: Option<(usize, usize)> = None;
        let mut select = None;
        let mut toggle_lock = None;
        let selected = self.selected;
        let list_id = ui.id().with("actions");
        egui::ScrollArea::vertical().max_height(300.0).id_salt(list_id).show(ui, |ui| {
            let frame = egui::Frame::default().inner_margin(2.0);
            let (_, _dropped) = ui.dnd_drop_zone::<usize, ()>(frame, |ui| {
                for (i, item) in self.route.actions.iter().enumerate() {
                    let step = &steps[i];
                    let on = selected == Some(i);
                    let row = ui.horizontal(|ui| {
                        ui.dnd_drag_source(egui::Id::new(("route-action", i)), i, |ui| {
                            ui.label("≡").on_hover_text("drag to reorder");
                        });
                        if ui.selectable_label(on, format!("{}. {}", i + 1, item.action.kind())).clicked() {
                            select = Some(i);
                        }
                        if ui
                            .small_button(if item.locked { "🔒" } else { "🔓" })
                            .on_hover_text(if item.locked {
                                "locked: click to unlock (⌘⇧L)"
                            } else {
                                "click to lock (⌘L)"
                            })
                            .clicked()
                        {
                            toggle_lock = Some(i);
                        }
                        if ui.small_button("↑").on_hover_text("earlier").clicked() {
                            nudge = Some((i, -1));
                        }
                        if ui.small_button("↓").on_hover_text("later").clicked() {
                            nudge = Some((i, 1));
                        }
                        if ui
                            .add_enabled(!item.locked, egui::Button::new("×").small())
                            .on_hover_text("remove")
                            .clicked()
                        {
                            remove = Some(i);
                        }
                    });
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
                    let link = if step.link.len() == 2 {
                        let d = ((step.link[1][0] - step.link[0][0]).powi(2) + (step.link[1][1] - step.link[0][1]).powi(2)).sqrt();
                        format!(" (drives {} mm to get there)", d.round())
                    } else {
                        String::new()
                    };
                    ui.weak(format!(
                        "     {}{} → ({}, {}) {}°",
                        item.action.text(),
                        link,
                        end.x_mm.round(),
                        end.y_mm.round(),
                        end.yaw_deg.round()
                    ));
                }
            });
        });
        if let Some(i) = select {
            self.selected = Some(i);
        }
        if let Some(i) = toggle_lock {
            let was = self.selected;
            self.selected = Some(i);
            let locked = self.is_locked(i);
            self.set_locked(!locked);
            self.selected = was.or(Some(i));
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
        // the selected action's parameters
        if let Some(i) = self.selected
            && i < self.route.actions.len()
        {
            let functions = self.route.functions();
            let wheel = self.chassis().map(|c| c.wheel_diameter_mm).unwrap_or(0.0);
            let locked = self.route.actions[i].locked;
            ui.add_space(4.0);
            ui.separator();
            ui.strong(format!(
                "Action {}: {}{}",
                i + 1,
                self.route.actions[i].action.kind(),
                if locked { " (locked)" } else { "" }
            ));
            let before = self.route.actions[i].action.clone();
            let mut edited = before.clone();
            let mut color = self.route.actions[i].color;
            let dark = ui.visuals().dark_mode;
            ui.add_enabled_ui(!locked, |ui| {
                action_fields(ui, &mut edited, &functions, wheel, "inspect");
                color_field(ui, &mut color, default_color(edited.kind(), dark));
            });
            if edited != before {
                self.record();
                self.route.actions[i].action = edited;
            }
            if color != self.route.actions[i].color {
                self.record();
                self.route.actions[i].color = color;
            }
            ui.horizontal(|ui| {
                let mut lock = locked;
                if ui.checkbox(&mut lock, "locked").on_hover_text("⌘L / ⌘⇧L").changed() {
                    self.set_locked(lock);
                }
                if ui.button("Duplicate").on_hover_text("⌘C, ⌘V").clicked()
                    && let Some(text) = self.copy_selected()
                {
                    self.paste(&text);
                }
                if ui
                    .add_enabled(!locked, egui::Button::new("Delete"))
                    .on_hover_text("Delete")
                    .clicked()
                {
                    self.remove_selected();
                }
            });
        }
        if !self.route.actions.is_empty() {
            let end = steps.last().map(|s| s.end).unwrap_or(self.route.start);
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
                .add_enabled(!self.history.is_empty(), egui::Button::new("Undo"))
                .on_hover_text("⌘Z")
                .clicked()
            {
                self.undo();
            }
            if ui.add_enabled(!self.route.actions.is_empty(), egui::Button::new("Clear")).clicked() {
                self.clear_route();
            }
        });
        ui.add_space(6.0);
        ui.separator();
        ui.horizontal(|ui| {
            ui.strong("Markers").on_hover_text(format!(
                "named points kept with this map on this machine:\n{}",
                Markers::file(&self.markers_dir, &self.world).display()
            ));
            let armed = self.placing.as_ref().is_some_and(|p| p.kind == "marker");
            if ui
                .add_enabled(has_map, egui::Button::selectable(armed, "◉ Marker"))
                .on_hover_text("click the map where it goes; a name follows")
                .clicked()
            {
                if armed {
                    self.cancel();
                } else {
                    self.arm("marker");
                }
            }
            ui.weak("route clicks snap to them");
        });
        let mut rename: Option<(usize, String)> = None;
        let mut remove_marker = None;
        let mut select_marker = None;
        for (i, m) in self.markers.markers.iter().enumerate() {
            let on = self.selected_marker == Some(i);
            ui.horizontal(|ui| {
                if ui.selectable_label(on, "◉").clicked() {
                    select_marker = Some(i);
                }
                let mut name = m.name.clone();
                if ui.add(egui::TextEdit::singleline(&mut name).desired_width(120.0)).changed() {
                    rename = Some((i, name));
                }
                ui.weak(format!("({}, {}) mm", m.at[0].round(), m.at[1].round()));
                if ui.small_button("×").on_hover_text("remove").clicked() {
                    remove_marker = Some(i);
                }
            });
        }
        if let Some(i) = select_marker {
            self.selected_marker = Some(i);
            self.selected = None;
        }
        if let Some((i, name)) = rename {
            self.rename_marker(i, &name);
        }
        if let Some(i) = remove_marker {
            self.selected_marker = Some(i);
            self.remove_selected_marker();
        }
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
fn end_combo(ui: &mut egui::Ui, scope: (&str, &str), then: &mut End, choices: &[End]) {
    let id = ui.id().with(scope);
    egui::ComboBox::from_id_salt(id)
        .selected_text(then.label())
        .width(84.0)
        .show_ui(ui, |ui| {
            for e in choices {
                ui.selectable_value(then, *e, e.label());
            }
        });
}

/// A speed in wheel degrees per second, with what that is in mm/s on
/// the loaded chassis.
fn speed_field(ui: &mut egui::Ui, label: &str, speed: &mut f64, wheel_mm: f64) {
    ui.horizontal(|ui| {
        ui.weak(label);
        ui.add(egui::DragValue::new(speed).speed(5.0).range(10.0..=2000.0).suffix(" °/s"));
        if wheel_mm > 0.0 {
            ui.weak(format!("≈ {} mm/s", (*speed * std::f64::consts::PI * wheel_mm / 360.0).round()));
        }
    });
}

/// The path's colour: a picker, and a way back to the kind's default.
fn color_field(ui: &mut egui::Ui, color: &mut Option<[u8; 3]>, default: [u8; 3]) {
    ui.horizontal(|ui| {
        ui.weak("colour");
        let mut rgb = color.unwrap_or(default);
        if egui::color_picker::color_edit_button_srgb(ui, &mut rgb).changed() {
            *color = Some(rgb);
        }
        if color.is_some() {
            if ui.small_button("default").on_hover_text("the kind's own colour").clicked() {
                *color = None;
            }
        } else {
            ui.weak("default");
        }
    });
}

/// A move's end: continuous, or one of the stop kinds.
fn end_fields(ui: &mut egui::Ui, then: &mut End, scope: &str) {
    let mut continuous = *then == End::Continue;
    if ui.checkbox(&mut continuous, "continuous (no stop at the end)").changed() {
        *then = if continuous { End::Continue } else { End::Coast };
    }
    if !continuous {
        ui.horizontal(|ui| {
            ui.weak("then");
            end_combo(ui, (scope, "then"), then, &End::STOPS);
        });
    }
}

/// The editable parameters of an action: the popup's and the inspector's.
fn action_fields(ui: &mut egui::Ui, action: &mut Action, functions: &[String], wheel_mm: f64, scope: &str) {
    let brief = action.brief();
    match action {
        Action::Straight { speed, then, .. } => {
            speed_field(ui, "speed", speed, wheel_mm);
            end_fields(ui, then, scope);
        }
        Action::Curve {
            heading_deg,
            end_heading_deg,
            speed,
            then,
            ..
        } => {
            ui.weak(format!(
                "enters facing {}°, ends facing {}° · {brief} · drag the arrow at its end to face elsewhere",
                heading_deg.round(),
                end_heading_deg.round()
            ));
            speed_field(ui, "speed", speed, wheel_mm);
            end_fields(ui, then, scope);
        }
        Action::Turn { heading_deg, speed, .. } => {
            ui.horizontal(|ui| {
                ui.weak("face");
                ui.add(egui::DragValue::new(heading_deg).speed(1.0).suffix("°"));
                ui.weak("(counter-clockwise from +x)");
            });
            speed_field(ui, "turn rate", speed, wheel_mm);
        }
        Action::Stop { then, wait_ms, .. } => {
            ui.horizontal(|ui| {
                ui.weak("then");
                end_combo(ui, (scope, "then"), then, &End::STOPS);
                ui.weak("wait");
                ui.add(egui::DragValue::new(wait_ms).speed(10.0).range(0.0..=600000.0).suffix(" ms"));
            });
        }
        Action::Custom { code, .. } => {
            ui.horizontal(|ui| {
                if !functions.is_empty() {
                    egui::ComboBox::from_id_salt(ui.id().with((scope, "call")))
                        .selected_text("call…")
                        .show_ui(ui, |ui| {
                            for f in functions {
                                if ui.selectable_label(false, f).clicked() {
                                    *code = format!("{f}()");
                                }
                            }
                        });
                }
                ui.add(egui::TextEdit::singleline(code).hint_text("line_follow()").desired_width(200.0));
            });
        }
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
    fn routes_are_placed_by_clicks_edited_copied_locked_and_turned_into_a_program() {
        let mut t = SimulateTab::new(None);
        assert!(!t.map_click(10.0, 20.0, 5.0), "nothing armed: the click is not used");
        assert!(t.route_lines(false).is_empty(), "no map: nothing to draw");
        assert_eq!(t.route_program().unwrap_err(), "load a map with a chassis first");
        // the map arrives: the route starts at the chassis's spawn
        t.apply(Event::Scene(Box::new(serde_json::from_str(SCENE_WITH_CHASSIS).unwrap())));
        assert_eq!(t.route.start, Pose2::at([-547.0, -150.0], 90.0));
        assert_eq!(t.route.world, "practice-line");
        assert_eq!(t.route_program().unwrap_err(), "add an action first");
        assert!(t.route_lines(true).len() >= 3, "the start's arrow");
        assert!(t.hint().starts_with("pan: drag"));
        // a straight: two clicks; the first snaps to the chassis's start; the popup opens; nothing is added until it says so
        t.arm("straight");
        assert!(
            t.placing.as_ref().unwrap().hint().starts_with("straight: click where it starts"),
            "{}",
            t.hint()
        );
        let bare = t.route_lines(false).len();
        t.hover = Some([-300.0, 0.0]);
        assert!(t.route_lines(false).len() > bare, "the pointer is marked before the first click");
        assert!(t.map_click(-540.0, -146.0, 20.0));
        assert_eq!(t.placing.as_ref().unwrap().points, vec![[-547.0, -150.0]], "snapped to the start");
        assert!(t.hint().contains("click where it ends"));
        t.hover = None;
        let marked = t.route_lines(false).len();
        assert!(marked >= bare + 10, "the clicked start is marked (crosshair + circle)");
        t.hover = Some([-547.0, 0.0]);
        assert!(t.route_lines(false).len() > marked + 8, "the rubber band and the pointer's circle");
        let cam = crate::viewport::Camera {
            target: Vec3::new(-400.0, 100.0, 0.0),
            distance: 1500.0,
            ..crate::viewport::Camera::top_down()
        };
        let live: Vec<String> = t.route_labels(&cam, 800.0, 600.0).into_iter().map(|l| l.1).collect();
        assert_eq!(live, vec!["150 mm".to_string()], "the length follows the pointer");
        // every click on the map leaves a marker for a moment
        t.note_click([-547.0, 0.0]);
        let now = std::time::Instant::now();
        assert_eq!(t.click_marker_at(now), Some([-547.0, 0.0]));
        assert_eq!(t.click_marker_at(now + CLICK_MARKER_FOR), None);
        assert!(t.route_lines(false).len() > marked + 16, "the click's ring");
        assert!(t.map_click(-547.04, 100.02, 20.0));
        assert!(t.placing.is_none());
        assert_eq!(t.hint(), "set the action's parameters in the popup");
        assert_eq!(
            t.draft.as_ref().expect("the popup").action,
            Action::Straight {
                start: [-547.0, -150.0],
                end: [-547.0, 100.0],
                speed: 350.0,
                then: End::Coast
            }
        );
        assert!(t.route.actions.is_empty());
        t.commit_draft();
        assert_eq!(t.route.actions.len(), 1);
        assert_eq!(t.selected, Some(0));
        // a curve from the straight's end (snapped) to a point out to the side, then a point to
        // face there: entered the straight's way, facing +x at the end — one right quarter circle
        t.arm("curve");
        assert!(t.map_click(-544.0, 104.0, 20.0) && t.map_click(-447.0, 200.0, 20.0));
        assert!(t.draft.is_none() && t.hint().contains("a point to face"), "{}", t.hint());
        assert!(t.map_click(-347.0, 200.0, 20.0));
        let d = t.draft.clone().unwrap();
        let pieces = d.action.pieces();
        let Action::Curve {
            start,
            heading_deg,
            end,
            end_heading_deg,
            ..
        } = d.action
        else {
            panic!("{d:?}")
        };
        assert_eq!(
            (start, heading_deg, end, end_heading_deg),
            ([-547.0, 100.0], 90.0, [-447.0, 200.0], 0.0)
        );
        assert!(matches!(pieces[..], [route::Piece::Arc { right: true, .. }]), "{pieces:?}");
        assert_eq!(t.heading_at([-547.0, 100.0]), 90.0, "at the last end: its heading");
        assert_eq!(t.heading_at([-447.0, 200.0]), 45.0, "elsewhere: the way the drive there faces");
        t.commit_draft();
        // the curve enters the way the robot arrives: edit the straight and it follows; a locked
        // curve keeps its own
        let entry = |t: &SimulateTab| match t.route.actions[1].action {
            Action::Curve { heading_deg, .. } => heading_deg,
            _ => panic!(),
        };
        t.sync_curves();
        assert_eq!(entry(&t), 90.0);
        t.route.actions[0].action.drag(Handle::Start, [-647.0, -150.0]);
        t.sync_curves();
        assert!((entry(&t) - 68.2).abs() < 0.1, "the way the edited straight leaves: {}", entry(&t));
        t.route.actions[1].locked = true;
        t.route.actions[0].action.drag(Handle::Start, [-547.0, -150.0]);
        t.sync_curves();
        assert!((entry(&t) - 68.2).abs() < 0.1, "a locked curve keeps its heading");
        t.route.actions[1].locked = false;
        t.sync_curves();
        assert_eq!(entry(&t), 90.0);
        // a stop somewhere else (the plan drives there), a turn by two clicks, a custom call that moves
        t.arm("stop");
        assert!(t.map_click(-300.0, 200.0, 20.0));
        t.commit_draft();
        t.arm("turn");
        assert!(t.map_click(-300.0, 200.0, 20.0));
        assert!(t.hint().contains("click a point to face"), "{}", t.hint());
        assert!(t.map_click(-300.0, 300.0, 20.0));
        assert!(matches!(t.draft.as_ref().unwrap().action, Action::Turn { heading_deg, .. } if heading_deg == 90.0));
        t.commit_draft();
        t.arm("custom");
        assert!(t.map_click(-300.0, 200.0, 20.0));
        let mut d = t.draft.take().unwrap();
        d.action = Action::Custom {
            at: [-300.0, 200.0],
            end: None,
            code: "line_follow()".into(),
        };
        d.moves = true;
        t.draft = Some(d);
        t.commit_draft();
        assert_eq!(t.placing.as_ref().map(|p| p.for_end), Some(Some(4)));
        assert!(t.hint().contains("leaves the robot"), "{}", t.hint());
        t.hover = Some([-300.0, 350.0]);
        assert!(t.route_lines(false).len() > 40);
        assert!(t.map_click(-300.0, 400.0, 20.0));
        assert!(t.placing.is_none());
        assert_eq!(t.route.actions[4].action.end(), [-300.0, 400.0]);
        assert_eq!(t.route.actions.len(), 5);
        // the program, with the definitions after the setup
        t.route.prelude = "def line_follow():\n    pass\n".into();
        let text = t.route_program().unwrap();
        assert!(text.contains("db.straight(250)") && text.contains("db.curve(100, 90)"), "{text}");
        assert!(text.contains("db.stop()") && text.contains("line_follow()"), "{text}");
        assert!(text.contains("wheel_diameter_mm=86.4, axle_track_mm=135"), "{text}");
        assert!(text.find("def line_follow").unwrap() > text.find("DriveBase(").unwrap());
        let steps = t.steps();
        assert_eq!(steps[2].link.len(), 2, "the drive to the stop");
        // running writes the program and asks the server to run it
        t.run_route();
        let last = t.sent.last().unwrap().clone();
        assert_eq!(last["cmd"], "run");
        let script = last["script"].as_str().unwrap().to_string();
        assert!(std::fs::read_to_string(&script).unwrap().contains("db.curve(100, 90)"));
        // labels: each action's number at its start and its key parameters beside its path
        let labels: Vec<String> = t.route_labels(&cam, 800.0, 600.0).into_iter().map(|l| l.1).collect();
        assert_eq!(labels.len(), 10);
        assert_eq!(labels[0], "1");
        assert_eq!(labels[1], "250 mm");
        assert_eq!(labels[3], "r 100 mm · 90°");
        assert_eq!(labels[5], "stop");
        assert_eq!(labels[7], "face 90°");
        assert_eq!(labels[9], "line_follow(), 200 mm");
        // selection on the map: the straight by its path, nothing out on the mat
        assert!(t.select_at([-547.0, 0.0], 10.0));
        assert_eq!(t.selected, Some(0));
        assert!(t.hint().starts_with("action 1:"), "{}", t.hint());
        assert!(!t.select_at([0.0, -800.0], 10.0));
        assert_eq!(t.selected, None);
        // copy and paste: the copy lands a little to the side, after the selection, selected
        t.selected = Some(0);
        let clip = t.copy_selected().unwrap();
        assert!(clip.contains(route::CLIP_FORMAT), "{clip}");
        assert_eq!(t.paste(&clip), 1);
        assert_eq!(t.route.actions.len(), 6);
        assert_eq!(t.selected, Some(1));
        assert_eq!(t.route.actions[1].action.start(), [-507.0, -110.0]);
        assert_eq!(t.paste("nonsense"), 0);
        assert_eq!(t.paste(r#"{"format":"openbricks-route-actions/1","actions":[]}"#), 0);
        assert_eq!(t.paste(r#"{"format":"other","actions":[]}"#), 0);
        assert_eq!(t.route.actions.len(), 6);
        // a locked action: shown as such, no drag, no delete; unlocked it goes
        t.set_locked(true);
        assert!(t.is_locked(1) && t.hint().contains("locked"));
        assert_eq!(t.route_labels(&cam, 800.0, 600.0)[2].1, "2 🔒");
        assert!(!t.begin_handle_drag(1, Handle::End, [0.0, 0.0]));
        t.remove_selected();
        assert_eq!(t.route.actions.len(), 6);
        assert!(t.message.contains("locked"), "{}", t.message);
        t.set_locked(false);
        t.remove_selected();
        assert_eq!(t.route.actions.len(), 5);
        assert!(t.selected.is_none());
        // undo brings it back (as it was: unlocked), then undoes the unlock, one change at a time
        assert!(t.undo());
        assert_eq!(t.route.actions.len(), 6);
        assert!(!t.route.actions[1].locked);
        assert!(t.undo());
        assert!(t.route.actions[1].locked, "the unlock is undone next");
        t.selected = Some(1);
        t.set_locked(false);
        // a handle drag: the straight's end along y, the ghost at its new end; a body drag shifts it whole
        assert!(t.begin_handle_drag(0, Handle::End, [-547.0, 100.0]));
        assert_eq!(t.selected, Some(0));
        assert_eq!(t.ghost.map(|g| g.y_mm), Some(100.0));
        t.drag_handle(-547.0, 150.0);
        assert_eq!(t.route.actions[0].action.end(), [-547.0, 150.0]);
        assert_eq!(t.ghost.map(|g| g.y_mm), Some(150.0));
        t.drag_chassis(Pose2::at([999.0, 999.0], 0.0));
        assert_ne!(t.ghost.unwrap().x_mm, 999.0, "a handle drag is not a chassis drag");
        t.end_handle_drag();
        assert!(t.ghost.is_none());
        assert!(t.begin_handle_drag(0, Handle::Body, [-547.0, 0.0]));
        t.drag_handle(-537.0, 10.0);
        assert_eq!(
            (t.route.actions[0].action.start(), t.route.actions[0].action.end()),
            ([-537.0, -140.0], [-537.0, 160.0])
        );
        t.end_handle_drag();
        t.drag_handle(0.0, 0.0);
        assert_eq!(
            t.route.actions[0].action.start(),
            [-537.0, -140.0],
            "nothing dragged: nothing changes"
        );
        assert!(!t.begin_handle_drag(42, Handle::End, [0.0, 0.0]));
        // handles on screen: the selected straight's end is found, a locked action's path is not
        t.selected = Some(0);
        let end = t.route.actions[0].action.end();
        let sp = cam.project(Vec3::new(end[0] as f32, end[1] as f32, 3.0), 800.0, 600.0).unwrap();
        assert_eq!(t.route_handle_at(&cam, sp.x, sp.y, 800.0, 600.0), Some((0, Handle::End)));
        let mid = [-537.0, 10.0];
        let mp = cam.project(Vec3::new(mid[0] as f32, mid[1] as f32, 3.0), 800.0, 600.0).unwrap();
        assert_eq!(t.route_handle_at(&cam, mp.x, mp.y, 800.0, 600.0), Some((0, Handle::Body)));
        t.set_locked(true);
        assert_eq!(t.route_handle_at(&cam, mp.x, mp.y, 800.0, 600.0), None);
        t.set_locked(false);
        assert_eq!(t.route_handle_at(&cam, 5.0, 5.0, 800.0, 600.0), None);
        // the list: move, nudge past the ends, out-of-range no-ops
        t.move_action(5, 0);
        assert!(matches!(t.route.actions[0].action, Action::Custom { .. }) && t.selected == Some(0));
        t.move_action(0, 6);
        assert!(matches!(t.route.actions[5].action, Action::Custom { .. }));
        t.move_action(9, 0);
        t.remove_action(42);
        assert_eq!(t.route.actions.len(), 6);
        // Escape drops a tool or a popup
        t.arm("stop");
        t.cancel();
        assert!(t.placing.is_none() && t.draft.is_none());
        t.clear_route();
        assert!(t.route.actions.is_empty() && t.selected.is_none());
        assert!(t.undo());
        assert_eq!(t.route.actions.len(), 6);
        t.clear_route();
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
        t.place_chassis(Pose2::at([10.04, -20.0], 370.0));
        let placed = Pose2::at([10.0, -20.0], 10.0);
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
        t.drag_chassis(Pose2::at([p0.x_mm + 50.0, p0.y_mm], p0.yaw_deg));
        assert_eq!(t.ghost.unwrap().x_mm, p0.x_mm + 50.0);
        assert_eq!(t.chassis_pose().unwrap().x_mm, p0.x_mm, "the chassis waits for the drop");
        t.end_chassis_drag();
        assert_eq!(t.route.start.x_mm, p0.x_mm + 50.0);
        assert!(t.ghost.is_none());
        assert_eq!(t.sent.last().unwrap()["cmd"], "place");
        // not while a program runs
        t.apply(state("running"));
        t.place_chassis(placed);
        assert!(t.message.contains("stop the program"), "{}", t.message);
        assert!(t.begin_chassis_drag().is_none());
        t.apply(state("stopped"));
        // markers: placed by a click, named in a popup, kept with the map, snapped to, dragged
        let mdir = std::env::temp_dir().join(format!("ob-tab-markers-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&mdir);
        t.markers_dir = mdir.clone();
        t.arm("marker");
        assert!(t.hint().starts_with("marker: click where it goes"), "{}", t.hint());
        assert!(t.map_click(100.0, 100.0, 20.0));
        assert!(t.placing.is_none() && t.draft.is_none());
        assert_eq!(
            t.marker_draft.as_ref().map(|d| (d.0, d.1.clone())),
            Some(([100.0, 100.0], "M1".into()))
        );
        assert_eq!(t.hint(), "name the marker in the popup");
        t.marker_draft.as_mut().unwrap().1 = "  junction ".into();
        t.commit_marker_draft();
        assert_eq!(t.markers.markers.len(), 1);
        assert_eq!(t.markers.markers[0].name, "junction");
        assert_eq!(t.selected_marker, Some(0));
        assert!(Markers::file(&mdir, "practice-line").exists(), "saved with the map");
        assert!(t.hint().starts_with("marker junction:"), "{}", t.hint());
        let lines = t.route_lines(false);
        assert!(lines.len() > 20, "the flag is drawn");
        let lit = [0.9, 0.1, 0.1, 1.0];
        assert_eq!(
            lines[0].color, lit,
            "the selected marker's flag comes first, so the route is drawn over it"
        );
        assert!(lines.iter().skip(4).any(|l| l.color != lit), "and the route follows");
        t.selected_marker = None;
        assert_eq!(t.route_lines(false)[0].color, [0.5, 0.2, 0.75, 1.0], "an unselected flag is purple");
        t.selected_marker = Some(0);
        // a path in a colour of the user's, and back to the kind's default
        t.route
            .actions
            .push(Action::placed("straight", &[[0.0, 0.0], [100.0, 0.0]], 0.0).into());
        assert!(
            t.route_lines(false)
                .iter()
                .any(|l| l.color == [26.0 / 255.0, 89.0 / 255.0, 204.0 / 255.0, 1.0]),
            "the default blue"
        );
        t.route.actions.last_mut().unwrap().color = Some([255, 0, 128]);
        assert!(
            t.route_lines(false).iter().any(|l| l.color == [1.0, 0.0, 128.0 / 255.0, 1.0]),
            "the chosen colour"
        );
        // the end of a path carries an arrowhead the way the robot faces there: for a straight
        // along +x ending at (100, 0), the wings reach back to (86, ±8) in the path's colour
        let rgb_of = |c: [u8; 3]| [c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0, 1.0];
        let pink = rgb_of([255, 0, 128]);
        let near = |v: Vec3, p: [f64; 2]| (v.x as f64 - p[0]).abs() < 1e-3 && (v.y as f64 - p[1]).abs() < 1e-3;
        let wing = |lines: &[Line], tip: [f64; 2], back: [f64; 2], c: [f32; 4]| {
            lines
                .iter()
                .any(|l| l.color == c && ((near(l.a, tip) && near(l.b, back)) || (near(l.a, back) && near(l.b, tip))))
        };
        let lines = t.route_lines(false);
        assert!(wing(&lines, [100.0, 0.0], [86.0, 8.0], pink), "left wing");
        assert!(wing(&lines, [100.0, 0.0], [86.0, -8.0], pink), "right wing");
        assert!(wing(&lines, [86.0, 8.0], [86.0, -8.0], pink), "and the base closes the head");
        // a curve's head follows the arc's end tangent: a quarter circle from (100, 0) to
        // (200, 100) bending left ends heading +y, so the wings reach back to (∓8, 86)
        t.route
            .actions
            .push(Action::placed("curve", &[[100.0, 0.0], [200.0, 100.0]], 0.0).into());
        let lines = t.route_lines(false);
        let green = [26.0 / 255.0, 140.0 / 255.0, 64.0 / 255.0, 1.0];
        let end_heading = t.steps().last().unwrap().end.yaw_deg;
        let (c, s) = (end_heading.to_radians().cos(), end_heading.to_radians().sin());
        let back = [200.0 - c * 14.0, 100.0 - s * 14.0];
        assert!(
            wing(&lines, [200.0, 100.0], [back[0] - s * 8.0, back[1] + c * 8.0], green),
            "the curve's head points along its end heading {end_heading}"
        );
        // a turn shows its heading with the arrow it already has (a shaft and the same closed
        // head) plus the square on its point: no second head
        let was_selected = t.selected.take();
        let before = t.route_lines(false);
        t.route
            .actions
            .push(Action::placed("turn", &[[200.0, 100.0], [200.0, 200.0]], 0.0).into());
        let after = t.route_lines(false);
        let extra: Vec<_> = after.iter().filter(|l| !before.contains(l)).collect();
        assert_eq!(extra.len(), 8, "shaft + 3 head segments + 4 square sides: {extra:?}");
        assert!(
            wing(&after, [200.0, 160.0], [192.0, 146.0], rgb_of([217, 115, 0])),
            "the turn's head at its arrow tip"
        );
        t.route.actions.truncate(t.route.actions.len() - 3);
        t.selected = was_selected;
        assert_eq!(default_color("curve", true), [102, 230, 128]);
        assert_eq!(default_color("turn", false), default_color("custom", false));
        t.route.actions.pop();
        assert!(t.route_labels(&cam, 800.0, 600.0).iter().any(|l| l.1 == "junction"));
        // a route click near it snaps to it
        t.arm("straight");
        assert!(t.map_click(104.0, 97.0, 20.0));
        assert_eq!(t.placing.as_ref().unwrap().points, vec![[100.0, 100.0]]);
        t.cancel();
        // selection: the marker wins over an action at the same spot and clears the action
        // selection; an action away from any marker is selected and clears the marker's
        t.route.actions.push(Action::placed("stop", &[[100.0, 100.0]], 0.0).into());
        t.route.actions.push(Action::placed("stop", &[[-300.0, 200.0]], 0.0).into());
        t.selected = Some(0);
        assert!(t.select_at([101.0, 100.0], 10.0));
        assert_eq!((t.selected_marker, t.selected), (Some(0), None));
        assert!(t.select_at([-300.0, 200.0], 10.0));
        assert_eq!((t.selected_marker, t.selected), (None, Some(1)));
        assert!(!t.select_at([-800.0, -800.0], 10.0));
        assert_eq!((t.selected_marker, t.selected), (None, None));
        t.route.actions.clear();
        // drag, rename, remove — each kept in the file
        assert!(t.begin_marker_drag(0));
        t.drag_marker(150.0, 120.04);
        t.end_marker_drag();
        assert_eq!(t.markers.markers[0].at, [150.0, 120.0]);
        assert!(!t.begin_marker_drag(7));
        t.rename_marker(0, "gate");
        t.rename_marker(0, "gate");
        t.rename_marker(9, "nobody");
        // the popups themselves, driven without a window: the marker's name on Enter
        t.marker_draft = Some(([5.0, 6.0], "".into(), None));
        let ctx = egui::Context::default();
        let mut input = egui::RawInput::default();
        input.events.push(egui::Event::Key {
            key: egui::Key::Enter,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        });
        let _ = ctx.run_ui(input, |ctx| t.draft_ui(ctx));
        assert_eq!(t.markers.markers.len(), 2);
        assert_eq!(
            t.markers.markers[1],
            Marker {
                name: "M1".into(),
                at: [5.0, 6.0]
            },
            "an empty name takes the next free one"
        );
        t.selected_marker = Some(1);
        t.remove_selected_marker();
        // a place that cannot be written names the failure and keeps the marker in memory
        let blocked = mdir.join("blocker");
        std::fs::write(&blocked, "x").unwrap();
        t.markers_dir = blocked.clone();
        t.marker_draft = Some(([7.0, 8.0], "x".into(), None));
        t.commit_marker_draft();
        assert_eq!(t.markers.markers.len(), 2);
        assert!(t.message.contains("could not"), "{}", t.message);
        t.selected_marker = Some(1);
        t.remove_selected_marker();
        t.markers_dir = mdir.clone();
        t.message.clear();
        let mut other = SimulateTab::new(None);
        other.markers_dir = mdir.clone();
        other.world = "practice-line".into();
        other.apply(Event::Scene(Box::new(serde_json::from_str(SCENE_WITH_CHASSIS).unwrap())));
        assert_eq!(
            other.markers.markers,
            vec![Marker {
                name: "gate".into(),
                at: [150.0, 120.0]
            }],
            "back with the map"
        );
        t.selected_marker = Some(0);
        t.remove_selected_marker();
        assert!(t.markers.markers.is_empty() && t.selected_marker.is_none());
        assert!(Markers::load(&mdir, "practice-line").unwrap().markers.is_empty());
        t.remove_selected_marker();
        let _ = std::fs::remove_dir_all(&mdir);
        // save and load, with a route for another map
        let dir = std::env::temp_dir().join(format!("ob-tab-route-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        t.route
            .actions
            .push(Action::placed("straight", &[[0.0, 0.0], [100.0, 0.0]], 0.0).into());
        t.save_route(dir.join("a.route.json"));
        assert!(t.message.starts_with("saved the route"), "{}", t.message);
        let mut other = SimulateTab::new(None);
        other.apply(Event::Scene(Box::new(serde_json::from_str(SCENE_WITH_CHASSIS).unwrap())));
        other.world = "wro-2026-senior".into();
        other.load_route(dir.join("a.route.json"));
        assert_eq!(other.route.actions.len(), 1);
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
        t.route
            .actions
            .push(Action::placed("straight", &[[250.0, -80.0], [250.0, 120.0]], 0.0).into());
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
                    lines: &[],
                    top_lines: &lines,
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
        // a straight north, a right quarter circle flowing (continuous) into a straight east, a held
        // stop, and a custom call: placed on the map, chained end to start
        t.route
            .actions
            .push(Action::placed("straight", &[[-400.0, -150.0], [-400.0, 100.0]], 0.0).into());
        t.route.actions.push(
            Action::Curve {
                start: [-400.0, 100.0],
                heading_deg: 90.0,
                end: [-300.0, 200.0],
                end_heading_deg: 0.0,
                speed: 350.0,
                then: End::Continue,
            }
            .into(),
        );
        t.route
            .actions
            .push(Action::placed("straight", &[[-300.0, 200.0], [-200.0, 200.0]], 0.0).into());
        t.route.actions.push(
            Action::Stop {
                at: [-200.0, 200.0],
                then: End::Hold,
                wait_ms: 200.0,
            }
            .into(),
        );
        t.route.actions.push(
            Action::Custom {
                at: [-200.0, 200.0],
                end: None,
                code: "say_hi()".into(),
            }
            .into(),
        );
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
                lines: &[],
                top_lines: &lines,
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
