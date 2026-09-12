//! Routes on the map: a list of actions placed on the mat — straights,
//! curves, turns, stops and custom calls, each with its own geometry —
//! the drives that join them, and the hub-style program that runs them.
//!
//! Map coordinates are millimetres with the yaw counter-clockwise from
//! +x (MuJoCo's); `DriveBase.turn`/`curve` take angles positive to the
//! RIGHT, so the program's relative angles are the negated heading
//! changes. Speeds are the drive base's: wheel degrees per second.

use serde::{Deserialize, Serialize};
use std::path::Path;

pub const FORMAT: &str = "openbricks-route/3";
const FORMAT_V2: &str = "openbricks-route/2";
const FORMAT_V1: &str = "openbricks-route/1";
/// The clipboard's shape for copied actions.
pub const CLIP_FORMAT: &str = "openbricks-route-actions/1";
/// The drive base's default cruise speeds, wheel degrees per second.
pub const DEFAULT_STRAIGHT_DPS: f64 = 350.0;
pub const DEFAULT_TURN_DPS: f64 = 300.0;
/// How far from a turn its heading handle sits.
pub const TURN_HANDLE_MM: f64 = 60.0;
/// Where a pasted copy lands, relative to the original.
pub const PASTE_OFFSET_MM: f64 = 40.0;
const EPS_MM: f64 = 0.5;

pub type Point = [f64; 2];

/// A pose on the map: mm, and the heading counter-clockwise from +x.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct Pose2 {
    pub x_mm: f64,
    pub y_mm: f64,
    pub yaw_deg: f64,
}

impl Pose2 {
    pub fn at(p: Point, yaw_deg: f64) -> Pose2 {
        Pose2 {
            x_mm: p[0],
            y_mm: p[1],
            yaw_deg,
        }
    }
    pub fn point(&self) -> Point {
        [self.x_mm, self.y_mm]
    }
    pub fn heading(&self) -> (f64, f64) {
        let r = self.yaw_deg.to_radians();
        (r.cos(), r.sin())
    }
}

/// How a move ends: the drive base's stop kinds, or none at all so the
/// next move flows straight on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum End {
    #[default]
    Coast,
    Brake,
    Hold,
    Continue,
}

impl End {
    pub const STOPS: [End; 3] = [End::Coast, End::Brake, End::Hold];

    pub fn label(self) -> &'static str {
        match self {
            End::Coast => "coast",
            End::Brake => "brake",
            End::Hold => "hold",
            End::Continue => "continue",
        }
    }
    /// The `Stop` member the program names.
    pub fn code(self) -> &'static str {
        match self {
            End::Coast => "Stop.COAST",
            End::Brake => "Stop.BRAKE",
            End::Hold => "Stop.HOLD",
            End::Continue => "Stop.NONE",
        }
    }
    /// The keyword argument a move takes (none for the default).
    pub fn arg(self) -> String {
        match self {
            End::Coast => String::new(),
            other => format!(", then={}", other.code()),
        }
    }
    pub fn suffix(self) -> &'static str {
        match self {
            End::Coast => "",
            End::Brake => ", then brake",
            End::Hold => ", then hold",
            End::Continue => ", then continue",
        }
    }
}

fn straight_dps() -> f64 {
    DEFAULT_STRAIGHT_DPS
}
fn turn_dps() -> f64 {
    DEFAULT_TURN_DPS
}
fn is_false(b: &bool) -> bool {
    !*b
}

/// One thing the robot does, placed on the map.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Action {
    /// From `start` to `end`, facing the way it drives.
    Straight {
        start: Point,
        end: Point,
        #[serde(default = "straight_dps")]
        speed: f64,
        #[serde(default)]
        then: End,
    },
    /// The arc of `radius_mm` from `start` to `end`, bending right or left.
    Curve {
        start: Point,
        end: Point,
        radius_mm: f64,
        #[serde(default)]
        right: bool,
        #[serde(default = "straight_dps")]
        speed: f64,
        #[serde(default)]
        then: End,
    },
    /// Turn in place at `at` to face `heading_deg`.
    Turn {
        at: Point,
        heading_deg: f64,
        #[serde(default = "turn_dps")]
        speed: f64,
    },
    /// Stop at `at`, then wait.
    Stop {
        at: Point,
        #[serde(default)]
        then: End,
        #[serde(default)]
        wait_ms: f64,
    },
    /// A call the definitions provide, at `at`; `end` is where it leaves
    /// the robot when it moves it.
    Custom {
        at: Point,
        #[serde(default)]
        end: Option<Point>,
        code: String,
    },
}

/// An action on the map, and whether it is locked against editing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Item {
    #[serde(flatten)]
    pub action: Action,
    #[serde(default, skip_serializing_if = "is_false")]
    pub locked: bool,
}

impl From<Action> for Item {
    fn from(action: Action) -> Self {
        Item { action, locked: false }
    }
}

/// The kinds, their tool labels, and what each asks the user to click.
pub const KINDS: [(&str, &str, &str); 5] = [
    ("straight", "→ Straight", "click where it starts, then where it ends"),
    ("curve", "⌒ Curve", "click where it starts, then where it ends"),
    ("turn", "↻ Turn", "click where it turns, then a point to face"),
    ("stop", "■ Stop", "click where it stops"),
    ("custom", "ƒ Custom", "click where it runs"),
];

/// How many map clicks place a kind.
pub fn clicks_needed(kind: &str) -> usize {
    match kind {
        "straight" | "curve" | "turn" => 2,
        _ => 1,
    }
}

/// A handle on an action: which point a drag moves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Handle {
    Start,
    End,
    /// A curve's midpoint: dragging it bends the arc.
    Mid,
    At,
    /// A turn's heading, `TURN_HANDLE_MM` along it.
    Face,
    /// The whole action.
    Body,
}

/// A curve's geometry, worked out from its two ends, radius and side.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arc {
    pub centre: Point,
    pub radius_mm: f64,
    /// The angle swept, degrees; positive when bending right.
    pub sweep_deg: f64,
    pub start_heading: f64,
    pub end_heading: f64,
}

fn dist(a: Point, b: Point) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

fn heading_to(a: Point, b: Point) -> f64 {
    (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees()
}

/// The arc from `start` to `end` of a radius (no shorter than half the
/// chord, else it is lengthened to that) bending to the right or the
/// left; None when the ends coincide.
pub fn arc_of(start: Point, end: Point, radius_mm: f64, right: bool) -> Option<Arc> {
    let c = dist(start, end);
    if c < EPS_MM {
        return None;
    }
    let r = radius_mm.max(c / 2.0);
    let h = (r * r - c * c / 4.0).max(0.0).sqrt();
    let mid = [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0];
    // the normal to the right of the chord
    let n = [(end[1] - start[1]) / c, -(end[0] - start[0]) / c];
    let side = if right { 1.0 } else { -1.0 };
    let centre = [mid[0] + n[0] * h * side, mid[1] + n[1] * h * side];
    let alpha = (c / (2.0 * r)).min(1.0).asin().to_degrees();
    let chord = heading_to(start, end);
    let (start_heading, end_heading) = if right {
        (wrap_deg(chord + alpha), wrap_deg(chord - alpha))
    } else {
        (wrap_deg(chord - alpha), wrap_deg(chord + alpha))
    };
    Some(Arc {
        centre,
        radius_mm: r,
        sweep_deg: 2.0 * alpha,
        start_heading,
        end_heading,
    })
}

fn arc_points(arc: &Arc, start: Point, right: bool) -> Vec<Point> {
    let a0 = (start[1] - arc.centre[1]).atan2(start[0] - arc.centre[0]);
    let sweep = arc.sweep_deg.to_radians() * if right { -1.0 } else { 1.0 };
    let n = ((arc.sweep_deg / 180.0 * 32.0).ceil() as usize).max(4);
    (0..=n)
        .map(|i| {
            let a = a0 + sweep * i as f64 / n as f64;
            [arc.centre[0] + arc.radius_mm * a.cos(), arc.centre[1] + arc.radius_mm * a.sin()]
        })
        .collect()
}

/// The circle through three points: centre and radius, None when they
/// are in a line.
fn circle_through(a: Point, b: Point, c: Point) -> Option<(Point, f64)> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() < 1e-9 {
        return None;
    }
    let (a2, b2, c2) = (a[0] * a[0] + a[1] * a[1], b[0] * b[0] + b[1] * b[1], c[0] * c[0] + c[1] * c[1]);
    let ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d;
    let uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d;
    Some(([ux, uy], dist([ux, uy], a)))
}

impl Action {
    /// A kind placed by its clicks, with default parameters: a curve
    /// starts as a quarter circle bending right, a turn faces the second
    /// click.
    pub fn placed(kind: &str, points: &[Point]) -> Action {
        let p = |i: usize| points.get(i).copied().unwrap_or([0.0, 0.0]);
        match kind {
            "straight" => Action::Straight {
                start: p(0),
                end: p(1),
                speed: DEFAULT_STRAIGHT_DPS,
                then: End::Coast,
            },
            "curve" => Action::Curve {
                start: p(0),
                end: p(1),
                radius_mm: round1(dist(p(0), p(1)) / 2f64.sqrt()),
                right: true,
                speed: DEFAULT_STRAIGHT_DPS,
                then: End::Coast,
            },
            "turn" => Action::Turn {
                at: p(0),
                heading_deg: round1(if points.len() > 1 { heading_to(p(0), p(1)) } else { 0.0 }),
                speed: DEFAULT_TURN_DPS,
            },
            "stop" => Action::Stop {
                at: p(0),
                then: End::Coast,
                wait_ms: 0.0,
            },
            _ => Action::Custom {
                at: p(0),
                end: None,
                code: String::new(),
            },
        }
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Action::Straight { .. } => "straight",
            Action::Curve { .. } => "curve",
            Action::Turn { .. } => "turn",
            Action::Stop { .. } => "stop",
            Action::Custom { .. } => "custom",
        }
    }

    /// Where it begins.
    pub fn start(&self) -> Point {
        match self {
            Action::Straight { start, .. } | Action::Curve { start, .. } => *start,
            Action::Turn { at, .. } | Action::Stop { at, .. } | Action::Custom { at, .. } => *at,
        }
    }

    /// Where it leaves the robot.
    pub fn end(&self) -> Point {
        match self {
            Action::Straight { end, .. } | Action::Curve { end, .. } => *end,
            Action::Turn { at, .. } | Action::Stop { at, .. } => *at,
            Action::Custom { at, end, .. } => end.unwrap_or(*at),
        }
    }

    pub fn arc(&self) -> Option<Arc> {
        match self {
            Action::Curve {
                start,
                end,
                radius_mm,
                right,
                ..
            } => arc_of(*start, *end, *radius_mm, *right),
            _ => None,
        }
    }

    /// The heading it must start with, when it has one.
    pub fn start_heading(&self) -> Option<f64> {
        match self {
            Action::Straight { start, end, .. } => (dist(*start, *end) >= EPS_MM).then(|| heading_to(*start, *end)),
            Action::Curve { .. } => self.arc().map(|a| a.start_heading),
            Action::Custom { at, end: Some(e), .. } => (dist(*at, *e) >= EPS_MM).then(|| heading_to(*at, *e)),
            _ => None,
        }
    }

    /// The heading it leaves the robot with, given the one it started with.
    pub fn end_heading(&self, start_heading: f64) -> f64 {
        match self {
            Action::Curve { .. } => self.arc().map(|a| a.end_heading).unwrap_or(start_heading),
            Action::Turn { heading_deg, .. } => wrap_deg(*heading_deg),
            _ => self.start_heading().unwrap_or(start_heading),
        }
    }

    /// The path it drives, for drawing and hit tests.
    pub fn path(&self) -> Vec<Point> {
        match self {
            Action::Straight { start, end, .. } => vec![*start, *end],
            Action::Curve { start, end, right, .. } => match self.arc() {
                Some(arc) => {
                    let mut pts = arc_points(&arc, *start, *right);
                    pts[0] = *start;
                    if let Some(last) = pts.last_mut() {
                        *last = *end;
                    }
                    pts
                }
                None => vec![*start, *end],
            },
            Action::Custom { at, end: Some(e), .. } => vec![*at, *e],
            other => vec![other.start()],
        }
    }

    /// Distance driven, mm.
    pub fn length_mm(&self) -> f64 {
        match self {
            Action::Straight { start, end, .. } => dist(*start, *end),
            Action::Curve { .. } => self.arc().map(|a| a.radius_mm * a.sweep_deg.to_radians()).unwrap_or(0.0),
            Action::Custom { at, end: Some(e), .. } => dist(*at, *e),
            _ => 0.0,
        }
    }

    /// The action in words.
    pub fn text(&self) -> String {
        match self {
            Action::Straight { speed, then, .. } => {
                format!("straight {} mm at {}°/s{}", fmt(self.length_mm()), fmt(*speed), then.suffix())
            }
            Action::Curve {
                radius_mm,
                right,
                speed,
                then,
                ..
            } => format!(
                "curve {} {}° on r {} mm at {}°/s{}",
                if *right { "right" } else { "left" },
                fmt(self.arc().map(|a| a.sweep_deg).unwrap_or(0.0)),
                fmt(*radius_mm),
                fmt(*speed),
                then.suffix()
            ),
            Action::Turn { heading_deg, speed, .. } => format!("turn to face {}° at {}°/s", fmt(*heading_deg), fmt(*speed)),
            Action::Stop { then, wait_ms, .. } => {
                if *wait_ms > 0.0 {
                    format!("stop ({}), wait {} ms", then.label(), fmt(*wait_ms))
                } else {
                    format!("stop ({})", then.label())
                }
            }
            Action::Custom { code, end, .. } => {
                let mut lines = code.lines().map(str::trim).filter(|l| !l.is_empty());
                let call = match (lines.next(), lines.next()) {
                    (None, _) => "custom (type the call)".to_string(),
                    (Some(first), None) => first.to_string(),
                    (Some(first), Some(_)) => format!("{first} …"),
                };
                if end.is_some() {
                    format!("{call}, moving {} mm", fmt(self.length_mm()))
                } else {
                    call
                }
            }
        }
    }

    pub fn translate(&mut self, dx: f64, dy: f64) {
        let mv = |p: &mut Point| {
            p[0] = round1(p[0] + dx);
            p[1] = round1(p[1] + dy);
        };
        match self {
            Action::Straight { start, end, .. } | Action::Curve { start, end, .. } => {
                mv(start);
                mv(end);
            }
            Action::Turn { at, .. } | Action::Stop { at, .. } => mv(at),
            Action::Custom { at, end, .. } => {
                mv(at);
                if let Some(e) = end {
                    mv(e);
                }
            }
        }
    }

    /// The handles a drag can take, and where they sit.
    pub fn handles(&self) -> Vec<(Handle, Point)> {
        match self {
            Action::Straight { start, end, .. } => vec![(Handle::Start, *start), (Handle::End, *end)],
            Action::Curve { start, end, .. } => {
                let mut v = vec![(Handle::Start, *start), (Handle::End, *end)];
                let pts = self.path();
                if pts.len() > 2 {
                    v.push((Handle::Mid, pts[pts.len() / 2]));
                }
                v
            }
            Action::Turn { at, heading_deg, .. } => {
                let r = heading_deg.to_radians();
                vec![
                    (Handle::At, *at),
                    (Handle::Face, [at[0] + r.cos() * TURN_HANDLE_MM, at[1] + r.sin() * TURN_HANDLE_MM]),
                ]
            }
            Action::Stop { at, .. } => vec![(Handle::At, *at)],
            Action::Custom { at, end, .. } => {
                let mut v = vec![(Handle::At, *at)];
                if let Some(e) = end {
                    v.push((Handle::End, *e));
                }
                v
            }
        }
    }

    /// Move a handle to `to`: an end point moves, a curve's midpoint
    /// bends the arc through it, a turn's face handle sets the heading.
    pub fn drag(&mut self, handle: Handle, to: Point) {
        let to = [round1(to[0]), round1(to[1])];
        match (self, handle) {
            (Action::Straight { start, .. }, Handle::Start) | (Action::Curve { start, .. }, Handle::Start) => *start = to,
            (Action::Straight { end, .. }, Handle::End) | (Action::Curve { end, .. }, Handle::End) => *end = to,
            (
                Action::Curve {
                    start,
                    end,
                    radius_mm,
                    right,
                    ..
                },
                Handle::Mid,
            ) => {
                if let Some((_, r)) = circle_through(*start, to, *end) {
                    // an arc bulging to the left of its chord bends right, and the other way round;
                    // a minor arc only: the midpoint stays within the chord's half-circle
                    let (cx, cy) = (end[0] - start[0], end[1] - start[1]);
                    let bulge_side = cx * (to[1] - start[1]) - cy * (to[0] - start[0]);
                    let c = dist(*start, *end);
                    let mid = [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0];
                    if dist(mid, to) <= c / 2.0 + 1e-6 {
                        *radius_mm = round1(r.max(c / 2.0));
                        *right = bulge_side > 0.0;
                    }
                }
            }
            (Action::Turn { at, .. }, Handle::At) | (Action::Stop { at, .. }, Handle::At) | (Action::Custom { at, .. }, Handle::At) => {
                *at = to
            }
            (Action::Turn { at, heading_deg, .. }, Handle::Face) => {
                if dist(*at, to) >= EPS_MM {
                    *heading_deg = round1(heading_to(*at, to));
                }
            }
            (Action::Custom { end, .. }, Handle::End) => *end = Some(to),
            _ => {}
        }
    }

    /// The program lines for the action itself, from the pose it starts at.
    fn code(&self, from: Pose2) -> Vec<String> {
        match self {
            Action::Straight { then, .. } => vec![format!("db.straight({}{})", num(self.length_mm()), then.arg())],
            Action::Curve { right, then, .. } => match self.arc() {
                Some(arc) => vec![format!(
                    "db.curve({}, {}{})",
                    num(arc.radius_mm),
                    num(if *right { arc.sweep_deg } else { -arc.sweep_deg }),
                    then.arg()
                )],
                None => vec![],
            },
            Action::Turn { heading_deg, .. } => {
                let rel = wrap_deg(heading_deg - from.yaw_deg);
                if rel.abs() < 0.05 {
                    vec!["# already facing that way".to_string()]
                } else {
                    vec![format!("db.turn({})", num(-rel))]
                }
            }
            Action::Stop { then, wait_ms, .. } => {
                let mut v = vec![match then {
                    End::Coast => "db.stop()".to_string(),
                    other => format!("db.stop(then={})", other.code()),
                }];
                if *wait_ms > 0.0 {
                    v.push(format!("time.sleep_ms({})", wait_ms.round() as i64));
                }
                v
            }
            Action::Custom { code, .. } => code
                .lines()
                .map(|l| l.trim_end().to_string())
                .filter(|l| !l.trim().is_empty())
                .collect(),
        }
    }
}

/// A route: the map, where the robot starts, what the custom actions
/// call, and the actions in order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Route {
    pub format: String,
    #[serde(default)]
    pub world: String,
    #[serde(default)]
    pub start: Pose2,
    /// Python placed before the actions: the `def`s the custom actions call.
    #[serde(default)]
    pub prelude: String,
    #[serde(default)]
    pub actions: Vec<Item>,
}

impl Default for Route {
    fn default() -> Self {
        Route {
            format: FORMAT.into(),
            world: String::new(),
            start: Pose2::default(),
            prelude: String::new(),
            actions: vec![],
        }
    }
}

/// One action planned: where the robot came from, the drive that takes
/// it to the action's start (`link`, empty when it is already there), the
/// pose it starts the action at, where it ends, and the path it draws.
#[derive(Clone, Debug, PartialEq)]
pub struct Step {
    pub from: Pose2,
    pub link: Vec<Point>,
    pub start: Pose2,
    pub end: Pose2,
    pub points: Vec<Point>,
}

fn fmt(v: f64) -> String {
    let r = (v * 10.0).round() / 10.0;
    if r == r.trunc() { format!("{}", r as i64) } else { format!("{r}") }
}

fn num(v: f64) -> String {
    let r = (v * 10.0).round() / 10.0;
    if r == r.trunc() { format!("{}", r as i64) } else { format!("{r}") }
}

fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

/// An angle wrapped to (-180, 180].
pub fn wrap_deg(a: f64) -> f64 {
    let mut a = a % 360.0;
    if a <= -180.0 {
        a += 360.0;
    } else if a > 180.0 {
        a -= 360.0;
    }
    a
}

/// The robot brought from `from` to `to`, facing `heading` when one is
/// asked for: the link it drives (empty when it is already there), the
/// program lines, and the pose it arrives in.
fn approach(from: Pose2, to: Point, heading: Option<f64>) -> (Vec<Point>, Vec<String>, Pose2) {
    let mut code = Vec::new();
    let mut pose = from;
    let mut link = Vec::new();
    let d = dist(from.point(), to);
    if d >= EPS_MM {
        let dir = heading_to(from.point(), to);
        let rel = wrap_deg(dir - pose.yaw_deg);
        if rel.abs() >= 0.05 {
            code.push(format!("db.turn({})", num(-rel)));
        }
        code.push(format!("db.straight({})", num(d)));
        link = vec![from.point(), to];
        pose = Pose2::at(to, dir);
    }
    if let Some(h) = heading {
        let rel = wrap_deg(h - pose.yaw_deg);
        if rel.abs() >= 0.05 {
            code.push(format!("db.turn({})", num(-rel)));
        }
        pose.yaw_deg = wrap_deg(h);
    }
    (link, code, pose)
}

/// Every action planned in order from the route's start.
pub fn plan(route: &Route) -> Vec<Step> {
    let mut pose = route.start;
    route
        .actions
        .iter()
        .map(|item| {
            let a = &item.action;
            let (link, _, start) = approach(pose, a.start(), a.start_heading());
            let end = Pose2::at(a.end(), a.end_heading(start.yaw_deg));
            let step = Step {
                from: pose,
                link,
                start,
                end,
                points: a.path(),
            };
            pose = end;
            step
        })
        .collect()
}

/// Distance driven over the whole route, links included, mm.
pub fn length_mm(route: &Route) -> f64 {
    plan(route)
        .iter()
        .zip(&route.actions)
        .map(|(s, item)| item.action.length_mm() + if s.link.len() == 2 { dist(s.link[0], s.link[1]) } else { 0.0 })
        .sum()
}

/// The nearest of `targets` within `tol_mm` of `p`, else `p` itself.
pub fn snap(p: Point, targets: &[Point], tol_mm: f64) -> Point {
    targets
        .iter()
        .map(|t| (dist(p, *t), *t))
        .filter(|(d, _)| *d <= tol_mm)
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, t)| t)
        .unwrap_or(p)
}

/// How far a point is from a polyline (a lone point counts as a point).
pub fn distance_to_path(p: Point, path: &[Point]) -> f64 {
    match path {
        [] => f64::INFINITY,
        [only] => dist(p, *only),
        _ => path
            .windows(2)
            .map(|w| {
                let (a, b) = (w[0], w[1]);
                let (vx, vy) = (b[0] - a[0], b[1] - a[1]);
                let len2 = vx * vx + vy * vy;
                let t = if len2 < 1e-12 {
                    0.0
                } else {
                    (((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / len2).clamp(0.0, 1.0)
                };
                // the ends exactly, so a marker on an end ties with the path and not by an ulp
                let nearest = if t <= 0.0 {
                    a
                } else if t >= 1.0 {
                    b
                } else {
                    [a[0] + vx * t, a[1] + vy * t]
                };
                dist(p, nearest)
            })
            .fold(f64::INFINITY, f64::min),
    }
}

/// The program that drives the route: a hub-style program the sim runs
/// unchanged (the shim binds the two serial motors to the chassis's
/// wheels), with the motor lines for the reference wiring to edit.
pub fn program(route: &Route, wheel_diameter_mm: f64, axle_track_mm: f64) -> String {
    let n = route.actions.len();
    let mut out = format!(
        "\"\"\"A route planned in the openbricks sim on {}: {} action{} from ({}, {}) mm heading {}°.\n\nThe sim binds the two motors to the chassis's wheels; on the hub, edit them for your wiring.\"\"\"\n",
        if route.world.is_empty() { "the map" } else { &route.world },
        n,
        if n == 1 { "" } else { "s" },
        num(route.start.x_mm),
        num(route.start.y_mm),
        num(route.start.yaw_deg)
    );
    let stops = route.actions.iter().any(|i| match &i.action {
        Action::Straight { then, .. } | Action::Curve { then, .. } | Action::Stop { then, .. } => *then != End::Coast,
        _ => false,
    });
    let waits = route
        .actions
        .iter()
        .any(|i| matches!(i.action, Action::Stop { wait_ms, .. } if wait_ms > 0.0));
    if waits {
        out.push_str("import time\n\n");
    }
    out.push_str("from openbricks.drivers.st3032 import ST3032Motor\n");
    if stops {
        out.push_str("from openbricks.parameters import Stop\n");
    }
    out.push_str("from openbricks.robotics import DriveBase\n");
    out.push_str(&format!(
        "\nleft = ST3032Motor(servo_id=1, tx=14, rx=41)\nright = ST3032Motor(servo_id=2, tx=14, rx=41, invert=True)\ndb = DriveBase(left, right, wheel_diameter_mm={}, axle_track_mm={})\n",
        num(wheel_diameter_mm),
        num(axle_track_mm)
    ));
    let prelude = route.prelude.trim();
    if !prelude.is_empty() {
        out.push_str("\n# definitions: what the custom actions call\n");
        out.push_str(prelude);
        out.push('\n');
    }
    let mut pose = route.start;
    let (mut straight_speed, mut turn_rate) = (DEFAULT_STRAIGHT_DPS, DEFAULT_TURN_DPS);
    for (i, item) in route.actions.iter().enumerate() {
        let a = &item.action;
        out.push_str(&format!("\n# {}: {}\n", i + 1, a.text()));
        match a {
            Action::Straight { speed, .. } | Action::Curve { speed, .. } if *speed != straight_speed => {
                straight_speed = *speed;
                out.push_str(&format!("db.settings(straight_speed={})\n", num(straight_speed)));
            }
            Action::Turn { speed, .. } if *speed != turn_rate => {
                turn_rate = *speed;
                out.push_str(&format!("db.settings(turn_rate={})\n", num(turn_rate)));
            }
            _ => {}
        }
        let (link, code, start) = approach(pose, a.start(), a.start_heading());
        if !code.is_empty() {
            out.push_str(&format!(
                "# to its start{}\n",
                if link.is_empty() { "" } else { ": the drive there" }
            ));
            for line in code {
                out.push_str(&line);
                out.push('\n');
            }
        }
        for line in a.code(start) {
            out.push_str(&line);
            out.push('\n');
        }
        pose = Pose2::at(a.end(), a.end_heading(start.yaw_deg));
    }
    out
}

/// The tangent arc from a pose through a point: its radius and side,
/// None when the point is dead ahead or behind (a line, not an arc).
fn tangent_arc(from: Pose2, to: Point) -> Option<(f64, bool)> {
    let (dx, dy) = (to[0] - from.x_mm, to[1] - from.y_mm);
    let d = (dx * dx + dy * dy).sqrt();
    if d < EPS_MM {
        return None;
    }
    let (hx, hy) = from.heading();
    let cross = hx * dy - hy * dx;
    let dot = hx * dx + hy * dy;
    if cross.abs() < 1e-6 * d {
        return None;
    }
    let alpha = cross.abs().atan2(dot);
    Some((round1(d / (2.0 * alpha.sin())), cross < 0.0))
}

impl Route {
    /// The names of the functions the definitions declare (`def name(`).
    pub fn functions(&self) -> Vec<String> {
        self.prelude
            .lines()
            .filter_map(|l| {
                let rest = l.trim_start().strip_prefix("def ")?;
                let name: String = rest.chars().take_while(|c| c.is_alphanumeric() || *c == '_').collect();
                (!name.is_empty() && rest[name.len()..].trim_start().starts_with('(')).then_some(name)
            })
            .collect()
    }

    /// Read a route file; the earlier formats' relative moves and point
    /// segments become placed actions.
    pub fn load(path: &Path) -> Result<Route, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        match v.get("format").and_then(|f| f.as_str()) {
            Some(FORMAT) => serde_json::from_value(v).map_err(|e| e.to_string()),
            Some(FORMAT_V2) => from_v2(v),
            Some(FORMAT_V1) => from_v1(v),
            other => Err(format!("not a route file (format {other:?}, expected {FORMAT})")),
        }
    }

    pub fn save(&self, path: &Path) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(path, text).map_err(|e| e.to_string())
    }
}

fn from_v1(v: serde_json::Value) -> Result<Route, String> {
    #[derive(Deserialize)]
    struct Segment {
        kind: String,
        to: [f64; 2],
    }
    #[derive(Deserialize)]
    struct V1 {
        #[serde(default)]
        world: String,
        start: Pose2,
        #[serde(default)]
        segments: Vec<Segment>,
    }
    let old: V1 = serde_json::from_value(v).map_err(|e| e.to_string())?;
    let mut route = Route {
        world: old.world,
        start: old.start,
        ..Default::default()
    };
    let mut pose = old.start;
    for seg in old.segments {
        let action = match (seg.kind.as_str(), tangent_arc(pose, seg.to)) {
            ("curve", Some((radius_mm, right))) => Action::Curve {
                start: pose.point(),
                end: seg.to,
                radius_mm,
                right,
                speed: DEFAULT_STRAIGHT_DPS,
                then: End::Coast,
            },
            _ => Action::Straight {
                start: pose.point(),
                end: seg.to,
                speed: DEFAULT_STRAIGHT_DPS,
                then: End::Coast,
            },
        };
        let heading = action.start_heading().unwrap_or(pose.yaw_deg);
        pose = Pose2::at(action.end(), action.end_heading(heading));
        route.actions.push(action.into());
    }
    Ok(route)
}

fn from_v2(v: serde_json::Value) -> Result<Route, String> {
    #[derive(Deserialize)]
    #[serde(tag = "kind", rename_all = "lowercase")]
    enum Old {
        Straight {
            mm: f64,
            #[serde(default)]
            then: End,
        },
        Turn {
            deg: f64,
        },
        Curve {
            radius_mm: f64,
            deg: f64,
            #[serde(default)]
            then: End,
        },
        Stop {
            #[serde(default)]
            then: End,
            #[serde(default)]
            wait_ms: f64,
        },
        Custom {
            code: String,
        },
    }
    #[derive(Deserialize)]
    struct V2 {
        #[serde(default)]
        world: String,
        #[serde(default)]
        start: Pose2,
        #[serde(default)]
        prelude: String,
        #[serde(default)]
        actions: Vec<Old>,
    }
    let old: V2 = serde_json::from_value(v).map_err(|e| e.to_string())?;
    let mut route = Route {
        world: old.world,
        start: old.start,
        prelude: old.prelude,
        ..Default::default()
    };
    let mut pose = old.start;
    for a in old.actions {
        let (hx, hy) = pose.heading();
        let action = match a {
            Old::Straight { mm, then } => Action::Straight {
                start: pose.point(),
                end: [round1(pose.x_mm + hx * mm), round1(pose.y_mm + hy * mm)],
                speed: DEFAULT_STRAIGHT_DPS,
                then,
            },
            Old::Turn { deg } => Action::Turn {
                at: pose.point(),
                heading_deg: round1(wrap_deg(pose.yaw_deg - deg)),
                speed: DEFAULT_TURN_DPS,
            },
            Old::Curve { radius_mm, deg, then } if radius_mm.abs() >= EPS_MM && deg.abs() >= 1e-6 => {
                // the old arc: the rigid body turns about the circle's centre, on the right for a
                // forward right turn and on the other side when driving backward
                let side = if (deg > 0.0) == (radius_mm > 0.0) { -1.0 } else { 1.0 };
                let r = radius_mm.abs();
                let centre = [pose.x_mm - hy * r * side, pose.y_mm + hx * r * side];
                let a0 = (pose.y_mm - centre[1]).atan2(pose.x_mm - centre[0]);
                let a1 = a0 - deg.to_radians();
                let end = [round1(centre[0] + r * a1.cos()), round1(centre[1] + r * a1.sin())];
                let (cx, cy) = (end[0] - pose.x_mm, end[1] - pose.y_mm);
                let cross = cx * (centre[1] - pose.y_mm) - cy * (centre[0] - pose.x_mm);
                Action::Curve {
                    start: pose.point(),
                    end,
                    radius_mm: r,
                    right: cross < 0.0,
                    speed: DEFAULT_STRAIGHT_DPS,
                    then,
                }
            }
            Old::Curve { deg, .. } => Action::Turn {
                at: pose.point(),
                heading_deg: round1(wrap_deg(pose.yaw_deg - deg)),
                speed: DEFAULT_TURN_DPS,
            },
            Old::Stop { then, wait_ms } => Action::Stop {
                at: pose.point(),
                then,
                wait_ms,
            },
            Old::Custom { code } => Action::Custom {
                at: pose.point(),
                end: None,
                code,
            },
        };
        let heading = action.start_heading().unwrap_or(pose.yaw_deg);
        pose = Pose2::at(action.end(), action.end_heading(heading));
        route.actions.push(action.into());
    }
    Ok(route)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    fn pose_near(p: Pose2, x: f64, y: f64, yaw: f64) -> bool {
        near(p.x_mm, x) && near(p.y_mm, y) && wrap_deg(p.yaw_deg - yaw).abs() < 1e-6
    }

    fn straight(start: Point, end: Point) -> Action {
        Action::placed("straight", &[start, end])
    }

    #[test]
    fn placed_kinds_take_their_defaults_from_the_clicks() {
        let s = straight([0.0, 0.0], [300.0, 0.0]);
        assert_eq!(s.start(), [0.0, 0.0]);
        assert_eq!(s.end(), [300.0, 0.0]);
        assert_eq!(s.start_heading(), Some(0.0));
        assert!(near(s.length_mm(), 300.0));
        assert_eq!(s.text(), "straight 300 mm at 350°/s");
        let c = Action::placed("curve", &[[0.0, 0.0], [100.0, 100.0]]);
        let Action::Curve { radius_mm, right, .. } = &c else { panic!() };
        assert!(
            *right && (radius_mm - 100.0).abs() < 0.1,
            "a quarter circle: r = chord / √2 = {radius_mm}"
        );
        assert!(near(c.arc().unwrap().sweep_deg, 90.0));
        let t = Action::placed("turn", &[[10.0, 10.0], [10.0, 60.0]]);
        assert_eq!(
            t,
            Action::Turn {
                at: [10.0, 10.0],
                heading_deg: 90.0,
                speed: 300.0
            }
        );
        assert_eq!(Action::placed("stop", &[[1.0, 2.0]]).text(), "stop (coast)");
        assert_eq!(Action::placed("custom", &[[1.0, 2.0]]).text(), "custom (type the call)");
        for (kind, _, _) in KINDS {
            assert_eq!(Action::placed(kind, &[[0.0, 0.0], [100.0, 0.0]]).kind(), kind);
        }
        assert_eq!(
            (
                clicks_needed("straight"),
                clicks_needed("turn"),
                clicks_needed("stop"),
                clicks_needed("custom")
            ),
            (2, 2, 1, 1)
        );
    }

    #[test]
    fn arcs_bend_right_or_left_between_their_ends() {
        // a right (clockwise) semicircle from (0,0) to (100,0) starts heading +y and ends heading -y
        let a = arc_of([0.0, 0.0], [100.0, 0.0], 50.0, true).unwrap();
        assert!(near(a.centre[0], 50.0) && near(a.centre[1], 0.0), "{a:?}");
        assert!(
            near(a.sweep_deg, 180.0) && near(a.start_heading, 90.0) && near(a.end_heading, -90.0),
            "{a:?}"
        );
        // to the left: mirrored
        let b = arc_of([0.0, 0.0], [100.0, 0.0], 50.0, false).unwrap();
        assert!(near(b.start_heading, -90.0) && near(b.end_heading, 90.0), "{b:?}");
        // a quarter circle of r 100 from (0,0) to (100,100) bending left starts heading +x
        let q = arc_of([0.0, 0.0], [100.0, 100.0], 100.0, false).unwrap();
        assert!(
            near(q.sweep_deg, 90.0) && near(q.start_heading, 0.0) && near(q.end_heading, 90.0),
            "{q:?}"
        );
        assert!(near(q.centre[0], 0.0) && near(q.centre[1], 100.0), "{q:?}");
        // the path runs from start to end along the circle
        let c = Action::Curve {
            start: [0.0, 0.0],
            end: [100.0, 100.0],
            radius_mm: 100.0,
            right: false,
            speed: 350.0,
            then: End::Coast,
        };
        let pts = c.path();
        assert_eq!(pts[0], [0.0, 0.0]);
        assert_eq!(*pts.last().unwrap(), [100.0, 100.0]);
        for p in &pts {
            assert!(near(dist(*p, [0.0, 100.0]), 100.0), "{p:?} on the circle");
        }
        assert!((c.length_mm() - 100.0 * std::f64::consts::FRAC_PI_2).abs() < 1e-6);
        // a radius shorter than half the chord is lengthened to it
        let s = arc_of([0.0, 0.0], [100.0, 0.0], 10.0, true).unwrap();
        assert!(near(s.radius_mm, 50.0) && near(s.sweep_deg, 180.0));
        assert!(arc_of([5.0, 5.0], [5.0, 5.0], 50.0, true).is_none());
    }

    #[test]
    fn handles_move_ends_bend_arcs_and_aim_turns() {
        let mut s = straight([0.0, 0.0], [100.0, 0.0]);
        assert_eq!(s.handles(), vec![(Handle::Start, [0.0, 0.0]), (Handle::End, [100.0, 0.0])]);
        s.drag(Handle::End, [200.04, 50.0]);
        assert_eq!(s.end(), [200.0, 50.0]);
        s.drag(Handle::Start, [10.0, 10.0]);
        assert_eq!(s.start(), [10.0, 10.0]);
        s.drag(Handle::Mid, [0.0, 0.0]);
        assert_eq!(s.start(), [10.0, 10.0], "a straight has no midpoint handle");
        // a curve's midpoint handle: dragged through (50, -50), below the chord, the arc becomes a
        // semicircle of r 50 bending left (it bulges to the right of the way it drives)
        let mut c = Action::Curve {
            start: [0.0, 0.0],
            end: [100.0, 0.0],
            radius_mm: 100.0,
            right: true,
            speed: 350.0,
            then: End::Coast,
        };
        assert_eq!(c.handles().len(), 3);
        c.drag(Handle::Mid, [50.0, -50.0]);
        let Action::Curve { radius_mm, right, .. } = &c else { panic!() };
        assert!(near(*radius_mm, 50.0) && !*right, "{c:?}");
        c.drag(Handle::Mid, [50.0, 20.0]);
        let Action::Curve { radius_mm, right, .. } = &c else { panic!() };
        assert!(*right && (radius_mm - 72.5).abs() < 0.1, "above the chord: bends right, {c:?}");
        c.drag(Handle::Mid, [50.0, 90.0]);
        let Action::Curve { radius_mm, .. } = &c else { panic!() };
        assert!((radius_mm - 72.5).abs() < 0.1, "a bulge past the half circle is refused: {c:?}");
        c.drag(Handle::Mid, [50.0, 0.0]);
        let Action::Curve { radius_mm, .. } = &c else { panic!() };
        assert!((radius_mm - 72.5).abs() < 0.1, "a flat midpoint (no circle) leaves the arc alone");
        // a turn: the face handle sets the heading, the at handle moves it
        let mut t = Action::placed("turn", &[[0.0, 0.0], [10.0, 0.0]]);
        assert_eq!(t.handles()[1], (Handle::Face, [TURN_HANDLE_MM, 0.0]));
        t.drag(Handle::Face, [0.0, 30.0]);
        assert_eq!(t.end_heading(0.0), 90.0);
        t.drag(Handle::At, [5.0, 5.0]);
        assert_eq!(t.start(), [5.0, 5.0]);
        t.drag(Handle::Face, [5.0, 5.0]);
        assert_eq!(t.end_heading(0.0), 90.0, "a face handle on the spot changes nothing");
        // a custom call's end handle appears once it moves the robot
        let mut k = Action::placed("custom", &[[0.0, 0.0]]);
        assert_eq!(k.handles().len(), 1);
        k.drag(Handle::End, [100.0, 0.0]);
        assert_eq!(k.handles().len(), 2);
        assert_eq!(k.start_heading(), Some(0.0));
        assert!(k.text().ends_with(", moving 100 mm"), "{}", k.text());
        k.translate(10.0, -10.0);
        assert_eq!((k.start(), k.end()), ([10.0, -10.0], [110.0, -10.0]));
        let mut st = Action::placed("stop", &[[3.0, 4.0]]);
        st.translate(1.0, 1.0);
        assert_eq!(st.start(), [4.0, 5.0]);
        assert_eq!(st.handles(), vec![(Handle::At, [4.0, 5.0])]);
    }

    #[test]
    fn the_plan_drives_to_each_action_and_keeps_the_sign_conventions() {
        let route = Route {
            start: Pose2::at([0.0, 0.0], 0.0),
            actions: vec![
                straight([0.0, 0.0], [300.0, 0.0]).into(),
                // a right quarter circle of r 100 from (300, 0): the robot must first turn to face +y
                Action::Curve {
                    start: [300.0, 0.0],
                    end: [400.0, 100.0],
                    radius_mm: 100.0,
                    right: true,
                    speed: 350.0,
                    then: End::Continue,
                }
                .into(),
                // a stop somewhere else: the plan drives there first
                Action::placed("stop", &[[400.0, 300.0]]).into(),
                Action::placed("turn", &[[400.0, 300.0], [300.0, 300.0]]).into(),
                Action::placed("custom", &[[400.0, 300.0]]).into(),
            ],
            ..Default::default()
        };
        let steps = plan(&route);
        assert_eq!(steps.len(), 5);
        assert!(steps[0].link.is_empty(), "already at the start");
        assert!(pose_near(steps[0].start, 0.0, 0.0, 0.0) && pose_near(steps[0].end, 300.0, 0.0, 0.0));
        assert!(steps[1].link.is_empty());
        assert!(pose_near(steps[1].start, 300.0, 0.0, 90.0), "{:?}", steps[1].start);
        assert!(pose_near(steps[1].end, 400.0, 100.0, 0.0), "{:?}", steps[1].end);
        assert_eq!(steps[2].link, vec![[400.0, 100.0], [400.0, 300.0]], "the drive to the stop");
        assert!(pose_near(steps[2].start, 400.0, 300.0, 90.0) && pose_near(steps[2].end, 400.0, 300.0, 90.0));
        assert!(steps[3].link.is_empty());
        assert!(pose_near(steps[3].end, 400.0, 300.0, 180.0), "faces -x");
        assert!(pose_near(steps[4].end, 400.0, 300.0, 180.0), "a call that stays put keeps the pose");
        assert!(near(length_mm(&route), 300.0 + 100.0 * std::f64::consts::FRAC_PI_2 + 200.0));
        let text = program(&route, 86.4, 135.0);
        assert!(
            text.contains("db = DriveBase(left, right, wheel_diameter_mm=86.4, axle_track_mm=135)\n"),
            "{text}"
        );
        assert!(text.contains("# 1: straight 300 mm at 350°/s\ndb.straight(300)\n"), "{text}");
        assert!(
            text.contains(
                "# 2: curve right 90° on r 100 mm at 350°/s, then continue\n# to its start\ndb.turn(-90)\ndb.curve(100, 90, then=Stop.NONE)\n"
            ),
            "{text}"
        );
        assert!(
            text.contains("# 3: stop (coast)\n# to its start: the drive there\ndb.turn(-90)\ndb.straight(200)\ndb.stop()\n"),
            "{text}"
        );
        assert!(text.contains("# 4: turn to face 180° at 300°/s\ndb.turn(-90)\n"), "{text}");
        assert!(text.contains("from openbricks.parameters import Stop\n"), "{text}");
        assert!(!text.contains("import time"), "{text}");
        assert!(!text.contains("db.settings"), "default speeds: no settings line");
    }

    #[test]
    fn speeds_waits_and_definitions_reach_the_program() {
        let mut route = Route {
            start: Pose2::at([0.0, 0.0], 90.0),
            prelude: "def line_follow():\n    db.drive(100, 0)\n".into(),
            ..Default::default()
        };
        route.actions.push(
            Action::Straight {
                start: [0.0, 0.0],
                end: [0.0, 250.0],
                speed: 200.0,
                then: End::Brake,
            }
            .into(),
        );
        route.actions.push(
            Action::Turn {
                at: [0.0, 250.0],
                heading_deg: 0.0,
                speed: 150.0,
            }
            .into(),
        );
        route.actions.push(
            Action::Stop {
                at: [0.0, 250.0],
                then: End::Hold,
                wait_ms: 500.0,
            }
            .into(),
        );
        route.actions.push(
            Action::Custom {
                at: [0.0, 250.0],
                end: Some([100.0, 250.0]),
                code: "line_follow()\n  \n".into(),
            }
            .into(),
        );
        route.actions.push(
            Action::Straight {
                start: [100.0, 250.0],
                end: [200.0, 250.0],
                speed: 200.0,
                then: End::Coast,
            }
            .into(),
        );
        let text = program(&route, 60.0, 150.0);
        assert!(
            text.starts_with("\"\"\"A route planned in the openbricks sim on the map: 5 actions from (0, 0) mm heading 90°."),
            "{text}"
        );
        assert!(text.contains("import time\n\n"), "{text}");
        assert!(
            text.contains("\n# definitions: what the custom actions call\ndef line_follow():\n    db.drive(100, 0)\n"),
            "{text}"
        );
        assert!(
            text.contains(
                "# 1: straight 250 mm at 200°/s, then brake\ndb.settings(straight_speed=200)\ndb.straight(250, then=Stop.BRAKE)\n"
            ),
            "{text}"
        );
        assert!(
            text.contains("# 2: turn to face 0° at 150°/s\ndb.settings(turn_rate=150)\ndb.turn(90)\n"),
            "{text}"
        );
        assert!(
            text.contains("# 3: stop (hold), wait 500 ms\ndb.stop(then=Stop.HOLD)\ntime.sleep_ms(500)\n"),
            "{text}"
        );
        assert!(text.contains("# 4: line_follow(), moving 100 mm\nline_follow()\n"), "{text}");
        assert!(
            text.contains("# 5: straight 100 mm at 200°/s\ndb.straight(100)\n"),
            "the speed is already set: no second settings line\n{text}"
        );
        assert_eq!(route.functions(), vec!["line_follow"]);
        let end = plan(&route).last().unwrap().end;
        assert!(pose_near(end, 200.0, 250.0, 0.0), "{end:?}");
    }

    #[test]
    fn snapping_and_hit_testing() {
        assert_eq!(snap([103.0, 2.0], &[[100.0, 0.0], [500.0, 0.0]], 5.0), [100.0, 0.0]);
        assert_eq!(snap([110.0, 2.0], &[[100.0, 0.0]], 5.0), [110.0, 2.0]);
        assert_eq!(snap([0.0, 0.0], &[], 5.0), [0.0, 0.0]);
        let path = vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0]];
        assert!(near(distance_to_path([50.0, 10.0], &path), 10.0));
        assert!(near(distance_to_path([120.0, 50.0], &path), 20.0));
        assert!(near(distance_to_path([-30.0, 0.0], &path), 30.0));
        assert!(near(distance_to_path([3.0, 4.0], &[[0.0, 0.0]]), 5.0));
        assert!(distance_to_path([0.0, 0.0], &[]).is_infinite());
    }

    #[test]
    fn routes_save_load_and_convert_the_earlier_formats() {
        let dir = std::env::temp_dir().join(format!("ob-route-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut route = Route {
            world: "wro-2026-senior".into(),
            start: Pose2::at([1.5, -2.0], 30.0),
            prelude: "X = 1".into(),
            actions: vec![
                Action::Curve {
                    start: [0.0, 0.0],
                    end: [100.0, 100.0],
                    radius_mm: 100.0,
                    right: false,
                    speed: 300.0,
                    then: End::Continue,
                }
                .into(),
                Action::placed("stop", &[[100.0, 100.0]]).into(),
            ],
            ..Default::default()
        };
        route.actions[1].locked = true;
        let p = dir.join("a.route.json");
        route.save(&p).unwrap();
        let text = std::fs::read_to_string(&p).unwrap();
        assert!(
            text.contains("\"format\": \"openbricks-route/3\"") && text.contains("\"kind\": \"curve\""),
            "{text}"
        );
        assert!(text.contains("\"locked\": true"), "{text}");
        assert_eq!(text.matches("\"locked\"").count(), 1, "unlocked actions do not mention it");
        assert_eq!(Route::load(&p).unwrap(), route);
        // the second format: relative moves from the start become placed actions
        let v2 = dir.join("v2.route.json");
        std::fs::write(
            &v2,
            r#"{"format":"openbricks-route/2","world":"practice-line","start":{"x_mm":0,"y_mm":0,"yaw_deg":90},"prelude":"def f():\n    pass\n","actions":[{"kind":"straight","mm":250,"then":"continue"},{"kind":"curve","radius_mm":100,"deg":90},{"kind":"turn","deg":-45},{"kind":"stop","then":"hold","wait_ms":200},{"kind":"custom","code":"f()"},{"kind":"curve","radius_mm":0,"deg":30}]}"#,
        )
        .unwrap();
        let old = Route::load(&v2).unwrap();
        assert_eq!(old.world, "practice-line");
        assert_eq!(old.prelude, "def f():\n    pass\n");
        assert_eq!(old.actions.len(), 6);
        assert_eq!(old.actions[0].action.start(), [0.0, 0.0]);
        assert_eq!(old.actions[0].action.end(), [0.0, 250.0]);
        let Action::Curve {
            start,
            end,
            radius_mm,
            right,
            ..
        } = &old.actions[1].action
        else {
            panic!("{:?}", old.actions[1])
        };
        assert_eq!((start, end, radius_mm, right), (&[0.0, 250.0], &[100.0, 350.0], &100.0, &true));
        assert_eq!(
            old.actions[2].action,
            Action::Turn {
                at: [100.0, 350.0],
                heading_deg: 45.0,
                speed: 300.0
            }
        );
        assert!(matches!(old.actions[3].action, Action::Stop { then: End::Hold, wait_ms, .. } if wait_ms == 200.0));
        assert!(matches!(&old.actions[4].action, Action::Custom { code, .. } if code == "f()"));
        assert_eq!(
            old.actions[5].action,
            Action::Turn {
                at: [100.0, 350.0],
                heading_deg: 15.0,
                speed: 300.0
            },
            "a zero-radius curve was a turn"
        );
        let steps = plan(&old);
        assert!(
            steps.iter().all(|s| s.link.is_empty()),
            "converted moves chain without drives between them"
        );
        assert!(pose_near(steps[1].end, 100.0, 350.0, 0.0), "{:?}", steps[1].end);
        // the first format: point segments
        let v1 = dir.join("v1.route.json");
        std::fs::write(
            &v1,
            r#"{"format":"openbricks-route/1","world":"practice-line","start":{"x_mm":0,"y_mm":0,"yaw_deg":90},"segments":[{"kind":"straight","to":[0,250]},{"kind":"curve","to":[100,350]},{"kind":"curve","to":[250,350]}]}"#,
        )
        .unwrap();
        let older = Route::load(&v1).unwrap();
        assert_eq!(older.actions.len(), 3);
        assert_eq!(older.actions[0].action.end(), [0.0, 250.0]);
        let Action::Curve { radius_mm, right, .. } = &older.actions[1].action else {
            panic!()
        };
        assert!(near(*radius_mm, 100.0) && *right);
        assert!(
            matches!(older.actions[2].action, Action::Straight { .. }),
            "a point dead ahead is a line"
        );
        assert!(pose_near(plan(&older).last().unwrap().end, 250.0, 350.0, 0.0));
        // other files are refused
        std::fs::write(dir.join("x.json"), r#"{"format":"something-else"}"#).unwrap();
        let e = Route::load(&dir.join("x.json")).unwrap_err();
        assert!(e.contains("not a route file"), "{e}");
        assert!(Route::load(&dir.join("missing.json")).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn end_states_and_helpers() {
        assert_eq!(End::STOPS.map(|e| e.label()), ["coast", "brake", "hold"]);
        assert_eq!(End::Continue.label(), "continue");
        assert_eq!(End::Continue.code(), "Stop.NONE");
        assert_eq!(End::Coast.arg(), "");
        assert_eq!(End::Hold.arg(), ", then=Stop.HOLD");
        assert_eq!(wrap_deg(370.0), 10.0);
        assert_eq!(wrap_deg(-180.0), 180.0);
        assert_eq!(wrap_deg(180.0), 180.0);
        assert_eq!(fmt(2.0), "2");
        assert_eq!(fmt(2.25), "2.3");
        assert_eq!(num(-45.04), "-45");
        let p = Pose2::at([3.0, 4.0], 90.0);
        assert_eq!(p.point(), [3.0, 4.0]);
        assert!(near(p.heading().1, 1.0));
        assert!(!Item::from(Action::placed("stop", &[[0.0, 0.0]])).locked);
    }
}
