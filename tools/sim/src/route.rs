//! Routes: a start pose on the map and an ordered list of actions —
//! straight, turn, curve, stop, or a custom call — each with the
//! parameters the drive base takes. The parameters are typed in, set
//! by dragging the action's handle on the map, or produced by the
//! point tools (a straight line or a tangent arc to a clicked point).
//! The maths here is plain so it runs under `cargo test`; the Simulate
//! tab draws it and turns it into a program.
//!
//! Frames: the map's world frame, mm, yaw counter-clockwise from +x in
//! degrees. Actions are in the drive base's own terms, where a positive
//! turn or curve angle is to the RIGHT (clockwise from above) like
//! `DriveBase.turn` and `DriveBase.curve`, and a negative curve radius
//! drives the arc backward.

use serde::{Deserialize, Serialize};
use std::path::Path;

pub const FORMAT: &str = "openbricks-route/2";
const FORMAT_V1: &str = "openbricks-route/1";
/// Points closer than this are the same point.
const EPS_MM: f64 = 0.5;
/// A turn's handle sits this far ahead along the new heading.
pub const TURN_HANDLE_MM: f64 = 60.0;

/// A pose on the map.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Debug, Default)]
pub struct Pose2 {
    pub x_mm: f64,
    pub y_mm: f64,
    pub yaw_deg: f64,
}

impl Pose2 {
    pub fn heading(&self) -> (f64, f64) {
        (self.yaw_deg.to_radians().cos(), self.yaw_deg.to_radians().sin())
    }
}

/// How a move or a stop ends: free-wheeling, braking, holding the
/// wheels, or — for a straight or a curve — not slowing at all, so the
/// next move takes over at cruise speed (`then=Stop.NONE`).
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Debug, Default)]
#[serde(rename_all = "lowercase")]
pub enum End {
    #[default]
    Coast,
    Brake,
    Hold,
    Continue,
}

impl End {
    /// What a stop can end with.
    pub const STOPS: [End; 3] = [End::Coast, End::Brake, End::Hold];
    /// What a straight or a curve can end with.
    pub const MOVES: [End; 4] = [End::Coast, End::Brake, End::Hold, End::Continue];
    pub fn label(self) -> &'static str {
        match self {
            End::Coast => "coast",
            End::Brake => "brake",
            End::Hold => "hold",
            End::Continue => "continue",
        }
    }
    fn code(self) -> &'static str {
        match self {
            End::Coast => "Stop.COAST",
            End::Brake => "Stop.BRAKE",
            End::Hold => "Stop.HOLD",
            End::Continue => "Stop.NONE",
        }
    }
    /// The `then=` argument, or nothing for the default.
    fn arg(self) -> String {
        match self {
            End::Coast => String::new(),
            other => format!(", then={}", other.code()),
        }
    }
    fn suffix(self) -> String {
        match self {
            End::Coast => String::new(),
            other => format!(", then {}", other.label()),
        }
    }
}

/// One thing the robot does, in the drive base's terms.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Action {
    /// Drive `mm` (negative backs up), ending as `then` says.
    Straight {
        mm: f64,
        #[serde(default)]
        then: End,
    },
    /// Turn in place; positive is to the right.
    Turn { deg: f64 },
    /// An arc of `|radius_mm|` turning `deg` (positive right); a
    /// negative radius drives it backward.
    Curve {
        radius_mm: f64,
        deg: f64,
        #[serde(default)]
        then: End,
    },
    /// Halt with an end state, then wait.
    Stop { then: End, wait_ms: f64 },
    /// A line of the program, as typed: a call to something the
    /// route's definitions provide.
    Custom { code: String },
}

pub const KINDS: [&str; 5] = ["straight", "turn", "curve", "stop", "custom"];

impl Action {
    /// A sensible action of a kind, for the "+" buttons.
    pub fn default_of(kind: &str) -> Action {
        match kind {
            "straight" => Action::Straight {
                mm: 200.0,
                then: End::Coast,
            },
            "turn" => Action::Turn { deg: 90.0 },
            "curve" => Action::Curve {
                radius_mm: 150.0,
                deg: 90.0,
                then: End::Coast,
            },
            "stop" => Action::Stop {
                then: End::Coast,
                wait_ms: 0.0,
            },
            _ => Action::Custom { code: String::new() },
        }
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Action::Straight { .. } => "straight",
            Action::Turn { .. } => "turn",
            Action::Curve { .. } => "curve",
            Action::Stop { .. } => "stop",
            Action::Custom { .. } => "custom",
        }
    }

    /// The action in words.
    pub fn text(&self) -> String {
        match self {
            Action::Straight { mm, then } => {
                if *mm < 0.0 {
                    format!("back {} mm{}", fmt(-mm), then.suffix())
                } else {
                    format!("straight {} mm{}", fmt(*mm), then.suffix())
                }
            }
            Action::Turn { deg } => format!("turn {} {}°", if *deg >= 0.0 { "right" } else { "left" }, fmt(deg.abs())),
            Action::Curve { radius_mm, deg, then } => format!(
                "curve {} {}° on r {} mm{}{}",
                if *deg >= 0.0 { "right" } else { "left" },
                fmt(deg.abs()),
                fmt(radius_mm.abs()),
                if *radius_mm < 0.0 { " backward" } else { "" },
                then.suffix()
            ),
            Action::Stop { then, wait_ms } => {
                if *wait_ms > 0.0 {
                    format!("stop ({}), wait {} ms", then.label(), fmt(*wait_ms))
                } else {
                    format!("stop ({})", then.label())
                }
            }
            Action::Custom { code } => {
                let mut lines = code.lines().map(str::trim).filter(|l| !l.is_empty());
                match (lines.next(), lines.next()) {
                    (None, _) => "custom (type the call)".into(),
                    (Some(first), None) => first.to_string(),
                    (Some(first), Some(_)) => format!("{first} …"),
                }
            }
        }
    }

    /// The lines of program that drive it.
    pub fn code(&self) -> Vec<String> {
        match self {
            Action::Straight { mm, then } => vec![format!("db.straight({}{})", num(*mm), then.arg())],
            Action::Turn { deg } => vec![format!("db.turn({})", num(*deg))],
            Action::Curve { radius_mm, deg, then } => vec![format!("db.curve({}, {}{})", num(*radius_mm), num(*deg), then.arg())],
            Action::Stop { then, wait_ms } => {
                let mut v = vec![match then {
                    End::Coast => "db.stop()".to_string(),
                    other => format!("db.stop(then={})", other.code()),
                }];
                if *wait_ms > 0.0 {
                    v.push(format!("time.sleep_ms({})", wait_ms.round() as i64));
                }
                v
            }
            Action::Custom { code } => code
                .lines()
                .map(|l| l.trim_end().to_string())
                .filter(|l| !l.trim().is_empty())
                .collect(),
        }
    }

    /// Distance driven, mm.
    pub fn length_mm(&self) -> f64 {
        match self {
            Action::Straight { mm, .. } => mm.abs(),
            Action::Curve { radius_mm, deg, .. } => radius_mm.abs() * deg.abs().to_radians(),
            _ => 0.0,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct Route {
    pub format: String,
    #[serde(default)]
    pub world: String,
    pub start: Pose2,
    /// Python the custom actions call: functions, imports, constants.
    #[serde(default)]
    pub prelude: String,
    #[serde(default)]
    pub actions: Vec<Action>,
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

/// An action worked out: where it starts and ends, and the points that
/// draw its path.
#[derive(Clone, PartialEq, Debug)]
pub struct Step {
    pub start: Pose2,
    pub end: Pose2,
    pub points: Vec<[f64; 2]>,
}

fn fmt(v: f64) -> String {
    let r = (v * 10.0).round() / 10.0;
    if r == r.trunc() { format!("{}", r as i64) } else { format!("{r}") }
}

fn num(v: f64) -> String {
    let r = (v * 100.0).round() / 100.0;
    if r == r.trunc() { format!("{}", r as i64) } else { format!("{r}") }
}

fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

/// An angle wrapped into (-180, 180].
pub fn wrap_deg(a: f64) -> f64 {
    let d = a.rem_euclid(360.0);
    if d > 180.0 { d - 360.0 } else { d }
}

/// Where an action leaves the robot, and the path it draws.
pub fn advance(from: Pose2, action: &Action) -> Step {
    let (hx, hy) = from.heading();
    match action {
        Action::Straight { mm, .. } => {
            let end = Pose2 {
                x_mm: from.x_mm + hx * mm,
                y_mm: from.y_mm + hy * mm,
                yaw_deg: from.yaw_deg,
            };
            Step {
                start: from,
                end,
                points: vec![[from.x_mm, from.y_mm], [end.x_mm, end.y_mm]],
            }
        }
        Action::Turn { deg } => Step {
            start: from,
            end: Pose2 {
                yaw_deg: wrap_deg(from.yaw_deg - deg),
                ..from
            },
            points: vec![[from.x_mm, from.y_mm]],
        },
        Action::Curve { radius_mm, deg, .. } => {
            if radius_mm.abs() < EPS_MM || deg.abs() < 1e-9 {
                // a turn in place, or nothing
                return advance(from, &Action::Turn { deg: *deg });
            }
            // the rigid body turns about the circle's centre by the heading change; the centre
            // is on the right for a forward right turn and swaps side when driving backward
            let side = if (*deg > 0.0) == (*radius_mm > 0.0) { -1.0 } else { 1.0 };
            let (cx, cy) = (from.x_mm - hy * radius_mm.abs() * side, from.y_mm + hx * radius_mm.abs() * side);
            let theta = -deg.to_radians();
            let a0 = (from.y_mm - cy).atan2(from.x_mm - cx);
            let n = ((deg.abs() / 180.0 * 32.0).ceil() as usize).max(4);
            let mut points: Vec<[f64; 2]> = (0..=n)
                .map(|i| {
                    let a = a0 + theta * i as f64 / n as f64;
                    [cx + radius_mm.abs() * a.cos(), cy + radius_mm.abs() * a.sin()]
                })
                .collect();
            // the arc starts exactly where the robot is
            points[0] = [from.x_mm, from.y_mm];
            let last = points[n];
            Step {
                start: from,
                end: Pose2 {
                    x_mm: last[0],
                    y_mm: last[1],
                    yaw_deg: wrap_deg(from.yaw_deg - deg),
                },
                points,
            }
        }
        Action::Stop { .. } | Action::Custom { .. } => Step {
            start: from,
            end: from,
            points: vec![[from.x_mm, from.y_mm]],
        },
    }
}

/// Every action planned in turn from the start pose.
pub fn plan(route: &Route) -> Vec<Step> {
    let mut pose = route.start;
    route
        .actions
        .iter()
        .map(|a| {
            let s = advance(pose, a);
            pose = s.end;
            s
        })
        .collect()
}

/// The total distance driven, mm.
pub fn length_mm(route: &Route) -> f64 {
    route.actions.iter().map(Action::length_mm).sum()
}

/// The actions that drive to a point in a straight line: the turn to
/// face it (if any), then the distance. Nothing for the point the robot
/// is already at.
pub fn straight_to(from: Pose2, to: [f64; 2]) -> Vec<Action> {
    let (dx, dy) = (to[0] - from.x_mm, to[1] - from.y_mm);
    let dist = (dx * dx + dy * dy).sqrt();
    if dist < EPS_MM {
        return vec![];
    }
    let delta = wrap_deg(dy.atan2(dx).to_degrees() - from.yaw_deg);
    let mut out = Vec::new();
    if delta.abs() >= 0.05 {
        out.push(Action::Turn { deg: round1(-delta) });
    }
    out.push(Action::Straight {
        mm: round1(dist),
        then: End::Coast,
    });
    out
}

/// The arc tangent to the heading through a point, as an action, with
/// a note when there is no such arc: a point dead ahead is a line, a
/// point straight behind gets a turn and a line.
pub fn curve_to(from: Pose2, to: [f64; 2]) -> (Vec<Action>, Option<String>) {
    let (dx, dy) = (to[0] - from.x_mm, to[1] - from.y_mm);
    let dist = (dx * dx + dy * dy).sqrt();
    if dist < EPS_MM {
        return (vec![], Some("the point is where the robot already is".into()));
    }
    let (hx, hy) = from.heading();
    let cross = hx * dy - hy * dx;
    let dot = hx * dx + hy * dy;
    if cross.abs() < 1e-6 * dist {
        return if dot > 0.0 {
            (straight_to(from, to), Some("straight ahead: a line, not an arc".into()))
        } else {
            (
                straight_to(from, to),
                Some("straight behind: no arc reaches it, so a turn and a line".into()),
            )
        };
    }
    // the tangent-chord angle: the arc turns twice the angle between heading and chord
    let alpha = cross.abs().atan2(dot);
    let radius = dist / (2.0 * alpha.sin());
    let deg = if cross > 0.0 {
        -2.0 * alpha.to_degrees()
    } else {
        2.0 * alpha.to_degrees()
    };
    (
        vec![Action::Curve {
            radius_mm: round1(radius),
            deg: round1(deg),
            then: End::Coast,
        }],
        None,
    )
}

/// Where an action's handle sits on the map: a move's end point, a
/// turn's new heading; stops and custom calls have none.
pub fn handle(step: &Step, action: &Action) -> Option<[f64; 2]> {
    match action {
        Action::Straight { .. } | Action::Curve { .. } => Some([step.end.x_mm, step.end.y_mm]),
        Action::Turn { .. } => {
            let (hx, hy) = step.end.heading();
            Some([step.end.x_mm + hx * TURN_HANDLE_MM, step.end.y_mm + hy * TURN_HANDLE_MM])
        }
        _ => None,
    }
}

/// The action with its handle dragged to `to`: a straight keeps its
/// line and takes the signed distance along it, a turn faces the
/// point, a curve becomes the tangent arc through it (unchanged when
/// no arc reaches the point).
pub fn dragged(action: &Action, start: Pose2, to: [f64; 2]) -> Action {
    let (hx, hy) = start.heading();
    let (dx, dy) = (to[0] - start.x_mm, to[1] - start.y_mm);
    match action {
        Action::Straight { then, .. } => Action::Straight {
            mm: round1(hx * dx + hy * dy),
            then: *then,
        },
        Action::Turn { .. } => {
            if dx.abs() < EPS_MM && dy.abs() < EPS_MM {
                return action.clone();
            }
            Action::Turn {
                deg: round1(-wrap_deg(dy.atan2(dx).to_degrees() - start.yaw_deg)),
            }
        }
        Action::Curve { then, .. } => match curve_to(start, to) {
            (v, None) if v.len() == 1 => match &v[0] {
                Action::Curve { radius_mm, deg, .. } => Action::Curve {
                    radius_mm: *radius_mm,
                    deg: *deg,
                    then: *then,
                },
                other => other.clone(),
            },
            _ => action.clone(),
        },
        other => other.clone(),
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
    let stops = route.actions.iter().any(|a| match a {
        Action::Straight { then, .. } | Action::Curve { then, .. } | Action::Stop { then, .. } => *then != End::Coast,
        _ => false,
    });
    let waits = route
        .actions
        .iter()
        .any(|a| matches!(a, Action::Stop { wait_ms, .. } if *wait_ms > 0.0));
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
    for (i, a) in route.actions.iter().enumerate() {
        out.push_str(&format!("\n# {}: {}\n", i + 1, a.text()));
        for line in a.code() {
            out.push_str(&line);
            out.push('\n');
        }
    }
    out
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

    /// Read a route file; the first format's point segments become actions.
    pub fn load(path: &Path) -> Result<Route, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        match v.get("format").and_then(|f| f.as_str()) {
            Some(FORMAT) => serde_json::from_value(v).map_err(|e| e.to_string()),
            Some(FORMAT_V1) => {
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
                    let actions = if seg.kind == "curve" {
                        curve_to(pose, seg.to).0
                    } else {
                        straight_to(pose, seg.to)
                    };
                    for a in actions {
                        pose = advance(pose, &a).end;
                        route.actions.push(a);
                    }
                }
                Ok(route)
            }
            other => Err(format!("not a route file (format {other:?}, expected {FORMAT})")),
        }
    }

    pub fn save(&self, path: &Path) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(path, text).map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(x: f64, y: f64, yaw: f64) -> Pose2 {
        Pose2 {
            x_mm: x,
            y_mm: y,
            yaw_deg: yaw,
        }
    }

    fn straight(mm: f64) -> Action {
        Action::Straight { mm, then: End::Coast }
    }

    fn curve(radius_mm: f64, deg: f64) -> Action {
        Action::Curve {
            radius_mm,
            deg,
            then: End::Coast,
        }
    }

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    fn pose_near(p: Pose2, x: f64, y: f64, yaw: f64) -> bool {
        near(p.x_mm, x) && near(p.y_mm, y) && near(wrap_deg(p.yaw_deg - yaw), 0.0)
    }

    #[test]
    fn every_action_moves_the_pose_as_the_drive_base_would() {
        let from = at(0.0, 0.0, 0.0);
        let s = advance(from, &straight(300.0));
        assert!(pose_near(s.end, 300.0, 0.0, 0.0));
        assert_eq!(s.points, vec![[0.0, 0.0], [300.0, 0.0]]);
        assert!(pose_near(advance(from, &straight(-50.0)).end, -50.0, 0.0, 0.0), "negative backs up");
        assert!(
            pose_near(advance(from, &Action::Turn { deg: 90.0 }).end, 0.0, 0.0, -90.0),
            "right is clockwise"
        );
        assert!(pose_near(advance(from, &Action::Turn { deg: -90.0 }).end, 0.0, 0.0, 90.0));
        // a forward quarter circle to the right: the centre is at (0, -100)
        let c = advance(from, &curve(100.0, 90.0));
        assert!(pose_near(c.end, 100.0, -100.0, -90.0), "{:?}", c.end);
        for q in &c.points {
            assert!(near((q[0] * q[0] + (q[1] + 100.0) * (q[1] + 100.0)).sqrt(), 100.0), "{q:?}");
        }
        assert_eq!(c.points[0], [0.0, 0.0]);
        // to the left
        let c = advance(from, &curve(100.0, -90.0));
        assert!(pose_near(c.end, 100.0, 100.0, 90.0), "{:?}", c.end);
        // backward, still turning right: the centre swaps to the left and the robot backs up
        let c = advance(from, &curve(-100.0, 90.0));
        assert!(pose_near(c.end, -100.0, 100.0, -90.0), "{:?}", c.end);
        // a zero radius is a turn in place; a zero angle goes nowhere
        assert!(pose_near(advance(from, &curve(0.0, 45.0)).end, 0.0, 0.0, -45.0));
        assert!(pose_near(advance(from, &curve(100.0, 0.0)).end, 0.0, 0.0, 0.0));
        // stops and custom calls stay put
        let stop = Action::Stop {
            then: End::Brake,
            wait_ms: 500.0,
        };
        assert_eq!(advance(from, &stop).end, from);
        assert_eq!(advance(from, &Action::Custom { code: "grab()".into() }).end, from);
        // turned frames: the same geometry rotated
        let c = advance(at(10.0, 20.0, 90.0), &curve(100.0, -90.0));
        assert!(pose_near(c.end, -90.0, 120.0, 180.0), "{:?}", c.end);
        // the end state changes nothing about where the robot goes
        let mut cont = straight(300.0);
        if let Action::Straight { then, .. } = &mut cont {
            *then = End::Continue;
        }
        assert_eq!(advance(from, &cont).end, s.end);
    }

    #[test]
    fn the_point_tools_turn_a_click_into_actions() {
        let from = at(0.0, 0.0, 0.0);
        assert_eq!(straight_to(from, [300.0, 0.0]), vec![straight(300.0)], "dead ahead: no turn");
        assert_eq!(
            straight_to(from, [0.0, 100.0]),
            vec![Action::Turn { deg: -90.0 }, straight(100.0)],
            "left: a left turn, then the drive"
        );
        assert_eq!(straight_to(from, [0.0, -100.0])[0], Action::Turn { deg: 90.0 });
        assert_eq!(straight_to(from, [-100.0, 0.0])[0], Action::Turn { deg: -180.0 });
        assert!(straight_to(from, [0.1, 0.1]).is_empty());
        let (a, note) = curve_to(from, [100.0, 100.0]);
        assert_eq!(a, vec![curve(100.0, -90.0)]);
        assert!(note.is_none());
        assert_eq!(curve_to(from, [100.0, -100.0]).0, vec![curve(100.0, 90.0)]);
        assert_eq!(curve_to(from, [0.0, 80.0]).0, vec![curve(40.0, -180.0)], "beside: a half circle");
        let (a, note) = curve_to(at(0.0, 0.0, 45.0), [100.0, 100.0]);
        assert_eq!(a.len(), 1);
        assert!(matches!(a[0], Action::Straight { .. }) && note.unwrap().contains("straight ahead"));
        let (a, note) = curve_to(at(0.0, 0.0, 45.0), [-100.0, -100.0]);
        assert_eq!(a[0], Action::Turn { deg: -180.0 });
        assert!(note.unwrap().contains("behind"));
        assert!(curve_to(from, [0.0, 0.0]).0.is_empty());
        // the curve tool's action lands on the point when advanced
        let (a, _) = curve_to(at(10.0, 20.0, 90.0), [-90.0, 120.0]);
        let end = advance(at(10.0, 20.0, 90.0), &a[0]).end;
        assert!(pose_near(end, -90.0, 120.0, 180.0), "{end:?}");
    }

    #[test]
    fn handles_sit_at_the_ends_and_dragging_them_rewrites_the_action() {
        let from = at(0.0, 0.0, 0.0);
        let s = Action::Straight {
            mm: 200.0,
            then: End::Continue,
        };
        let step = advance(from, &s);
        assert_eq!(handle(&step, &s), Some([200.0, 0.0]));
        // a straight keeps its line and its end state: the pointer's projection is the new distance, negative behind
        assert_eq!(
            dragged(&s, from, [150.0, 40.0]),
            Action::Straight {
                mm: 150.0,
                then: End::Continue
            }
        );
        assert_eq!(
            dragged(&s, from, [-30.0, 10.0]),
            Action::Straight {
                mm: -30.0,
                then: End::Continue
            }
        );
        let turn = Action::Turn { deg: 90.0 };
        let step = advance(from, &turn);
        let h = handle(&step, &turn).unwrap();
        assert!(near(h[0], 0.0) && near(h[1], -TURN_HANDLE_MM), "{h:?}");
        assert_eq!(dragged(&turn, from, [0.0, 100.0]), Action::Turn { deg: -90.0 });
        assert_eq!(dragged(&turn, from, [-100.0, 0.0]), Action::Turn { deg: -180.0 });
        assert_eq!(dragged(&turn, from, [0.1, 0.0]), turn, "a point on the robot changes nothing");
        let c = Action::Curve {
            radius_mm: 150.0,
            deg: 90.0,
            then: End::Brake,
        };
        assert_eq!(
            dragged(&c, from, [100.0, 100.0]),
            Action::Curve {
                radius_mm: 100.0,
                deg: -90.0,
                then: End::Brake
            }
        );
        assert_eq!(dragged(&c, from, [100.0, 0.0]), c, "no arc reaches a point dead ahead: unchanged");
        assert_eq!(dragged(&c, from, [0.0, 0.0]), c);
        let stop = Action::Stop {
            then: End::Hold,
            wait_ms: 0.0,
        };
        assert_eq!(handle(&advance(from, &stop), &stop), None);
        assert_eq!(dragged(&stop, from, [5.0, 5.0]), stop);
    }

    #[test]
    fn the_program_drives_the_actions_with_only_the_imports_it_needs() {
        let mut route = Route {
            world: "practice-line".into(),
            start: at(-547.0, -150.0, 90.0),
            actions: vec![straight(250.0), Action::Turn { deg: -45.5 }],
            ..Default::default()
        };
        let text = program(&route, 86.4, 135.0);
        assert!(text.contains("2 actions from (-547, -150) mm heading 90°"));
        assert!(!text.contains("import time") && !text.contains("parameters import Stop"), "{text}");
        assert!(
            text.contains("db = DriveBase(left, right, wheel_diameter_mm=86.4, axle_track_mm=135)\n"),
            "{text}"
        );
        assert!(
            text.contains("# 1: straight 250 mm\ndb.straight(250)\n\n# 2: turn left 45.5°\ndb.turn(-45.5)\n"),
            "{text}"
        );
        // continuous mode: the move ends at cruise speed and the next one carries it on
        route.actions[0] = Action::Straight {
            mm: 250.0,
            then: End::Continue,
        };
        let text = program(&route, 86.4, 135.0);
        assert!(text.contains("from openbricks.parameters import Stop\n"), "{text}");
        assert!(
            text.contains("# 1: straight 250 mm, then continue\ndb.straight(250, then=Stop.NONE)\n"),
            "{text}"
        );
        route.actions.push(Action::Stop {
            then: End::Brake,
            wait_ms: 500.0,
        });
        route.actions.push(Action::Curve {
            radius_mm: -120.0,
            deg: 30.0,
            then: End::Hold,
        });
        route.actions.push(Action::Custom {
            code: "grab()\n  \nrelease()  \n".into(),
        });
        route.prelude = "def grab():\n    print('grab')\n\ndef line_follow(speed=120):\n    db.drive(speed, 0)\nX = 1\n  def indented_helper(a, b):\n    pass\ndefault = 2\n".into();
        assert_eq!(route.functions(), vec!["grab", "line_follow", "indented_helper"]);
        let text = program(&route, 86.4, 135.0);
        assert!(text.starts_with("\"\"\"A route"), "{text}");
        assert!(text.contains("import time\n"), "{text}");
        assert!(text.contains("from openbricks.parameters import Stop\n"), "{text}");
        assert!(
            text.contains("db = DriveBase(left, right, wheel_diameter_mm=86.4, axle_track_mm=135)\n\n# definitions: what the custom actions call\ndef grab():\n    print('grab')\n"),
            "{text}"
        );
        assert!(
            text.contains("# 3: stop (brake), wait 500 ms\ndb.stop(then=Stop.BRAKE)\ntime.sleep_ms(500)\n"),
            "{text}"
        );
        assert!(
            text.contains("# 4: curve right 30° on r 120 mm backward, then hold\ndb.curve(-120, 30, then=Stop.HOLD)\n"),
            "{text}"
        );
        assert!(text.contains("# 5: grab() …\ngrab()\nrelease()\n"), "{text}");
        assert_eq!(
            Action::Custom {
                code: "  grab()  \n".into()
            }
            .text(),
            "grab()"
        );
        assert_eq!(
            Action::Stop {
                then: End::Coast,
                wait_ms: 0.0
            }
            .code(),
            vec!["db.stop()".to_string()]
        );
        assert_eq!(Action::Custom { code: String::new() }.text(), "custom (type the call)");
        assert_eq!(straight(-80.0).text(), "back 80 mm");
        assert!(near(length_mm(&route), 250.0 + 120.0 * 30f64.to_radians()));
        for k in KINDS {
            assert_eq!(Action::default_of(k).kind(), k);
        }
        assert_eq!(End::STOPS.map(|e| e.label()), ["coast", "brake", "hold"]);
        assert_eq!(End::MOVES.map(|e| e.label()), ["coast", "brake", "hold", "continue"]);
    }

    #[test]
    fn routes_save_load_convert_the_first_format_and_refuse_other_files() {
        let dir = std::env::temp_dir().join(format!("ob-route-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let route = Route {
            world: "wro-2026-senior".into(),
            start: at(1.5, -2.0, 30.0),
            prelude: "X = 1".into(),
            actions: vec![
                Action::Curve {
                    radius_mm: 100.0,
                    deg: 50.0,
                    then: End::Continue,
                },
                Action::Stop {
                    then: End::Hold,
                    wait_ms: 250.0,
                },
                Action::Custom { code: "grab()".into() },
            ],
            ..Default::default()
        };
        let path = dir.join("lap.route.json");
        route.save(&path).unwrap();
        assert_eq!(Route::load(&path).unwrap(), route);
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(
            text.contains("\"format\": \"openbricks-route/2\"")
                && text.contains("\"kind\": \"curve\"")
                && text.contains("\"then\": \"hold\"")
        );
        assert!(text.contains("\"then\": \"continue\""), "{text}");
        // a move without an end state in the file coasts
        std::fs::write(
            dir.join("bare.route.json"),
            r#"{"format": "openbricks-route/2", "start": {"x_mm": 0, "y_mm": 0, "yaw_deg": 0}, "actions": [{"kind": "straight", "mm": 10}]}"#,
        )
        .unwrap();
        assert_eq!(Route::load(&dir.join("bare.route.json")).unwrap().actions, vec![straight(10.0)]);
        // the first format: point segments become the actions the point tools would make
        std::fs::write(
            dir.join("old.route.json"),
            r#"{"format": "openbricks-route/1", "world": "practice-line", "start": {"x_mm": 0, "y_mm": 0, "yaw_deg": 0},
                "segments": [{"kind": "straight", "to": [0, 100]}, {"kind": "curve", "to": [100, 200]}]}"#,
        )
        .unwrap();
        let old = Route::load(&dir.join("old.route.json")).unwrap();
        assert_eq!(old.world, "practice-line");
        assert_eq!(old.actions, vec![Action::Turn { deg: -90.0 }, straight(100.0), curve(100.0, 90.0)]);
        let end = plan(&old).last().unwrap().end;
        assert!(pose_near(end, 100.0, 200.0, 0.0), "{end:?}");
        std::fs::write(
            dir.join("other.json"),
            r#"{"format": "openbricks-assembly/1", "start": {"x_mm": 0, "y_mm": 0, "yaw_deg": 0}}"#,
        )
        .unwrap();
        assert!(Route::load(&dir.join("other.json")).unwrap_err().contains("not a route file"));
        assert!(Route::load(&dir.join("missing.json")).is_err());
        std::fs::write(dir.join("bad.json"), "nope").unwrap();
        assert!(Route::load(&dir.join("bad.json")).is_err());
        assert!(route.save(&dir.join("no-such-dir").join("x.json")).is_err());
        assert_eq!(wrap_deg(190.0), -170.0);
        assert_eq!(wrap_deg(-180.0), 180.0);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
