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

pub const FORMAT: &str = "openbricks-route/4";
const FORMAT_V3: &str = "openbricks-route/3";
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
    /// A smooth drive from `start`, entered facing `heading_deg` (the way
    /// the robot arrives there; the tab keeps it in step with the previous
    /// action), to `end` facing `end_heading_deg`: one arc when the end
    /// pose lies on the circle tangent to the start pose, else two arcs
    /// meeting tangentially — see [`biarc`].
    Curve {
        start: Point,
        #[serde(default)]
        heading_deg: f64,
        end: Point,
        #[serde(default)]
        end_heading_deg: f64,
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

/// An action on the map, whether it is locked against editing, and the
/// colour its path is drawn in when not the kind's default.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Item {
    #[serde(flatten)]
    pub action: Action,
    #[serde(default, skip_serializing_if = "is_false")]
    pub locked: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<[u8; 3]>,
}

impl From<Action> for Item {
    fn from(action: Action) -> Self {
        Item {
            action,
            locked: false,
            color: None,
        }
    }
}

/// The kinds, their tool labels, and what each asks the user to click.
pub const KINDS: [(&str, &str, &str); 5] = [
    ("straight", "→ Straight", "click where it starts, then where it ends"),
    (
        "curve",
        "⌒ Curve",
        "click where it starts, where it ends, then a point to face there",
    ),
    ("turn", "↻ Turn", "click where it turns, then a point to face"),
    ("stop", "■ Stop", "click where it stops"),
    ("custom", "ƒ Custom", "click where it runs"),
];

/// How many map clicks place a kind.
pub fn clicks_needed(kind: &str) -> usize {
    match kind {
        "straight" | "turn" => 2,
        "curve" => 3,
        _ => 1,
    }
}

/// A handle on an action: which point a drag moves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Handle {
    Start,
    End,
    At,
    /// A turn's heading, or the heading a curve ends facing, `TURN_HANDLE_MM` along it.
    Face,
    /// The whole action.
    Body,
}

/// A curve's geometry: its centre, the angle it sweeps, the headings at
/// its ends, and where it ends.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arc {
    pub centre: Point,
    pub radius_mm: f64,
    /// The angle swept, degrees, always positive; `right` says which way.
    pub sweep_deg: f64,
    pub start_heading: f64,
    pub end_heading: f64,
    pub end: Point,
}

fn dist(a: Point, b: Point) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

fn heading_to(a: Point, b: Point) -> f64 {
    (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees()
}

/// The arc a robot drives from `start` facing `heading_deg`: `angle_deg`
/// of a circle of `radius_mm` to its right or its left; None when there
/// is nothing to drive (no radius, or no angle). The end is rounded to
/// 0.1 mm, as placed points are.
pub fn arc_from(start: Point, heading_deg: f64, radius_mm: f64, angle_deg: f64, right: bool) -> Option<Arc> {
    let angle = angle_deg.abs();
    if radius_mm < EPS_MM || angle < 1e-6 {
        return None;
    }
    let (c, s) = (heading_deg.to_radians().cos(), heading_deg.to_radians().sin());
    // the centre lies on the normal to the heading, on the side the arc bends to
    let side = if right { -1.0 } else { 1.0 };
    let centre = [start[0] - s * radius_mm * side, start[1] + c * radius_mm * side];
    let a0 = (start[1] - centre[1]).atan2(start[0] - centre[0]);
    let a1 = a0 + angle.to_radians() * side;
    Some(Arc {
        centre,
        radius_mm,
        sweep_deg: angle,
        start_heading: wrap_deg(heading_deg),
        end_heading: wrap_deg(heading_deg + angle * side),
        end: [round1(centre[0] + radius_mm * a1.cos()), round1(centre[1] + radius_mm * a1.sin())],
    })
}

/// The arc through two points of a radius (no shorter than half the
/// chord, else it is lengthened to that) bending to the right or the
/// left; None when the ends coincide. How the third format placed a
/// curve; kept to read those files.
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
        end,
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

/// One piece of a curve: an arc, or a straight run where the poses line
/// up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Piece {
    Arc { from: Point, arc: Arc, right: bool },
    Line { from: Point, to: Point },
}

impl Piece {
    pub fn end(&self) -> Point {
        match self {
            Piece::Arc { arc, .. } => arc.end,
            Piece::Line { to, .. } => *to,
        }
    }

    /// The heading at its end, given the one at its start.
    pub fn end_heading(&self, start_heading: f64) -> f64 {
        match self {
            Piece::Arc { arc, .. } => arc.end_heading,
            Piece::Line { .. } => start_heading,
        }
    }

    pub fn length_mm(&self) -> f64 {
        match self {
            Piece::Arc { arc, .. } => arc.radius_mm * arc.sweep_deg.to_radians(),
            Piece::Line { from, to } => dist(*from, *to),
        }
    }

    /// Its path, start to end.
    pub fn points(&self) -> Vec<Point> {
        match self {
            Piece::Arc { from, arc, right } => {
                let mut pts = arc_points(arc, *from, *right);
                pts[0] = *from;
                if let Some(last) = pts.last_mut() {
                    *last = arc.end;
                }
                pts
            }
            Piece::Line { from, to } => vec![*from, *to],
        }
    }
}

/// The arc from a pose that a tangent fit found, ending exactly at `to`,
/// or the straight run there when the fit found none (or the arc is too
/// small to be one).
fn arc_or_line(from: Point, heading_deg: f64, to: Point, fit: Option<(f64, f64, bool)>) -> Piece {
    match fit.and_then(|(r, a, right)| arc_from(from, heading_deg, r, a, right).map(|arc| (arc, right))) {
        Some((mut arc, right)) => {
            arc.end = to;
            Piece::Arc { from, arc, right }
        }
        None => Piece::Line { from, to },
    }
}

/// The pieces a curve drives from one pose to another: the single arc
/// when the end pose lies on the circle tangent to the start pose (the
/// straight run when they line up), else two arcs meeting tangentially
/// — the biarc with equal tangent lengths from both ends, which is the
/// single arc again whenever one exists — a piece flattening to a
/// straight run where its ends line up. Nothing when the poses share a
/// point.
pub fn biarc(start: Point, heading_deg: f64, end: Point, end_heading_deg: f64) -> Vec<Piece> {
    let v = [end[0] - start[0], end[1] - start[1]];
    let vv = v[0] * v[0] + v[1] * v[1];
    if vv.sqrt() < EPS_MM {
        return vec![];
    }
    let from = Pose2::at(start, heading_deg);
    let (h1, h2) = (from.heading(), Pose2::at(end, end_heading_deg).heading());
    let turn = wrap_deg(end_heading_deg - heading_deg);
    // one arc, or one line
    let fit = tangent_arc(from, end);
    match fit {
        Some((_, a, right)) => {
            if wrap_deg(if right { -a } else { a } - turn).abs() < 0.5 {
                return vec![arc_or_line(start, heading_deg, end, fit)];
            }
        }
        None => {
            if turn.abs() < 0.5 && v[0] * h1.0 + v[1] * h1.1 > 0.0 {
                return vec![Piece::Line { from: start, to: end }];
            }
        }
    }
    // the joint: equal tangent lengths d from both ends — the chord's midpoint when the headings
    // are parallel, where that equation has no root
    let dot = h1.0 * h2.0 + h1.1 * h2.1;
    let joint = if 1.0 - dot < 1e-9 {
        [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0]
    } else {
        let vt = v[0] * (h1.0 + h2.0) + v[1] * (h1.1 + h2.1);
        let d = (-vt + (vt * vt + 2.0 * (1.0 - dot) * vv).sqrt()) / (2.0 * (1.0 - dot));
        [
            (start[0] + d * h1.0 + end[0] - d * h2.0) / 2.0,
            (start[1] + d * h1.1 + end[1] - d * h2.1) / 2.0,
        ]
    };
    let first = arc_or_line(start, heading_deg, joint, tangent_arc(from, joint));
    let (mid, mid_heading) = (first.end(), first.end_heading(heading_deg));
    // the second arc, fitted backwards from the end pose: the same radius and angle, the other side
    let back = tangent_arc(Pose2::at(end, end_heading_deg + 180.0), mid).map(|(r, a, side)| (r, a, !side));
    let mut second = arc_or_line(mid, mid_heading, end, back);
    if let Piece::Arc { arc, .. } = &mut second {
        arc.end_heading = wrap_deg(end_heading_deg);
    }
    vec![first, second]
}

impl Action {
    /// A kind placed by its clicks, with default parameters: a curve runs
    /// from the first click to the second and faces the third there —
    /// before that click, the way the arc tangent to `heading_deg` (how
    /// the robot arrives at the first click) through the second faces;
    /// a turn faces the second click.
    pub fn placed(kind: &str, points: &[Point], heading_deg: f64) -> Action {
        let p = |i: usize| points.get(i).copied().unwrap_or([0.0, 0.0]);
        match kind {
            "straight" => Action::Straight {
                start: p(0),
                end: p(1),
                speed: DEFAULT_STRAIGHT_DPS,
                then: End::Coast,
            },
            "curve" => {
                let (start, end) = (p(0), p(1));
                let facing = points.get(2).filter(|f| dist(**f, end) >= EPS_MM).map(|f| heading_to(end, *f));
                let end_heading_deg = facing.unwrap_or_else(|| match tangent_arc(Pose2::at(start, heading_deg), end) {
                    Some((_, a, right)) => heading_deg + if right { -a } else { a },
                    None => heading_deg,
                });
                Action::Curve {
                    start,
                    heading_deg: round1(wrap_deg(heading_deg)),
                    end,
                    end_heading_deg: round1(wrap_deg(end_heading_deg)),
                    speed: DEFAULT_STRAIGHT_DPS,
                    then: End::Coast,
                }
            }
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

    /// The pieces a curve drives; nothing for the other kinds.
    pub fn pieces(&self) -> Vec<Piece> {
        match self {
            Action::Curve {
                start,
                heading_deg,
                end,
                end_heading_deg,
                ..
            } => biarc(*start, *heading_deg, *end, *end_heading_deg),
            _ => vec![],
        }
    }

    /// The heading it must start with, when it has one.
    pub fn start_heading(&self) -> Option<f64> {
        match self {
            Action::Straight { start, end, .. } => (dist(*start, *end) >= EPS_MM).then(|| heading_to(*start, *end)),
            Action::Curve { heading_deg, .. } => Some(wrap_deg(*heading_deg)),
            Action::Custom { at, end: Some(e), .. } => (dist(*at, *e) >= EPS_MM).then(|| heading_to(*at, *e)),
            _ => None,
        }
    }

    /// The heading it leaves the robot with, given the one it started with.
    pub fn end_heading(&self, start_heading: f64) -> f64 {
        match self {
            Action::Curve { end_heading_deg, .. } => wrap_deg(*end_heading_deg),
            Action::Turn { heading_deg, .. } => wrap_deg(*heading_deg),
            _ => self.start_heading().unwrap_or(start_heading),
        }
    }

    /// The path it drives, for drawing and hit tests.
    pub fn path(&self) -> Vec<Point> {
        match self {
            Action::Straight { start, end, .. } => vec![*start, *end],
            Action::Curve { start, .. } => {
                let mut pts = vec![*start];
                for piece in self.pieces() {
                    pts.extend(piece.points().into_iter().skip(1));
                }
                pts
            }
            Action::Custom { at, end: Some(e), .. } => vec![*at, *e],
            other => vec![other.start()],
        }
    }

    /// Distance driven, mm.
    pub fn length_mm(&self) -> f64 {
        match self {
            Action::Straight { start, end, .. } => dist(*start, *end),
            Action::Curve { .. } => self.pieces().iter().map(Piece::length_mm).sum(),
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
            Action::Curve { speed, then, .. } => {
                let pieces = self.pieces();
                let what = if pieces.is_empty() {
                    "curve (nowhere to go)".to_string()
                } else {
                    pieces.iter().map(piece_text).collect::<Vec<_>>().join(", then ")
                };
                format!("{what} at {}°/s{}", fmt(*speed), then.suffix())
            }
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

    /// The key parameters, short enough to sit beside the path on the map.
    pub fn brief(&self) -> String {
        match self {
            Action::Straight { .. } => format!("{} mm", fmt(self.length_mm())),
            Action::Curve { .. } => {
                let pieces = self.pieces();
                match pieces.as_slice() {
                    [] => "curve".into(),
                    [Piece::Arc { arc, .. }] => format!("r {} mm · {}°", fmt(arc.radius_mm), fmt(arc.sweep_deg)),
                    many => many
                        .iter()
                        .map(|p| match p {
                            Piece::Arc { arc, .. } => format!("r {}", fmt(arc.radius_mm)),
                            Piece::Line { from, to } => format!("{} mm", fmt(dist(*from, *to))),
                        })
                        .collect::<Vec<_>>()
                        .join(" · "),
                }
            }
            Action::Turn { heading_deg, .. } => format!("face {}°", fmt(*heading_deg)),
            Action::Stop { wait_ms, .. } => {
                if *wait_ms > 0.0 {
                    format!("stop, {} ms", fmt(*wait_ms))
                } else {
                    "stop".into()
                }
            }
            Action::Custom { code, end, .. } => {
                let call = code.lines().map(str::trim).find(|l| !l.is_empty()).unwrap_or("custom").to_string();
                if end.is_some() {
                    format!("{call}, {} mm", fmt(self.length_mm()))
                } else {
                    call
                }
            }
        }
    }

    /// Where a label about the action sits: the middle of its path, or
    /// the point itself.
    pub fn label_point(&self) -> Point {
        let pts = self.path();
        pts[pts.len() / 2]
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
            Action::Curve {
                start,
                end,
                end_heading_deg,
                ..
            } => {
                let r = end_heading_deg.to_radians();
                vec![
                    (Handle::Start, *start),
                    (Handle::End, *end),
                    (Handle::Face, [end[0] + r.cos() * TURN_HANDLE_MM, end[1] + r.sin() * TURN_HANDLE_MM]),
                ]
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

    /// Move a handle to `to`: an end point moves (a curve's start carries
    /// its end along), a face handle sets a turn's heading or the heading
    /// a curve ends facing.
    pub fn drag(&mut self, handle: Handle, to: Point) {
        let to = [round1(to[0]), round1(to[1])];
        match (self, handle) {
            (Action::Straight { start, .. }, Handle::Start) => *start = to,
            (Action::Curve { start, end, .. }, Handle::Start) => {
                let (dx, dy) = (to[0] - start[0], to[1] - start[1]);
                *start = to;
                *end = [round1(end[0] + dx), round1(end[1] + dy)];
            }
            (Action::Straight { end, .. }, Handle::End) | (Action::Curve { end, .. }, Handle::End) => *end = to,
            (Action::Curve { end, end_heading_deg, .. }, Handle::Face) => {
                if dist(*end, to) >= EPS_MM {
                    *end_heading_deg = round1(heading_to(*end, to));
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
            Action::Curve { end_heading_deg, then, .. } => {
                let pieces = self.pieces();
                if pieces.is_empty() {
                    // nowhere to drive: only a heading to take up
                    let rel = wrap_deg(end_heading_deg - from.yaw_deg);
                    return if rel.abs() < 0.05 {
                        vec![]
                    } else {
                        vec![format!("db.turn({})", num(-rel))]
                    };
                }
                let mut heading = from.yaw_deg;
                let mut out = Vec::new();
                for (i, p) in pieces.iter().enumerate() {
                    // the pieces flow into one another; the action's own end comes last
                    let arg = if i + 1 == pieces.len() { then.arg() } else { End::Continue.arg() };
                    out.push(match p {
                        Piece::Arc { arc, right, .. } => format!(
                            "db.curve({}, {}{arg})",
                            num(arc.radius_mm),
                            num(if *right { arc.sweep_deg } else { -arc.sweep_deg })
                        ),
                        Piece::Line { from, to } => {
                            let (c, s) = (heading.to_radians().cos(), heading.to_radians().sin());
                            // backward when the run lies behind the heading
                            format!("db.straight({}{arg})", num((to[0] - from[0]) * c + (to[1] - from[1]) * s))
                        }
                    });
                    heading = p.end_heading(heading);
                }
                out
            }
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

/// One piece of a curve in words.
fn piece_text(p: &Piece) -> String {
    match p {
        Piece::Arc { arc, right, .. } => format!(
            "curve {} {}° on r {} mm",
            if *right { "right" } else { "left" },
            fmt(arc.sweep_deg),
            fmt(arc.radius_mm)
        ),
        Piece::Line { from, to } => format!("straight {} mm", fmt(dist(*from, *to))),
    }
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
pub fn approach(from: Pose2, to: Point, heading: Option<f64>) -> (Vec<Point>, Vec<String>, Pose2) {
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
        // a curve of two pieces flows from one into the other with then=Stop.NONE
        Action::Curve { then, .. } => *then != End::Coast || i.action.pieces().len() > 1,
        Action::Straight { then, .. } | Action::Stop { then, .. } => *then != End::Coast,
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

/// The arc tangent to a pose through a point: its radius, the angle it
/// sweeps and its side; None when the point is on the spot, dead ahead
/// or behind (a line, not an arc).
pub fn tangent_arc(from: Pose2, to: Point) -> Option<(f64, f64, bool)> {
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
    Some((round1(d / (2.0 * alpha.sin())), round1(2.0 * alpha.to_degrees()), cross < 0.0))
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
            Some(FORMAT_V3) => from_v3(v),
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

/// The third format placed a curve by its two ends, a radius and a side;
/// the arc keeps its shape here: its ends and the headings at them.
fn from_v3(mut v: serde_json::Value) -> Result<Route, String> {
    if let Some(actions) = v.get_mut("actions").and_then(|a| a.as_array_mut()) {
        for a in actions.iter_mut().filter_map(|a| a.as_object_mut()) {
            if a.get("kind").and_then(|k| k.as_str()) != Some("curve") {
                continue;
            }
            let point = |k: &str| -> Option<Point> {
                let p = a.get(k)?.as_array()?;
                Some([p.first()?.as_f64()?, p.get(1)?.as_f64()?])
            };
            let (Some(start), Some(end)) = (point("start"), point("end")) else {
                return Err("a curve without its two ends".into());
            };
            let radius = a.get("radius_mm").and_then(|r| r.as_f64()).unwrap_or(0.0);
            let right = a.get("right").and_then(|r| r.as_bool()).unwrap_or(false);
            let (heading, end_heading) = match arc_of(start, end, radius, right) {
                Some(arc) => (arc.start_heading, arc.end_heading),
                None => (0.0, 0.0),
            };
            a.remove("radius_mm");
            a.remove("right");
            a.insert("heading_deg".into(), serde_json::json!(round1(heading)));
            a.insert("end_heading_deg".into(), serde_json::json!(round1(end_heading)));
        }
    }
    v["format"] = serde_json::json!(FORMAT);
    serde_json::from_value(v).map_err(|e| e.to_string())
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
            ("curve", Some((_, angle_deg, right))) => Action::Curve {
                start: pose.point(),
                heading_deg: round1(pose.yaw_deg),
                end: seg.to,
                end_heading_deg: round1(wrap_deg(pose.yaw_deg + if right { -angle_deg } else { angle_deg })),
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
                // a positive angle was a forward right turn; driving backward it bent the other way
                let right = (deg > 0.0) == (radius_mm > 0.0);
                let arc = arc_from(pose.point(), pose.yaw_deg, radius_mm.abs(), deg.abs(), right).expect("a radius and an angle");
                Action::Curve {
                    start: pose.point(),
                    heading_deg: round1(pose.yaw_deg),
                    end: arc.end,
                    end_heading_deg: round1(arc.end_heading),
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
        Action::placed("straight", &[start, end], 0.0)
    }

    #[test]
    fn placed_kinds_take_their_defaults_from_the_clicks() {
        let s = straight([0.0, 0.0], [300.0, 0.0]);
        assert_eq!(s.start(), [0.0, 0.0]);
        assert_eq!(s.end(), [300.0, 0.0]);
        assert_eq!(s.start_heading(), Some(0.0));
        assert!(near(s.length_mm(), 300.0));
        assert_eq!(s.text(), "straight 300 mm at 350°/s");
        // a curve from (0, 0) arriving facing +y, to (100, 100): before the third click it faces
        // the way the tangent arc does there (+x) — a right quarter circle of r 100
        let c = Action::placed("curve", &[[0.0, 0.0], [100.0, 100.0]], 90.0);
        let Action::Curve {
            heading_deg,
            end,
            end_heading_deg,
            ..
        } = &c
        else {
            panic!()
        };
        assert_eq!((heading_deg, end, end_heading_deg), (&90.0, &[100.0, 100.0], &0.0), "{c:?}");
        let pieces = c.pieces();
        let [Piece::Arc { arc, right, .. }] = pieces[..] else {
            panic!("{pieces:?}")
        };
        assert!(right && near(arc.radius_mm, 100.0) && near(arc.sweep_deg, 90.0), "{arc:?}");
        assert_eq!((c.start(), c.end()), ([0.0, 0.0], [100.0, 100.0]));
        assert_eq!((c.start_heading(), c.end_heading(0.0)), (Some(90.0), 0.0));
        assert_eq!(c.text(), "curve right 90° on r 100 mm at 350°/s");
        // the third click says which way to face at the end: facing +y there takes two arcs
        let two = Action::placed("curve", &[[0.0, 0.0], [100.0, 100.0], [100.0, 200.0]], 90.0);
        assert_eq!(two.end_heading(0.0), 90.0);
        assert_eq!(two.pieces().len(), 2, "{:?}", two.pieces());
        assert!(
            two.text().starts_with("curve ") && two.text().contains(", then curve "),
            "{}",
            two.text()
        );
        assert!(two.brief().starts_with("r ") && two.brief().contains(" · r "), "{}", two.brief());
        // a third click on the end itself decides nothing
        assert_eq!(Action::placed("curve", &[[0.0, 0.0], [100.0, 100.0], [100.0, 100.0]], 90.0), c);
        // a point dead ahead, facing the same way: a straight run
        let d = Action::placed("curve", &[[0.0, 0.0], [100.0, 0.0]], 0.0);
        assert_eq!(
            d.pieces(),
            vec![Piece::Line {
                from: [0.0, 0.0],
                to: [100.0, 0.0]
            }]
        );
        assert_eq!(d.text(), "straight 100 mm at 350°/s");
        assert_eq!(d.brief(), "100 mm");
        let none = Action::placed("curve", &[[5.0, 5.0], [5.0, 5.0]], 0.0);
        assert!(none.pieces().is_empty() && none.path() == vec![[5.0, 5.0]] && none.length_mm() == 0.0);
        assert_eq!(none.brief(), "curve");
        assert_eq!(none.text(), "curve (nowhere to go) at 350°/s");
        let t = Action::placed("turn", &[[10.0, 10.0], [10.0, 60.0]], 0.0);
        assert_eq!(
            t,
            Action::Turn {
                at: [10.0, 10.0],
                heading_deg: 90.0,
                speed: 300.0
            }
        );
        assert_eq!(Action::placed("stop", &[[1.0, 2.0]], 0.0).text(), "stop (coast)");
        // the short labels beside the paths, and where they sit
        assert_eq!(s.brief(), "300 mm");
        assert_eq!(s.label_point(), [300.0, 0.0]);
        assert_eq!(c.brief(), "r 100 mm · 90°");
        assert_eq!(t.brief(), "face 90°");
        assert_eq!(t.label_point(), [10.0, 10.0]);
        assert_eq!(Action::placed("stop", &[[1.0, 2.0]], 0.0).brief(), "stop");
        assert_eq!(
            Action::Stop {
                at: [0.0, 0.0],
                then: End::Hold,
                wait_ms: 500.0
            }
            .brief(),
            "stop, 500 ms"
        );
        assert_eq!(Action::placed("custom", &[[1.0, 2.0]], 0.0).brief(), "custom");
        assert_eq!(
            Action::Custom {
                at: [0.0, 0.0],
                end: Some([0.0, 120.0]),
                code: "\nline_follow()\n".into()
            }
            .brief(),
            "line_follow(), 120 mm"
        );
        assert_eq!(Action::placed("custom", &[[1.0, 2.0]], 0.0).text(), "custom (type the call)");
        for (kind, _, _) in KINDS {
            assert_eq!(Action::placed(kind, &[[0.0, 0.0], [100.0, 0.0]], 0.0).kind(), kind);
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
            heading_deg: 0.0,
            end: [100.0, 100.0],
            end_heading_deg: 90.0,
            speed: 350.0,
            then: End::Coast,
        };
        let pts = c.path();
        assert_eq!(pts[0], [0.0, 0.0]);
        assert_eq!(*pts.last().unwrap(), [100.0, 100.0]);
        // the arc from a pose: a right quarter circle from (0, 0) facing +x ends at (100, -100)
        // facing -y, its centre below the start
        let a = arc_from([0.0, 0.0], 0.0, 100.0, 90.0, true).unwrap();
        assert_eq!(a.end, [100.0, -100.0]);
        assert!(
            near(a.end_heading, -90.0) && near(a.start_heading, 0.0) && near(a.sweep_deg, 90.0),
            "{a:?}"
        );
        assert!(near(a.centre[0], 0.0) && near(a.centre[1], -100.0), "{a:?}");
        let big = arc_from([0.0, 0.0], 90.0, 50.0, 270.0, false).unwrap();
        assert_eq!(
            big.end,
            [-50.0, -50.0],
            "three quarters of a circle, left, from facing +y: round the centre (-50, 0)"
        );
        assert!(near(big.end_heading, 0.0), "{big:?}");
        assert!(arc_from([0.0, 0.0], 0.0, 0.0, 90.0, true).is_none(), "no radius");
        assert!(arc_from([0.0, 0.0], 0.0, 100.0, 0.0, true).is_none(), "no angle");
        let none = Action::Curve {
            start: [1.0, 2.0],
            heading_deg: 0.0,
            end: [1.0, 2.0],
            end_heading_deg: 45.0,
            speed: 350.0,
            then: End::Coast,
        };
        assert_eq!((none.end(), none.path(), none.length_mm()), ([1.0, 2.0], vec![[1.0, 2.0]], 0.0));
        assert_eq!(none.end_heading(0.0), 45.0, "nowhere to go, but a heading to take up");
        // two arcs meeting tangentially wherever one arc cannot reach the end pose
        let check = |start: Point, h: f64, end: Point, eh: f64| {
            let pieces = biarc(start, h, end, eh);
            assert!(!pieces.is_empty(), "{start:?} {h} -> {end:?} {eh}");
            let (mut p, mut heading) = (start, h);
            for piece in &pieces {
                assert!(dist(piece.points()[0], p) < 0.3, "the pieces chain: {pieces:?}");
                p = piece.end();
                heading = piece.end_heading(heading);
            }
            assert!(dist(p, end) < 0.3, "ends at the end: {pieces:?}");
            assert!(
                wrap_deg(heading - eh).abs() < 0.3,
                "faces the way asked: {heading} vs {eh}, {pieces:?}"
            );
            pieces
        };
        // an S: sideways, facing the same way — two semicircles of r 25
        let s = check([0.0, 0.0], 0.0, [0.0, 100.0], 0.0);
        let [Piece::Arc { arc: a1, right: r1, .. }, Piece::Arc { arc: a2, right: r2, .. }] = s[..] else {
            panic!("{s:?}")
        };
        assert!(!r1 && r2 && near(a1.radius_mm, 25.0) && near(a2.radius_mm, 25.0), "{s:?}");
        assert!(near(a1.sweep_deg, 180.0) && near(a2.sweep_deg, 180.0), "{s:?}");
        // a U: back the way it came, 100 mm over — the semicircle of r 50 is a single arc
        let u = check([0.0, 0.0], 0.0, [0.0, 100.0], 180.0);
        let [Piece::Arc { arc: a1, right, .. }] = u[..] else {
            panic!("{u:?}")
        };
        assert!(!right && near(a1.radius_mm, 50.0) && near(a1.sweep_deg, 180.0), "{u:?}");
        // facing elsewhere at the same point takes two arcs
        assert_eq!(check([0.0, 0.0], 0.0, [0.0, 100.0], 270.0).len(), 2);
        // ahead and to the side, facing the same way: two quarter circles of r 50
        let z = check([0.0, 0.0], 0.0, [100.0, 100.0], 0.0);
        assert!(
            matches!(z[..], [Piece::Arc { .. }, Piece::Arc { .. }]) && near(z[0].length_mm(), z[1].length_mm()),
            "{z:?}"
        );
        // the single arc when the poses allow it, and the straight run when they line up
        assert_eq!(check([0.0, 0.0], 0.0, [100.0, 100.0], 90.0).len(), 1);
        assert_eq!(
            check([0.0, 0.0], 0.0, [100.0, 0.0], 0.0),
            vec![Piece::Line {
                from: [0.0, 0.0],
                to: [100.0, 0.0]
            }]
        );
        // ends and headings all round, every one reached facing the way asked
        let ends = [
            ([200.0, 50.0], 30.0),
            ([-50.0, 120.0], 200.0),
            ([80.0, -140.0], -90.0),
            ([30.0, 30.0], 180.0),
        ];
        for (i, (end, eh)) in ends.into_iter().enumerate() {
            for h in [0.0, 20.0 * (i + 1) as f64, -135.0] {
                check([10.0, -5.0], h, end, eh);
            }
        }
        assert!(biarc([1.0, 1.0], 0.0, [1.0, 1.0], 90.0).is_empty(), "nowhere to go");
        // the tangent arc through a point: radius, angle, side
        assert_eq!(tangent_arc(Pose2::at([0.0, 0.0], 0.0), [100.0, 100.0]), Some((100.0, 90.0, false)));
        assert_eq!(tangent_arc(Pose2::at([0.0, 0.0], 0.0), [100.0, -100.0]), Some((100.0, 90.0, true)));
        assert_eq!(
            tangent_arc(Pose2::at([0.0, 0.0], 0.0), [-100.0, 100.0]),
            Some((100.0, 270.0, false)),
            "behind: a big arc"
        );
        assert_eq!(tangent_arc(Pose2::at([0.0, 0.0], 0.0), [100.0, 0.0]), None, "dead ahead");
        assert_eq!(tangent_arc(Pose2::at([0.0, 0.0], 0.0), [0.0, 0.0]), None, "on the spot");
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
        s.drag(Handle::Face, [0.0, 0.0]);
        assert_eq!(s.start(), [10.0, 10.0], "a straight has no face handle");
        // a curve's handles: its start (which carries its end along), its end, and the heading it
        // ends facing, TURN_HANDLE_MM along
        let mut c = Action::Curve {
            start: [0.0, 0.0],
            heading_deg: 0.0,
            end: [100.0, -100.0],
            end_heading_deg: -90.0,
            speed: 350.0,
            then: End::Coast,
        };
        assert_eq!(c.pieces().len(), 1, "a right quarter circle: {:?}", c.pieces());
        let h = c.handles();
        assert_eq!(&h[..2], &[(Handle::Start, [0.0, 0.0]), (Handle::End, [100.0, -100.0])]);
        assert!(
            h[2].0 == Handle::Face && near(h[2].1[0], 100.0) && near(h[2].1[1], -100.0 - TURN_HANDLE_MM),
            "{h:?}"
        );
        c.drag(Handle::Face, [200.0, -100.0]);
        assert_eq!(c.end_heading(0.0), 0.0, "the face handle sets the heading it ends facing");
        assert_eq!(c.pieces().len(), 2, "which now takes two arcs: {:?}", c.pieces());
        c.drag(Handle::Face, [100.0, -100.0]);
        assert_eq!(c.end_heading(0.0), 0.0, "a face handle on the spot changes nothing");
        c.drag(Handle::End, [100.0, 100.0]);
        assert_eq!(c.end(), [100.0, 100.0]);
        c.drag(Handle::Start, [10.0, 10.0]);
        assert_eq!(
            (c.start(), c.end()),
            ([10.0, 10.0], [110.0, 110.0]),
            "the start carries the end along"
        );
        c.translate(-10.0, -10.0);
        assert_eq!((c.start(), c.end()), ([0.0, 0.0], [100.0, 100.0]));
        // a turn: the face handle sets the heading, the at handle moves it
        let mut t = Action::placed("turn", &[[0.0, 0.0], [10.0, 0.0]], 0.0);
        assert_eq!(t.handles()[1], (Handle::Face, [TURN_HANDLE_MM, 0.0]));
        t.drag(Handle::Face, [0.0, 30.0]);
        assert_eq!(t.end_heading(0.0), 90.0);
        t.drag(Handle::At, [5.0, 5.0]);
        assert_eq!(t.start(), [5.0, 5.0]);
        t.drag(Handle::Face, [5.0, 5.0]);
        assert_eq!(t.end_heading(0.0), 90.0, "a face handle on the spot changes nothing");
        // a custom call's end handle appears once it moves the robot
        let mut k = Action::placed("custom", &[[0.0, 0.0]], 0.0);
        assert_eq!(k.handles().len(), 1);
        k.drag(Handle::End, [100.0, 0.0]);
        assert_eq!(k.handles().len(), 2);
        assert_eq!(k.start_heading(), Some(0.0));
        assert!(k.text().ends_with(", moving 100 mm"), "{}", k.text());
        k.translate(10.0, -10.0);
        assert_eq!((k.start(), k.end()), ([10.0, -10.0], [110.0, -10.0]));
        let mut st = Action::placed("stop", &[[3.0, 4.0]], 0.0);
        st.translate(1.0, 1.0);
        assert_eq!(st.start(), [4.0, 5.0]);
        assert_eq!(st.handles(), vec![(Handle::At, [4.0, 5.0])]);
    }

    #[test]
    fn a_two_arc_curve_flows_from_one_piece_into_the_other() {
        let curve = |end: Point, end_heading_deg: f64| Action::Curve {
            start: [0.0, 0.0],
            heading_deg: 0.0,
            end,
            end_heading_deg,
            speed: 350.0,
            then: End::Coast,
        };
        let route = Route {
            start: Pose2::at([0.0, 0.0], 0.0),
            actions: vec![curve([0.0, 100.0], 0.0).into()],
            ..Default::default()
        };
        let text = program(&route, 86.4, 135.0);
        assert!(
            text.contains("from openbricks.parameters import Stop\n"),
            "the first piece needs Stop.NONE: {text}"
        );
        assert!(
            text.contains(
                "# 1: curve left 180° on r 25 mm, then curve right 180° on r 25 mm at 350°/s\ndb.curve(25, -180, then=Stop.NONE)\ndb.curve(25, 180)\n"
            ),
            "{text}"
        );
        assert!(pose_near(plan(&route)[0].end, 0.0, 100.0, 0.0));
        assert!(near(length_mm(&route), 50.0 * std::f64::consts::PI));
        // a curve going nowhere but turning is a turn; one dead ahead is a straight (the plan
        // turns back to its entry heading first: the file says it enters facing +x)
        let spin = Route {
            actions: vec![curve([0.0, 0.0], 90.0).into(), curve([100.0, 0.0], 0.0).into()],
            ..Default::default()
        };
        let text = program(&spin, 86.4, 135.0);
        assert!(text.contains("# 1: curve (nowhere to go) at 350°/s\ndb.turn(-90)\n"), "{text}");
        assert!(
            text.contains("\n# 2: straight 100 mm at 350°/s\n# to its start\ndb.turn(90)\ndb.straight(100)\n"),
            "{text}"
        );
    }

    #[test]
    fn the_plan_drives_to_each_action_and_keeps_the_sign_conventions() {
        let route = Route {
            start: Pose2::at([0.0, 0.0], 0.0),
            actions: vec![
                straight([0.0, 0.0], [300.0, 0.0]).into(),
                // a right quarter circle of r 100 from (300, 0) entered facing +y: the robot must
                // first turn to face that way
                Action::Curve {
                    start: [300.0, 0.0],
                    heading_deg: 90.0,
                    end: [400.0, 100.0],
                    end_heading_deg: 0.0,
                    speed: 350.0,
                    then: End::Continue,
                }
                .into(),
                // a stop somewhere else: the plan drives there first
                Action::placed("stop", &[[400.0, 300.0]], 0.0).into(),
                Action::placed("turn", &[[400.0, 300.0], [300.0, 300.0]], 0.0).into(),
                Action::placed("custom", &[[400.0, 300.0]], 0.0).into(),
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
                    heading_deg: 0.0,
                    end: [100.0, 100.0],
                    end_heading_deg: 90.0,
                    speed: 300.0,
                    then: End::Continue,
                }
                .into(),
                Action::placed("stop", &[[100.0, 100.0]], 0.0).into(),
            ],
            ..Default::default()
        };
        route.actions[1].locked = true;
        route.actions[0].color = Some([200, 30, 30]);
        let p = dir.join("a.route.json");
        route.save(&p).unwrap();
        let text = std::fs::read_to_string(&p).unwrap();
        assert_eq!(text.matches("\"color\"").count(), 1, "only a chosen colour is written");
        assert!(
            text.contains("\"format\": \"openbricks-route/4\"") && text.contains("\"kind\": \"curve\""),
            "{text}"
        );
        assert!(
            text.contains("\"end_heading_deg\": 90.0") && !text.contains("\"radius_mm\""),
            "two poses, no radius: {text}"
        );
        assert!(text.contains("\"locked\": true"), "{text}");
        assert_eq!(text.matches("\"locked\"").count(), 1, "unlocked actions do not mention it");
        assert_eq!(Route::load(&p).unwrap(), route);
        // the third format placed a curve by its ends: it keeps its shape, now as an arc from its
        // start, and the rest of the file (colours, locks) comes along
        let v3 = dir.join("v3.route.json");
        std::fs::write(
            &v3,
            r#"{"format":"openbricks-route/3","world":"practice-line","start":{"x_mm":0,"y_mm":0,"yaw_deg":0},"actions":[{"kind":"curve","start":[0,0],"end":[100,100],"radius_mm":100,"right":false,"color":[1,2,3]},{"kind":"straight","start":[100,100],"end":[100,300],"locked":true},{"kind":"curve","start":[5,5],"end":[5,5],"radius_mm":40}]}"#,
        )
        .unwrap();
        let third = Route::load(&v3).unwrap();
        assert_eq!(third.format, FORMAT);
        let Action::Curve {
            start,
            heading_deg,
            end,
            end_heading_deg,
            ..
        } = &third.actions[0].action
        else {
            panic!("{:?}", third.actions[0])
        };
        assert_eq!(
            (start, heading_deg, end, end_heading_deg),
            (&[0.0, 0.0], &0.0, &[100.0, 100.0], &90.0)
        );
        assert_eq!(third.actions[0].action.pieces().len(), 1, "the same single arc");
        assert_eq!(third.actions[0].color, Some([1, 2, 3]));
        assert!(third.actions[1].locked);
        assert_eq!(third.actions[2].action.end(), [5.0, 5.0], "a curve with no arc keeps its start");
        std::fs::write(&v3, r#"{"format":"openbricks-route/3","actions":[{"kind":"curve","start":[0,0]}]}"#).unwrap();
        assert!(Route::load(&v3).unwrap_err().contains("two ends"));
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
            heading_deg,
            end,
            end_heading_deg,
            ..
        } = &old.actions[1].action
        else {
            panic!("{:?}", old.actions[1])
        };
        assert_eq!(
            (start, heading_deg, end, end_heading_deg),
            (&[0.0, 250.0], &90.0, &[100.0, 350.0], &0.0)
        );
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
        let Action::Curve { end, end_heading_deg, .. } = &older.actions[1].action else {
            panic!()
        };
        assert_eq!((end, end_heading_deg), (&[100.0, 350.0], &0.0));
        let pieces = older.actions[1].action.pieces();
        assert!(matches!(pieces[..], [Piece::Arc { right: true, .. }]), "{pieces:?}");
        assert!(near(pieces[0].length_mm(), 100.0 * std::f64::consts::FRAC_PI_2));
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
        assert!(!Item::from(Action::placed("stop", &[[0.0, 0.0]], 0.0)).locked);
    }
}
