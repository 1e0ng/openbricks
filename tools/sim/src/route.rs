//! Routes: a start pose on the map and a sequence of segments, each
//! ending at a point the user chose, driven as a straight line (turn
//! in place, then drive the distance) or as the arc tangent to the
//! current heading through the point (its radius and angle worked
//! out). The maths here is plain so it runs under `cargo test`; the
//! Simulate tab draws it and turns it into a program.
//!
//! Frames: the map's world frame, mm, yaw counter-clockwise from +x
//! in degrees. Legs are in the drive base's own terms, where a
//! positive turn or curve angle is to the RIGHT (clockwise from
//! above) like `DriveBase.turn` and `DriveBase.curve`.

use serde::{Deserialize, Serialize};
use std::path::Path;

pub const FORMAT: &str = "openbricks-route/1";
/// Points closer than this are the same point.
const EPS_MM: f64 = 0.5;

/// A pose on the map.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Debug, Default)]
pub struct Pose2 {
    pub x_mm: f64,
    pub y_mm: f64,
    pub yaw_deg: f64,
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Debug)]
#[serde(rename_all = "lowercase")]
pub enum Kind {
    Straight,
    Curve,
}

impl Kind {
    pub fn label(self) -> &'static str {
        match self {
            Kind::Straight => "straight",
            Kind::Curve => "curve",
        }
    }
}

/// One segment: how to reach `to` from wherever the previous one ended.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct Segment {
    pub kind: Kind,
    pub to: [f64; 2],
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct Route {
    pub format: String,
    #[serde(default)]
    pub world: String,
    pub start: Pose2,
    #[serde(default)]
    pub segments: Vec<Segment>,
}

impl Default for Route {
    fn default() -> Self {
        Route {
            format: FORMAT.into(),
            world: String::new(),
            start: Pose2::default(),
            segments: vec![],
        }
    }
}

/// What the robot does for one segment, in the drive base's terms.
#[derive(Clone, PartialEq, Debug)]
pub enum Leg {
    /// Turn in place; positive is to the right.
    Turn {
        deg: f64,
    },
    Straight {
        mm: f64,
    },
    /// An arc of `radius_mm`; positive `deg` is to the right.
    Curve {
        radius_mm: f64,
        deg: f64,
    },
}

impl Leg {
    pub fn text(&self) -> String {
        match self {
            Leg::Turn { deg } => format!("turn {} {}°", if *deg >= 0.0 { "right" } else { "left" }, fmt(deg.abs())),
            Leg::Straight { mm } => format!("straight {} mm", fmt(*mm)),
            Leg::Curve { radius_mm, deg } => format!(
                "curve {} {}° on r {} mm",
                if *deg >= 0.0 { "right" } else { "left" },
                fmt(deg.abs()),
                fmt(*radius_mm)
            ),
        }
    }

    /// The line of program that drives it.
    pub fn code(&self) -> String {
        match self {
            Leg::Turn { deg } => format!("db.turn({})", num(*deg)),
            Leg::Straight { mm } => format!("db.straight({})", num(*mm)),
            Leg::Curve { radius_mm, deg } => format!("db.curve({}, {})", num(*radius_mm), num(*deg)),
        }
    }

    /// Distance driven, mm.
    pub fn length_mm(&self) -> f64 {
        match self {
            Leg::Turn { .. } => 0.0,
            Leg::Straight { mm } => mm.abs(),
            Leg::Curve { radius_mm, deg } => radius_mm.abs() * deg.abs().to_radians(),
        }
    }
}

/// A segment worked out: its legs, where it ends, the points that draw
/// it, and a note when the chosen kind could not be honoured.
#[derive(Clone, PartialEq, Debug)]
pub struct Planned {
    pub legs: Vec<Leg>,
    pub end: Pose2,
    pub points: Vec<[f64; 2]>,
    pub note: Option<String>,
}

fn fmt(v: f64) -> String {
    let r = (v * 10.0).round() / 10.0;
    if r == r.trunc() { format!("{}", r as i64) } else { format!("{r}") }
}

fn num(v: f64) -> String {
    let r = (v * 100.0).round() / 100.0;
    if r == r.trunc() { format!("{}", r as i64) } else { format!("{r}") }
}

/// An angle wrapped into (-180, 180].
pub fn wrap_deg(a: f64) -> f64 {
    let d = a.rem_euclid(360.0);
    if d > 180.0 { d - 360.0 } else { d }
}

/// Plan one segment from `from`.
pub fn plan_segment(from: Pose2, seg: &Segment) -> Planned {
    let (dx, dy) = (seg.to[0] - from.x_mm, seg.to[1] - from.y_mm);
    let dist = (dx * dx + dy * dy).sqrt();
    if dist < EPS_MM {
        return Planned {
            legs: vec![],
            end: from,
            points: vec![[from.x_mm, from.y_mm]],
            note: Some("the point is where the robot already is".into()),
        };
    }
    let (hx, hy) = (from.yaw_deg.to_radians().cos(), from.yaw_deg.to_radians().sin());
    let cross = hx * dy - hy * dx;
    let dot = hx * dx + hy * dy;
    let straight = |note: Option<String>| {
        let heading = dy.atan2(dx).to_degrees();
        let delta = wrap_deg(heading - from.yaw_deg);
        let mut legs = Vec::new();
        if delta.abs() >= 0.05 {
            legs.push(Leg::Turn { deg: -delta });
        }
        legs.push(Leg::Straight { mm: dist });
        Planned {
            legs,
            end: Pose2 {
                x_mm: seg.to[0],
                y_mm: seg.to[1],
                yaw_deg: wrap_deg(heading),
            },
            points: vec![[from.x_mm, from.y_mm], seg.to],
            note,
        }
    };
    match seg.kind {
        Kind::Straight => straight(None),
        Kind::Curve => {
            if cross.abs() < 1e-6 * dist {
                if dot > 0.0 {
                    // dead ahead: the arc is a line
                    let mut p = straight(None);
                    p.note = Some("straight ahead: a line, not an arc".into());
                    return p;
                }
                return straight(Some("straight behind: no arc reaches it, so a turn and a line".into()));
            }
            // the tangent-chord angle: the arc turns twice the angle between heading and chord
            let alpha = cross.abs().atan2(dot);
            let radius = dist / (2.0 * alpha.sin());
            let left = cross > 0.0;
            let sweep = 2.0 * alpha; // radians, unsigned
            let world_delta = if left { sweep } else { -sweep };
            // the centre sits one radius to the side the arc bends towards
            let (cx, cy) = if left {
                (from.x_mm - hy * radius, from.y_mm + hx * radius)
            } else {
                (from.x_mm + hy * radius, from.y_mm - hx * radius)
            };
            let a0 = (from.y_mm - cy).atan2(from.x_mm - cx);
            let n = ((sweep / std::f64::consts::PI * 32.0).ceil() as usize).max(4);
            let points = (0..=n)
                .map(|i| {
                    let a = a0 + world_delta * i as f64 / n as f64;
                    [cx + radius * a.cos(), cy + radius * a.sin()]
                })
                .collect();
            Planned {
                legs: vec![Leg::Curve {
                    radius_mm: radius,
                    deg: -world_delta.to_degrees(),
                }],
                end: Pose2 {
                    x_mm: seg.to[0],
                    y_mm: seg.to[1],
                    yaw_deg: wrap_deg(from.yaw_deg + world_delta.to_degrees()),
                },
                points,
                note: None,
            }
        }
    }
}

/// Every segment planned in turn from the start pose.
pub fn plan(route: &Route) -> Vec<Planned> {
    let mut pose = route.start;
    route
        .segments
        .iter()
        .map(|s| {
            let p = plan_segment(pose, s);
            pose = p.end;
            p
        })
        .collect()
}

/// The total distance driven, mm.
pub fn length_mm(planned: &[Planned]) -> f64 {
    planned.iter().flat_map(|p| p.legs.iter()).map(Leg::length_mm).sum()
}

/// The program that drives the route: a hub-style program the sim runs
/// unchanged (the shim binds the two serial motors to the chassis's
/// wheels), with the motor lines for the reference wiring to edit.
pub fn program(route: &Route, wheel_diameter_mm: f64, axle_track_mm: f64) -> String {
    let planned = plan(route);
    let mut out = String::new();
    out.push_str(&format!(
        "\"\"\"A route planned in the openbricks sim on {}: {} segment{} from ({}, {}) mm heading {}°.\n\nThe sim binds the two motors to the chassis's wheels; on the hub, edit them for your wiring.\"\"\"\nfrom openbricks.drivers.st3032 import ST3032Motor\nfrom openbricks.robotics import DriveBase\n\nleft = ST3032Motor(servo_id=1, tx=14, rx=41)\nright = ST3032Motor(servo_id=2, tx=14, rx=41, invert=True)\ndb = DriveBase(left, right, wheel_diameter_mm={}, axle_track_mm={})\n",
        if route.world.is_empty() { "the map" } else { &route.world },
        route.segments.len(),
        if route.segments.len() == 1 { "" } else { "s" },
        num(route.start.x_mm),
        num(route.start.y_mm),
        num(route.start.yaw_deg),
        num(wheel_diameter_mm),
        num(axle_track_mm)
    ));
    for (i, (seg, p)) in route.segments.iter().zip(&planned).enumerate() {
        out.push_str(&format!(
            "\n# {}: {} to ({}, {})\n",
            i + 1,
            seg.kind.label(),
            num(seg.to[0]),
            num(seg.to[1])
        ));
        if let Some(n) = &p.note {
            out.push_str(&format!("# {n}\n"));
        }
        for leg in &p.legs {
            out.push_str(&leg.code());
            out.push('\n');
        }
    }
    out
}

impl Route {
    pub fn load(path: &Path) -> Result<Route, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let route: Route = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        if route.format != FORMAT {
            return Err(format!("not a route file (format {:?}, expected {FORMAT})", route.format));
        }
        Ok(route)
    }

    pub fn save(&self, path: &Path) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(path, text).map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seg(kind: Kind, x: f64, y: f64) -> Segment {
        Segment { kind, to: [x, y] }
    }

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn a_straight_segment_turns_to_face_the_point_then_drives() {
        let from = Pose2 {
            x_mm: 0.0,
            y_mm: 0.0,
            yaw_deg: 0.0,
        };
        let p = plan_segment(from, &seg(Kind::Straight, 300.0, 0.0));
        assert_eq!(p.legs, vec![Leg::Straight { mm: 300.0 }], "dead ahead: no turn");
        assert_eq!(p.end.yaw_deg, 0.0);
        let p = plan_segment(from, &seg(Kind::Straight, 0.0, 100.0));
        assert_eq!(
            p.legs,
            vec![Leg::Turn { deg: -90.0 }, Leg::Straight { mm: 100.0 }],
            "a point to the left: turn left (negative), then drive"
        );
        assert_eq!(
            p.end,
            Pose2 {
                x_mm: 0.0,
                y_mm: 100.0,
                yaw_deg: 90.0
            }
        );
        let p = plan_segment(from, &seg(Kind::Straight, 0.0, -100.0));
        assert_eq!(p.legs[0], Leg::Turn { deg: 90.0 }, "a point to the right: turn right");
        let p = plan_segment(from, &seg(Kind::Straight, -100.0, 0.0));
        assert_eq!(p.legs[0], Leg::Turn { deg: -180.0 }, "behind: half a turn");
        assert_eq!(p.points, vec![[0.0, 0.0], [-100.0, 0.0]]);
        let p = plan_segment(from, &seg(Kind::Straight, 0.1, 0.1));
        assert!(p.legs.is_empty() && p.note.is_some(), "the same point: nothing to do");
    }

    #[test]
    fn a_curve_is_the_arc_tangent_to_the_heading_through_the_point() {
        let from = Pose2 {
            x_mm: 0.0,
            y_mm: 0.0,
            yaw_deg: 0.0,
        };
        let p = plan_segment(from, &seg(Kind::Curve, 100.0, 100.0));
        let [Leg::Curve { radius_mm, deg }] = p.legs.as_slice() else {
            panic!("{:?}", p.legs)
        };
        assert!(near(*radius_mm, 100.0), "{radius_mm}");
        assert!(near(*deg, -90.0), "a quarter circle to the left is negative: {deg}");
        assert!(near(p.end.yaw_deg, 90.0));
        assert!(near(p.end.x_mm, 100.0) && near(p.end.y_mm, 100.0));
        let first = p.points[0];
        let last = p.points[p.points.len() - 1];
        assert!(near(first[0], 0.0) && near(first[1], 0.0));
        assert!(near(last[0], 100.0) && near(last[1], 100.0));
        for q in &p.points {
            // every point is one radius from the centre (0, 100)
            assert!(near((q[0] * q[0] + (q[1] - 100.0) * (q[1] - 100.0)).sqrt(), 100.0), "{q:?}");
        }
        assert!(near(p.legs[0].length_mm(), 100.0 * std::f64::consts::FRAC_PI_2));
        // the mirror: to the right, positive angle
        let p = plan_segment(from, &seg(Kind::Curve, 100.0, -100.0));
        let [Leg::Curve { radius_mm, deg }] = p.legs.as_slice() else {
            panic!("{:?}", p.legs)
        };
        assert!(near(*radius_mm, 100.0) && near(*deg, 90.0), "{radius_mm} {deg}");
        assert!(near(p.end.yaw_deg, -90.0));
        // a point beside the robot: a half circle of radius d/2
        let p = plan_segment(from, &seg(Kind::Curve, 0.0, 80.0));
        let [Leg::Curve { radius_mm, deg }] = p.legs.as_slice() else {
            panic!("{:?}", p.legs)
        };
        assert!(near(*radius_mm, 40.0) && near(*deg, -180.0), "{radius_mm} {deg}");
        // a heading that is not along x: the same geometry, turned
        let from = Pose2 {
            x_mm: 10.0,
            y_mm: 20.0,
            yaw_deg: 90.0,
        };
        let p = plan_segment(from, &seg(Kind::Curve, -90.0, 120.0));
        let [Leg::Curve { radius_mm, deg }] = p.legs.as_slice() else {
            panic!("{:?}", p.legs)
        };
        assert!(near(*radius_mm, 100.0) && near(*deg, -90.0), "{radius_mm} {deg}");
        assert!(near(p.end.yaw_deg, 180.0), "{}", p.end.yaw_deg);
    }

    #[test]
    fn a_curve_to_a_point_on_the_heading_line_becomes_a_line() {
        let from = Pose2 {
            x_mm: 0.0,
            y_mm: 0.0,
            yaw_deg: 45.0,
        };
        let p = plan_segment(from, &seg(Kind::Curve, 100.0, 100.0));
        assert_eq!(p.legs, vec![Leg::Straight { mm: 200f64.sqrt() * 10.0 }]);
        assert!(p.note.as_deref().unwrap().contains("straight ahead"));
        let p = plan_segment(from, &seg(Kind::Curve, -100.0, -100.0));
        assert_eq!(p.legs[0], Leg::Turn { deg: -180.0 });
        assert!(p.note.as_deref().unwrap().contains("behind"));
    }

    #[test]
    fn segments_chain_from_each_other_and_sum_their_length() {
        let route = Route {
            world: "practice-line".into(),
            start: Pose2 {
                x_mm: 0.0,
                y_mm: 0.0,
                yaw_deg: 0.0,
            },
            segments: vec![
                seg(Kind::Straight, 200.0, 0.0),
                seg(Kind::Curve, 300.0, 100.0),
                seg(Kind::Straight, 300.0, 300.0),
            ],
            ..Default::default()
        };
        let planned = plan(&route);
        assert_eq!(planned.len(), 3);
        assert_eq!(planned[0].end.yaw_deg, 0.0);
        assert!(near(planned[1].end.yaw_deg, 90.0), "{}", planned[1].end.yaw_deg);
        assert_eq!(planned[2].legs, vec![Leg::Straight { mm: 200.0 }], "already facing +y: no turn");
        assert!(near(length_mm(&planned), 200.0 + 100.0 * std::f64::consts::FRAC_PI_2 + 200.0));
    }

    #[test]
    fn the_program_drives_the_legs_on_the_chassis() {
        let route = Route {
            world: "practice-line".into(),
            start: Pose2 {
                x_mm: -547.0,
                y_mm: -150.0,
                yaw_deg: 90.0,
            },
            segments: vec![
                seg(Kind::Straight, -547.0, 100.0),
                seg(Kind::Curve, -447.0, 200.0),
                seg(Kind::Straight, -447.0, 200.0),
            ],
            ..Default::default()
        };
        let text = program(&route, 86.4, 135.0);
        assert!(text.contains("from openbricks.robotics import DriveBase"));
        assert!(text.contains("from openbricks.drivers.st3032 import ST3032Motor\n"));
        assert!(
            text.contains("left = ST3032Motor(servo_id=1, tx=14, rx=41)\nright = ST3032Motor(servo_id=2, tx=14, rx=41, invert=True)\n")
        );
        assert!(
            text.contains("db = DriveBase(left, right, wheel_diameter_mm=86.4, axle_track_mm=135)"),
            "{text}"
        );
        assert!(text.contains("# 1: straight to (-547, 100)\ndb.straight(250)\n"), "{text}");
        assert!(text.contains("# 2: curve to (-447, 200)\ndb.curve(100, 90)\n"), "{text}");
        assert!(
            text.contains("# 3: straight to (-447, 200)\n# the point is where the robot already is\n"),
            "{text}"
        );
        assert!(text.contains("3 segments from (-547, -150) mm heading 90°"));
        assert_eq!(Leg::Turn { deg: -32.26 }.text(), "turn left 32.3°");
        assert_eq!(
            Leg::Curve {
                radius_mm: 180.0,
                deg: 60.0
            }
            .text(),
            "curve right 60° on r 180 mm"
        );
        assert_eq!(Leg::Straight { mm: 245.04 }.text(), "straight 245 mm");
        assert_eq!(
            Leg::Curve {
                radius_mm: 123.456,
                deg: -12.345
            }
            .code(),
            "db.curve(123.46, -12.35)"
        );
    }

    #[test]
    fn routes_save_load_and_refuse_other_files() {
        let dir = std::env::temp_dir().join(format!("ob-route-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let route = Route {
            world: "wro-2026-senior".into(),
            start: Pose2 {
                x_mm: 1.5,
                y_mm: -2.0,
                yaw_deg: 30.0,
            },
            segments: vec![seg(Kind::Curve, 100.0, 50.0)],
            ..Default::default()
        };
        let path = dir.join("lap.route.json");
        route.save(&path).unwrap();
        let back = Route::load(&path).unwrap();
        assert_eq!(back, route);
        assert!(
            std::fs::read_to_string(&path)
                .unwrap()
                .contains("\"format\": \"openbricks-route/1\"")
        );
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
        assert_eq!(Kind::Straight.label(), "straight");
        assert_eq!(wrap_deg(190.0), -170.0);
        assert_eq!(wrap_deg(-180.0), 180.0);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
