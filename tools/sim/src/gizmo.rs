//! Move and rotate handles on the selection: three world-axis arrows
//! or three rings drawn over the scene, picked in screen space and
//! dragged with the maths below. Everything here is plain geometry so
//! it runs under `cargo test`; the app wires it to the pointer.

use crate::assembly::{self, euler_from, rot_mat};
use crate::viewport::{Camera, DrawItem, srgb};
use glam::{DMat3, DVec3, Mat4, Quat, Vec2, Vec3};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Mode {
    Move,
    Rotate,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Handle {
    /// An arrow along world axis 0, 1 or 2.
    Axis(usize),
    /// A ring around world axis 0, 1 or 2.
    Ring(usize),
}

pub const MESH_SHAFT: &str = "gizmo:shaft";
pub const MESH_CONE: &str = "gizmo:cone";
pub const MESH_RING: &str = "gizmo:ring";

/// The handles' size on screen, in pixels, whatever the zoom.
pub const PIXELS: f32 = 90.0;
/// How close (pixels) the pointer must come to grab a handle.
pub const GRAB_PX: f32 = 9.0;
/// The rings' radius as a fraction of the arrows' length.
pub const RING_FRACTION: f32 = 0.8;
/// Rotation steps while the grid snap is on.
pub const ANGLE_STEP_DEG: f32 = 15.0;

const AXIS_COLORS: [u32; 3] = [0xC4442A, 0x3F7D5B, 0x1F6FB2];
const HOT_COLOR: u32 = 0xF2B233;

pub struct Gizmo {
    pub center: Vec3,
    pub mode: Mode,
    /// The arrows' length in world units (mm).
    pub length: f32,
}

pub fn axis(i: usize) -> Vec3 {
    [Vec3::X, Vec3::Y, Vec3::Z][i]
}

fn daxis(i: usize) -> DVec3 {
    [DVec3::X, DVec3::Y, DVec3::Z][i]
}

/// The rotation that carries +z onto world axis `i` (the unit meshes
/// are built along z).
fn orient(i: usize) -> Quat {
    match i {
        0 => Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
        1 => Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
        _ => Quat::IDENTITY,
    }
}

impl Gizmo {
    /// Handles at `center`, sized so the arrows span `PIXELS` on a
    /// view `h` pixels tall seen through `cam`.
    pub fn new(center: Vec3, mode: Mode, cam: &Camera, h: f32) -> Self {
        let depth = (center - cam.eye()).dot(-cam.direction()).max(1.0);
        let per_px = 2.0 * depth * (cam.fov_deg.to_radians() / 2.0).tan() / h.max(1.0);
        Gizmo {
            center,
            mode,
            length: PIXELS * per_px,
        }
    }

    pub fn ring_radius(&self) -> f32 {
        self.length * RING_FRACTION
    }

    /// The handle a screen-space pick would grab, if any.
    pub fn handle_at(&self, cam: &Camera, px: f32, py: f32, w: f32, h: f32) -> Option<Handle> {
        let p = Vec2::new(px, py);
        let mut best: Option<(f32, Handle)> = None;
        let mut consider = |d: f32, handle: Handle| {
            if d <= GRAB_PX && best.map(|b| d < b.0).unwrap_or(true) {
                best = Some((d, handle));
            }
        };
        for i in 0..3 {
            match self.mode {
                Mode::Move => {
                    let (Some(a), Some(b)) = (
                        cam.project(self.center, w, h),
                        cam.project(self.center + axis(i) * self.length, w, h),
                    ) else {
                        continue;
                    };
                    consider(dist_to_segment(p, a, b), Handle::Axis(i));
                }
                Mode::Rotate => {
                    let pts: Vec<Vec2> = ring_points(self.center, self.ring_radius(), i, 48)
                        .into_iter()
                        .filter_map(|q| cam.project(q, w, h))
                        .collect();
                    for k in 0..pts.len() {
                        consider(dist_to_segment(p, pts[k], pts[(k + 1) % pts.len()]), Handle::Ring(i));
                    }
                }
            }
        }
        best.map(|b| b.1)
    }

    /// The drag parameter of a handle for a pointer ray: the position
    /// along the arrow's axis, or the angle (degrees) around the ring.
    pub fn param(&self, handle: Handle, origin: Vec3, dir: Vec3) -> Option<f32> {
        match handle {
            Handle::Axis(i) => axis_param(origin, dir, self.center, axis(i)),
            Handle::Ring(i) => ring_angle(origin, dir, self.center, i),
        }
    }

    /// The handles as draw items over the scene; `hot` is drawn lit.
    pub fn draw(&self, hot: Option<Handle>) -> Vec<DrawItem> {
        let mut items = Vec::new();
        for (i, &base) in AXIS_COLORS.iter().enumerate() {
            let mine = match self.mode {
                Mode::Move => Handle::Axis(i),
                Mode::Rotate => Handle::Ring(i),
            };
            let color = srgb(if hot == Some(mine) { HOT_COLOR } else { base });
            let model = Mat4::from_scale_rotation_translation(Vec3::splat(self.length), orient(i), self.center);
            let meshes: &[&str] = match self.mode {
                Mode::Move => &[MESH_SHAFT, MESH_CONE],
                Mode::Rotate => &[MESH_RING],
            };
            for mesh in meshes {
                items.push(DrawItem {
                    mesh: mesh.to_string(),
                    model,
                    color,
                    texture: None,
                });
            }
        }
        items
    }
}

/// Points around the ring of world axis `i`.
pub fn ring_points(center: Vec3, radius: f32, i: usize, n: usize) -> Vec<Vec3> {
    let (u, v) = (axis((i + 1) % 3), axis((i + 2) % 3));
    (0..n)
        .map(|k| {
            let a = k as f32 / n as f32 * std::f32::consts::TAU;
            center + (u * a.cos() + v * a.sin()) * radius
        })
        .collect()
}

fn dist_to_segment(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let l2 = ab.length_squared();
    let t = if l2 < 1e-9 { 0.0 } else { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) };
    (p - (a + ab * t)).length()
}

/// Where along the line through `p` with unit direction `a` the ray
/// `(origin, dir)` passes closest; None when they are parallel.
pub fn axis_param(origin: Vec3, dir: Vec3, p: Vec3, a: Vec3) -> Option<f32> {
    let w = origin - p;
    let b = dir.dot(a);
    let denom = 1.0 - b * b;
    if denom < 1e-6 {
        return None;
    }
    Some((a.dot(w) - b * dir.dot(w)) / denom)
}

/// The angle, in degrees about world axis `i`, of the point where the
/// ray meets the ring's plane through `c`; None when the ray misses it.
pub fn ring_angle(origin: Vec3, dir: Vec3, c: Vec3, i: usize) -> Option<f32> {
    let a = axis(i);
    let denom = dir.dot(a);
    if denom.abs() < 1e-6 {
        return None;
    }
    let t = (c - origin).dot(a) / denom;
    if t < 0.0 {
        return None;
    }
    let q = origin + dir * t - c;
    let (u, v) = (axis((i + 1) % 3), axis((i + 2) % 3));
    Some(q.dot(v).atan2(q.dot(u)).to_degrees())
}

/// `deg` wrapped into (-180, 180].
pub fn wrap_deg(deg: f32) -> f32 {
    let d = deg.rem_euclid(360.0);
    if d > 180.0 { d - 360.0 } else { d }
}

/// The angle a ring drag applies: `ANGLE_STEP_DEG` multiples while
/// snapping, hundredths of a degree otherwise.
pub fn snap_angle(deg: f32, snapping: bool) -> f64 {
    let d = wrap_deg(deg);
    if snapping {
        ((d / ANGLE_STEP_DEG).round() * ANGLE_STEP_DEG) as f64
    } else {
        ((d * 100.0).round() / 100.0) as f64
    }
}

/// A position moved by `delta` along world axis `i`, that coordinate
/// snapped to `snap_mm` (no snap when it is 0).
pub fn moved(start: [f64; 3], i: usize, delta: f64, snap_mm: f64) -> [f64; 3] {
    let mut p = start;
    let v = start[i] + delta;
    p[i] = if snap_mm > 0.0 {
        (v / snap_mm).round() * snap_mm
    } else {
        assembly::round3(v)
    };
    p
}

/// A pose turned by `deg` about world axis `i` through `pivot`.
pub fn rotated(pos: [f64; 3], rot: [f64; 3], pivot: DVec3, i: usize, deg: f64) -> ([f64; 3], [f64; 3]) {
    let r = DMat3::from_axis_angle(daxis(i), deg.to_radians());
    let p = pivot + r * (DVec3::from_array(pos) - pivot);
    (p.to_array().map(assembly::round3), euler_from(&(r * rot_mat(rot))))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam() -> Camera {
        Camera {
            target: Vec3::ZERO,
            yaw: 0.0,
            pitch: 0.0,
            distance: 500.0,
            fov_deg: 38.0,
            ortho: false,
        }
    }

    #[test]
    fn arrows_span_the_pixel_size_at_any_distance() {
        for distance in [100.0, 500.0, 5000.0] {
            let cam = Camera { distance, ..cam() };
            let g = Gizmo::new(Vec3::ZERO, Mode::Move, &cam, 480.0);
            // the camera looks down -x from +x, so the y arrow lies across the screen
            let a = cam.project(g.center, 640.0, 480.0).unwrap();
            let b = cam.project(g.center + Vec3::Y * g.length, 640.0, 480.0).unwrap();
            assert!(((a - b).length() - PIXELS).abs() < 1.0, "{distance}: {} px", (a - b).length());
        }
    }

    #[test]
    fn picks_the_arrow_under_the_pointer_and_nothing_far_away() {
        let cam = cam();
        let g = Gizmo::new(Vec3::ZERO, Mode::Move, &cam, 480.0);
        let tip = cam.project(g.center + Vec3::Y * g.length * 0.5, 640.0, 480.0).unwrap();
        assert_eq!(g.handle_at(&cam, tip.x, tip.y + 4.0, 640.0, 480.0), Some(Handle::Axis(1)));
        let z = cam.project(g.center + Vec3::Z * g.length * 0.7, 640.0, 480.0).unwrap();
        assert_eq!(g.handle_at(&cam, z.x + 3.0, z.y, 640.0, 480.0), Some(Handle::Axis(2)));
        assert_eq!(g.handle_at(&cam, 10.0, 10.0, 640.0, 480.0), None);
        // the x arrow points straight at the camera: its projection is a dot at the centre
        assert_eq!(g.handle_at(&cam, 320.0, 240.0, 640.0, 480.0), Some(Handle::Axis(0)));
    }

    #[test]
    fn picks_the_ring_under_the_pointer() {
        let cam = cam();
        let g = Gizmo::new(Vec3::ZERO, Mode::Rotate, &cam, 480.0);
        // the ring about x faces the camera as a full circle
        let on = cam.project(Vec3::new(0.0, g.ring_radius(), 0.0), 640.0, 480.0).unwrap();
        assert_eq!(g.handle_at(&cam, on.x, on.y + 2.0, 640.0, 480.0), Some(Handle::Ring(0)));
        // the rings about y and z are edge-on: lines through the centre, still grabbable
        assert!(g.handle_at(&cam, 320.0, 240.0, 640.0, 480.0).is_some());
        // inside the x ring, away from both lines: nothing
        let r = (on - glam::Vec2::new(320.0, 240.0)).length();
        assert_eq!(g.handle_at(&cam, 320.0 + 0.5 * r, 240.0 + 0.5 * r, 640.0, 480.0), None);
    }

    #[test]
    fn axis_param_is_the_closest_point_along_the_axis() {
        let t = axis_param(Vec3::new(5.0, 0.0, 10.0), Vec3::NEG_Z, Vec3::ZERO, Vec3::X).unwrap();
        assert!((t - 5.0).abs() < 1e-5);
        let t = axis_param(
            Vec3::new(0.0, 3.0, 10.0),
            Vec3::new(0.0, -0.6, -0.8),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::X,
        )
        .unwrap();
        assert!((t + 2.0).abs() < 1e-5, "{t}");
        assert!(axis_param(Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::X).is_none());
    }

    #[test]
    fn ring_angle_follows_the_right_hand_rule() {
        let from_above = |x: f32, y: f32| ring_angle(Vec3::new(x, y, 50.0), Vec3::NEG_Z, Vec3::ZERO, 2).unwrap();
        assert!((from_above(1.0, 0.0)).abs() < 1e-4);
        assert!((from_above(0.0, 1.0) - 90.0).abs() < 1e-4);
        assert!((from_above(-1.0, 0.0)).abs() - 180.0 < 1e-4);
        // about x: y turns towards z
        let a = ring_angle(Vec3::new(50.0, 0.0, 1.0), Vec3::NEG_X, Vec3::ZERO, 0).unwrap();
        assert!((a - 90.0).abs() < 1e-4);
        // about y: z turns towards x
        let a = ring_angle(Vec3::new(1.0, 50.0, 0.0), Vec3::NEG_Y, Vec3::ZERO, 1).unwrap();
        assert!((a - 90.0).abs() < 1e-4);
        assert!(ring_angle(Vec3::new(0.0, 0.0, 50.0), Vec3::X, Vec3::ZERO, 2).is_none());
        assert!(ring_angle(Vec3::new(0.0, 0.0, 50.0), Vec3::Z, Vec3::ZERO, 2).is_none());
    }

    #[test]
    fn angles_wrap_and_snap() {
        assert_eq!(wrap_deg(190.0), -170.0);
        assert_eq!(wrap_deg(-190.0), 170.0);
        assert_eq!(wrap_deg(180.0), 180.0);
        assert_eq!(snap_angle(22.0, true), 15.0);
        assert_eq!(snap_angle(-52.0, true), -45.0);
        assert_eq!(snap_angle(22.004, false), 22.0);
        assert_eq!(snap_angle(370.0, false), 10.0);
    }

    #[test]
    fn moves_snap_on_one_axis_only() {
        assert_eq!(moved([1.0, 2.5, 3.0], 2, 5.1, 8.0), [1.0, 2.5, 8.0]);
        assert_eq!(moved([1.0, 2.5, 3.0], 0, 0.0004, 0.0), [1.0, 2.5, 3.0]);
        assert_eq!(moved([1.0, 2.5, 3.0], 1, 1.23456, 0.0), [1.0, 3.735, 3.0]);
    }

    #[test]
    fn rotation_about_a_pivot_turns_the_pose() {
        let (p, r) = rotated([10.0, 0.0, 0.0], [0.0; 3], DVec3::ZERO, 2, 90.0);
        assert_eq!(p, [0.0, 10.0, 0.0]);
        assert_eq!(r, [0.0, 0.0, 90.0]);
        let (p, r) = rotated([10.0, 0.0, 0.0], [0.0, 0.0, 90.0], DVec3::new(10.0, 0.0, 0.0), 0, 90.0);
        assert_eq!(p, [10.0, 0.0, 0.0]);
        // Rx(90)·Rz(90): the same matrix, whatever euler triple names it
        let want = DMat3::from_axis_angle(DVec3::X, 90f64.to_radians()) * rot_mat([0.0, 0.0, 90.0]);
        let got = rot_mat(r);
        for c in 0..3 {
            assert!((want.col(c) - got.col(c)).length() < 1e-6, "{r:?}");
        }
    }

    #[test]
    fn draws_arrows_or_rings_with_the_hot_one_lit() {
        let cam = cam();
        let g = Gizmo::new(Vec3::new(1.0, 2.0, 3.0), Mode::Move, &cam, 480.0);
        let items = g.draw(Some(Handle::Axis(1)));
        assert_eq!(items.len(), 6);
        assert_eq!(items[0].mesh, MESH_SHAFT);
        assert_eq!(items[1].mesh, MESH_CONE);
        assert_eq!(items[2].color, srgb(HOT_COLOR));
        assert_ne!(items[0].color, srgb(HOT_COLOR));
        // the x arrow's model carries +z onto +x
        let tip = items[0].model.transform_point3(Vec3::Z);
        assert!((tip - (g.center + Vec3::X * g.length)).length() < 1e-3, "{tip:?}");
        let tip = items[2].model.transform_point3(Vec3::Z);
        assert!((tip - (g.center + Vec3::Y * g.length)).length() < 1e-3, "{tip:?}");
        let rings = Gizmo { mode: Mode::Rotate, ..g }.draw(None);
        assert_eq!(rings.len(), 3);
        assert!(rings.iter().all(|r| r.mesh == MESH_RING));
    }
}
