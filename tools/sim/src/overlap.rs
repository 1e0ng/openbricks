//! Whether two placed parts overlap — one's material inside the
//! other's — or merely touch. Touching is what mating is: a plate's
//! underside on a brick's top, a pin's side against its hole, a stud's
//! top against the cavity it sits in. Overlap is a face of one part
//! crossing a face of the other deeper than `TOL_MM`, or two faces
//! lying in one plane with the same outward normal over more than a
//! sliver (both parts solid on the same side of that plane: the same
//! brick placed twice, a brick pushed into its neighbour at the same
//! height). Faces in one plane with opposite normals are contact.
//!
//! Each part's triangles sit in a bounding-volume tree in the part's
//! own frame; a pair is tested by bringing the second part's triangles
//! into the first's frame.

use crate::bundle::MeshData;
use glam::{DMat3, DVec2, DVec3};

/// How deep a face may cross another before the parts overlap, in mm:
/// above the 0.05 mm two 16-sided cylinders of one radius push into
/// each other when turned against each other, below anything a hand
/// would place. (Mated parts are excused altogether by the editor:
/// LDraw's friction pin lip is 2.55 mm in radius in a 2.4 mm hole.)
pub const TOL_MM: f64 = 0.2;
/// Faces this close, with parallel normals, lie in one plane.
pub const PLANE_MM: f64 = 0.05;
/// How the two parts overlap, when they do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Overlap {
    /// A face crosses a face this deep, in mm.
    Crossing(f64),
    /// Faces in one plane, facing the same way, share this area in mm².
    Coplanar(f64),
    /// One part sits wholly inside the other, no face meeting any.
    Inside,
}

impl Overlap {
    /// What a status line says this part would do to `other`. No
    /// number: a crossing's depth and a shared area are what one pair of
    /// faces does, not how far the parts interpenetrate.
    pub fn account(&self, other: &str) -> String {
        match self {
            Overlap::Crossing(_) | Overlap::Coplanar(_) => format!("overlap {other}"),
            Overlap::Inside => format!("lie wholly inside {other}"),
        }
    }
}

/// Where a part is: world = rot × local + pos.
#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub pos: DVec3,
    pub rot: DMat3,
}

struct Node {
    lo: DVec3,
    hi: DVec3,
    /// Children when `count == 0`, else the leaf's range in `order`.
    left: u32,
    right: u32,
    first: u32,
    count: u32,
}

/// A part's triangles in its own frame, under a bounding-volume tree.
pub struct Shape {
    tris: Vec<[DVec3; 3]>,
    normals: Vec<DVec3>,
    order: Vec<u32>,
    nodes: Vec<Node>,
    pub bbox: (DVec3, DVec3),
}

const LEAF: usize = 8;

impl Shape {
    pub fn from_mesh(mesh: &MeshData) -> Shape {
        let mut tris = Vec::with_capacity(mesh.indices.len() / 3);
        let mut normals = Vec::with_capacity(mesh.indices.len() / 3);
        for t in mesh.indices.chunks_exact(3) {
            let p = |i: u32| DVec3::from(mesh.positions[i as usize].map(f64::from));
            let tri = [p(t[0]), p(t[1]), p(t[2])];
            let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
            if n.length_squared() < 1e-12 {
                continue;
            }
            tris.push(tri);
            normals.push(n.normalize());
        }
        let mut order: Vec<u32> = (0..tris.len() as u32).collect();
        let mut nodes = Vec::new();
        let bbox = if tris.is_empty() {
            (DVec3::ZERO, DVec3::ZERO)
        } else {
            build(&tris, &mut order, 0, tris.len(), &mut nodes);
            (nodes[0].lo, nodes[0].hi)
        };
        Shape {
            tris,
            normals,
            order,
            nodes,
            bbox,
        }
    }

    #[cfg(test)]
    pub fn triangles(&self) -> usize {
        self.tris.len()
    }

    /// The part's box in the world, at `pose`.
    pub fn world_bbox(&self, pose: &Pose) -> (DVec3, DVec3) {
        let (lo, hi) = self.bbox;
        let mut wlo = DVec3::splat(f64::INFINITY);
        let mut whi = DVec3::splat(f64::NEG_INFINITY);
        for k in 0..8 {
            let c = DVec3::new(
                if k & 1 == 0 { lo.x } else { hi.x },
                if k & 2 == 0 { lo.y } else { hi.y },
                if k & 4 == 0 { lo.z } else { hi.z },
            );
            let w = pose.rot * c + pose.pos;
            wlo = wlo.min(w);
            whi = whi.max(w);
        }
        (wlo, whi)
    }

    /// Whether `p`, in the part's own frame, lies inside its material:
    /// a ray from it crosses the surface an odd number of times. The
    /// ray runs off any axis so it meets no edge of a well-behaved mesh.
    pub fn contains(&self, p: DVec3) -> bool {
        let dir = DVec3::new(0.31, 0.53, 0.79).normalize();
        let mut crossings = 0;
        for t in &self.tris {
            // Möller–Trumbore
            let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
            let h = dir.cross(e2);
            let a = e1.dot(h);
            if a.abs() < 1e-12 {
                continue;
            }
            let f = 1.0 / a;
            let s = p - t[0];
            let u = f * s.dot(h);
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = s.cross(e1);
            let v = f * dir.dot(q);
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            if f * e2.dot(q) > 1e-9 {
                crossings += 1;
            }
        }
        crossings % 2 == 1
    }

    /// The triangles whose boxes meet `[lo, hi]`.
    fn candidates(&self, lo: DVec3, hi: DVec3, out: &mut Vec<u32>) {
        if self.nodes.is_empty() {
            return;
        }
        let mut stack = vec![0usize];
        while let Some(i) = stack.pop() {
            let n = &self.nodes[i];
            if n.lo.x > hi.x || n.hi.x < lo.x || n.lo.y > hi.y || n.hi.y < lo.y || n.lo.z > hi.z || n.hi.z < lo.z {
                continue;
            }
            if n.count > 0 {
                out.extend_from_slice(&self.order[n.first as usize..(n.first + n.count) as usize]);
            } else {
                stack.push(n.left as usize);
                stack.push(n.right as usize);
            }
        }
    }
}

fn bounds(tris: &[[DVec3; 3]], order: &[u32]) -> (DVec3, DVec3) {
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for &i in order {
        for p in &tris[i as usize] {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
    }
    (lo, hi)
}

/// Builds the node for `order[from..to]`, splitting at the median
/// centroid along the longest axis, and returns its index.
fn build(tris: &[[DVec3; 3]], order: &mut [u32], from: usize, to: usize, nodes: &mut Vec<Node>) -> u32 {
    let (lo, hi) = bounds(tris, &order[from..to]);
    let idx = nodes.len() as u32;
    nodes.push(Node {
        lo,
        hi,
        left: 0,
        right: 0,
        first: from as u32,
        count: (to - from) as u32,
    });
    if to - from <= LEAF {
        return idx;
    }
    let size = hi - lo;
    let axis = if size.x >= size.y && size.x >= size.z {
        0
    } else if size.y >= size.z {
        1
    } else {
        2
    };
    let centroid = |i: u32| (tris[i as usize][0][axis] + tris[i as usize][1][axis] + tris[i as usize][2][axis]) / 3.0;
    let mid = from + (to - from) / 2;
    order[from..to].select_nth_unstable_by(mid - from, |&a, &b| centroid(a).total_cmp(&centroid(b)));
    let left = build(tris, order, from, mid, nodes);
    let right = build(tris, order, mid, to, nodes);
    nodes[idx as usize].left = left;
    nodes[idx as usize].right = right;
    nodes[idx as usize].count = 0;
    idx
}

/// Whether `b` at `pb` overlaps `a` at `pa`: the deepest crossing found,
/// else the largest shared coplanar area, else — when one part's box
/// lies within the other's and its centre inside the other's material —
/// wholly inside, else None (touching or apart).
pub fn overlap(a: &Shape, pa: &Pose, b: &Shape, pb: &Pose) -> Option<Overlap> {
    let (alo, ahi) = a.world_bbox(pa);
    let (blo, bhi) = b.world_bbox(pb);
    let pad = DVec3::splat(TOL_MM);
    if (alo - pad).cmpgt(bhi).any() || (ahi + pad).cmplt(blo).any() {
        return None;
    }
    // b's triangles in a's frame
    let inv = pa.rot.transpose();
    let to_a = |p: DVec3| inv * (pb.rot * p + pb.pos - pa.pos);
    let mut deepest: Option<f64> = None;
    let mut widest: Option<f64> = None;
    let mut cands = Vec::new();
    for (tb, nb) in b.tris.iter().zip(&b.normals) {
        let t = [to_a(tb[0]), to_a(tb[1]), to_a(tb[2])];
        let n = inv * (pb.rot * *nb);
        let lo = t[0].min(t[1]).min(t[2]) - pad;
        let hi = t[0].max(t[1]).max(t[2]) + pad;
        cands.clear();
        a.candidates(lo, hi, &mut cands);
        for &i in &cands {
            let (ta, na) = (&a.tris[i as usize], a.normals[i as usize]);
            if let Some(d) = crossing(ta, na, &t, n) {
                deepest = Some(deepest.map_or(d, |x: f64| x.max(d)));
            } else if let Some(area) = coplanar_same_way(ta, na, &t, n) {
                widest = Some(widest.map_or(area, |x: f64| x.max(area)));
            }
        }
    }
    if let Some(found) = deepest.map(Overlap::Crossing).or(widest.map(Overlap::Coplanar)) {
        return Some(found);
    }
    // no face meets any: one part may still sit wholly inside the other
    let within = |lo: DVec3, hi: DVec3, olo: DVec3, ohi: DVec3| lo.cmpge(olo - pad).all() && hi.cmple(ohi + pad).all();
    if within(blo, bhi, alo, ahi) && a.contains(to_a((b.bbox.0 + b.bbox.1) * 0.5)) {
        return Some(Overlap::Inside);
    }
    if within(alo, ahi, blo, bhi) {
        let to_b = |p: DVec3| pb.rot.transpose() * (pa.rot * p + pa.pos - pb.pos);
        if b.contains(to_b((a.bbox.0 + a.bbox.1) * 0.5)) {
            return Some(Overlap::Inside);
        }
    }
    None
}

/// The points where a triangle meets a plane, given its vertices'
/// signed distances: the vertices on it and the crossings of its edges.
fn clip(t: &[DVec3; 3], d: [f64; 3]) -> Vec<DVec3> {
    let mut pts = Vec::with_capacity(3);
    for i in 0..3 {
        let j = (i + 1) % 3;
        if d[i].abs() <= 1e-9 {
            pts.push(t[i]);
        } else if d[i] * d[j] < 0.0 {
            pts.push(t[i] + (t[j] - t[i]) * (d[i] / (d[i] - d[j])));
        }
    }
    pts
}

/// How far a triangle can be pushed along `axis` before the two no
/// longer overlap there: the lesser of the two ways apart.
fn apart_along(a: &[DVec3; 3], b: &[DVec3; 3], axis: DVec3) -> f64 {
    let span = |t: &[DVec3; 3]| {
        t.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p.dot(axis)), hi.max(p.dot(axis)))
        })
    };
    let ((alo, ahi), (blo, bhi)) = (span(a), span(b));
    (ahi - blo).min(bhi - alo)
}

/// The least translation that separates two triangles that cross: the
/// smallest push apart over the separating-axis candidates (each face's
/// normal and every pair of edges' cross product). A face reaching far
/// behind another's plane is not the measure — a wall pushed 0.1 mm
/// into a tall neighbour is 0.1 mm in, however tall both are.
fn least_apart(a: &[DVec3; 3], na: DVec3, b: &[DVec3; 3], nb: DVec3) -> f64 {
    let mut least = apart_along(a, b, na).min(apart_along(a, b, nb));
    for i in 0..3 {
        let ea = a[(i + 1) % 3] - a[i];
        for j in 0..3 {
            let axis = ea.cross(b[(j + 1) % 3] - b[j]);
            if axis.length_squared() > 1e-12 {
                least = least.min(apart_along(a, b, axis.normalize()));
            }
        }
    }
    least.max(0.0)
}

/// The depth by which triangle `a` (normal `na`) and triangle `b`
/// (normal `nb`) cross into each other's material beyond `TOL_MM`: the
/// least push that would part them, when each passes through the
/// other's plane (a vertex clearly on either side — an edge lying in
/// the plane is contact) and the two meet along their planes' line.
fn crossing(a: &[DVec3; 3], na: DVec3, b: &[DVec3; 3], nb: DVec3) -> Option<f64> {
    let da = [(a[0] - b[0]).dot(nb), (a[1] - b[0]).dot(nb), (a[2] - b[0]).dot(nb)];
    let inside_a = -da[0].min(da[1]).min(da[2]);
    if inside_a <= TOL_MM || da[0].max(da[1]).max(da[2]) <= PLANE_MM {
        return None;
    }
    let db = [(b[0] - a[0]).dot(na), (b[1] - a[0]).dot(na), (b[2] - a[0]).dot(na)];
    let inside_b = -db[0].min(db[1]).min(db[2]);
    if inside_b <= TOL_MM || db[0].max(db[1]).max(db[2]) <= PLANE_MM {
        return None;
    }
    let dir = na.cross(nb);
    if dir.length_squared() < 1e-12 {
        return None;
    }
    let span = |pts: &[DVec3]| {
        pts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
            let t = p.dot(dir);
            (lo.min(t), hi.max(t))
        })
    };
    let (alo, ahi) = span(&clip(a, da));
    let (blo, bhi) = span(&clip(b, db));
    // the two cuts lie on that line: they must share a length, not just an end (an axle
    // arm's tip meeting a hole's countersink ring at one point is contact)
    if ahi.min(bhi) - alo.max(blo) <= 1e-4 {
        return None;
    }
    let depth = least_apart(a, na, b, nb);
    (depth > TOL_MM).then_some(depth)
}

/// The area two triangles share when they lie in one plane and face
/// the same way, unless it is a sliver narrower than `TOL_MM`.
fn coplanar_same_way(a: &[DVec3; 3], na: DVec3, b: &[DVec3; 3], nb: DVec3) -> Option<f64> {
    if na.dot(nb) < 0.99939 {
        return None; // not the same way within 2°
    }
    if b.iter().any(|p| (*p - a[0]).dot(na).abs() > PLANE_MM) {
        return None;
    }
    // a 2D frame in the plane
    let u = (a[1] - a[0]).normalize();
    let v = na.cross(u);
    let flat = |p: DVec3| DVec2::new((p - a[0]).dot(u), (p - a[0]).dot(v));
    let mut poly: Vec<DVec2> = b.iter().map(|p| flat(*p)).collect();
    let tri: Vec<DVec2> = a.iter().map(|p| flat(*p)).collect();
    let ccw = signed_area(&tri) > 0.0;
    for i in 0..3 {
        let (p, q) = (tri[i], tri[(i + 1) % 3]);
        poly = clip_polygon(&poly, p, q, ccw);
        if poly.len() < 3 {
            return None;
        }
    }
    let area = signed_area(&poly).abs();
    let perimeter: f64 = (0..poly.len()).map(|i| (poly[(i + 1) % poly.len()] - poly[i]).length()).sum();
    if perimeter <= 0.0 || 2.0 * area / perimeter <= TOL_MM {
        return None;
    }
    Some(area)
}

fn signed_area(poly: &[DVec2]) -> f64 {
    (0..poly.len())
        .map(|i| {
            let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
            p.x * q.y - q.x * p.y
        })
        .sum::<f64>()
        * 0.5
}

/// Sutherland–Hodgman: `poly` cut to the inner side of the edge p→q of
/// a triangle wound counter-clockwise when `ccw`.
fn clip_polygon(poly: &[DVec2], p: DVec2, q: DVec2, ccw: bool) -> Vec<DVec2> {
    let side = |x: DVec2| {
        let s = (q.x - p.x) * (x.y - p.y) - (q.y - p.y) * (x.x - p.x);
        if ccw { s } else { -s }
    };
    let mut out = Vec::with_capacity(poly.len() + 2);
    for i in 0..poly.len() {
        let (cur, next) = (poly[i], poly[(i + 1) % poly.len()]);
        let (sc, sn) = (side(cur), side(next));
        if sc >= 0.0 {
            out.push(cur);
        }
        if (sc >= 0.0) != (sn >= 0.0) && (sc - sn).abs() > 1e-15 {
            out.push(cur + (next - cur) * (sc / (sc - sn)));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::editor::testing::real_bundle;
    use crate::geometry;

    fn at(x: f64, y: f64, z: f64) -> Pose {
        Pose {
            pos: DVec3::new(x, y, z),
            rot: DMat3::IDENTITY,
        }
    }

    fn turned(x: f64, y: f64, z: f64, rot: DMat3) -> Pose {
        Pose {
            pos: DVec3::new(x, y, z),
            rot,
        }
    }

    fn shape(num: &str) -> Shape {
        Shape::from_mesh(&real_bundle().parts[num].mesh.decode().unwrap())
    }

    #[test]
    fn boxes_touch_side_by_side_and_overlap_when_pushed_in() {
        let b = Shape::from_mesh(&geometry::box_mesh([10.0, 10.0, 10.0], [0.0; 3]));
        assert_eq!(overlap(&b, &at(0.0, 0.0, 0.0), &b, &at(10.0, 0.0, 0.0)), None, "flush");
        assert_eq!(overlap(&b, &at(0.0, 0.0, 0.0), &b, &at(10.1, 0.0, 0.0)), None, "a gap");
        assert_eq!(overlap(&b, &at(0.0, 0.0, 0.0), &b, &at(0.0, 0.0, 10.0)), None, "stacked");
        // pushed in: the top faces share a 1 × 10 strip in one plane, facing the same way
        // (the area is the largest one triangle pair shares: 9 of the strip's 10)
        match overlap(&b, &at(0.0, 0.0, 0.0), &b, &at(9.0, 0.0, 0.0)) {
            Some(Overlap::Coplanar(area)) => assert!((area - 9.0).abs() < 1e-6, "{area}"),
            other => panic!("{other:?}"),
        }
        // the same box twice: the same place
        assert!(matches!(
            overlap(&b, &at(0.0, 0.0, 0.0), &b, &at(0.0, 0.0, 0.0)),
            Some(Overlap::Coplanar(_))
        ));
        // a smaller box sunk into the top: its walls cross the top face by the sinking
        let s = Shape::from_mesh(&geometry::box_mesh([4.0, 4.0, 4.0], [0.0; 3]));
        match overlap(&b, &at(0.0, 0.0, 0.0), &s, &at(0.0, 0.0, 6.0)) {
            Some(Overlap::Crossing(d)) => assert!((d - 1.0).abs() < 1e-6, "{d}"),
            other => panic!("{other:?}"),
        }
        assert_eq!(overlap(&b, &at(0.0, 0.0, 0.0), &s, &at(0.0, 0.0, 6.9)), None, "0.1 mm is touching");
        // a box turned 45° on top touches; sunk it crosses
        let r = DMat3::from_rotation_z(45f64.to_radians());
        assert_eq!(overlap(&b, &at(0.0, 0.0, 0.0), &s, &turned(0.0, 0.0, 7.0, r)), None);
        assert!(matches!(
            overlap(&b, &at(0.0, 0.0, 0.0), &s, &turned(0.0, 0.0, 5.0, r)),
            Some(Overlap::Crossing(_))
        ));
        assert!(matches!(
            overlap(&b, &at(0.0, 0.0, 0.0), &s, &turned(1.0, 0.0, 5.0, r)),
            Some(Overlap::Crossing(_))
        ));
        assert_eq!(overlap(&b, &at(0.0, 0.0, 0.0), &s, &at(30.0, 0.0, 0.0)), None, "apart");
        assert_eq!(b.triangles(), 12);
        // a wall pushed a little into a much taller neighbour is in by that little, not by
        // how far it reaches behind the neighbour's faces
        let tall = Shape::from_mesh(&geometry::box_mesh([10.0, 10.0, 40.0], [0.0; 3]));
        assert_eq!(
            overlap(&tall, &at(0.0, 0.0, 0.0), &s, &at(6.9, 0.0, 0.0)),
            None,
            "0.1 mm in: touching"
        );
        match overlap(&tall, &at(0.0, 0.0, 0.0), &s, &at(6.7, 0.0, 0.0)) {
            Some(Overlap::Crossing(d)) => assert!((d - 0.3).abs() < 1e-6, "{d}"),
            other => panic!("{other:?}"),
        }
        // a small box wholly inside the big one meets no face of it: still an overlap, both ways round
        assert_eq!(overlap(&b, &at(0.0, 0.0, 0.0), &s, &at(1.0, -1.0, 2.0)), Some(Overlap::Inside));
        assert_eq!(overlap(&s, &at(1.0, -1.0, 2.0), &b, &at(0.0, 0.0, 0.0)), Some(Overlap::Inside));
        assert!(b.contains(DVec3::new(4.9, -4.9, 4.9)) && !b.contains(DVec3::new(5.1, 0.0, 0.0)));
    }

    #[test]
    fn a_plate_on_its_stud_grid_and_a_brick_on_a_brick_touch() {
        let (brick, plate) = (shape("3001"), shape("3022"));
        // the origin is the top face: a plate's body is 3.2 mm below it, a brick's 9.6
        assert_eq!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &plate, &at(0.0, 0.0, 3.2)),
            None,
            "plate on four studs"
        );
        assert_eq!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &plate, &at(8.0, 0.0, 3.2)),
            None,
            "plate on the end studs"
        );
        assert_eq!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &brick, &at(0.0, 0.0, 9.6)),
            None,
            "brick on brick"
        );
        assert_eq!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &brick, &at(8.0, 0.0, 9.6)),
            None,
            "brick on brick, offset"
        );
        assert_eq!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &brick, &at(32.0, 0.0, 0.0)),
            None,
            "bricks end to end"
        );
        assert_eq!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &brick, &at(0.0, 16.0, 0.0)),
            None,
            "bricks side by side"
        );
        // sunk into the brick, the plate's walls cross its top face: the least push apart
        // is the sinking, or what is left of the wall above the top face when that is less
        for (dz, want) in [(1.0, 1.0), (2.0, 1.2)] {
            match overlap(&brick, &at(0.0, 0.0, 0.0), &plate, &at(0.0, 0.0, 3.2 - dz)) {
                Some(Overlap::Crossing(d)) => assert!((d - want).abs() < 0.02, "sunk {dz}: {d}"),
                other => panic!("sunk {dz}: {other:?}"),
            }
        }
        assert_eq!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &plate, &at(0.0, 0.0, 3.0)),
            None,
            "0.2 mm low is touching"
        );
        assert!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &brick, &at(31.0, 0.0, 0.0)).is_some(),
            "pushed in 1 mm"
        );
        assert!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &brick, &at(0.0, 0.0, 0.0)).is_some(),
            "the same place"
        );
        assert!(
            overlap(&brick, &at(0.0, 0.0, 0.0), &brick, &at(4.0, 0.0, 4.8)).is_some(),
            "half a brick up"
        );
    }

    #[test]
    fn pins_axles_and_bushes_touch_in_their_holes_and_overlap_off_centre() {
        let (beam, pin, axle, bush) = (shape("32316"), shape("2780"), shape("3705"), shape("3713"));
        // the beam's holes run along z at y = -16, -8, 0, 8, 16; a pin's halves run along x
        let up = DMat3::from_rotation_y(-90f64.to_radians());
        assert_eq!(
            overlap(&beam, &at(0.0, 0.0, 0.0), &pin, &turned(0.0, -16.0, 4.0, up)),
            None,
            "pin in a hole"
        );
        assert_eq!(
            overlap(&beam, &at(0.0, 0.0, 0.0), &pin, &turned(0.0, 8.0, -4.0, up)),
            None,
            "pin from below"
        );
        assert!(
            overlap(&beam, &at(0.0, 0.0, 0.0), &pin, &turned(0.5, -16.0, 4.0, up)).is_some(),
            "pin off centre"
        );
        assert!(
            overlap(&beam, &at(0.0, 0.0, 0.0), &pin, &turned(0.0, -16.0, 2.0, up)).is_some(),
            "pin collar sunk"
        );
        assert_eq!(
            overlap(&beam, &at(0.0, 0.0, 0.0), &axle, &turned(0.0, 0.0, 0.0, up)),
            None,
            "axle through a hole"
        );
        // turned about its own axis in the round hole it is as free: its arm tips only graze
        for deg in [5.0f64, 10.0, 22.5] {
            let turned_axle = up * DMat3::from_rotation_x(deg.to_radians());
            assert_eq!(
                overlap(&beam, &at(0.0, 0.0, 0.0), &axle, &turned(0.0, 0.0, 0.0, turned_axle)),
                None,
                "axle turned {deg}°"
            );
        }
        assert!(
            overlap(&beam, &at(0.0, 0.0, 0.0), &axle, &turned(0.0, 1.0, 0.0, up)).is_some(),
            "axle off centre"
        );
        // a bush's hole runs along y; on the axle (along x) it turns about z
        let on = DMat3::from_rotation_z(90f64.to_radians());
        assert_eq!(
            overlap(&axle, &at(0.0, 0.0, 0.0), &bush, &turned(8.0, 0.0, 0.0, on)),
            None,
            "bush on the axle"
        );
        assert!(
            overlap(&axle, &at(0.0, 0.0, 0.0), &bush, &turned(8.0, 1.0, 0.0, on)).is_some(),
            "bush off the axle"
        );
        // two pins in neighbouring holes never meet; a pin in a Technic brick's hole is fine
        assert_eq!(overlap(&pin, &turned(0.0, -16.0, 4.0, up), &pin, &turned(0.0, -8.0, 4.0, up)), None);
        let tb = shape("3700");
        let along_y = DMat3::from_rotation_z(90f64.to_radians());
        assert_eq!(
            overlap(&tb, &at(0.0, 0.0, 0.0), &pin, &turned(0.0, 4.0, -4.0, along_y)),
            None,
            "pin in a Technic brick"
        );
    }

    #[test]
    fn meshed_gears_touch_and_jammed_ones_overlap() {
        // two 24-tooth gears (module 1: pitch radius 12 mm; the axle runs along y) 24 mm
        // apart, one turned half a tooth so their teeth interleave; unturned, tooth on tooth
        let gear = shape("3648");
        let half_tooth = DMat3::from_rotation_y((360.0f64 / 24.0 / 2.0).to_radians());
        let meshed = overlap(&gear, &at(0.0, 0.0, 0.0), &gear, &turned(24.0, 0.0, 0.0, half_tooth));
        assert_eq!(meshed, None, "meshed gears");
        assert!(
            matches!(
                overlap(&gear, &at(0.0, 0.0, 0.0), &gear, &at(24.0, 0.0, 0.0)),
                Some(Overlap::Crossing(_))
            ),
            "tooth on tooth"
        );
    }

    #[test]
    fn the_test_is_quick_enough_for_a_drag() {
        let (brick, plate) = (shape("3001"), shape("3022"));
        let t0 = std::time::Instant::now();
        for _ in 0..20 {
            assert_eq!(overlap(&brick, &at(0.0, 0.0, 0.0), &plate, &at(0.0, 0.0, 3.2)), None);
        }
        let per = t0.elapsed().as_secs_f64() / 20.0;
        eprintln!("plate on brick: {:.2} ms per test", per * 1000.0);
        assert!(per < 0.05, "{per} s per test");
        let (wlo, whi) = brick.world_bbox(&turned(10.0, 0.0, 0.0, DMat3::from_rotation_z(90f64.to_radians())));
        assert!(
            (wlo.x - 2.0).abs() < 1e-9 && (whi.x - 18.0).abs() < 1e-9 && (wlo.y + 16.0).abs() < 1e-9,
            "{wlo} {whi}"
        );
    }

    #[test]
    fn the_account_names_the_other_part_and_whole_containment() {
        assert_eq!(Overlap::Crossing(1.26).account("b"), "overlap b");
        assert_eq!(Overlap::Coplanar(512.0).account("b"), "overlap b");
        assert_eq!(Overlap::Inside.account("b"), "lie wholly inside b");
        let e = Shape::from_mesh(&MeshData::default());
        assert_eq!(overlap(&e, &at(0.0, 0.0, 0.0), &e, &at(0.0, 0.0, 0.0)), None);
    }
}
