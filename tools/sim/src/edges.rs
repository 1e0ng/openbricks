//! The edges of a brick worth a line: where a stack of bricks would
//! otherwise render as one mass, the seams; on a stud, its rims. They
//! come from the mesh itself — the boundary of every open shell and
//! every crease — which on LDraw parts is close to the set of edge
//! lines LDraw's own files draw: every one of a 2x4 brick's 472, all
//! but a few on beams, pins and bushes, fewer on parts whose detail
//! bends by less than the crease angle (tyres, rims, gear teeth).

use crate::bundle::MeshData;
use glam::Vec3;
use std::collections::HashMap;

/// The angle between two faces from which their shared edge is drawn,
/// in degrees (strictly more than it, so a twelve-sided cylinder's
/// facets at exactly 30° stay silent). The brick converter smooths
/// normals below the same angle, so drawn edges and shading creases
/// agree.
pub const CREASE_DEG: f32 = 30.0;

/// How close two vertices must be to count as one, in mm: far below
/// the bundle's 0.01 mm position step, far above the rounding that
/// separates a seam's two sides (sin 2π is not quite 0; an f32 at
/// 600 mm steps by 0.00006).
pub const WELD_MM: f32 = 0.002;

/// The feature edges of a mesh, as pairs of endpoints. Vertices within
/// `WELD_MM` are joined (a stud's base and the face it stands on meet
/// exactly in the bundle; a cylinder's seam only up to rounding); an
/// edge is kept when it belongs to one face only (the boundary of an
/// open shell), or when two of its faces meet at more than `CREASE_DEG`.
/// Triangulation diagonals and the facets of a 16-sided cylinder stay
/// silent, and so does a boundary edge that merely lies along the edge
/// of a neighbouring face in the same surface (LDraw builds a Technic
/// brick's face from primitives that meet at T-junctions: those seams
/// are not edges). Degenerate triangles are ignored.
/// A line in space, by its direction and the point nearest the origin,
/// quantised so edges on one line share a key; and an edge's extent
/// along its line.
type LineKey = ([i32; 3], [i32; 3]);
type Span = (f32, f32);
/// The edges on one line, each with its extent.
type OnLine = Vec<((u32, u32), Span)>;

pub fn feature_edges(mesh: &MeshData) -> Vec<[f32; 3]> {
    let (welded, remap) = weld(&mesh.positions, WELD_MM);
    let mut normals: Vec<Vec3> = Vec::new();
    let mut faces_of: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [remap[tri[0] as usize], remap[tri[1] as usize], remap[tri[2] as usize]];
        let (pa, pb, pc) = (
            Vec3::from(welded[a as usize]),
            Vec3::from(welded[b as usize]),
            Vec3::from(welded[c as usize]),
        );
        // degenerate: the corner's sine below rounding noise (three points in a line on
        // the 0.01 mm grid leave a cross product of residue, not a normal)
        let (ab, ac) = (pb - pa, pc - pa);
        let n = ab.cross(ac);
        if n.length_squared() <= 1e-12 * ab.length_squared() * ac.length_squared() {
            continue;
        }
        let f = normals.len() as u32;
        normals.push(n.normalize());
        for (u, v) in [(a, b), (b, c), (c, a)] {
            if u != v {
                faces_of.entry((u.min(v), u.max(v))).or_default().push(f);
            }
        }
    }
    let flat = CREASE_DEG.to_radians().cos() - 1e-4;
    let creased = |faces: &[u32]| {
        faces
            .iter()
            .enumerate()
            .any(|(i, &x)| faces[i + 1..].iter().any(|&y| normals[x as usize].dot(normals[y as usize]) < flat))
    };
    // every edge by the line it lies on, for the T-junction test
    let point = |i: u32| Vec3::from(welded[i as usize]);
    let line_of = |u: u32, v: u32| -> Option<(LineKey, Span)> {
        let (p, q) = (point(u), point(v));
        let mut d = q - p;
        let len = d.length();
        if len < 1e-6 {
            return None;
        }
        d /= len;
        let lead = d.to_array().into_iter().find(|c| c.abs() > 1e-6).unwrap_or(1.0);
        if lead < 0.0 {
            d = -d;
        }
        let foot = p - d * p.dot(d);
        let key = (
            d.to_array().map(|c| (c * 1000.0).round() as i32),
            foot.to_array().map(|c| (c * 200.0).round() as i32),
        );
        let (a, b) = (p.dot(d), q.dot(d));
        Some((key, (a.min(b), a.max(b))))
    };
    let mut on_line: HashMap<LineKey, OnLine> = HashMap::new();
    for &(u, v) in faces_of.keys() {
        if let Some((key, span)) = line_of(u, v) {
            on_line.entry(key).or_default().push(((u, v), span));
        }
    }
    // a boundary edge lying along other edges whose faces are all smooth with its own is a
    // seam within one surface, not an edge
    let seam = |e: (u32, u32), faces: &[u32]| -> bool {
        let Some((key, (lo, hi))) = line_of(e.0, e.1) else { return false };
        let f = faces[0] as usize;
        let mut partners = 0;
        for &(other, (olo, ohi)) in &on_line[&key] {
            if other == e || hi.min(ohi) - lo.max(olo) <= 5e-3 {
                continue;
            }
            partners += 1;
            if faces_of[&other].iter().any(|&g| normals[f].dot(normals[g as usize]) < flat) {
                return false;
            }
        }
        partners > 0
    };
    let mut kept: Vec<(u32, u32)> = faces_of
        .iter()
        .filter(|(e, faces)| if faces.len() == 1 { !seam(**e, faces) } else { creased(faces) })
        .map(|(k, _)| *k)
        .collect();
    kept.sort_unstable();
    kept.iter().flat_map(|&(u, v)| [welded[u as usize], welded[v as usize]]).collect()
}

/// Vertices joined by position: the distinct positions and, for every
/// input vertex, the index of its position. Two vertices within `tol`
/// are one; the lookup is by grid cell and the 26 cells around it, so a
/// pair that straddles a cell boundary still meets.
pub fn weld(positions: &[[f32; 3]], tol: f32) -> (Vec<[f32; 3]>, Vec<u32>) {
    let mut cells: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
    let mut welded: Vec<[f32; 3]> = Vec::new();
    let cell = |p: &[f32; 3]| {
        [
            (p[0] / tol).floor() as i64,
            (p[1] / tol).floor() as i64,
            (p[2] / tol).floor() as i64,
        ]
    };
    let remap = positions
        .iter()
        .map(|p| {
            let c = cell(p);
            let q = Vec3::from(*p);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(ids) = cells.get(&[c[0] + dx, c[1] + dy, c[2] + dz])
                            && let Some(&id) = ids.iter().find(|&&id| (Vec3::from(welded[id as usize]) - q).length() <= tol)
                        {
                            return id;
                        }
                    }
                }
            }
            welded.push(*p);
            let id = (welded.len() - 1) as u32;
            cells.entry(c).or_default().push(id);
            id
        })
        .collect();
    (welded, remap)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry;

    fn segments(edges: &[[f32; 3]]) -> Vec<(Vec3, Vec3)> {
        edges.chunks_exact(2).map(|e| (Vec3::from(e[0]), Vec3::from(e[1]))).collect()
    }

    #[test]
    fn a_box_has_its_twelve_edges_and_no_diagonals() {
        let m = geometry::box_mesh([10.0, 20.0, 30.0], [1.0, 2.0, 3.0]);
        let segs = segments(&feature_edges(&m));
        assert_eq!(segs.len(), 12);
        let mut lengths: Vec<i32> = segs.iter().map(|(a, b)| (a - b).length().round() as i32).collect();
        lengths.sort_unstable();
        assert_eq!(lengths, [10, 10, 10, 10, 20, 20, 20, 20, 30, 30, 30, 30]);
    }

    #[test]
    fn a_cylinder_shows_its_rims_and_no_side_lines() {
        // 16 facets meet at 22.5°, below the crease: only the two rims are drawn (the seam
        // where the last facet meets the first differs by the rounding of sin 2π, and welds)
        let m = geometry::cylinder_mesh(5.0, 10.0, "z", [0.0; 3], 16);
        let segs = segments(&feature_edges(&m));
        assert_eq!(segs.len(), 32);
        let chord = 2.0 * 5.0 * (std::f32::consts::PI / 16.0).sin();
        for (a, b) in &segs {
            assert!((a.z.abs() - 5.0).abs() < 1e-5 && (a.z - b.z).abs() < 1e-5, "{a} {b}");
            assert!(((a - b).length() - chord).abs() < 1e-4);
        }
        // 8 facets meet at 45°: the side lines join the rims
        let m = geometry::cylinder_mesh(5.0, 10.0, "y", [3.0, 0.0, 0.0], 8);
        assert_eq!(feature_edges(&m).len() / 2, 16 + 8);
    }

    #[test]
    fn an_open_shell_s_rim_is_drawn_as_its_boundary() {
        // the box without its bottom face: the four bottom edges belong to one face each
        let mut m = geometry::box_mesh([8.0, 8.0, 9.6], [0.0; 3]);
        m.indices.truncate(m.indices.len() - 6);
        assert_eq!(feature_edges(&m).len() / 2, 12);
    }

    #[test]
    fn a_fin_on_a_flat_seam_is_drawn_once_and_the_seam_is_not() {
        // two squares in a plane sharing an edge, and a fin standing on that edge
        let mut m = MeshData::default();
        let quad = |m: &mut MeshData, pts: [[f32; 3]; 4]| {
            let b = m.positions.len() as u32;
            m.positions.extend(pts);
            m.normals.extend([[0.0, 0.0, 1.0]; 4]);
            m.indices.extend([b, b + 1, b + 2, b, b + 2, b + 3]);
        };
        quad(&mut m, [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]);
        quad(&mut m, [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 1.0, 0.0], [1.0, 1.0, 0.0]]);
        quad(&mut m, [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [1.0, 1.0, 1.0], [1.0, 0.0, 1.0]]);
        let segs = segments(&feature_edges(&m));
        // 3 outer edges of each square, the fin's 3 free edges, and the seam under the fin
        assert_eq!(segs.len(), 10);
        let seam = segs
            .iter()
            .filter(|(a, b)| a.x == 1.0 && b.x == 1.0 && a.z == 0.0 && b.z == 0.0)
            .count();
        assert_eq!(seam, 1);
        // without the fin the seam is silent: two coplanar faces
        m.indices.truncate(12);
        m.positions.truncate(8);
        assert_eq!(feature_edges(&m).len() / 2, 6);
    }

    #[test]
    fn welding_joins_neighbours_across_cell_walls_and_keeps_the_bundle_s_grid_apart() {
        // 0.0019 and 0.0021 fall in different cells and still weld; 0.01 apart stays apart
        let pts = [[0.0019, 0.0, 0.0], [0.0021, 0.0, 0.0], [0.012, 0.0, 0.0], [0.0019, 0.0, 0.0]];
        let (welded, remap) = weld(&pts, WELD_MM);
        assert_eq!(welded.len(), 2);
        assert_eq!(remap, [0, 0, 1, 0]);
    }

    #[test]
    fn a_t_junction_within_one_face_is_silent_and_on_a_crease_is_kept() {
        // two squares side by side and, above them, one twice as wide whose lower edge runs
        // along both their upper edges: three shells meeting at T-junctions in one plane
        let quad = |m: &mut MeshData, pts: [[f32; 3]; 4], n: [f32; 3]| {
            let b = m.positions.len() as u32;
            m.positions.extend(pts);
            m.normals.extend([n; 4]);
            m.indices.extend([b, b + 1, b + 2, b, b + 2, b + 3]);
        };
        let mut flat = MeshData::default();
        quad(
            &mut flat,
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        quad(
            &mut flat,
            [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 1.0, 0.0], [1.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        quad(
            &mut flat,
            [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0], [2.0, 2.0, 0.0], [0.0, 2.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let segs = segments(&feature_edges(&flat));
        // the outline of the 2 × 2 square only (in the seven pieces the shells cut it into):
        // the seam at y = 1 is silent
        assert_eq!(segs.len(), 7, "{segs:?}");
        assert!(segs.iter().all(|(a, b)| !(a.y == 1.0 && b.y == 1.0)), "{segs:?}");
        // the wide one standing up instead: its lower edge is a crease, and stays
        let mut bent = MeshData::default();
        quad(
            &mut bent,
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        quad(
            &mut bent,
            [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 1.0, 0.0], [1.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        quad(
            &mut bent,
            [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0], [2.0, 1.0, 1.0], [0.0, 1.0, 1.0]],
            [0.0, -1.0, 0.0],
        );
        let segs = segments(&feature_edges(&bent));
        assert_eq!(
            segs.iter()
                .filter(|(a, b)| a.y == 1.0 && b.y == 1.0 && a.z == 0.0 && b.z == 0.0)
                .count(),
            3,
            "{segs:?}"
        );
    }

    #[test]
    fn degenerate_triangles_and_empty_meshes_draw_nothing() {
        assert!(feature_edges(&MeshData::default()).is_empty());
        let m = MeshData {
            positions: vec![[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            normals: vec![[0.0, 0.0, 1.0]; 3],
            indices: vec![0, 1, 2],
            uvs: vec![],
        };
        assert!(feature_edges(&m).is_empty());
        // three grid points in a line whose f32 cross product is rounding residue, not zero
        let m = MeshData {
            positions: vec![[0.07, 0.11, 0.13], [10.07, 20.11, 30.13], [20.07, 40.11, 60.13]],
            normals: vec![[0.0, 0.0, 1.0]; 3],
            indices: vec![0, 1, 2],
            uvs: vec![],
        };
        assert!(feature_edges(&m).is_empty());
    }

    #[test]
    fn the_two_by_four_brick_draws_exactly_the_edges_ldraw_drew() {
        // 3001.dat carries 472 edge lines (type 2); the mesh yields the same 472
        let bundle = crate::editor::testing::real_bundle();
        let m = bundle.parts["3001"].mesh.decode().unwrap();
        assert_eq!(feature_edges(&m).len() / 2, 472);
        // and a beam's every hole rim: at least LDraw's 900, at most a few more where a
        // chamfer creases without an edge line of its own
        let m = bundle.parts["32316"].mesh.decode().unwrap();
        let n = feature_edges(&m).len() / 2;
        assert!((900..=1100).contains(&n), "{n}");
        // a Technic brick's faces are built from primitives meeting at T-junctions: none of
        // those seams is drawn (LDraw draws 308 edge lines on 3700)
        let m = bundle.parts["3700"].mesh.decode().unwrap();
        let n = feature_edges(&m).len() / 2;
        assert!((308..=320).contains(&n), "{n}");
        // pinned counts: a beam 5 keeps the sixteen pieces of each of its four corner lines
        // (1028, not 960 without them), a beam 15, a 16-long Technic brick with holes and the
        // frame, whose flat faces would carry hundreds of seams unfiltered
        for (num, want) in [("32316", 1028), ("32278", 3268), ("3703", 3392), ("64178", 4930)] {
            let m = bundle.parts[num].mesh.decode().unwrap();
            assert_eq!(feature_edges(&m).len() / 2, want, "{num}");
        }
    }
}
