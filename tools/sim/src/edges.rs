//! The edges of a brick worth a line: where a stack of bricks would
//! otherwise render as one mass, the seams; on a stud, its rims. They
//! come from the mesh itself — the boundary of every open shell and
//! every crease — which on LDraw parts is exactly the set of edge lines
//! LDraw's own files draw (checked on 3001, 32316, 2780, 3700, 3713,
//! 32523 and 3648: every one of their edge lines is found).

use crate::bundle::MeshData;
use glam::Vec3;
use std::collections::HashMap;

/// The angle between two faces from which their shared edge is drawn,
/// in degrees. The brick converter smooths normals below the same
/// angle, so drawn edges and shading creases agree.
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
/// open shell), or when two of its faces meet at `CREASE_DEG` or more.
/// Triangulation diagonals and the facets of a 16-sided cylinder stay
/// silent. Degenerate triangles are ignored.
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
        let n = (pb - pa).cross(pc - pa);
        if n.length_squared() < 1e-12 {
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
    let flat = CREASE_DEG.to_radians().cos();
    let creased = |faces: &[u32]| {
        faces
            .iter()
            .enumerate()
            .any(|(i, &x)| faces[i + 1..].iter().any(|&y| normals[x as usize].dot(normals[y as usize]) < flat))
    };
    let mut kept: Vec<(u32, u32)> = faces_of
        .iter()
        .filter(|(_, faces)| faces.len() == 1 || creased(faces))
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
    fn degenerate_triangles_and_empty_meshes_draw_nothing() {
        assert!(feature_edges(&MeshData::default()).is_empty());
        let m = MeshData {
            positions: vec![[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
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
    }
}
