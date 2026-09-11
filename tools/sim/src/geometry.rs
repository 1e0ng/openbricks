//! Triangle meshes for everything the viewport draws: decoded LDraw
//! and imported meshes, and tessellated boxes, cylinders and spheres.

use crate::assembly::Shape;
use crate::bundle::MeshData;
use std::f32::consts::PI;

pub fn shape_mesh(shape: &Shape) -> MeshData {
    match shape {
        Shape::Box { size, pos } => box_mesh(
            [size[0] as f32, size[1] as f32, size[2] as f32],
            [pos[0] as f32, pos[1] as f32, pos[2] as f32],
        ),
        Shape::Cylinder { radius, length, axis, pos } => cylinder_mesh(
            *radius as f32,
            *length as f32,
            axis,
            [pos[0] as f32, pos[1] as f32, pos[2] as f32],
            32,
        ),
        Shape::Sphere { radius, pos } => sphere_mesh(*radius as f32, [pos[0] as f32, pos[1] as f32, pos[2] as f32], 24, 16),
    }
}

pub fn box_mesh(size: [f32; 3], at: [f32; 3]) -> MeshData {
    let h = [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0];
    let mut m = MeshData::default();
    for (axis, sign) in [(0usize, 1.0f32), (0, -1.0), (1, 1.0), (1, -1.0), (2, 1.0), (2, -1.0)] {
        let mut n = [0.0f32; 3];
        n[axis] = sign;
        let (o1, o2) = match axis {
            0 => (1, 2),
            1 => (2, 0),
            _ => (0, 1),
        };
        let base = m.positions.len() as u32;
        for (u, v) in [(-1.0f32, -1.0f32), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            let mut p = [0.0f32; 3];
            p[axis] = sign * h[axis];
            p[o1] = u * h[o1];
            p[o2] = v * h[o2];
            m.positions.push([p[0] + at[0], p[1] + at[1], p[2] + at[2]]);
            m.normals.push(n);
        }
        if sign > 0.0 {
            m.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        } else {
            m.indices.extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
        }
    }
    m
}

pub fn cylinder_mesh(radius: f32, length: f32, axis: &str, at: [f32; 3], segments: u32) -> MeshData {
    // built along z, then the axes are permuted
    let perm = |v: [f32; 3]| -> [f32; 3] {
        match axis {
            "x" => [v[2], v[0], v[1]],
            "y" => [v[1], v[2], v[0]],
            _ => v,
        }
    };
    let mut m = MeshData::default();
    let h = length / 2.0;
    for i in 0..segments {
        let a0 = i as f32 / segments as f32 * 2.0 * PI;
        let a1 = (i + 1) as f32 / segments as f32 * 2.0 * PI;
        let (c0, s0, c1, s1) = (a0.cos(), a0.sin(), a1.cos(), a1.sin());
        let base = m.positions.len() as u32;
        for (c, s, z) in [(c0, s0, -h), (c1, s1, -h), (c1, s1, h), (c0, s0, h)] {
            let p = perm([radius * c, radius * s, z]);
            m.positions.push([p[0] + at[0], p[1] + at[1], p[2] + at[2]]);
            m.normals.push(perm([c, s, 0.0]));
        }
        m.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        for (z, nz, flip) in [(h, 1.0f32, false), (-h, -1.0, true)] {
            let base = m.positions.len() as u32;
            for p in [[0.0, 0.0, z], [radius * c0, radius * s0, z], [radius * c1, radius * s1, z]] {
                let q = perm(p);
                m.positions.push([q[0] + at[0], q[1] + at[1], q[2] + at[2]]);
                m.normals.push(perm([0.0, 0.0, nz]));
            }
            if flip {
                m.indices.extend_from_slice(&[base, base + 2, base + 1]);
            } else {
                m.indices.extend_from_slice(&[base, base + 1, base + 2]);
            }
        }
    }
    m
}

pub fn sphere_mesh(radius: f32, at: [f32; 3], segments: u32, rings: u32) -> MeshData {
    let mut m = MeshData::default();
    for r in 0..=rings {
        let phi = r as f32 / rings as f32 * PI;
        for s in 0..=segments {
            let theta = s as f32 / segments as f32 * 2.0 * PI;
            let n = [phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos()];
            m.positions
                .push([n[0] * radius + at[0], n[1] * radius + at[1], n[2] * radius + at[2]]);
            m.normals.push(n);
        }
    }
    let w = segments + 1;
    for r in 0..rings {
        for s in 0..segments {
            let a = r * w + s;
            let b = a + w;
            m.indices.extend_from_slice(&[a, b, a + 1, a + 1, b, b + 1]);
        }
    }
    m
}

/// A closed mesh's signed volume (mm³): positive when wound outward.
#[cfg(test)]
pub fn signed_volume(m: &MeshData) -> f64 {
    let mut v = 0.0f64;
    for t in m.indices.chunks_exact(3) {
        let a = m.positions[t[0] as usize].map(|x| x as f64);
        let b = m.positions[t[1] as usize].map(|x| x as f64);
        let c = m.positions[t[2] as usize].map(|x| x as f64);
        let cr = [b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]];
        v += (a[0] * cr[0] + a[1] * cr[1] + a[2] * cr[2]) / 6.0;
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_is_closed_and_outward() {
        let m = box_mesh([40.0, 20.0, 10.0], [0.0; 3]);
        assert_eq!(m.indices.len(), 36);
        assert!((signed_volume(&m) - 8000.0).abs() < 1e-3);
    }

    #[test]
    fn cylinder_and_sphere_volumes() {
        for axis in ["x", "y", "z"] {
            let m = cylinder_mesh(10.0, 20.0, axis, [5.0, -3.0, 2.0], 64);
            let exact = std::f64::consts::PI * 100.0 * 20.0;
            assert!((signed_volume(&m) - exact).abs() / exact < 0.01, "{axis}");
        }
        let s = sphere_mesh(10.0, [0.0; 3], 48, 24);
        let exact = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
        assert!((signed_volume(&s) - exact).abs() / exact < 0.01);
    }

    #[test]
    fn shapes_dispatch() {
        let m = shape_mesh(&Shape::Sphere {
            radius: 1.0,
            pos: [1.0, 2.0, 3.0],
        });
        assert!(!m.positions.is_empty());
        let m = shape_mesh(&Shape::Cylinder {
            radius: 1.0,
            length: 2.0,
            axis: "y".into(),
            pos: [0.0; 3],
        });
        assert_eq!(m.indices.len() % 3, 0);
    }
}
