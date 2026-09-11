//! STL import: a file becomes a brick with exact mass properties, a
//! packed mesh record like the bundle's, and the pin holes found on it
//! — the record the web workbench writes, so the Python loader and the
//! Simulate view read it as they do any imported part.

use crate::assembly::{self, Part};
use crate::bundle::{Connector, MeshRecord};
use base64::Engine;
use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

/// One triangle: three corners, in the file's units.
pub type Tri = [[f32; 3]; 3];
/// A quantised position (0.01 mm) and normal (1/127): one packed vertex.
type VertexKey = ((i32, i32, i32), [i8; 3]);
/// A face's vote for a bore axis: face index, u, v, and its midpoint along the axis.
type Vote = (usize, f64, f64, f64);
/// A vertex position quantised to 0.001 mm, the key edges are matched by.
type Point = (i64, i64, i64);

/// More than this and the record would dwarf the assembly file.
pub const MAX_TRIS: usize = 1_500_000;
pub const CATEGORIES: [&str; 5] = ["other", "lego", "electronics", "servo", "wheel"];
pub const DENSITIES: [(&str, f64); 5] = [("PLA", 1.24), ("PETG", 1.27), ("ABS", 1.05), ("nylon", 1.10), ("aluminium", 2.70)];
const BORE_RADIUS_MM: f64 = 2.4;
const BORE_MIN_VOTES: usize = 12;

/// Triangles from a binary or ASCII STL file.
pub fn parse(bytes: &[u8]) -> Result<Vec<Tri>, String> {
    if bytes.len() >= 84 {
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if 84 + 50 * n == bytes.len() {
            let f = |o: usize| f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]);
            return Ok((0..n)
                .map(|i| {
                    let o = 84 + 50 * i + 12;
                    [
                        [f(o), f(o + 4), f(o + 8)],
                        [f(o + 12), f(o + 16), f(o + 20)],
                        [f(o + 24), f(o + 28), f(o + 32)],
                    ]
                })
                .collect());
        }
    }
    let text = String::from_utf8_lossy(bytes);
    if text.trim_start().get(..5).map(|s| s.eq_ignore_ascii_case("solid")).unwrap_or(false) {
        let mut verts: Vec<[f32; 3]> = Vec::new();
        let mut words = text.split_ascii_whitespace();
        while let Some(w) = words.next() {
            if w.eq_ignore_ascii_case("vertex") {
                let mut v = [0f32; 3];
                for c in v.iter_mut() {
                    *c = words
                        .next()
                        .and_then(|s| s.parse().ok())
                        .ok_or("a vertex line with fewer than three numbers")?;
                }
                verts.push(v);
            }
        }
        if !verts.is_empty() && verts.len().is_multiple_of(3) {
            return Ok(verts.as_chunks::<3>().0.to_vec());
        }
    }
    Err("not a binary or ASCII STL file".into())
}

fn corners(t: &Tri) -> [DVec3; 3] {
    t.map(|v| DVec3::new(v[0] as f64, v[1] as f64, v[2] as f64))
}

/// Whether the mesh is a closed, consistently wound surface: every
/// directed edge has its reverse in another triangle. Only such a
/// mesh has a volume worth trusting for mass properties.
pub fn is_closed(tris: &[Tri]) -> bool {
    let q = |v: [f32; 3]| {
        (
            (v[0] * 1000.0).round() as i64,
            (v[1] * 1000.0).round() as i64,
            (v[2] * 1000.0).round() as i64,
        )
    };
    let mut edges: HashMap<(Point, Point), i32> = HashMap::new();
    for t in tris {
        let k = t.map(q);
        for i in 0..3 {
            let (a, b) = (k[i], k[(i + 1) % 3]);
            if a == b {
                return false;
            }
            *edges.entry((a, b)).or_insert(0) += 1;
            *edges.entry((b, a)).or_insert(0) -= 1;
        }
    }
    !tris.is_empty() && edges.values().all(|n| *n == 0)
}

/// The unit normal of a triangle, None when it is degenerate.
fn normal(t: &Tri) -> Option<DVec3> {
    let [a, b, c] = corners(t);
    let n = (b - a).cross(c - a);
    let l = n.length();
    if l > 1e-9 { Some(n / l) } else { None }
}

/// Volume, centroid and the inertia tensor about the centroid for unit
/// density, with the bounding box, by signed tetrahedra against the
/// origin. Positive volume means the winding faces outward.
pub struct Props {
    pub volume: f64,
    pub com: DVec3,
    pub inertia: DMat3,
    pub min: DVec3,
    pub max: DVec3,
}

pub fn props(tris: &[Tri]) -> Props {
    let mut vol = 0.0;
    let mut mc = DVec3::ZERO;
    let mut cov = [[0.0f64; 3]; 3];
    let mut mn = DVec3::splat(f64::INFINITY);
    let mut mx = DVec3::splat(f64::NEG_INFINITY);
    for t in tris {
        let [a, b, c] = corners(t);
        for v in [a, b, c] {
            mn = mn.min(v);
            mx = mx.max(v);
        }
        let det = a.dot(b.cross(c));
        vol += det / 6.0;
        let sm = a + b + c;
        mc += sm * (det / 24.0);
        for (i, row) in cov.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell += det * (sm[i] * sm[j] + a[i] * a[j] + b[i] * b[j] + c[i] * c[j]) / 120.0;
            }
        }
    }
    if tris.is_empty() {
        mn = DVec3::ZERO;
        mx = DVec3::ZERO;
    }
    let com = if vol.abs() > 1e-9 { mc / vol } else { (mn + mx) * 0.5 };
    let tr = cov[0][0] + cov[1][1] + cov[2][2];
    let i0 = assembly::mat_from_rows(&[
        [tr - cov[0][0], -cov[0][1], -cov[0][2]],
        [-cov[1][0], tr - cov[1][1], -cov[1][2]],
        [-cov[2][0], -cov[2][1], tr - cov[2][2]],
    ]);
    Props {
        volume: vol,
        com,
        inertia: i0 + assembly::shift_term(-vol, com),
        min: mn,
        max: mx,
    }
}

/// The bundle's mesh encoding: int16 positions (0.01 mm), int8 normals
/// smoothed across edges below a 30° crease, 16- or 32-bit indices.
pub fn pack(tris: &[Tri]) -> MeshRecord {
    let faces: Vec<(Tri, DVec3)> = tris.iter().filter_map(|t| normal(t).map(|n| (*t, n))).collect();
    let key = |p: [f32; 3]| {
        (
            (p[0] * 100.0).round() as i32,
            (p[1] * 100.0).round() as i32,
            (p[2] * 100.0).round() as i32,
        )
    };
    let mut groups: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
    for (f, (t, _)) in faces.iter().enumerate() {
        for c in t {
            groups.entry(key(*c)).or_default().push(f);
        }
    }
    let crease = 30f64.to_radians().cos();
    let mut uniq: HashMap<VertexKey, u32> = HashMap::new();
    let mut pos: Vec<i16> = Vec::new();
    let mut nrm: Vec<i8> = Vec::new();
    let mut idx: Vec<u32> = Vec::new();
    for (t, n) in &faces {
        for c in t {
            let k = key(*c);
            let mut acc = DVec3::ZERO;
            for &o in &groups[&k] {
                let on = faces[o].1;
                if n.dot(on) > crease {
                    acc += on;
                }
            }
            let nn = acc.normalize_or_zero();
            let q = [
                (nn.x * 127.0).round() as i8,
                (nn.y * 127.0).round() as i8,
                (nn.z * 127.0).round() as i8,
            ];
            let next = (pos.len() / 3) as u32;
            let vi = *uniq.entry((k, q)).or_insert_with(|| {
                pos.extend([k.0, k.1, k.2].map(|v| v.clamp(-32767, 32767) as i16));
                nrm.extend(q);
                next
            });
            idx.push(vi);
        }
    }
    let verts = pos.len() / 3;
    let idx32 = verts >= 65536;
    let b64 = base64::engine::general_purpose::STANDARD;
    let idx_bytes: Vec<u8> = if idx32 {
        idx.iter().flat_map(|v| v.to_le_bytes()).collect()
    } else {
        idx.iter().flat_map(|v| (*v as u16).to_le_bytes()).collect()
    };
    MeshRecord {
        verts,
        tris: faces.len(),
        pos: b64.encode(pos.iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>()),
        nrm: b64.encode(nrm.iter().map(|v| *v as u8).collect::<Vec<u8>>()),
        idx: b64.encode(idx_bytes),
        idx32,
    }
}

/// Round bores of the pin radius on a mesh: every wall face of a
/// 16-gon bore has its centroid one apothem from the axis along its
/// inward normal, so faces vote for axis positions; a real bore
/// collects votes from normals all the way round. Bores run along one
/// of the part's axes and are cut into 8 mm modules.
pub fn detect_bores(tris: &[Tri], radius: f64, min_votes: usize) -> Vec<Connector> {
    struct Face {
        n: DVec3,
        c: DVec3,
        lo: DVec3,
        hi: DVec3,
    }
    let faces: Vec<Face> = tris
        .iter()
        .filter_map(|t| {
            let n = normal(t)?;
            let [a, b, c] = corners(t);
            Some(Face {
                n,
                c: (a + b + c) / 3.0,
                lo: a.min(b).min(c),
                hi: a.max(b).max(c),
            })
        })
        .collect();
    let apothem = radius * (std::f64::consts::PI / 16.0).cos();
    let mut out = Vec::new();
    for ax in 0..3 {
        let oth = [(ax + 1) % 3, (ax + 2) % 3];
        let oth = if oth[0] < oth[1] { oth } else { [oth[1], oth[0]] };
        // face index, u, v, midpoint along the axis
        let mut bins: HashMap<(i64, i64), Vec<Vote>> = HashMap::new();
        for (fi, f) in faces.iter().enumerate() {
            if f.n[ax].abs() >= 0.08 {
                continue;
            }
            let pc = f.c + f.n * apothem;
            let (u, v) = (pc[oth[0]], pc[oth[1]]);
            bins.entry(((u / 0.5).round() as i64, (v / 0.5).round() as i64))
                .or_default()
                .push((fi, u, v, (f.lo[ax] + f.hi[ax]) / 2.0));
        }
        let mut keys: Vec<(i64, i64)> = bins.keys().copied().collect();
        keys.sort();
        for k in keys {
            let mut list = bins.remove(&k).unwrap();
            if list.len() < min_votes {
                continue;
            }
            list.sort_by(|p, q| p.3.partial_cmp(&q.3).unwrap());
            let mut start = 0;
            for i in 1..=list.len() {
                if i < list.len() && list[i].3 - list[i - 1].3 <= 7.5 {
                    continue;
                }
                let grp = &list[start..i];
                start = i;
                if grp.len() < min_votes {
                    continue;
                }
                let mut sectors = [false; 16];
                for p in grp {
                    let n = &faces[p.0].n;
                    let a = n[oth[1]].atan2(n[oth[0]]);
                    sectors[(((a + std::f64::consts::PI) / std::f64::consts::TAU * 16.0).floor() as usize) % 16] = true;
                }
                if sectors.iter().filter(|s| **s).count() < 10 {
                    continue; // a wall or a fillet, not a bore
                }
                let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
                let (mut su, mut sv) = (0.0, 0.0);
                for p in grp {
                    lo = lo.min(faces[p.0].lo[ax]);
                    hi = hi.max(faces[p.0].hi[ax]);
                    su += p.1;
                    sv += p.2;
                }
                if hi - lo < 3.0 {
                    continue;
                }
                if (5.0..=8.5).contains(&(hi - lo)) {
                    // chamfered rings sit inside the module
                    let mid = (lo + hi) / 2.0;
                    lo = mid - 4.0;
                    hi = mid + 4.0;
                }
                let (cu, cv) = (su / grp.len() as f64, sv / grp.len() as f64);
                let len = hi - lo;
                let segs = if len > 8.5 { (len / 8.0).round().max(1.0) as usize } else { 1 };
                let mut axis = [0.0; 3];
                axis[ax] = 1.0;
                for s in 0..segs {
                    let mut centre = [0.0; 3];
                    centre[oth[0]] = cu;
                    centre[oth[1]] = cv;
                    centre[ax] = lo + len * (s as f64 + 0.5) / segs as f64;
                    out.push(Connector {
                        kind: "pin_hole".into(),
                        centre: centre.map(assembly::round3),
                        axis,
                        length: assembly::round3(len / segs as f64),
                        r: radius,
                    });
                }
            }
        }
    }
    out
}

/// What an imported part records beside its name and mass: the same
/// fields as a bundle part.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Record {
    pub mesh: MeshRecord,
    pub bbox: [[f64; 3]; 2],
    pub volume_mm3: f64,
    pub com: [f64; 3],
    pub inertia_per_g: [[f64; 3]; 3],
    pub mass_model: String,
    pub connectors: Vec<Connector>,
}

/// The record of a mesh in mm: inside-out files are flipped; open or
/// inconsistently wound meshes get a box's inertia and no volume.
pub fn record(mut tris: Vec<Tri>) -> Record {
    let mut pr = props(&tris);
    if pr.volume < 0.0 {
        for t in tris.iter_mut() {
            t.swap(1, 2);
        }
        pr = props(&tris);
    }
    let closed = pr.volume > 1.0 && is_closed(&tris);
    let size = pr.max - pr.min;
    let rows = |m: DMat3| {
        [
            [m.col(0)[0], m.col(1)[0], m.col(2)[0]],
            [m.col(0)[1], m.col(1)[1], m.col(2)[1]],
            [m.col(0)[2], m.col(1)[2], m.col(2)[2]],
        ]
    };
    let ipg = if closed {
        rows(pr.inertia * (1.0 / pr.volume))
    } else {
        [
            [(size.y * size.y + size.z * size.z) / 12.0, 0.0, 0.0],
            [0.0, (size.x * size.x + size.z * size.z) / 12.0, 0.0],
            [0.0, 0.0, (size.x * size.x + size.y * size.y) / 12.0],
        ]
    };
    let com = if closed { pr.com } else { (pr.min + pr.max) * 0.5 };
    Record {
        mesh: pack(&tris),
        bbox: [pr.min.to_array().map(assembly::round3), pr.max.to_array().map(assembly::round3)],
        volume_mm3: if closed { (pr.volume * 100.0).round() / 100.0 } else { 0.0 },
        com: com.to_array().map(assembly::round3),
        inertia_per_g: ipg.map(|r| r.map(|v| (v * 10000.0).round() / 10000.0)),
        mass_model: if closed { "mesh" } else { "box" }.into(),
        connectors: detect_bores(&tris, BORE_RADIUS_MM, BORE_MIN_VOTES),
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Units {
    Mm,
    Cm,
    Inch,
    M,
}

impl Units {
    pub const ALL: [Units; 4] = [Units::Mm, Units::Cm, Units::Inch, Units::M];
    pub fn scale(self) -> f32 {
        match self {
            Units::Mm => 1.0,
            Units::Cm => 10.0,
            Units::Inch => 25.4,
            Units::M => 1000.0,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Units::Mm => "mm",
            Units::Cm => "cm",
            Units::Inch => "inch",
            Units::M => "m",
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Origin {
    File,
    Centre,
    Bottom,
}

impl Origin {
    pub const ALL: [Origin; 3] = [Origin::File, Origin::Centre, Origin::Bottom];
    pub fn label(self) -> &'static str {
        match self {
            Origin::File => "as in the file",
            Origin::Centre => "bounding-box centre",
            Origin::Bottom => "bottom centre",
        }
    }
}

/// A file being imported and the choices that turn it into a part.
pub struct Import {
    pub file: String,
    pub raw: Vec<Tri>,
    pub name: String,
    pub category: String,
    pub units: Units,
    pub origin: Origin,
    /// A weighed mass; 0 means "use the density".
    pub mass_g: f64,
    pub density: f64,
}

/// The import worked out for the current choices.
pub struct Prepared {
    pub record: Record,
    pub mass_g: Option<f64>,
    pub measured: bool,
    pub summary: String,
}

impl Import {
    pub fn new(file: &str, raw: Vec<Tri>) -> Result<Import, String> {
        if raw.is_empty() {
            return Err("the file has no triangles".into());
        }
        if raw.len() > MAX_TRIS {
            return Err(format!("that is {} triangles; decimate it below {MAX_TRIS} first", raw.len()));
        }
        let name = Path::new(file)
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| file.to_string());
        Ok(Import {
            file: file.to_string(),
            raw,
            name,
            category: "other".into(),
            units: Units::Mm,
            origin: Origin::File,
            mass_g: 0.0,
            density: DENSITIES[0].1,
        })
    }

    pub fn from_path(path: &Path) -> Result<Import, String> {
        let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
        let raw = parse(&bytes)?;
        let file = path.file_name().map(|f| f.to_string_lossy().to_string()).unwrap_or_default();
        Self::new(&file, raw)
    }

    /// Scale, shift, measure and pack the mesh for the current choices.
    pub fn prepare(&self) -> Prepared {
        let scale = self.units.scale();
        let mut tris: Vec<Tri> = self.raw.iter().map(|t| t.map(|c| c.map(|v| v * scale))).collect();
        let pr0 = props(&tris);
        let shift = match self.origin {
            Origin::File => DVec3::ZERO,
            Origin::Centre => -(pr0.min + pr0.max) * 0.5,
            Origin::Bottom => DVec3::new(-(pr0.min.x + pr0.max.x) / 2.0, -(pr0.min.y + pr0.max.y) / 2.0, -pr0.min.z),
        };
        if shift != DVec3::ZERO {
            let s = shift.as_vec3();
            for t in tris.iter_mut() {
                for c in t.iter_mut() {
                    c[0] += s.x;
                    c[1] += s.y;
                    c[2] += s.z;
                }
            }
        }
        let record = record(tris);
        let measured = self.mass_g > 0.0;
        let mass_g = if measured {
            Some(self.mass_g)
        } else if record.mass_model == "mesh" {
            Some(record.volume_mm3 / 1000.0 * self.density)
        } else {
            None
        };
        let size: Vec<String> = (0..3).map(|k| assembly::fmt(record.bbox[1][k] - record.bbox[0][k])).collect();
        let holes = record.connectors.len();
        let summary = format!(
            "{} triangles · {} mm · {} · {}{}",
            record.mesh.tris,
            size.join(" × "),
            if record.mass_model == "mesh" {
                format!("volume {:.2} cm³", record.volume_mm3 / 1000.0)
            } else {
                "open mesh, box inertia".to_string()
            },
            match mass_g {
                Some(m) => format!(
                    "mass {:.2} g{}",
                    m,
                    if measured {
                        String::new()
                    } else {
                        format!(" at {} g/cm³", self.density)
                    }
                ),
                None => "give a mass".to_string(),
            },
            match holes {
                0 => String::new(),
                1 => " · 1 pin hole found".to_string(),
                n => format!(" · {n} pin holes found"),
            }
        );
        Prepared {
            record,
            mass_g,
            measured,
            summary,
        }
    }

    /// The part to record, once a mass is known.
    pub fn part(&self, p: &Prepared) -> Option<Part> {
        let mass = p.mass_g?;
        let serde_json::Value::Object(extra) = serde_json::to_value(&p.record).ok()? else {
            return None;
        };
        let name = self.name.trim();
        Some(Part {
            name: if name.is_empty() { self.file.clone() } else { name.to_string() },
            category: self.category.clone(),
            mass_g: assembly::round3(mass),
            source: if p.measured { "measured" } else { "placeholder" }.into(),
            source_note: format!(
                "imported from {}{}",
                self.file,
                if p.measured {
                    String::new()
                } else {
                    format!("; mass from the mesh volume at {} g/cm³: weigh it", self.density)
                }
            ),
            ldraw: None,
            shapes: vec![],
            extra,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle::{Bundle, MeshData};
    use crate::geometry;
    use std::f32::consts::PI;

    fn tris_of(m: &MeshData) -> Vec<Tri> {
        m.indices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|t| [m.positions[t[0] as usize], m.positions[t[1] as usize], m.positions[t[2] as usize]])
            .collect()
    }

    fn binary_stl(tris: &[Tri]) -> Vec<u8> {
        let mut out = vec![0u8; 80];
        out.extend((tris.len() as u32).to_le_bytes());
        for t in tris {
            out.extend([0u8; 12]);
            for c in t {
                for v in c {
                    out.extend(v.to_le_bytes());
                }
            }
            out.extend([0u8; 2]);
        }
        out
    }

    fn ascii_stl(tris: &[Tri]) -> String {
        let mut s = String::from("solid test\n");
        for t in tris {
            s.push_str("  facet normal 0 0 0\n    outer loop\n");
            for c in t {
                s.push_str(&format!("      vertex {} {} {}\n", c[0], c[1], c[2]));
            }
            s.push_str("    endloop\n  endfacet\n");
        }
        s.push_str("endsolid test\n");
        s
    }

    /// A 16-gon tube wall of `radius` along an axis, from `z0` to `z1`,
    /// wound so its normals point inward: the wall of a pin hole.
    fn bore(radius: f32, axis: usize, z0: f32, z1: f32, sides: usize, centre: [f32; 3]) -> Vec<Tri> {
        let mut out = Vec::new();
        let place = |r: f32, a: f32, z: f32| -> [f32; 3] {
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            let mut p = centre;
            p[u] += r * a.cos();
            p[v] += r * a.sin();
            p[axis] += z;
            p
        };
        for i in 0..sides {
            let a0 = i as f32 / 16.0 * 2.0 * PI;
            let a1 = (i + 1) as f32 / 16.0 * 2.0 * PI;
            let (p0, p1, p2, p3) = (
                place(radius, a0, z0),
                place(radius, a1, z0),
                place(radius, a1, z1),
                place(radius, a0, z1),
            );
            out.push([p0, p2, p1]);
            out.push([p0, p3, p2]);
        }
        out
    }

    #[test]
    fn parses_binary_and_ascii_and_refuses_the_rest() {
        let tris = tris_of(&geometry::box_mesh([10.0, 20.0, 30.0], [1.0, 2.0, 3.0]));
        let b = parse(&binary_stl(&tris)).unwrap();
        assert_eq!(b.len(), 12);
        assert_eq!(b, tris);
        let a = parse(ascii_stl(&tris).as_bytes()).unwrap();
        assert_eq!(a, tris);
        assert!(parse(b"garbage").is_err());
        assert!(parse(b"solid x\n vertex 1 2\nendsolid").is_err());
        assert!(parse(b"solid empty\nendsolid empty\n").is_err());
        let mut short = binary_stl(&tris);
        short.truncate(100);
        assert!(parse(&short).is_err(), "a binary file with the wrong length is refused");
    }

    #[test]
    fn props_of_a_box_are_exact() {
        let tris = tris_of(&geometry::box_mesh([40.0, 20.0, 10.0], [5.0, -3.0, 2.0]));
        let p = props(&tris);
        assert!((p.volume - 8000.0).abs() < 1e-6);
        assert!((p.com - DVec3::new(5.0, -3.0, 2.0)).length() < 1e-9);
        assert!((p.inertia.col(0)[0] - 8000.0 * (400.0 + 100.0) / 12.0).abs() < 1e-3);
        assert!((p.inertia.col(1)[1] - 8000.0 * (1600.0 + 100.0) / 12.0).abs() < 1e-3);
        assert!((p.inertia.col(2)[2] - 8000.0 * (1600.0 + 400.0) / 12.0).abs() < 1e-3);
        assert!(p.inertia.col(0)[1].abs() < 1e-6);
        assert_eq!((p.min, p.max), (DVec3::new(-15.0, -13.0, -3.0), DVec3::new(25.0, 7.0, 7.0)));
        let e = props(&[]);
        assert_eq!((e.volume, e.com), (0.0, DVec3::ZERO));
    }

    #[test]
    fn inside_out_files_are_flipped_and_open_meshes_get_a_box() {
        let mut tris = tris_of(&geometry::box_mesh([10.0, 10.0, 10.0], [0.0; 3]));
        for t in tris.iter_mut() {
            t.swap(1, 2);
        }
        assert!(props(&tris).volume < 0.0);
        let r = record(tris.clone());
        assert_eq!(r.mass_model, "mesh");
        assert_eq!(r.volume_mm3, 1000.0);
        assert_eq!(r.com, [0.0; 3]);
        assert!((r.inertia_per_g[0][0] - 200.0 / 12.0).abs() < 1e-3);
        assert!(is_closed(&tris));
        let mut torn = tris.clone();
        torn.pop();
        assert!(!is_closed(&torn), "a missing face opens the mesh");
        let mut twisted = tris.clone();
        twisted[0].swap(1, 2);
        assert!(!is_closed(&twisted), "one face wound the other way");
        assert!(!is_closed(&[]));
        assert_eq!(record(twisted).mass_model, "box");
        let open = record(vec![tris[0]]);
        assert_eq!(open.mass_model, "box");
        assert_eq!(open.volume_mm3, 0.0);
        assert_eq!(
            open.com,
            [
                (open.bbox[0][0] + open.bbox[1][0]) / 2.0,
                (open.bbox[0][1] + open.bbox[1][1]) / 2.0,
                (open.bbox[0][2] + open.bbox[1][2]) / 2.0
            ]
        );
        assert_eq!(open.inertia_per_g[0][1], 0.0);
        assert!(open.connectors.is_empty());
    }

    #[test]
    fn packed_meshes_decode_like_the_bundle() {
        let tris = tris_of(&geometry::box_mesh([10.0, 20.0, 30.0], [0.0; 3]));
        let rec = pack(&tris);
        assert_eq!((rec.verts, rec.tris, rec.idx32), (24, 12, false));
        let m = rec.decode().unwrap();
        assert_eq!(m.indices.len(), 36);
        for (i, p) in m.positions.iter().enumerate() {
            assert!(p.iter().all(|v| v.abs() == 5.0 || v.abs() == 10.0 || v.abs() == 15.0), "{i}: {p:?}");
            let n = m.normals[i];
            assert!((n.iter().map(|v| v * v).sum::<f32>() - 1.0).abs() < 0.02, "unit normal: {n:?}");
        }
        assert_eq!(crate::geometry::signed_volume(&m).round(), 6000.0);
        let degenerate = [[[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]];
        assert_eq!(pack(&degenerate).tris, 0);
    }

    #[test]
    fn finds_pin_holes_along_every_axis_and_cuts_them_into_modules() {
        let one = detect_bores(&bore(2.4, 2, -4.0, 4.0, 16, [10.0, 20.0, 30.0]), 2.4, 12);
        assert_eq!(one.len(), 1, "{one:?}");
        assert_eq!(one[0].kind, "pin_hole");
        assert_eq!(one[0].axis, [0.0, 0.0, 1.0]);
        assert_eq!(one[0].length, 8.0);
        assert!(
            (one[0].centre[0] - 10.0).abs() < 0.1 && (one[0].centre[1] - 20.0).abs() < 0.1 && (one[0].centre[2] - 30.0).abs() < 1e-6,
            "{:?}",
            one[0].centre
        );
        let three = detect_bores(&bore(2.4, 0, 0.0, 24.0, 16, [0.0; 3]), 2.4, 12);
        assert_eq!(three.len(), 3, "{three:?}");
        assert!(three.iter().all(|c| c.axis == [1.0, 0.0, 0.0] && c.length == 8.0));
        let mut xs: Vec<f64> = three.iter().map(|c| c.centre[0]).collect();
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(xs, vec![4.0, 12.0, 20.0]);
        assert_eq!(
            detect_bores(&bore(2.4, 1, 0.0, 7.0, 16, [0.0; 3]), 2.4, 12)[0].length,
            8.0,
            "a chamfered ring fills its module"
        );
        assert!(
            detect_bores(&bore(2.4, 2, 0.0, 8.0, 7, [0.0; 3]), 2.4, 12).is_empty(),
            "half a bore is a fillet"
        );
        assert!(detect_bores(&bore(2.4, 2, 0.0, 2.0, 16, [0.0; 3]), 2.4, 12).is_empty(), "too short");
        assert!(
            detect_bores(&bore(6.0, 2, 0.0, 8.0, 16, [0.0; 3]), 2.4, 12).is_empty(),
            "the wrong radius votes nowhere"
        );
    }

    #[test]
    fn import_scales_shifts_weighs_and_records() {
        let tris = tris_of(&geometry::box_mesh([1.0, 2.0, 0.5], [0.0, 0.0, 0.25]));
        assert!(Import::new("x.stl", vec![]).is_err());
        let mut imp = Import::new("bracket.stl", tris).unwrap();
        assert_eq!(imp.name, "bracket");
        assert_eq!(
            (imp.units, imp.origin, imp.mass_g, imp.density),
            (Units::Mm, Origin::File, 0.0, 1.24)
        );
        imp.units = Units::Inch;
        let p = imp.prepare();
        assert_eq!(p.record.bbox[1][2], 12.7);
        assert!((p.record.volume_mm3 - 25.4f64.powi(3)).abs() < 1.0);
        assert_eq!(
            p.mass_g.map(|m| (m * 100.0).round() / 100.0),
            Some(((25.4f64.powi(3) / 1000.0) * 1.24 * 100.0).round() / 100.0)
        );
        assert!(!p.measured);
        assert!(
            p.summary.contains("12 triangles") && p.summary.contains("at 1.24 g/cm³"),
            "{}",
            p.summary
        );
        imp.units = Units::Cm;
        imp.origin = Origin::Centre;
        let p = imp.prepare();
        assert_eq!(p.record.bbox, [[-5.0, -10.0, -2.5], [5.0, 10.0, 2.5]]);
        imp.origin = Origin::Bottom;
        let p = imp.prepare();
        assert_eq!(p.record.bbox, [[-5.0, -10.0, 0.0], [5.0, 10.0, 5.0]]);
        imp.mass_g = 42.0;
        imp.name = "  ".into();
        imp.category = "electronics".into();
        let p = imp.prepare();
        assert!(p.measured && p.mass_g == Some(42.0));
        assert!(p.summary.contains("mass 42.00 g") && !p.summary.contains("at "), "{}", p.summary);
        let part = imp.part(&p).unwrap();
        assert_eq!(
            (part.name.as_str(), part.category.as_str(), part.mass_g, part.source.as_str()),
            ("bracket.stl", "electronics", 42.0, "measured")
        );
        assert_eq!(part.source_note, "imported from bracket.stl");
        match assembly::geometry_of(
            &part,
            &Bundle {
                format: String::new(),
                source: String::new(),
                parts: Default::default(),
                missing: vec![],
            },
        ) {
            assembly::Geometry::Imported { volume_mm3, tris, .. } => {
                assert_eq!(volume_mm3, 1000.0);
                assert_eq!(tris, 12);
            }
            _ => panic!("the part should carry an imported mesh"),
        }
        let pr = assembly::part_props(
            &part,
            &Bundle {
                format: String::new(),
                source: String::new(),
                parts: Default::default(),
                missing: vec![],
            },
        );
        assert_eq!(pr.mass, 42.0);
        assert!((pr.com.z - 2.5).abs() < 1e-9);
        // a mesh with no volume and no mass cannot become a part
        let open = Import::new("sheet.stl", vec![imp.raw[0]]).unwrap();
        let p = open.prepare();
        assert!(p.mass_g.is_none() && p.summary.contains("give a mass"), "{}", p.summary);
        assert!(open.part(&p).is_none());
        assert_eq!(Units::ALL.map(|u| u.label()), ["mm", "cm", "inch", "m"]);
        assert_eq!(
            Origin::ALL.map(|o| o.label()),
            ["as in the file", "bounding-box centre", "bottom centre"]
        );
    }

    #[test]
    fn reads_a_file_from_disk() {
        let dir = std::env::temp_dir().join(format!("ob-stl-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let tris = tris_of(&geometry::box_mesh([8.0, 8.0, 8.0], [0.0; 3]));
        std::fs::write(dir.join("cube.stl"), binary_stl(&tris)).unwrap();
        let imp = Import::from_path(&dir.join("cube.stl")).unwrap();
        assert_eq!((imp.file.as_str(), imp.name.as_str(), imp.raw.len()), ("cube.stl", "cube", 12));
        assert!(Import::from_path(&dir.join("missing.stl")).is_err());
        std::fs::write(dir.join("bad.stl"), b"nope").unwrap();
        assert!(Import::from_path(&dir.join("bad.stl")).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
