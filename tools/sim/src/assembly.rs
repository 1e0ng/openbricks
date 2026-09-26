//! The assembly document (`openbricks-assembly/1`) and its maths:
//! bricks recorded once, components made of bricks and components,
//! the robot as the top component. Mass, centre of mass, inertia and
//! extents are computed at every level, never stored.
//!
//! Frames: X forward, Y left, Z up, millimetres and grams. An instance
//! pose is a position plus `[roll, pitch, yaw]` in degrees with
//! `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.

use crate::bundle::{Bundle, Connector, PartRecord};
use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};

pub const FORMAT: &str = "openbricks-assembly/1";
pub const ROLES: [(&str, &str); 8] = [
    ("wheel_left", "left drive wheel (a cylinder)"),
    ("wheel_right", "right drive wheel"),
    ("caster", "the free support"),
    ("line_sensor", "first QTR array → chassis_line"),
    ("line_sensor_2", "second array → chassis_line2"),
    ("color_sensor", "TCS34725 → chassis_cam_down"),
    ("distance_sensor", "range sensor → chassis_dist"),
    ("imu", "gyro → chassis_imu"),
];

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Document {
    pub format: String,
    #[serde(default)]
    pub units: serde_json::Value,
    pub parts: BTreeMap<String, Part>,
    pub components: BTreeMap<String, Component>,
    pub robot: Robot,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Part {
    pub name: String,
    #[serde(default)]
    pub category: String,
    pub mass_g: f64,
    #[serde(default)]
    pub source: String,
    #[serde(default)]
    pub source_note: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ldraw: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub shapes: Vec<Shape>,
    /// Everything else (imported-mesh records, frames) rides along untouched.
    #[serde(flatten)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Shape {
    Box {
        size: [f64; 3],
        #[serde(default)]
        pos: [f64; 3],
    },
    Cylinder {
        radius: f64,
        length: f64,
        #[serde(default = "default_axis")]
        axis: String,
        #[serde(default)]
        pos: [f64; 3],
    },
    Sphere {
        radius: f64,
        #[serde(default)]
        pos: [f64; 3],
    },
}

fn default_axis() -> String {
    "z".into()
}

impl Shape {
    pub fn pos(&self) -> DVec3 {
        match self {
            Shape::Box { pos, .. } | Shape::Cylinder { pos, .. } | Shape::Sphere { pos, .. } => DVec3::from_array(*pos),
        }
    }
    pub fn volume(&self) -> f64 {
        match self {
            Shape::Box { size, .. } => size[0] * size[1] * size[2],
            Shape::Cylinder { radius, length, .. } => std::f64::consts::PI * radius * radius * length,
            Shape::Sphere { radius, .. } => 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3),
        }
    }
    pub fn half_extents(&self) -> DVec3 {
        match self {
            Shape::Box { size, .. } => DVec3::new(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0),
            Shape::Cylinder { radius, length, axis, .. } => match axis.as_str() {
                "x" => DVec3::new(length / 2.0, *radius, *radius),
                "y" => DVec3::new(*radius, length / 2.0, *radius),
                _ => DVec3::new(*radius, *radius, length / 2.0),
            },
            Shape::Sphere { radius, .. } => DVec3::splat(*radius),
        }
    }
    /// Inertia about the shape's own centre for mass `m`, in its own frame.
    pub fn inertia(&self, m: f64) -> DMat3 {
        match self {
            Shape::Box { size, .. } => {
                let (a, b, c) = (size[0], size[1], size[2]);
                DMat3::from_diagonal(DVec3::new(
                    m / 12.0 * (b * b + c * c),
                    m / 12.0 * (a * a + c * c),
                    m / 12.0 * (a * a + b * b),
                ))
            }
            Shape::Cylinder { radius, length, axis, .. } => {
                let ax = m * radius * radius / 2.0;
                let tr = m / 12.0 * (3.0 * radius * radius + length * length);
                match axis.as_str() {
                    "x" => DMat3::from_diagonal(DVec3::new(ax, tr, tr)),
                    "y" => DMat3::from_diagonal(DVec3::new(tr, ax, tr)),
                    _ => DMat3::from_diagonal(DVec3::new(tr, tr, ax)),
                }
            }
            Shape::Sphere { radius, .. } => DMat3::from_diagonal(DVec3::splat(0.4 * m * radius * radius)),
        }
    }
    pub fn text(&self) -> String {
        match self {
            Shape::Box { size, .. } => format!("box {} × {} × {} mm", fmt(size[0]), fmt(size[1]), fmt(size[2])),
            Shape::Cylinder { radius, length, axis, .. } => format!("cylinder ⌀{} × {} mm along {axis}", fmt(2.0 * radius), fmt(*length)),
            Shape::Sphere { radius, .. } => format!("sphere ⌀{} mm", fmt(2.0 * radius)),
        }
    }
}

pub fn fmt(v: f64) -> String {
    let r = (v * 100.0).round() / 100.0;
    if r == r.trunc() { format!("{}", r as i64) } else { format!("{r}") }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Instance {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub part: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub component: Option<String>,
    pub pos: [f64; 3],
    #[serde(default)]
    pub rot: [f64; 3],
    /// A locked instance stays where it is: the pointer, the keys and
    /// the inspector cannot move, turn or remove it until it is unlocked.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub locked: bool,
    /// The LEGO colour this brick is placed in: an LDraw colour id the
    /// library's palette names (a part comes in a known set of them, each
    /// its own LEGO element number). Unset, it draws in its category's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<u32>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Component {
    #[serde(default)]
    pub note: String,
    #[serde(default)]
    pub children: Vec<Instance>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Robot {
    #[serde(default)]
    pub name: String,
    pub root: String,
    #[serde(default)]
    pub roles: BTreeMap<String, String>,
    #[serde(default)]
    pub spawn: Spawn,
    #[serde(default)]
    pub measured_mass_g: Option<f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Spawn {
    #[serde(default)]
    pub pos_mm: [f64; 2],
    #[serde(default)]
    pub yaw_deg: f64,
}

// ---------------------------------------------------------------- maths

pub fn rot_mat(rpy: [f64; 3]) -> DMat3 {
    let [r, p, y] = rpy.map(f64::to_radians);
    let (sr, cr) = r.sin_cos();
    let (sp, cp) = p.sin_cos();
    let (sy, cy) = y.sin_cos();
    DMat3::from_cols(
        DVec3::new(cy * cp, sy * cp, -sp),
        DVec3::new(cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
        DVec3::new(cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr),
    )
}

/// `[roll, pitch, yaw]` in degrees from a rotation matrix (rounded to 0.01°).
pub fn euler_from(m: &DMat3) -> [f64; 3] {
    let r = |row: usize, col: usize| m.col(col)[row];
    let sp = -r(2, 0);
    let (roll, pitch, yaw) = if sp.abs() > 0.999_999 {
        (0.0, sp.clamp(-1.0, 1.0).asin(), (-r(0, 1)).atan2(r(1, 1)))
    } else {
        (r(2, 1).atan2(r(2, 2)), sp.asin(), r(1, 0).atan2(r(0, 0)))
    };
    [roll, pitch, yaw].map(|a| {
        let d = (a.to_degrees() * 100.0).round() / 100.0;
        if d == 0.0 { 0.0 } else { d }
    })
}

pub fn shift_term(m: f64, d: DVec3) -> DMat3 {
    let dd = d.dot(d);
    DMat3::from_cols(
        DVec3::new(m * (dd - d.x * d.x), -m * d.y * d.x, -m * d.z * d.x),
        DVec3::new(-m * d.x * d.y, m * (dd - d.y * d.y), -m * d.z * d.y),
        DVec3::new(-m * d.x * d.z, -m * d.y * d.z, m * (dd - d.z * d.z)),
    )
}

#[derive(Clone, Debug, PartialEq)]
pub struct Bbox {
    pub min: DVec3,
    pub max: DVec3,
}

impl Bbox {
    fn union(a: Option<Bbox>, b: Option<Bbox>) -> Option<Bbox> {
        match (a, b) {
            (None, x) | (x, None) => x,
            (Some(a), Some(b)) => Some(Bbox {
                min: a.min.min(b.min),
                max: a.max.max(b.max),
            }),
        }
    }
    fn transformed(&self, p: DVec3, r: &DMat3) -> Bbox {
        let mut out: Option<Bbox> = None;
        for i in 0..8 {
            let c = DVec3::new(
                if i & 1 != 0 { self.max.x } else { self.min.x },
                if i & 2 != 0 { self.max.y } else { self.min.y },
                if i & 4 != 0 { self.max.z } else { self.min.z },
            );
            let w = p + *r * c;
            out = Bbox::union(out, Some(Bbox { min: w, max: w }));
        }
        out.unwrap()
    }
    pub fn size(&self) -> DVec3 {
        self.max - self.min
    }
}

#[derive(Clone, Debug)]
pub struct Props {
    pub mass: f64,
    pub com: DVec3,
    pub inertia: DMat3,
    pub bbox: Option<Bbox>,
    pub count: usize,
}

impl Props {
    fn empty() -> Props {
        Props {
            mass: 0.0,
            com: DVec3::ZERO,
            inertia: DMat3::ZERO,
            bbox: None,
            count: 0,
        }
    }
}

/// Where a part's geometry comes from: the bundle (LDraw), its own
/// imported mesh record, or recorded shapes.
pub enum Geometry<'a> {
    Record(&'a PartRecord),
    Imported {
        bbox: Bbox,
        com: DVec3,
        inertia_per_g: DMat3,
        connectors: Vec<Connector>,
        volume_mm3: f64,
        tris: usize,
    },
    Shapes,
    None,
}

pub fn geometry_of<'a>(part: &'a Part, bundle: &'a Bundle) -> Geometry<'a> {
    if let Some(num) = &part.ldraw {
        if let Some(rec) = bundle.parts.get(num) {
            return Geometry::Record(rec);
        }
        // a part fetched by number carries its record in the build, for a library without it
        if !part.extra.contains_key("mesh") {
            return Geometry::None;
        }
    }
    if part.extra.contains_key("mesh") {
        let v = |k: &str| part.extra.get(k).cloned().unwrap_or(serde_json::Value::Null);
        let bbox: Option<[[f64; 3]; 2]> = serde_json::from_value(v("bbox")).ok();
        let com: Option<[f64; 3]> = serde_json::from_value(v("com")).ok();
        let ipg: Option<[[f64; 3]; 3]> = serde_json::from_value(v("inertia_per_g")).ok();
        if let (Some(bb), Some(c), Some(i)) = (bbox, com, ipg) {
            let connectors: Vec<Connector> = serde_json::from_value(v("connectors")).unwrap_or_default();
            let tris = v("mesh").get("tris").and_then(|t| t.as_u64()).unwrap_or(0) as usize;
            return Geometry::Imported {
                bbox: Bbox {
                    min: DVec3::from_array(bb[0]),
                    max: DVec3::from_array(bb[1]),
                },
                com: DVec3::from_array(c),
                inertia_per_g: mat_from_rows(&i),
                connectors,
                volume_mm3: v("volume_mm3").as_f64().unwrap_or(0.0),
                tris,
            };
        }
    }
    if part.shapes.is_empty() { Geometry::None } else { Geometry::Shapes }
}

pub fn mat_from_rows(rows: &[[f64; 3]; 3]) -> DMat3 {
    DMat3::from_cols(
        DVec3::new(rows[0][0], rows[1][0], rows[2][0]),
        DVec3::new(rows[0][1], rows[1][1], rows[2][1]),
        DVec3::new(rows[0][2], rows[1][2], rows[2][2]),
    )
}

pub fn part_props(part: &Part, bundle: &Bundle) -> Props {
    match geometry_of(part, bundle) {
        Geometry::Record(rec) => Props {
            mass: part.mass_g,
            com: DVec3::from_array(rec.com),
            inertia: mat_from_rows(&rec.inertia_per_g) * part.mass_g,
            bbox: Some(Bbox {
                min: DVec3::from_array(rec.bbox[0]),
                max: DVec3::from_array(rec.bbox[1]),
            }),
            count: 1,
        },
        Geometry::Imported {
            bbox, com, inertia_per_g, ..
        } => Props {
            mass: part.mass_g,
            com,
            inertia: inertia_per_g * part.mass_g,
            bbox: Some(bbox),
            count: 1,
        },
        Geometry::Shapes | Geometry::None => {
            let shapes = &part.shapes;
            let vols: Vec<f64> = shapes.iter().map(Shape::volume).collect();
            let vt: f64 = vols.iter().sum();
            let masses: Vec<f64> = shapes
                .iter()
                .enumerate()
                .map(|(i, _)| {
                    if vt > 0.0 {
                        part.mass_g * vols[i] / vt
                    } else {
                        part.mass_g / shapes.len().max(1) as f64
                    }
                })
                .collect();
            let mut mc = DVec3::ZERO;
            let mut bbox = None;
            for (s, m) in shapes.iter().zip(&masses) {
                mc += s.pos() * *m;
                let h = s.half_extents();
                bbox = Bbox::union(
                    bbox,
                    Some(Bbox {
                        min: s.pos() - h,
                        max: s.pos() + h,
                    }),
                );
            }
            let com = if part.mass_g > 0.0 && !shapes.is_empty() {
                mc / part.mass_g
            } else {
                DVec3::ZERO
            };
            let mut inertia = DMat3::ZERO;
            for (s, m) in shapes.iter().zip(&masses) {
                inertia += s.inertia(*m) + shift_term(*m, s.pos() - com);
            }
            Props {
                mass: part.mass_g,
                com,
                inertia,
                bbox,
                count: 1,
            }
        }
    }
}

/// Mass properties of a component, rolled up from its children with
/// the parallel-axis theorem. Memoised per component id.
pub fn component_props(
    doc: &Document,
    bundle: &Bundle,
    id: &str,
    memo: &mut HashMap<String, Props>,
    stack: &mut Vec<String>,
    errors: &mut Vec<String>,
) -> Props {
    if let Some(p) = memo.get(id) {
        return p.clone();
    }
    let Some(comp) = doc.components.get(id) else {
        errors.push(format!("component '{id}' does not exist"));
        return Props::empty();
    };
    if stack.iter().any(|s| s == id) {
        errors.push(format!("component '{id}' contains itself"));
        return Props::empty();
    }
    stack.push(id.to_string());
    let mut mass = 0.0;
    let mut mc = DVec3::ZERO;
    let mut count = 0;
    let mut bbox = None;
    let mut placed: Vec<(f64, DVec3, DMat3)> = Vec::new();
    for ch in &comp.children {
        let props = if let Some(pid) = &ch.part {
            match doc.parts.get(pid) {
                Some(part) => part_props(part, bundle),
                None => {
                    errors.push(format!("'{}' refers to a brick '{pid}' that is not in the library", ch.name));
                    continue;
                }
            }
        } else if let Some(cid) = &ch.component {
            component_props(doc, bundle, cid, memo, stack, errors)
        } else {
            continue;
        };
        let r = rot_mat(ch.rot);
        let p = DVec3::from_array(ch.pos);
        let c = p + r * props.com;
        placed.push((props.mass, c, r * props.inertia * r.transpose()));
        mass += props.mass;
        mc += c * props.mass;
        count += props.count;
        if let Some(b) = &props.bbox {
            bbox = Bbox::union(bbox, Some(b.transformed(p, &r)));
        }
    }
    stack.pop();
    let com = if mass > 0.0 { mc / mass } else { DVec3::ZERO };
    let mut inertia = DMat3::ZERO;
    for (m, c, i) in placed {
        inertia += i + shift_term(m, c - com);
    }
    let out = Props {
        mass,
        com,
        inertia,
        bbox,
        count,
    };
    memo.insert(id.to_string(), out.clone());
    out
}

/// One brick instance placed in the frame of the component being
/// viewed, with the path of instance names that leads to it.
#[derive(Clone, Debug)]
pub struct Leaf {
    pub path: Vec<String>,
    pub part_id: String,
    pub pos: DVec3,
    pub rot: DMat3,
    /// The brick's LEGO colour id, when it was placed in one.
    pub color: Option<u32>,
}

pub fn flatten(doc: &Document, comp_id: &str) -> Vec<Leaf> {
    let mut out = Vec::new();
    let mut stack = vec![comp_id.to_string()];
    walk(doc, comp_id, DVec3::ZERO, DMat3::IDENTITY, &mut Vec::new(), &mut out, &mut stack);
    out
}

fn walk(doc: &Document, comp_id: &str, p0: DVec3, r0: DMat3, path: &mut Vec<String>, out: &mut Vec<Leaf>, stack: &mut Vec<String>) {
    let Some(comp) = doc.components.get(comp_id) else { return };
    for ch in &comp.children {
        let r = r0 * rot_mat(ch.rot);
        let p = p0 + r0 * DVec3::from_array(ch.pos);
        path.push(ch.name.clone());
        if let Some(pid) = &ch.part {
            out.push(Leaf {
                path: path.clone(),
                part_id: pid.clone(),
                pos: p,
                rot: r,
                color: ch.color,
            });
        } else if let Some(cid) = &ch.component
            && !stack.contains(cid)
        {
            stack.push(cid.clone());
            walk(doc, cid, p, r, path, out, stack);
            stack.pop();
        }
        path.pop();
    }
}

pub fn component_contains(doc: &Document, id: &str, target: &str, stack: &mut Vec<String>) -> bool {
    if id == target {
        return true;
    }
    let Some(comp) = doc.components.get(id) else { return false };
    if stack.iter().any(|s| s == id) {
        return false;
    }
    stack.push(id.to_string());
    let hit = comp.children.iter().any(|ch| {
        ch.component
            .as_deref()
            .map(|c| component_contains(doc, c, target, stack))
            .unwrap_or(false)
    });
    stack.pop();
    hit
}

/// The part of a document that one component needs: the components it
/// reaches and the parts they use, with that component as the robot's
/// root and no roles — what a map takes as a prop.
pub fn subset(doc: &Document, root: &str) -> Option<Document> {
    if !doc.components.contains_key(root) {
        return None;
    }
    let mut components = BTreeMap::new();
    let mut parts = BTreeMap::new();
    let mut todo = vec![root.to_string()];
    while let Some(id) = todo.pop() {
        if components.contains_key(&id) {
            continue;
        }
        let Some(comp) = doc.components.get(&id) else { continue };
        for ch in &comp.children {
            if let Some(p) = &ch.part
                && let Some(part) = doc.parts.get(p)
            {
                parts.insert(p.clone(), part.clone());
            }
            if let Some(c) = &ch.component {
                todo.push(c.clone());
            }
        }
        components.insert(id, comp.clone());
    }
    Some(Document {
        format: doc.format.clone(),
        units: doc.units.clone(),
        parts,
        components,
        robot: Robot {
            name: root.to_string(),
            root: root.to_string(),
            roles: BTreeMap::new(),
            spawn: Spawn::default(),
            measured_mass_g: None,
        },
    })
}

/// What an import brought into a library.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Imported {
    pub components: Vec<String>,
    pub parts: Vec<String>,
    /// Ids the import had to change: the library held something else by that name.
    pub renamed: Vec<(String, String)>,
}

/// `base`, or `base_2`, `base_3`… — the first not `taken`.
fn fresh_id(base: &str, taken: impl Fn(&str) -> bool) -> String {
    if !taken(base) {
        return base.to_string();
    }
    let mut k = 2;
    loop {
        let id = format!("{base}_{k}");
        if !taken(&id) {
            return id;
        }
        k += 1;
    }
}

/// The parts and components of a saved build join `doc`'s library.
/// What the library already holds, the same, is left as it is (under
/// whatever id it has here); what it holds under the same id but
/// different comes in under a new id, and the build's own references
/// follow. Nothing is placed: the components are there to add. Bringing
/// the same build in twice adds nothing the second time.
pub fn merge_library(doc: &mut Document, other: &Document) -> Imported {
    let mut out = Imported::default();
    let mut part_ids: BTreeMap<String, String> = BTreeMap::new();
    for (id, part) in &other.parts {
        let target = match doc.parts.iter().find(|(_, mine)| *mine == part) {
            Some((same, _)) => same.clone(),
            None => {
                let id2 = fresh_id(id, |k| doc.parts.contains_key(k));
                doc.parts.insert(id2.clone(), part.clone());
                out.parts.push(id2.clone());
                id2
            }
        };
        if target != *id {
            out.renamed.push((id.clone(), target.clone()));
        }
        part_ids.insert(id.clone(), target);
    }
    let remap = |comp: &Component, comp_ids: &BTreeMap<String, String>| -> Component {
        let mut c = comp.clone();
        for ch in &mut c.children {
            if let Some(p) = &ch.part {
                ch.part = Some(part_ids.get(p).cloned().unwrap_or_else(|| p.clone()));
            }
            if let Some(cid) = &ch.component {
                ch.component = Some(comp_ids.get(cid).cloned().unwrap_or_else(|| cid.clone()));
            }
        }
        c
    };
    // components come in after the ones they refer to, so a component whose
    // sub-components the library already has compares equal to its copy here
    let mut comp_ids: BTreeMap<String, String> = BTreeMap::new();
    let mut pending: Vec<String> = other.components.keys().cloned().collect();
    while !pending.is_empty() {
        let ready: Vec<String> = pending
            .iter()
            .filter(|id| {
                other.components[*id]
                    .children
                    .iter()
                    .filter_map(|ch| ch.component.as_ref())
                    .all(|c| comp_ids.contains_key(c) || c == *id || !other.components.contains_key(c))
            })
            .cloned()
            .collect();
        let batch = if ready.is_empty() { vec![pending[0].clone()] } else { ready };
        for id in batch {
            pending.retain(|p| p != &id);
            let comp = remap(&other.components[&id], &comp_ids);
            let target = match doc.components.iter().find(|(_, mine)| **mine == comp) {
                Some((same, _)) => same.clone(),
                None => {
                    let id2 = fresh_id(&id, |k| doc.components.contains_key(k));
                    doc.components.insert(id2.clone(), comp);
                    out.components.push(id2.clone());
                    id2
                }
            };
            if target != id {
                out.renamed.push((id.clone(), target.clone()));
            }
            comp_ids.insert(id, target);
        }
    }
    out
}

/// One library brick as a document of its own — what a map takes as a
/// prop when a brick is added from the library.
pub fn brick_document(bundle: &Bundle, num: &str) -> Option<Document> {
    let rec = bundle.parts.get(num)?;
    let id = format!("lego_{num}");
    let mut parts = BTreeMap::new();
    parts.insert(
        id.clone(),
        Part {
            name: rec.name.clone(),
            category: "lego".into(),
            mass_g: rec.mass_g,
            source: rec.source.clone(),
            source_note: rec.source_note.clone(),
            ldraw: Some(num.to_string()),
            shapes: vec![],
            extra: Default::default(),
        },
    );
    let mut components = BTreeMap::new();
    components.insert(
        "brick".to_string(),
        Component {
            note: String::new(),
            children: vec![Instance {
                name: "brick".into(),
                part: Some(id),
                component: None,
                pos: [0.0; 3],
                rot: [0.0; 3],
                locked: false,
                color: None,
            }],
        },
    );
    Some(Document {
        format: "openbricks-assembly/1".into(),
        units: serde_json::json!({"length": "mm", "mass": "g"}),
        parts,
        components,
        robot: Robot {
            name: rec.name.clone(),
            root: "brick".into(),
            roles: BTreeMap::new(),
            spawn: Spawn::default(),
            measured_mass_g: None,
        },
    })
}

pub fn usage_count(doc: &Document, comp_id: &str) -> usize {
    doc.components
        .values()
        .flat_map(|c| c.children.iter())
        .filter(|ch| ch.component.as_deref() == Some(comp_id))
        .count()
}

pub fn unique_name(siblings: &[Instance], base: &str) -> String {
    let stem = match base.rfind('_') {
        Some(i) if base[i + 1..].chars().all(|c| c.is_ascii_digit()) && i + 1 < base.len() => &base[..i],
        _ => base,
    };
    let mut i = 2;
    loop {
        let name = format!("{stem}_{i}");
        if !siblings.iter().any(|s| s.name == name) {
            return name;
        }
        i += 1;
    }
}

pub fn slug(s: &str) -> String {
    let mut out = String::new();
    let mut last_us = true;
    for c in s.to_lowercase().chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c);
            last_us = false;
        } else if !last_us {
            out.push('_');
            last_us = true;
        }
    }
    out.trim_matches('_').to_string()
}

// ------------------------------------------------------------ editing

/// Rewrite every role path by walking it from the root; `f` sees each
/// segment inside `editing` and answers: unchanged (`None`), dropped
/// (`Some(vec![])`) or replaced by segments.
pub fn remap_roles(doc: &mut Document, editing: &str, f: &dyn Fn(&str) -> Option<Vec<String>>) {
    let root = doc.robot.root.clone();
    let mut roles = doc.robot.roles.clone();
    for value in roles.values_mut() {
        if value.is_empty() {
            continue;
        }
        let segs: Vec<String> = value.split('/').map(String::from).collect();
        let mut out: Vec<String> = Vec::new();
        let mut comp: Option<String> = Some(root.clone());
        let mut k = 0;
        while k < segs.len() {
            let seg = &segs[k];
            let here = comp.as_deref() == Some(editing);
            match if here { f(seg) } else { None } {
                None => out.push(seg.clone()),
                Some(reps) => out.extend(reps),
            }
            let child = comp
                .as_deref()
                .and_then(|c| doc.components.get(c))
                .and_then(|c| c.children.iter().find(|ch| &ch.name == seg));
            comp = child.and_then(|ch| ch.component.clone());
            k += 1;
            if comp.is_none() {
                out.extend(segs[k..].iter().cloned());
                break;
            }
        }
        *value = out.join("/");
    }
    doc.robot.roles = roles;
}

/// The library id a component named by the user gets: the name's slug,
/// which must be something and not taken.
pub fn component_id(doc: &Document, name: &str) -> Result<String, String> {
    let id = slug(name);
    if id.is_empty() {
        return Err("Give the component a name".into());
    }
    if doc.components.contains_key(&id) {
        return Err(format!("A component named {id} already exists"));
    }
    Ok(id)
}

/// A new component with nothing in it joins the library under the
/// name's id, to be built from bricks.
pub fn new_component(doc: &mut Document, name: &str) -> Result<String, String> {
    let id = component_id(doc, name)?;
    doc.components.insert(id.clone(), Component::default());
    Ok(id)
}

/// Group the named top-level instances of `editing` into a new
/// component `new_id`: the first one's frame becomes the new origin;
/// world poses do not move; roles follow.
pub fn group(doc: &mut Document, editing: &str, names: &[String], new_id: &str) -> Result<String, String> {
    let new_id = component_id(doc, new_id)?;
    let comp = doc.components.get(editing).ok_or("no such component")?.clone();
    let chosen: Vec<Instance> = names
        .iter()
        .filter_map(|n| comp.children.iter().find(|c| &c.name == n).cloned())
        .collect();
    if chosen.is_empty() {
        return Err("Nothing selected".into());
    }
    let first = &chosen[0];
    let p0 = DVec3::from_array(first.pos);
    let r0 = rot_mat(first.rot);
    let r0t = r0.transpose();
    let children: Vec<Instance> = chosen
        .iter()
        .map(|ch| {
            let mut c = ch.clone();
            c.pos = (r0t * (DVec3::from_array(ch.pos) - p0)).to_array().map(round3);
            c.rot = euler_from(&(r0t * rot_mat(ch.rot)));
            c
        })
        .collect();
    let inst_name = if comp.children.iter().any(|c| c.name == new_id && !names.contains(&c.name)) {
        unique_name(&comp.children, &new_id)
    } else {
        new_id.clone()
    };
    let grouped: HashSet<String> = names.iter().cloned().collect();
    let iname = inst_name.clone();
    remap_roles(doc, editing, &|seg| {
        if grouped.contains(seg) {
            Some(vec![iname.clone(), seg.to_string()])
        } else {
            None
        }
    });
    doc.components.insert(
        new_id.clone(),
        Component {
            note: String::new(),
            children,
        },
    );
    let comp = doc.components.get_mut(editing).unwrap();
    let idx = comp.children.iter().position(|c| c.name == first.name).unwrap_or(0);
    comp.children.retain(|c| !grouped.contains(&c.name));
    let idx = idx.min(comp.children.len());
    comp.children.insert(
        idx,
        Instance {
            name: inst_name.clone(),
            part: None,
            component: Some(new_id.clone()),
            pos: p0.to_array(),
            rot: first.rot,
            locked: false,
            color: None,
        },
    );
    Ok(inst_name)
}

/// Replace a component instance by its children, placed in the parent
/// frame; the component definition stays in the library.
pub fn ungroup(doc: &mut Document, editing: &str, name: &str) -> Result<Vec<String>, String> {
    let comp = doc.components.get(editing).ok_or("no such component")?.clone();
    let inst = comp.children.iter().find(|c| c.name == name).ok_or("no such instance")?.clone();
    let cid = inst.component.clone().ok_or("not a component instance")?;
    let inner = doc.components.get(&cid).ok_or("the component definition is missing")?.clone();
    let r = rot_mat(inst.rot);
    let p = DVec3::from_array(inst.pos);
    let mut siblings: Vec<Instance> = comp.children.iter().filter(|c| c.name != name).cloned().collect();
    let mut made = Vec::new();
    let mut rename: HashMap<String, String> = HashMap::new();
    for ch in &inner.children {
        let mut c = ch.clone();
        c.pos = (p + r * DVec3::from_array(ch.pos)).to_array().map(round3);
        c.rot = euler_from(&(r * rot_mat(ch.rot)));
        if siblings.iter().any(|s| s.name == c.name) {
            let nn = unique_name(&siblings, &c.name);
            rename.insert(ch.name.clone(), nn.clone());
            c.name = nn;
        }
        siblings.push(c.clone());
        made.push(c);
    }
    let dropped = name.to_string();
    remap_roles(doc, editing, &|seg| if seg == dropped { Some(vec![]) } else { None });
    if !rename.is_empty() {
        let editing_owned = editing.to_string();
        let root = doc.robot.root.clone();
        let comps = doc.components.clone();
        for value in doc.robot.roles.values_mut() {
            let mut segs: Vec<String> = value.split('/').map(String::from).collect();
            let mut comp_id = Some(root.clone());
            for seg in segs.iter_mut() {
                if comp_id.as_deref() == Some(editing_owned.as_str())
                    && let Some(nn) = rename.get(seg)
                {
                    *seg = nn.clone();
                }
                let child = comp_id
                    .as_deref()
                    .and_then(|c| comps.get(c))
                    .and_then(|c| c.children.iter().find(|ch| &ch.name == seg));
                comp_id = child.and_then(|ch| ch.component.clone());
            }
            *value = segs.join("/");
        }
    }
    let comp = doc.components.get_mut(editing).unwrap();
    let idx = comp.children.iter().position(|c| c.name == name).unwrap();
    comp.children.remove(idx);
    let names: Vec<String> = made.iter().map(|c| c.name.clone()).collect();
    for (i, c) in made.into_iter().enumerate() {
        comp.children.insert(idx + i, c);
    }
    Ok(names)
}

pub fn round3(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}

// ---------------------------------------------------------- connectors

#[derive(Clone, Debug)]
pub struct WorldConnector {
    pub kind: String,
    pub centre: DVec3,
    pub axis: DVec3,
    pub length: f64,
    pub path: Vec<String>,
}

pub fn connectors_of_leaf(doc: &Document, bundle: &Bundle, leaf: &Leaf) -> Vec<WorldConnector> {
    let Some(part) = doc.parts.get(&leaf.part_id) else { return vec![] };
    let list: Vec<Connector> = match geometry_of(part, bundle) {
        Geometry::Record(rec) => with_stud_sockets(&rec.connectors, &rec.bbox, &rec.name),
        // a fetched part's record travels without its sockets: derived here as for the library's
        // (nothing to derive for an STL import, which has no studs)
        Geometry::Imported { connectors, bbox, .. } => {
            with_stud_sockets(&connectors, &[bbox.min.to_array(), bbox.max.to_array()], &part.name)
        }
        _ => vec![],
    };
    list.into_iter()
        .map(|c| WorldConnector {
            kind: c.kind,
            centre: leaf.pos + leaf.rot * DVec3::from_array(c.centre),
            axis: (leaf.rot * DVec3::from_array(c.axis)).normalize_or_zero(),
            length: c.length,
            path: leaf.path.clone(),
        })
        .collect()
}

/// The stud sockets a part's underside has: one under every stud, as
/// deep in the underside as the stud is tall, so a stud below meets it
/// where the part's own stud stands above — a brick or plate stacks on
/// another with the two stud grids in step. Only studs along a bbox
/// axis get one. A tile has no studs and the same underside: it gets
/// one under every place of its footprint's stud grid, a stud's height
/// up its bottom face — see [`tile_sockets`].
pub fn with_stud_sockets(connectors: &[Connector], bbox: &[[f64; 3]; 2], name: &str) -> Vec<Connector> {
    let mut out = connectors.to_vec();
    if !connectors.iter().any(|c| c.kind == "stud") {
        out.extend(tile_sockets(bbox, name));
    }
    for c in connectors.iter().filter(|c| c.kind == "stud") {
        let a = c.axis;
        let k = (0..3)
            .max_by(|&i, &j| a[i].abs().partial_cmp(&a[j].abs()).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or(2);
        if a[k].abs() < 0.99 {
            continue;
        }
        let face = if a[k] > 0.0 { bbox[1][k] } else { bbox[0][k] };
        let depth = (face - c.centre[k]).abs();
        let mut centre = c.centre;
        centre[k] = round3(c.centre[k] + a[k].signum() * (depth - c.length / 2.0));
        out.push(Connector {
            kind: "stud_socket".into(),
            centre,
            axis: c.axis,
            length: c.length,
            r: c.r,
        });
    }
    out
}

/// The sockets under a tile (a part named one, with no studs of its
/// own): one per stud place of the 8 mm grid its footprint covers,
/// when the footprint is whole studs — a stud's height up its bottom
/// face, pointing down, as a stud below stands.
pub fn tile_sockets(bbox: &[[f64; 3]; 2], name: &str) -> Vec<Connector> {
    use crate::editor::MODULE_MM;
    const STUD_MM: f64 = 1.6;
    const STUD_R_MM: f64 = 2.4;
    let is_tile = name.split_whitespace().any(|w| w.eq_ignore_ascii_case("tile"));
    let dx = bbox[1][0] - bbox[0][0];
    let dy = bbox[1][1] - bbox[0][1];
    let (nx, ny) = ((dx / MODULE_MM).round(), (dy / MODULE_MM).round());
    if !is_tile || nx < 1.0 || ny < 1.0 || (dx - nx * MODULE_MM).abs() > 0.6 || (dy - ny * MODULE_MM).abs() > 0.6 {
        return vec![];
    }
    let z = round3(bbox[0][2] + STUD_MM / 2.0);
    let mut out = Vec::new();
    for i in 0..nx as usize {
        for j in 0..ny as usize {
            out.push(Connector {
                kind: "stud_socket".into(),
                centre: [
                    round3(bbox[0][0] + MODULE_MM / 2.0 + i as f64 * MODULE_MM),
                    round3(bbox[0][1] + MODULE_MM / 2.0 + j as f64 * MODULE_MM),
                    z,
                ],
                axis: [0.0, 0.0, -1.0],
                length: STUD_MM,
                r: STUD_R_MM,
            });
        }
    }
    out
}

fn mates(male: &str) -> &'static [&'static str] {
    match male {
        "pin" => &["pin_hole"],
        "axle" => &["axle_hole", "pin_hole"],
        "stud" => &["stud_socket", "stud_hole", "pin_hole"],
        _ => &[],
    }
}

/// Whether one of the two is the male of the other.
fn compatible(a: &WorldConnector, b: &WorldConnector) -> bool {
    mates(&a.kind).contains(&b.kind.as_str()) || mates(&b.kind).contains(&a.kind.as_str())
}

/// How close a feature must come to a hole (or a stud to a socket) for
/// the magnet to take it: on letting go, and while it is dragged near.
pub const SNAP_MM: f64 = 6.0;
pub const PULL_MM: f64 = 4.0;

/// A seat the magnet may settle on: (mates, spread, movement), the
/// shift, the mated path.
type Choice = ((usize, f64, f64), DVec3, Vec<String>);

pub struct Mate<'a> {
    pub m: &'a WorldConnector,
    pub o: &'a WorldConnector,
    pub dist: f64,
}

pub fn mated_pairs<'a>(mine: &'a [WorldConnector], others: &'a [WorldConnector], tol_mm: f64, tol_deg: f64) -> Vec<Mate<'a>> {
    let mut out = Vec::new();
    for m in mine {
        for o in others {
            let (male, female) = if !mates(&m.kind).is_empty() {
                (m, o)
            } else if !mates(&o.kind).is_empty() {
                (o, m)
            } else {
                continue;
            };
            if !mates(&male.kind).contains(&female.kind.as_str()) {
                continue;
            }
            let cos_a = m.axis.dot(o.axis).abs();
            if cos_a < tol_deg.to_radians().cos() {
                continue;
            }
            let d = o.centre - m.centre;
            let along = d.dot(o.axis);
            let off = (d - o.axis * along).length();
            let slack = ((m.length.max(o.length) - m.length.min(o.length)) / 2.0).max(0.0);
            let dist = off.hypot((along.abs() - slack).max(0.0));
            if dist <= tol_mm {
                out.push(Mate { m, o, dist });
            }
        }
    }
    out.sort_by(|a, b| a.dist.partial_cmp(&b.dist).unwrap_or(std::cmp::Ordering::Equal));
    out
}

/// Whether any feature of `mine` sits seated in one of `others`: a
/// mate within `MATE_MM` and `MATE_DEG` with no slack along the axis (a
/// pin all the way in its hole, a stud down on its socket — a brick
/// sunk into the one below is not), or an axle anywhere along a hole
/// it runs through; which the overlap rule excuses as a joint.
pub fn seated(mine: &[WorldConnector], others: &[WorldConnector]) -> bool {
    mated_pairs(mine, others, MATE_MM, MATE_DEG).iter().any(|p| {
        let male = if mates(&p.m.kind).is_empty() { p.o } else { p.m };
        let d = p.o.centre - p.m.centre;
        male.kind == "axle" || d.dot(p.o.axis).abs() <= MATE_MM
    })
}

/// A place the magnet could put a group: every member's position
/// (all shifted alike — the magnet never turns a part), how many
/// features it mates there, and the path of what it mates.
#[derive(Clone, Debug, PartialEq)]
pub struct Seat {
    pub poses: Vec<(String, [f64; 3])>,
    pub mates: usize,
    pub path: Vec<String>,
}

/// How many seats the magnet offers, best first.
pub const SEATS_MAX: usize = 8;

/// Move the instances `names` (top-level in `editing`), as one rigid
/// group, so their features sit in the holes and on the studs of the
/// other instances: the best of [`group_seats`]. Returns the mated
/// path. (The editor takes the first seat its overlap rule accepts.)
#[cfg(test)]
pub fn snap_group_within(doc: &mut Document, bundle: &Bundle, editing: &str, names: &[String], tol_mm: f64) -> Option<Vec<String>> {
    let seat = group_seats(doc, bundle, editing, names, tol_mm).into_iter().next()?;
    seat_group(doc, editing, &seat);
    Some(seat.path)
}

/// Put the group where `seat` says.
pub fn seat_group(doc: &mut Document, editing: &str, seat: &Seat) {
    let Some(comp) = doc.components.get_mut(editing) else { return };
    for (name, pos) in &seat.poses {
        if let Some(child) = comp.children.iter_mut().find(|c| c.name == *name) {
            child.pos = *pos;
        }
    }
}

/// The seats within `tol_mm` for the instances `names` (top-level in
/// `editing`) moved as one rigid group, best first: the shift each
/// compatible pair of features asks for (a feature within `tol_mm` of
/// a hole along its own axis, within 5° of it), clustered; the shift
/// the most features agree on ranks first (a plate lands on a brick's
/// whole stud grid, a two-pin connector in both holes), the tightest
/// agreement next, the least movement last; at most [`SEATS_MAX`]
/// distinct ones. The magnet never turns a part: a heading is the
/// user's to set (a pair of 1 x 6 let go under a 2 x 4 lying across
/// them used to be turned along it for the four extra studs, and land
/// where the bricks beside it stood). A caller that can judge a seat
/// (the editor, by its overlap rule) takes the first it accepts among
/// those mating as many features as the best.
pub fn group_seats(doc: &Document, bundle: &Bundle, editing: &str, names: &[String], tol_mm: f64) -> Vec<Seat> {
    let leaves = flatten(doc, editing);
    let is_mine = |top: &str| names.iter().any(|n| n == top);
    let mine: Vec<WorldConnector> = leaves
        .iter()
        .filter(|l| is_mine(&l.path[0]))
        .flat_map(|l| connectors_of_leaf(doc, bundle, l))
        .collect();
    let Some(first) = names.first() else { return vec![] };
    let Some(comp) = doc.components.get(editing) else { return vec![] };
    let Some(inst) = comp.children.iter().find(|c| c.name == *first) else {
        return vec![];
    };
    let pos = DVec3::from_array(inst.pos);
    let reach = mine.iter().map(|c| (c.centre - pos).length()).fold(0.0, f64::max) * 2.0 + tol_mm + 20.0;
    let others: Vec<WorldConnector> = leaves
        .iter()
        .filter(|l| !is_mine(&l.path[0]))
        .flat_map(|l| connectors_of_leaf(doc, bundle, l))
        .filter(|c| (c.centre - pos).length() <= reach)
        .collect();
    if mine.is_empty() || others.is_empty() {
        return vec![];
    }
    // the shift every pair asks for, clustered; the most agreeing features rank first, then
    // the tightest agreement, then the least movement. Seats within a stud's height along
    // the axis come first: a plate dragged over another slides on it rather than dropping
    // through it onto the brick below, which offers more studs.
    let cos_tol = 5.0_f64.to_radians().cos();
    let mut choices: Vec<Choice> = Vec::new();
    for along_tol in [1.8_f64.min(tol_mm), tol_mm] {
        if !choices.is_empty() {
            break;
        }
        let mut asks: Vec<(usize, DVec3, &WorldConnector)> = Vec::new();
        for (i, m) in mine.iter().enumerate() {
            for o in &others {
                if !compatible(m, o) || m.axis.dot(o.axis).abs() < cos_tol {
                    continue;
                }
                let off = m.centre - o.centre;
                let along = off.dot(o.axis);
                let perp = off - o.axis * along;
                if perp.length() > tol_mm {
                    continue;
                }
                let slack = ((m.length.max(o.length) - m.length.min(o.length)) / 2.0).max(0.0);
                let kept = if (o.length - m.length).abs() < 1.0 {
                    0.0
                } else if m.kind == "stud" {
                    // a stud in a tube or a Technic hole goes in up to its base: it sits at
                    // the hole's mouth on its own side, not anywhere along
                    slack * m.axis.dot(o.axis).signum()
                } else if o.kind == "stud" {
                    -slack
                } else {
                    along.clamp(-slack, slack)
                };
                if (along - kept).abs() > along_tol {
                    continue;
                }
                asks.push((i, -perp + o.axis * (kept - along), o));
            }
        }
        // a feature is where it is, whatever it is called twice over: a tube is a stud
        // hole and a pin hole in one place, and votes once
        let spot = |i: usize| {
            let c = mine[i].centre;
            [
                (c.x * 10.0).round() as i64,
                (c.y * 10.0).round() as i64,
                (c.z * 10.0).round() as i64,
            ]
        };
        for (_, shift, o) in &asks {
            let mut seen = HashSet::new();
            let members: Vec<DVec3> = asks
                .iter()
                .filter(|(i, s, _)| (*s - *shift).length() <= 1.0 && seen.insert(spot(*i)))
                .map(|(_, s, _)| *s)
                .collect();
            let count = members.len();
            let mean = members.iter().fold(DVec3::ZERO, |acc, s| acc + *s) / count as f64;
            let residual: f64 = members.iter().map(|s| (*s - mean).length()).sum();
            choices.push(((count, residual, mean.length()), mean, o.path.clone()));
        }
    }
    // best first; a tie (to a micron) keeps the order found
    let micron = |v: f64| (v * 1e6).round() as i64;
    choices.sort_by_key(|((count, residual, movement), ..)| (std::cmp::Reverse(*count), micron(*residual), micron(*movement)));
    let round2 = |v: f64| (v * 100.0).round() / 100.0;
    let mut seats: Vec<Seat> = Vec::new();
    for ((mates, ..), shift, path) in choices {
        let poses: Vec<(String, [f64; 3])> = comp
            .children
            .iter()
            .filter(|c| is_mine(&c.name))
            .map(|child| (child.name.clone(), (DVec3::from_array(child.pos) + shift).to_array().map(round2)))
            .collect();
        if seats.iter().any(|s| s.poses == poses) {
            continue;
        }
        seats.push(Seat { poses, mates, path });
        if seats.len() >= SEATS_MAX {
            break;
        }
    }
    seats
}

/// How close two features must sit, in mm and degrees, to count as
/// mated: what the inspector lists, and what the overlap rule excuses.
pub const MATE_MM: f64 = 0.4;
pub const MATE_DEG: f64 = 3.0;

pub fn connections_of(doc: &Document, bundle: &Bundle, editing: &str, name: &str) -> Vec<(String, String, Vec<String>)> {
    let leaves = flatten(doc, editing);
    let mine: Vec<WorldConnector> = leaves
        .iter()
        .filter(|l| l.path[0] == name)
        .flat_map(|l| connectors_of_leaf(doc, bundle, l))
        .collect();
    let others: Vec<WorldConnector> = leaves
        .iter()
        .filter(|l| l.path[0] != name)
        .flat_map(|l| connectors_of_leaf(doc, bundle, l))
        .collect();
    let mut seen = HashSet::new();
    mated_pairs(&mine, &others, MATE_MM, MATE_DEG)
        .into_iter()
        .filter(|p| seen.insert((p.m.centre.to_array().map(|v| (v * 10.0).round() as i64), p.o.path.join("/"))))
        .map(|p| (p.m.kind.clone(), p.o.kind.clone(), p.o.path.clone()))
        .collect()
}

pub const EXAMPLE: &str = include_str!("../assets/example.assembly.json");

pub fn example() -> Document {
    serde_json::from_str(EXAMPLE).expect("the built-in example is valid")
}

/// Problems a file has before it can be edited.
pub fn validate(doc: &Document, bundle: &Bundle) -> Vec<String> {
    let mut errs = Vec::new();
    if doc.format != FORMAT {
        errs.push(format!("format must be \"{FORMAT}\""));
    }
    if !doc.components.contains_key(&doc.robot.root) {
        errs.push("robot.root must name a component".into());
    }
    for (id, p) in &doc.parts {
        if p.mass_g <= 0.0 {
            errs.push(format!("brick {id} needs mass_g > 0"));
        }
        if let Some(num) = &p.ldraw {
            if !bundle.parts.contains_key(num) && !p.extra.contains_key("mesh") {
                errs.push(format!("brick {id} needs LDraw part {num}, which this library does not carry"));
            }
        } else if p.shapes.is_empty() && !p.extra.contains_key("mesh") {
            errs.push(format!("brick {id} needs a shape, an LDraw number or a mesh"));
        }
    }
    for (id, c) in &doc.components {
        let mut names = HashSet::new();
        for ch in &c.children {
            if ch.name.is_empty() || ch.name.contains('/') || !names.insert(ch.name.clone()) {
                errs.push(format!("component {id}: every child needs a unique name without /"));
            }
            match (&ch.part, &ch.component) {
                (Some(p), _) if !doc.parts.contains_key(p) => errs.push(format!("component {id}: brick {p} is not in the library")),
                (None, Some(cc)) if !doc.components.contains_key(cc) => errs.push(format!("component {id}: component {cc} does not exist")),
                (None, None) => errs.push(format!("component {id}: child {} needs part or component", ch.name)),
                _ => {}
            }
        }
    }
    errs
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bundle() -> Bundle {
        Bundle {
            format: String::new(),
            source: String::new(),
            parts: BTreeMap::new(),
            missing: vec![],
            sets: BTreeMap::new(),
            colors: BTreeMap::new(),
        }
    }

    fn part(mass: f64, shape: Shape) -> Part {
        Part {
            name: "p".into(),
            category: String::new(),
            mass_g: mass,
            source: String::new(),
            source_note: String::new(),
            ldraw: None,
            shapes: vec![shape],
            extra: Default::default(),
        }
    }

    fn doc_with(parts: Vec<(&str, Part)>, comps: Vec<(&str, Vec<Instance>)>) -> Document {
        Document {
            format: FORMAT.into(),
            units: serde_json::Value::Null,
            parts: parts.into_iter().map(|(k, v)| (k.to_string(), v)).collect(),
            components: comps
                .into_iter()
                .map(|(k, ch)| {
                    (
                        k.to_string(),
                        Component {
                            note: String::new(),
                            children: ch,
                        },
                    )
                })
                .collect(),
            robot: Robot {
                name: "t".into(),
                root: "robot".into(),
                roles: BTreeMap::new(),
                spawn: Spawn::default(),
                measured_mass_g: None,
            },
        }
    }

    fn inst(name: &str, part: &str, pos: [f64; 3], rot: [f64; 3]) -> Instance {
        Instance {
            name: name.into(),
            part: Some(part.into()),
            component: None,
            pos,
            rot,
            locked: false,
            color: None,
        }
    }

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn two_half_boxes_roll_up_to_one_box() {
        let full = part(
            8.0,
            Shape::Box {
                size: [40.0, 20.0, 10.0],
                pos: [0.0; 3],
            },
        );
        let half = part(
            4.0,
            Shape::Box {
                size: [20.0, 20.0, 10.0],
                pos: [0.0; 3],
            },
        );
        let doc = doc_with(
            vec![("full", full.clone()), ("half", half)],
            vec![(
                "robot",
                vec![
                    inst("a", "half", [-10.0, 0.0, 0.0], [0.0; 3]),
                    inst("b", "half", [10.0, 0.0, 0.0], [0.0; 3]),
                ],
            )],
        );
        let b = bundle();
        let f = part_props(&full, &b);
        let mut memo = HashMap::new();
        let mut errs = vec![];
        let pr = component_props(&doc, &b, "robot", &mut memo, &mut vec![], &mut errs);
        assert!(errs.is_empty());
        assert!(near(pr.mass, 8.0));
        assert!(pr.com.length() < 1e-9);
        for i in 0..3 {
            for j in 0..3 {
                assert!(near(pr.inertia.col(j)[i], f.inertia.col(j)[i]), "{i}{j}");
            }
        }
        assert!(near(f.inertia.col(0)[0], 8.0 / 12.0 * 500.0));
    }

    #[test]
    fn rotation_conventions_round_trip() {
        for rpy in [
            [0.0, 0.0, 90.0],
            [90.0, 0.0, 0.0],
            [0.0, 90.0, 0.0],
            [30.0, -40.0, 120.0],
            [-90.0, 0.0, 180.0],
        ] {
            let m = rot_mat(rpy);
            let back = rot_mat(euler_from(&m));
            for c in 0..3 {
                assert!((m.col(c) - back.col(c)).length() < 1e-6, "{rpy:?}");
            }
        }
        let yawed = rot_mat([0.0, 0.0, 90.0]) * DVec3::X;
        assert!((yawed - DVec3::Y).length() < 1e-9);
    }

    #[test]
    fn parallel_axis_moves_inertia() {
        let full = part(
            8.0,
            Shape::Box {
                size: [40.0, 20.0, 10.0],
                pos: [0.0; 3],
            },
        );
        let doc = doc_with(
            vec![("full", full.clone())],
            vec![(
                "robot",
                vec![
                    inst("a", "full", [100.0, 0.0, 0.0], [0.0; 3]),
                    inst("b", "full", [-100.0, 0.0, 0.0], [0.0; 3]),
                ],
            )],
        );
        let b = bundle();
        let f = part_props(&full, &b);
        let pr = component_props(&doc, &b, "robot", &mut HashMap::new(), &mut vec![], &mut vec![]);
        assert!(near(pr.inertia.col(1)[1], 2.0 * f.inertia.col(1)[1] + 2.0 * 8.0 * 10000.0));
        assert!(near(pr.inertia.col(0)[0], 2.0 * f.inertia.col(0)[0]));
    }

    #[test]
    fn group_and_ungroup_keep_world_poses_and_roles() {
        let full = part(
            8.0,
            Shape::Box {
                size: [40.0, 20.0, 10.0],
                pos: [0.0; 3],
            },
        );
        let mut doc = doc_with(
            vec![("full", full)],
            vec![(
                "robot",
                vec![
                    inst("a", "full", [10.0, 20.0, 30.0], [0.0, 0.0, 90.0]),
                    inst("b", "full", [-5.0, 8.0, 0.0], [45.0, 0.0, 0.0]),
                    inst("c", "full", [0.0; 3], [0.0; 3]),
                ],
            )],
        );
        doc.robot.roles.insert("imu".into(), "b".into());
        doc.robot.roles.insert("caster".into(), "c".into());
        let snapshot = |d: &Document| -> Vec<(String, [i64; 3])> {
            let mut v: Vec<_> = flatten(d, "robot")
                .into_iter()
                .map(|l| (l.part_id, l.pos.to_array().map(|x| (x * 100.0).round() as i64)))
                .collect();
            v.sort();
            v
        };
        let before = snapshot(&doc);
        let name = group(&mut doc, "robot", &["a".into(), "b".into()], "unit").unwrap();
        assert_eq!(name, "unit");
        assert_eq!(snapshot(&doc), before);
        assert_eq!(doc.robot.roles["imu"], "unit/b");
        assert_eq!(doc.robot.roles["caster"], "c");
        assert_eq!(doc.components["unit"].children[0].pos, [0.0; 3]);
        assert_eq!(usage_count(&doc, "unit"), 1);
        let made = ungroup(&mut doc, "robot", "unit").unwrap();
        assert_eq!(made, vec!["a".to_string(), "b".to_string()]);
        assert_eq!(snapshot(&doc), before);
        assert_eq!(doc.robot.roles["imu"], "b");
        assert!(doc.components.contains_key("unit"));
        assert!(group(&mut doc, "robot", &["a".into()], "unit").is_err());
        assert!(group(&mut doc, "robot", &[], "x").is_err());
    }

    #[test]
    fn containment_and_names() {
        let mut doc = doc_with(vec![], vec![("robot", vec![]), ("u", vec![])]);
        doc.components.get_mut("robot").unwrap().children.push(Instance {
            name: "u1".into(),
            part: None,
            component: Some("u".into()),
            pos: [0.0; 3],
            rot: [0.0; 3],
            locked: false,
            color: None,
        });
        assert!(component_contains(&doc, "robot", "u", &mut vec![]));
        assert!(!component_contains(&doc, "u", "robot", &mut vec![]));
        let sib = vec![inst("pin", "p", [0.0; 3], [0.0; 3]), inst("pin_2", "p", [0.0; 3], [0.0; 3])];
        assert_eq!(unique_name(&sib, "pin"), "pin_3");
        assert_eq!(slug("Drive Unit #2!"), "drive_unit_2");
    }

    #[test]
    fn a_new_component_starts_empty_in_the_library() {
        let b = bundle();
        let box10 = Shape::Box {
            size: [10.0, 10.0, 10.0],
            pos: [0.0; 3],
        };
        let mut doc = doc_with(
            vec![("p", part(10.0, box10))],
            vec![("robot", vec![inst("a", "p", [0.0; 3], [0.0; 3])])],
        );
        assert_eq!(component_id(&doc, " - "), Err("Give the component a name".into()));
        assert_eq!(component_id(&doc, "Robot"), Err("A component named robot already exists".into()));
        assert_eq!(new_component(&mut doc, "Sensor Mast"), Ok("sensor_mast".into()));
        assert_eq!(
            new_component(&mut doc, "sensor mast"),
            Err("A component named sensor_mast already exists".into())
        );
        let comp = &doc.components["sensor_mast"];
        assert!(comp.children.is_empty() && comp.note.is_empty());
        assert!(validate(&doc, &b).is_empty());
        assert_eq!(usage_count(&doc, "sensor_mast"), 0);
        assert!(flatten(&doc, "sensor_mast").is_empty());
        let mut errs = vec![];
        let pr = component_props(&doc, &b, "sensor_mast", &mut HashMap::new(), &mut vec![], &mut errs);
        assert!(errs.is_empty(), "{errs:?}");
        assert_eq!((pr.mass, pr.count), (0.0, 0));
        assert!(pr.bbox.is_none() && pr.com == DVec3::ZERO);
        assert_eq!(doc.components["robot"].children.len(), 1, "the robot is untouched");
        // an instance of it in the robot adds nothing, and is valid
        doc.components.get_mut("robot").unwrap().children.push(Instance {
            name: "mast".into(),
            part: None,
            component: Some("sensor_mast".into()),
            pos: [0.0; 3],
            rot: [0.0; 3],
            locked: false,
            color: None,
        });
        assert!(validate(&doc, &b).is_empty());
        let pr = component_props(&doc, &b, "robot", &mut HashMap::new(), &mut vec![], &mut errs);
        assert!(errs.is_empty(), "{errs:?}");
        assert_eq!((pr.mass, pr.count), (10.0, 1));
        assert_eq!(usage_count(&doc, "sensor_mast"), 1);
    }

    #[test]
    fn a_brick_keeps_its_own_lego_colour_and_the_file_names_it() {
        let b = bundle();
        let box10 = Shape::Box {
            size: [10.0, 10.0, 10.0],
            pos: [0.0; 3],
        };
        let mut doc = doc_with(
            vec![("p", part(1.0, box10))],
            vec![
                (
                    "robot",
                    vec![
                        inst("a", "p", [0.0; 3], [0.0; 3]),
                        Instance {
                            name: "sub".into(),
                            part: None,
                            component: Some("sub".into()),
                            pos: [0.0; 3],
                            rot: [0.0; 3],
                            locked: false,
                            color: None,
                        },
                    ],
                ),
                ("sub", vec![inst("b", "p", [0.0; 3], [0.0; 3]), inst("c", "p", [0.0; 3], [0.0; 3])]),
            ],
        );
        // nothing placed in a colour: no leaf carries one, and the file says nothing about it
        assert!(flatten(&doc, "robot").iter().all(|l| l.color.is_none()));
        assert!(!serde_json::to_string(&doc).unwrap().contains("color"));
        doc.components.get_mut("sub").unwrap().children[0].color = Some(72);
        let leaves = flatten(&doc, "robot");
        let of = |name: &str| {
            leaves
                .iter()
                .find(|l| l.path.last().map(String::as_str) == Some(name))
                .unwrap()
                .color
        };
        assert_eq!(of("a"), None);
        assert_eq!(of("b"), Some(72), "the brick placed in dark bluish gray");
        assert_eq!(of("c"), None, "its neighbour in the same component keeps its category colour");
        // the file names the colour by its LDraw id, and reads back the same
        let text = serde_json::to_string(&doc).unwrap();
        assert_eq!(text.matches("\"color\":72").count(), 1, "{text}");
        let back: Document = serde_json::from_str(&text).unwrap();
        assert_eq!(back, doc);
        assert!(validate(&doc, &b).is_empty());
    }

    #[test]
    fn a_saved_build_joins_a_library_and_clashes_come_in_renamed() {
        let box10 = Shape::Box {
            size: [10.0, 10.0, 10.0],
            pos: [0.0; 3],
        };
        let box20 = Shape::Box {
            size: [20.0, 10.0, 10.0],
            pos: [0.0; 3],
        };
        let cinst = |name: &str, comp: &str| Instance {
            name: name.into(),
            part: None,
            component: Some(comp.into()),
            pos: [0.0; 3],
            rot: [0.0; 3],
            locked: false,
            color: None,
        };
        // the library: brick p, component x of one p
        let mut doc = doc_with(
            vec![("p", part(1.0, box10.clone()))],
            vec![("robot", vec![cinst("x", "x")]), ("x", vec![inst("a", "p", [0.0; 3], [0.0; 3])])],
        );
        // the saved build: the same p, a new q, an x that differs, and y made of x and q
        let other = doc_with(
            vec![("p", part(1.0, box10.clone())), ("q", part(2.0, box20.clone()))],
            vec![
                ("robot", vec![cinst("y", "y")]),
                ("x", vec![inst("a", "p", [8.0, 0.0, 0.0], [0.0; 3])]),
                ("y", vec![cinst("x", "x"), inst("b", "q", [0.0; 3], [0.0; 3])]),
            ],
        );
        let got = merge_library(&mut doc, &other);
        assert_eq!(got.parts, vec!["q".to_string()], "p is here already");
        assert_eq!(got.components, vec!["x_2".to_string(), "y".to_string(), "robot_2".to_string()]);
        assert_eq!(
            got.renamed,
            vec![("x".to_string(), "x_2".to_string()), ("robot".to_string(), "robot_2".to_string())]
        );
        assert_eq!(
            doc.components["y"].children[0].component.as_deref(),
            Some("x_2"),
            "y follows the rename"
        );
        assert_eq!(doc.components["x"].children[0].pos, [0.0; 3], "the library's x is untouched");
        assert_eq!(doc.components["robot_2"].children[0].component.as_deref(), Some("y"));
        assert!(validate(&doc, &bundle()).is_empty());
        // the same build again brings nothing: everything compares equal to what is here
        let again = merge_library(&mut doc, &other);
        assert_eq!(
            again,
            Imported {
                components: vec![],
                parts: vec![],
                renamed: vec![("x".into(), "x_2".into()), ("robot".into(), "robot_2".into())]
            }
        );
        assert_eq!(doc.components.len(), 5);
        // a brick that differs under the same id comes in renamed, and its users follow
        let other2 = doc_with(
            vec![("p", part(9.0, box10))],
            vec![("robot", vec![inst("c", "p", [0.0; 3], [0.0; 3])])],
        );
        let got2 = merge_library(&mut doc, &other2);
        assert_eq!(got2.parts, vec!["p_2".to_string()]);
        assert_eq!(doc.components["robot_3"].children[0].part.as_deref(), Some("p_2"));
        assert_eq!(fresh_id("z", |_| false), "z");
    }

    #[test]
    fn a_component_becomes_a_document_of_its_own_and_so_does_a_brick() {
        let bundle_path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../openbricks/openbricks_sim/bricks/technic_bundle.json.zlib");
        let bundle = crate::bundle::load_bundle(&bundle_path).expect("the shipped brick bundle");
        let doc = example();
        let root = doc.robot.root.clone();
        let sub = subset(&doc, &root).unwrap();
        assert_eq!((sub.robot.root.as_str(), sub.robot.name.as_str()), (root.as_str(), root.as_str()));
        assert!(sub.robot.roles.is_empty(), "a prop has no roles");
        assert_eq!(sub.format, doc.format);
        // every part the reachable components use, and nothing else
        let used: std::collections::BTreeSet<String> = flatten(&sub, &root).iter().map(|l| l.part_id.clone()).collect();
        assert_eq!(sub.parts.keys().cloned().collect::<std::collections::BTreeSet<_>>(), used);
        assert_eq!(flatten(&sub, &root).len(), flatten(&doc, &root).len());
        let (some_child, _) = doc
            .components
            .iter()
            .find(|(id, _)| **id != root)
            .map(|(id, c)| (id.clone(), c.clone()))
            .expect("the example has a component besides the root");
        let part = subset(&doc, &some_child).unwrap();
        assert!(part.components.len() < doc.components.len());
        assert!(subset(&doc, "nothing").is_none());
        // a library brick: one part, one component, one instance
        let num = bundle.parts.keys().next().unwrap().clone();
        let b = brick_document(&bundle, &num).unwrap();
        assert_eq!(b.format, "openbricks-assembly/1");
        assert_eq!(b.robot.root, "brick");
        assert_eq!(b.parts.len(), 1);
        assert_eq!(b.parts.values().next().unwrap().ldraw.as_deref(), Some(num.as_str()));
        assert_eq!(flatten(&b, "brick").len(), 1);
        assert!(validate(&b, &bundle).is_empty(), "{:?}", validate(&b, &bundle));
        assert!(brick_document(&bundle, "no-such-part").is_none());
        let text = serde_json::to_string(&b).unwrap();
        let back: Document = serde_json::from_str(&text).unwrap();
        assert_eq!(back.robot.root, "brick");
    }

    #[test]
    fn example_document_is_valid_and_rolls_up() {
        let doc = example();
        assert_eq!(doc.format, FORMAT);
        let b = bundle();
        let errs = validate(&doc, &b);
        assert!(errs.iter().all(|e| e.contains("LDraw part")), "{errs:?}");
    }

    #[test]
    fn a_socket_sits_under_every_stud_along_a_box_axis() {
        let stud = |centre: [f64; 3], axis: [f64; 3]| Connector {
            kind: "stud".into(),
            centre,
            axis,
            length: 1.6,
            r: 2.4,
        };
        let hole = Connector {
            kind: "pin_hole".into(),
            centre: [0.0, 0.0, -4.0],
            axis: [0.0, 1.0, 0.0],
            length: 8.0,
            r: 2.4,
        };
        // a 2 x 2 plate: studs 0.8 mm up its top face, the body 3.2 mm below it
        let bbox = [[-8.0, -8.0, -3.2], [8.0, 8.0, 1.6]];
        let out = with_stud_sockets(
            &[
                stud([4.0, 4.0, 0.8], [0.0, 0.0, -1.0]),
                stud([1.0, 1.0, 0.8], [0.7, 0.0, -0.7]),
                hole.clone(),
            ],
            &bbox,
            "Plate  2 x  2",
        );
        let sockets: Vec<&Connector> = out.iter().filter(|c| c.kind == "stud_socket").collect();
        assert_eq!(sockets.len(), 1, "a stud at an angle gets none");
        assert_eq!(sockets[0].centre, [4.0, 4.0, -2.4]);
        assert_eq!((sockets[0].axis, sockets[0].length, sockets[0].r), ([0.0, 0.0, -1.0], 1.6, 2.4));
        assert_eq!(out.len(), 4, "the rest is kept");
        assert_eq!(out[2].kind, "pin_hole");
        // studs pointing up a part (a SNOT face) get a socket at the far face along that axis
        let out = with_stud_sockets(&[stud([0.0, 0.0, -2.4], [0.0, 0.0, 1.0])], &bbox, "Brick  1 x  1 with Stud on Side");
        assert_eq!(out[1].centre, [0.0, 0.0, 0.8]);
        // a tile: no studs, a socket under every stud place of its footprint; a part with studs,
        // one that is not a tile, or an odd footprint gets none of those
        let tile = [[-8.0, -4.0, -3.2], [8.0, 4.0, 0.0]];
        let out = with_stud_sockets(&[], &tile, "Tile  1 x  2 with Groove");
        assert_eq!(out.len(), 2);
        assert_eq!((out[0].centre, out[1].centre), ([-4.0, 0.0, -2.4], [4.0, 0.0, -2.4]));
        assert_eq!(
            (out[0].kind.as_str(), out[0].axis, out[0].length, out[0].r),
            ("stud_socket", [0.0, 0.0, -1.0], 1.6, 2.4)
        );
        assert_eq!(
            tile_sockets(&[[-8.0, -8.0, -3.2], [8.0, 8.0, 0.0]], "Tile  2 x  2 with Groove").len(),
            4
        );
        assert_eq!(tile_sockets(&tile, "Technic Tile  1 x  2 with Two Holes").len(), 2);
        assert!(tile_sockets(&tile, "Technic Beam  1 x  2").is_empty(), "not a tile");
        assert!(
            tile_sockets(&[[-3.6, -11.6, 0.0], [3.6, 11.6, 8.0]], "Tile of no such size").is_empty(),
            "not whole studs"
        );
        assert!(
            with_stud_sockets(&[stud([4.0, 4.0, 0.8], [0.0, 0.0, -1.0])], &bbox, "Tile with a stud").len() == 2,
            "its own stud rules"
        );
        let a = WorldConnector {
            kind: "stud".into(),
            centre: DVec3::ZERO,
            axis: DVec3::Z,
            length: 1.6,
            path: vec![],
        };
        let b = WorldConnector {
            kind: "stud_socket".into(),
            ..a.clone()
        };
        let c = WorldConnector {
            kind: "axle".into(),
            ..a.clone()
        };
        assert!(compatible(&a, &b) && compatible(&b, &a) && !compatible(&a, &c));
    }

    #[test]
    fn snapping_puts_a_pin_into_a_hole() {
        let mut b = bundle();
        let hole = Connector {
            kind: "pin_hole".into(),
            centre: [0.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            length: 8.0,
            r: 2.4,
        };
        let pin = Connector {
            kind: "pin".into(),
            centre: [0.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            length: 8.0,
            r: 2.4,
        };
        let mesh = crate::bundle::MeshRecord {
            verts: 0,
            tris: 0,
            pos: String::new(),
            nrm: String::new(),
            idx: String::new(),
            idx32: false,
            scale: None,
        };
        let rec = |c: Connector| PartRecord {
            name: "r".into(),
            ldraw: "1".into(),
            mesh: mesh.clone(),
            bbox: [[-4.0; 3], [4.0; 3]],
            volume_mm3: 1.0,
            com: [0.0; 3],
            inertia_per_g: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mass_model: "mesh".into(),
            connectors: vec![c],
            mass_g: 1.0,
            source: String::new(),
            source_note: String::new(),
            density_g_cm3: None,
            sets: BTreeMap::new(),
            aliases: vec![],
            fetched: false,
            colors: BTreeMap::new(),
        };
        b.parts.insert("beam".into(), rec(hole));
        b.parts.insert("pin".into(), rec(pin));
        let ld = |num: &str| Part {
            name: num.into(),
            category: "lego".into(),
            mass_g: 1.0,
            source: String::new(),
            source_note: String::new(),
            ldraw: Some(num.into()),
            shapes: vec![],
            extra: Default::default(),
        };
        let mut doc = doc_with(
            vec![("beam", ld("beam")), ("pin", ld("pin"))],
            vec![(
                "robot",
                vec![
                    inst("beam", "beam", [50.0, 0.0, 0.0], [0.0; 3]),
                    inst("pin", "pin", [51.5, 0.8, 0.9], [2.0, 1.0, 0.0]),
                ],
            )],
        );
        let seats = group_seats(&doc, &b, "robot", &["pin".to_string()], SNAP_MM);
        assert_eq!(seats.len(), 1, "{seats:?}");
        assert_eq!((seats[0].mates, &seats[0].path), (1, &vec!["beam".to_string()]));
        assert!(group_seats(&doc, &b, "robot", &["no such".to_string()], SNAP_MM).is_empty());
        assert!(group_seats(&doc, &b, "robot", &[], SNAP_MM).is_empty());
        assert!(group_seats(&doc, &b, "nowhere", &["pin".to_string()], SNAP_MM).is_empty());
        let mated = snap_group_within(&mut doc, &b, "robot", &["pin".to_string()], SNAP_MM).unwrap();
        assert_eq!(mated, vec!["beam".to_string()]);
        let p = &doc.components["robot"].children[1];
        assert!(
            (p.pos[0] - 50.0).abs() < 0.05 && p.pos[1].abs() < 0.05 && p.pos[2].abs() < 0.05,
            "{:?}",
            p.pos
        );
        assert_eq!(p.rot, [2.0, 1.0, 0.0], "the magnet never turns a part");
        seat_group(&mut doc, "nowhere", &seats[0]);
        assert_eq!(connections_of(&doc, &b, "robot", "pin").len(), 1);
        // an axle keeps its position along a hole
        b.parts.get_mut("pin").unwrap().connectors = vec![Connector {
            kind: "axle".into(),
            centre: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            length: 30.0,
            r: 2.4,
        }];
        let inst_axle = doc.components.get_mut("robot").unwrap().children.get_mut(1).unwrap();
        inst_axle.pos = [50.4, 0.3, 7.0];
        inst_axle.rot = [0.0; 3];
        snap_group_within(&mut doc, &b, "robot", &["pin".to_string()], SNAP_MM).unwrap();
        let p = &doc.components["robot"].children[1];
        assert!((p.pos[2] - 7.0).abs() < 0.05 && (p.pos[0] - 50.0).abs() < 0.05, "{:?}", p.pos);
    }
}
