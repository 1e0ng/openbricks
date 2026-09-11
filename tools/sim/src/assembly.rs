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

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Document {
    pub format: String,
    #[serde(default)]
    pub units: serde_json::Value,
    pub parts: BTreeMap<String, Part>,
    pub components: BTreeMap<String, Component>,
    pub robot: Robot,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
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
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Component {
    #[serde(default)]
    pub note: String,
    #[serde(default)]
    pub children: Vec<Instance>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
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

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
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

fn shift_term(m: f64, d: DVec3) -> DMat3 {
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
        return match bundle.parts.get(num) {
            Some(rec) => Geometry::Record(rec),
            None => Geometry::None,
        };
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

/// Group the named top-level instances of `editing` into a new
/// component `new_id`: the first one's frame becomes the new origin;
/// world poses do not move; roles follow.
pub fn group(doc: &mut Document, editing: &str, names: &[String], new_id: &str) -> Result<String, String> {
    let new_id = slug(new_id);
    if new_id.is_empty() {
        return Err("Give the component a name".into());
    }
    if doc.components.contains_key(&new_id) {
        return Err(format!("A component named {new_id} already exists"));
    }
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
        Geometry::Record(rec) => rec.connectors.clone(),
        Geometry::Imported { connectors, .. } => connectors,
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

fn mates(male: &str) -> &'static [&'static str] {
    match male {
        "pin" => &["pin_hole"],
        "axle" => &["axle_hole", "pin_hole"],
        "stud" => &["stud_hole", "pin_hole"],
        _ => &[],
    }
}

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

/// Move `inst` (top-level in `editing`) so its nearest compatible
/// feature sits in a hole of another instance. Returns the mated path.
pub fn snap_instance(doc: &mut Document, bundle: &Bundle, editing: &str, name: &str) -> Option<Vec<String>> {
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
    if mine.is_empty() || others.is_empty() {
        return None;
    }
    let pairs = mated_pairs(&mine, &others, 6.0, 30.0);
    let best = pairs.first()?;
    let (m, o) = (best.m.clone(), best.o.clone());
    let inst = doc.components.get(editing)?.children.iter().find(|c| c.name == name)?.clone();
    let s = if m.axis.dot(o.axis) >= 0.0 { 1.0 } else { -1.0 };
    let ut = o.axis * s;
    let r0 = rot_mat(inst.rot);
    let cr = m.axis.cross(ut);
    let sn = cr.length();
    let cs = m.axis.dot(ut);
    let dr = if sn > 1e-6 {
        let k = cr / sn;
        let kk = DMat3::from_cols(DVec3::new(0.0, k.z, -k.y), DVec3::new(-k.z, 0.0, k.x), DVec3::new(k.y, -k.x, 0.0));
        DMat3::IDENTITY + kk * sn + kk * kk * (1.0 - cs)
    } else {
        DMat3::IDENTITY
    };
    let r1 = dr * r0;
    let pos = DVec3::from_array(inst.pos);
    let c_local = r0.transpose() * (m.centre - pos);
    let c_now = pos + r1 * c_local;
    let mut t = (c_now - o.centre).dot(ut);
    let slack = (o.length - m.length).abs() / 2.0;
    t = if (o.length - m.length).abs() < 1.0 {
        0.0
    } else {
        t.clamp(-slack, slack)
    };
    let target = o.centre + ut * t;
    let new_pos = (target - r1 * c_local).to_array().map(|v| (v * 100.0).round() / 100.0);
    let comp = doc.components.get_mut(editing)?;
    let inst = comp.children.iter_mut().find(|c| c.name == name)?;
    inst.rot = euler_from(&r1);
    inst.pos = new_pos;
    Some(o.path.clone())
}

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
    mated_pairs(&mine, &others, 0.4, 3.0)
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
            if !bundle.parts.contains_key(num) {
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
        });
        assert!(component_contains(&doc, "robot", "u", &mut vec![]));
        assert!(!component_contains(&doc, "u", "robot", &mut vec![]));
        let sib = vec![inst("pin", "p", [0.0; 3], [0.0; 3]), inst("pin_2", "p", [0.0; 3], [0.0; 3])];
        assert_eq!(unique_name(&sib, "pin"), "pin_3");
        assert_eq!(slug("Drive Unit #2!"), "drive_unit_2");
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
                    inst("pin", "pin", [51.5, 0.8, 0.9], [4.0, 3.0, 0.0]),
                ],
            )],
        );
        let mated = snap_instance(&mut doc, &b, "robot", "pin").unwrap();
        assert_eq!(mated, vec!["beam".to_string()]);
        let p = &doc.components["robot"].children[1];
        assert!(
            (p.pos[0] - 50.0).abs() < 0.05 && p.pos[1].abs() < 0.05 && p.pos[2].abs() < 0.05,
            "{:?}",
            p.pos
        );
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
        snap_instance(&mut doc, &b, "robot", "pin").unwrap();
        let p = &doc.components["robot"].children[1];
        assert!((p.pos[2] - 7.0).abs() < 0.05 && (p.pos[0] - 50.0).abs() < 0.05, "{:?}", p.pos);
    }
}
