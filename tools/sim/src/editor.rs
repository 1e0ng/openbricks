//! The Workbench's editing state, apart from any widget: the document,
//! the component being edited, the selection, undo, snapping and every
//! operation the toolbar, the keys, the inspector and the pointer
//! perform on them. `app.rs` only draws it and feeds it input, so
//! everything here runs under `cargo test`.

use crate::assembly::{self, Component, Document, Instance, Leaf, Part, Props};
use crate::bundle::Bundle;
use crate::gizmo::{self, Handle};
use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};

pub const MODULE_MM: f64 = 8.0;
const UNDO_DEPTH: usize = 60;
pub const CLIPBOARD_FORMAT: &str = "openbricks-clipboard/1";

/// What a copy puts on the clipboard: the instances, the component
/// they were copied from, and every part and component definition
/// they need, so a paste into another file has them too.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Clipboard {
    pub format: String,
    pub from: String,
    pub instances: Vec<Instance>,
    #[serde(default)]
    pub parts: BTreeMap<String, Part>,
    #[serde(default)]
    pub components: BTreeMap<String, Component>,
}

pub struct Editor {
    pub bundle: Bundle,
    pub doc: Document,
    pub path: Option<PathBuf>,
    pub editing: String,
    pub crumbs: Vec<String>,
    pub selection: Vec<String>,
    undo: Vec<Document>,
    pub dirty: bool,
    pub snap_mm: f64,
    pub magnet: bool,
    pub status: String,
    pub errors: Vec<String>,
    /// Set when the view should frame the edited component again.
    pub fit_pending: bool,
    pub memo: HashMap<String, Props>,
    pub root_props: Props,
    pub leaves: Vec<Leaf>,
}

impl Editor {
    pub fn new(bundle: Bundle, doc: Option<(PathBuf, Document)>) -> Self {
        let (path, doc) = match doc {
            Some((p, d)) => (Some(p), d),
            None => (None, assembly::example()),
        };
        let editing = doc.robot.root.clone();
        let mut ed = Editor {
            bundle,
            doc,
            path,
            editing: editing.clone(),
            crumbs: vec![editing],
            selection: vec![],
            undo: vec![],
            dirty: false,
            snap_mm: MODULE_MM,
            magnet: true,
            status: String::new(),
            errors: vec![],
            fit_pending: true,
            memo: HashMap::new(),
            root_props: Props {
                mass: 0.0,
                com: DVec3::ZERO,
                inertia: DMat3::ZERO,
                bbox: None,
                count: 0,
            },
            leaves: vec![],
        };
        ed.errors = assembly::validate(&ed.doc, &ed.bundle);
        ed.recompute();
        ed
    }

    // ------------------------------------------------------------ model

    pub fn recompute(&mut self) {
        self.memo.clear();
        let mut errs = vec![];
        let root = self.doc.robot.root.clone();
        self.root_props = assembly::component_props(&self.doc, &self.bundle, &root, &mut self.memo, &mut vec![], &mut errs);
        if !self.doc.components.contains_key(&self.editing) {
            self.editing = root.clone();
            self.crumbs = vec![root];
            self.selection.clear();
        }
        let editing = self.editing.clone();
        assembly::component_props(&self.doc, &self.bundle, &editing, &mut self.memo, &mut vec![], &mut errs);
        self.leaves = assembly::flatten(&self.doc, &editing);
        let names: HashSet<String> = self.doc.components[&editing].children.iter().map(|c| c.name.clone()).collect();
        self.selection.retain(|n| names.contains(n));
    }

    /// Mass properties of the component being edited.
    pub fn edited_props(&mut self) -> Props {
        let editing = self.editing.clone();
        let mut errs = vec![];
        assembly::component_props(&self.doc, &self.bundle, &editing, &mut self.memo, &mut vec![], &mut errs)
    }

    pub fn push_undo(&mut self) {
        self.undo.push(self.doc.clone());
        if self.undo.len() > UNDO_DEPTH {
            self.undo.remove(0);
        }
        self.dirty = true;
    }

    #[cfg(test)]
    pub fn undo_depth(&self) -> usize {
        self.undo.len()
    }

    pub fn undo(&mut self) {
        if let Some(d) = self.undo.pop() {
            self.doc = d;
            self.recompute();
            self.status = "Undone".into();
        } else {
            self.status = "Nothing to undo".into();
        }
    }

    pub fn is_root(&self) -> bool {
        self.editing == self.doc.robot.root
    }

    pub fn children(&self) -> &[Instance] {
        &self.doc.components[&self.editing].children
    }

    pub fn selected_instances(&self) -> Vec<Instance> {
        let comp = &self.doc.components[&self.editing];
        self.selection
            .iter()
            .filter_map(|n| comp.children.iter().find(|c| &c.name == n).cloned())
            .collect()
    }

    pub fn set_instance(&mut self, name: &str, f: impl FnOnce(&mut Instance)) {
        if let Some(inst) = self
            .doc
            .components
            .get_mut(&self.editing)
            .and_then(|c| c.children.iter_mut().find(|c| c.name == name))
        {
            f(inst);
        }
    }

    pub fn is_locked(&self, name: &str) -> bool {
        self.children().iter().any(|c| c.name == name && c.locked)
    }

    /// The selected instances that may be moved, turned or removed.
    pub fn unlocked_selection(&self) -> Vec<String> {
        self.selection.iter().filter(|n| !self.is_locked(n)).cloned().collect()
    }

    pub fn locked_count(&self) -> usize {
        self.selection.len() - self.unlocked_selection().len()
    }

    /// Lock (or unlock) everything selected: locked instances stay
    /// where they are until unlocked.
    pub fn lock_selection(&mut self, locked: bool) {
        if self.selection.is_empty() {
            self.status = "Select something to lock".into();
            return;
        }
        self.push_undo();
        let n = self.selection.len();
        for name in self.selection.clone() {
            self.set_instance(&name, |i| i.locked = locked);
        }
        self.status = format!("{} {n}", if locked { "Locked" } else { "Unlocked" });
    }

    /// A click on `top`: shift toggles it in the selection, otherwise it
    /// becomes the selection unless it already is part of it.
    pub fn select(&mut self, top: &str, shift: bool) {
        if shift {
            if let Some(i) = self.selection.iter().position(|n| n == top) {
                self.selection.remove(i);
            } else {
                self.selection.push(top.to_string());
            }
        } else if !self.selection.iter().any(|n| n == top) {
            self.selection = vec![top.to_string()];
        }
    }

    pub fn open_component(&mut self, id: &str, push: bool) {
        if !self.doc.components.contains_key(id) {
            return;
        }
        if push {
            self.crumbs.push(id.to_string());
        } else if let Some(i) = self.crumbs.iter().position(|c| c == id) {
            self.crumbs.truncate(i + 1);
        }
        self.editing = id.to_string();
        self.selection.clear();
        self.fit_pending = true;
        self.recompute();
    }

    /// The component instance `name` refers to, if it is one.
    pub fn component_of(&self, name: &str) -> Option<String> {
        self.children().iter().find(|c| c.name == name).and_then(|c| c.component.clone())
    }

    pub fn add_instance(&mut self, part: Option<String>, component: Option<String>, pos: [f64; 3]) {
        if let Some(c) = &component
            && assembly::component_contains(&self.doc, c, &self.editing, &mut vec![])
        {
            self.status = format!("That would put {c} inside itself");
            return;
        }
        self.push_undo();
        let base = part.clone().or(component.clone()).unwrap_or_default();
        let editing = self.editing.clone();
        let comp = self.doc.components.get_mut(&editing).unwrap();
        let name = if comp.children.iter().any(|c| c.name == base) {
            assembly::unique_name(&comp.children, &base)
        } else {
            base
        };
        comp.children.push(Instance {
            name: name.clone(),
            part,
            component,
            pos,
            rot: [0.0; 3],
            locked: false,
        });
        self.selection = vec![name.clone()];
        self.recompute();
        if self.magnet {
            self.snap_selection(false);
        }
        self.status = format!("Added {name} to {editing}");
    }

    /// The document's part for an LDraw number, recorded from the bundle
    /// on first use.
    pub fn ensure_ldraw_part(&mut self, num: &str) -> Option<String> {
        let rec = self.bundle.parts.get(num)?;
        let id = format!("lego_{num}");
        if !self.doc.parts.contains_key(&id) {
            self.doc.parts.insert(
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
        }
        Some(id)
    }

    pub fn remove_selection(&mut self) {
        if self.selection.is_empty() {
            return;
        }
        let kept = self.locked_count();
        let names: HashSet<String> = self.unlocked_selection().into_iter().collect();
        if names.is_empty() {
            self.status = format!("{kept} locked: unlock first");
            return;
        }
        self.push_undo();
        let editing = self.editing.clone();
        let root = self.doc.robot.root.clone();
        let comps = self.doc.components.clone();
        for value in self.doc.robot.roles.values_mut() {
            let segs: Vec<&str> = value.split('/').collect();
            let mut comp = Some(root.clone());
            for seg in segs {
                if comp.as_deref() == Some(editing.as_str()) && names.contains(seg) {
                    value.clear();
                    break;
                }
                comp = comp
                    .as_deref()
                    .and_then(|c| comps.get(c))
                    .and_then(|c| c.children.iter().find(|ch| ch.name == seg))
                    .and_then(|ch| ch.component.clone());
            }
        }
        self.doc
            .components
            .get_mut(&editing)
            .unwrap()
            .children
            .retain(|c| !names.contains(&c.name));
        self.status = format!(
            "Removed {}{}",
            names.len(),
            if kept > 0 { format!(", kept {kept} locked") } else { String::new() }
        );
        self.selection.retain(|n| !names.contains(n));
        self.recompute();
    }

    pub fn duplicate_selection(&mut self) {
        let insts = self.selected_instances();
        if insts.is_empty() {
            return;
        }
        self.push_undo();
        let editing = self.editing.clone();
        let comp = self.doc.components.get_mut(&editing).unwrap();
        let mut made = vec![];
        for inst in insts {
            let mut copy = inst.clone();
            copy.name = assembly::unique_name(&comp.children, &inst.name);
            copy.pos[1] = assembly::round3(copy.pos[1] - 2.0 * MODULE_MM);
            copy.locked = false;
            let idx = comp
                .children
                .iter()
                .position(|c| c.name == inst.name)
                .map(|i| i + 1)
                .unwrap_or(comp.children.len());
            comp.children.insert(idx, copy.clone());
            made.push(copy.name);
        }
        self.selection = made;
        self.recompute();
    }

    /// The unlocked selection, or a status line when everything selected is locked.
    fn movable(&mut self) -> Vec<String> {
        let names = self.unlocked_selection();
        if names.is_empty() && !self.selection.is_empty() {
            self.status = format!("{} locked: unlock first", self.selection.len());
        }
        names
    }

    pub fn rotate_selection(&mut self, axis: usize, deg: f64) {
        let names = self.movable();
        if names.is_empty() {
            return;
        }
        self.push_undo();
        for name in names {
            self.set_instance(&name, |i| i.rot[axis] = wrap_deg(i.rot[axis] + deg));
        }
        self.recompute();
    }

    pub fn nudge_selection(&mut self, d: [f64; 3]) {
        let names = self.movable();
        if names.is_empty() {
            return;
        }
        self.push_undo();
        for name in names {
            self.set_instance(&name, |i| {
                for (k, dk) in d.iter().enumerate() {
                    i.pos[k] = assembly::round3(i.pos[k] + dk);
                }
            });
        }
        self.recompute();
    }

    /// The keyboard's nudge step: one grid step, or 1 mm with shift.
    pub fn nudge_step(&self, shift: bool) -> f64 {
        if shift { 1.0 } else { self.snap_mm.max(1.0) }
    }

    pub fn snap_selection(&mut self, announce: bool) {
        if self.selection.len() != 1 {
            if announce {
                self.status = "Select one item to snap".into();
            }
            return;
        }
        let name = self.selection[0].clone();
        if self.is_locked(&name) {
            if announce {
                self.status = format!("{name} is locked");
            }
            return;
        }
        let before = self.doc.clone();
        match assembly::snap_instance(&mut self.doc, &self.bundle, &self.editing, &name) {
            Some(path) => {
                self.undo.push(before);
                self.dirty = true;
                self.status = format!("Snapped {name} into {}", path.join("/"));
                self.recompute();
            }
            None => {
                if announce {
                    self.status = "No hole within reach".into();
                }
            }
        }
    }

    pub fn group_selection(&mut self, new_id: &str) -> bool {
        let before = self.doc.clone();
        let names = self.selection.clone();
        match assembly::group(&mut self.doc, &self.editing, &names, new_id) {
            Ok(inst) => {
                self.undo.push(before);
                self.dirty = true;
                self.status = format!("{} is now in the library", assembly::slug(new_id));
                self.selection = vec![inst];
                self.recompute();
                true
            }
            Err(e) => {
                self.doc = before;
                self.status = e;
                false
            }
        }
    }

    pub fn ungroup_selection(&mut self) {
        if self.selection.len() != 1 {
            return;
        }
        let before = self.doc.clone();
        let name = self.selection[0].clone();
        match assembly::ungroup(&mut self.doc, &self.editing, &name) {
            Ok(names) => {
                self.undo.push(before);
                self.dirty = true;
                self.status = format!("Ungrouped {name}; the component stays in the library");
                self.selection = names;
                self.recompute();
            }
            Err(e) => {
                self.doc = before;
                self.status = e;
            }
        }
    }

    /// Rename the selected instance; roles that point at it follow.
    pub fn rename_selected(&mut self, new: &str) -> bool {
        let Some(inst) = self.selected_instances().into_iter().next() else {
            return false;
        };
        if new == inst.name {
            return false;
        }
        let siblings = &self.doc.components[&self.editing].children;
        if new.is_empty() || new.contains('/') || siblings.iter().any(|c| c.name == new) {
            self.status = "Names must be unique among siblings and contain no /".into();
            return false;
        }
        self.push_undo();
        let editing = self.editing.clone();
        let old = inst.name.clone();
        let new_name = new.to_string();
        assembly::remap_roles(&mut self.doc, &editing, &|seg| {
            if seg == old { Some(vec![new_name.clone()]) } else { None }
        });
        self.set_instance(&old, |i| i.name = new_name.clone());
        self.selection = vec![new.to_string()];
        self.recompute();
        true
    }

    /// The inspector's pose fields: angles wrapped, positions rounded.
    pub fn set_pose(&mut self, name: &str, pos: [f64; 3], rot: [f64; 3]) {
        if self.is_locked(name) {
            self.status = format!("{name} is locked");
            return;
        }
        self.push_undo();
        self.set_instance(name, |i| {
            i.pos = pos.map(assembly::round3);
            i.rot = rot.map(wrap_deg);
        });
        self.recompute();
    }

    pub fn set_role(&mut self, role: &str, value: String) {
        self.push_undo();
        self.doc.robot.roles.insert(role.to_string(), value);
    }

    /// Instance paths the roles can point at.
    pub fn role_options(&self) -> Vec<String> {
        assembly::flatten(&self.doc, &self.doc.robot.root)
            .iter()
            .map(|l| l.path.join("/"))
            .collect()
    }

    pub fn reset_to_example(&mut self) {
        self.push_undo();
        self.doc = assembly::example();
        self.path = None;
        self.editing = self.doc.robot.root.clone();
        self.crumbs = vec![self.editing.clone()];
        self.selection.clear();
        self.errors = assembly::validate(&self.doc, &self.bundle);
        self.fit_pending = true;
        self.recompute();
    }

    // ------------------------------------------------------- clipboard

    /// The selection as clipboard text, with every definition it needs.
    pub fn copy_selection(&self) -> Option<String> {
        let insts = self.selected_instances();
        if insts.is_empty() {
            return None;
        }
        let mut parts = BTreeMap::new();
        let mut components = BTreeMap::new();
        let mut stack: Vec<String> = Vec::new();
        let mut take = |inst: &Instance, stack: &mut Vec<String>| {
            if let Some(p) = &inst.part
                && let Some(rec) = self.doc.parts.get(p)
            {
                parts.insert(p.clone(), rec.clone());
            }
            if let Some(c) = &inst.component {
                stack.push(c.clone());
            }
        };
        for inst in &insts {
            take(inst, &mut stack);
        }
        while let Some(cid) = stack.pop() {
            if components.contains_key(&cid) {
                continue;
            }
            let Some(comp) = self.doc.components.get(&cid) else { continue };
            components.insert(cid.clone(), comp.clone());
            for ch in &comp.children {
                take(ch, &mut stack);
            }
        }
        let clip = Clipboard {
            format: CLIPBOARD_FORMAT.into(),
            from: self.editing.clone(),
            instances: insts,
            parts,
            components,
        };
        serde_json::to_string(&clip).ok()
    }

    /// Paste clipboard text into the component being edited: the
    /// definitions it carries join the library when missing, the
    /// instances come in unlocked with unique names, two modules over
    /// when pasted back where they were copied from. Returns how many.
    pub fn paste(&mut self, text: &str) -> usize {
        let clip: Clipboard = match serde_json::from_str::<Clipboard>(text) {
            Ok(c) if c.format == CLIPBOARD_FORMAT => c,
            _ => {
                self.status = "The clipboard holds no bricks".into();
                return 0;
            }
        };
        let before = self.doc.clone();
        for (id, p) in clip.parts {
            self.doc.parts.entry(id).or_insert(p);
        }
        for (id, c) in clip.components {
            self.doc.components.entry(id).or_insert(c);
        }
        let same = clip.from == self.editing;
        let editing = self.editing.clone();
        let mut made = Vec::new();
        let mut refused = 0;
        for mut inst in clip.instances {
            if let Some(c) = &inst.component
                && (!self.doc.components.contains_key(c) || assembly::component_contains(&self.doc, c, &editing, &mut vec![]))
            {
                refused += 1;
                continue;
            }
            if let Some(p) = &inst.part
                && !self.doc.parts.contains_key(p)
            {
                refused += 1;
                continue;
            }
            let comp = self.doc.components.get_mut(&editing).unwrap();
            if comp.children.iter().any(|c| c.name == inst.name) {
                inst.name = assembly::unique_name(&comp.children, &inst.name);
            }
            if same {
                inst.pos[1] = assembly::round3(inst.pos[1] - 2.0 * MODULE_MM);
            }
            inst.locked = false;
            made.push(inst.name.clone());
            comp.children.push(inst);
        }
        if made.is_empty() {
            self.doc = before;
            self.status = format!("Nothing pasted: {refused} would put a component inside itself or lack a definition");
            return 0;
        }
        self.undo.push(before);
        self.dirty = true;
        self.selection = made.clone();
        self.recompute();
        self.status = format!(
            "Pasted {}{}",
            made.len(),
            if refused > 0 {
                format!(" ({refused} skipped: would put a component inside itself)")
            } else {
                String::new()
            }
        );
        made.len()
    }

    /// Copy, then remove what is not locked.
    pub fn cut_selection(&mut self) -> Option<String> {
        let text = self.copy_selection()?;
        self.remove_selection();
        Some(text)
    }

    // ----------------------------------------------------------- files

    pub fn load_path(&mut self, p: PathBuf) {
        match std::fs::read_to_string(&p)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str::<Document>(&t).map_err(|e| e.to_string()))
        {
            Ok(doc) => {
                let errs = assembly::validate(&doc, &self.bundle);
                if errs.iter().any(|e| e.starts_with("format") || e.starts_with("robot.root")) {
                    self.status = format!("Not an assembly file: {}", errs.join("; "));
                    return;
                }
                self.errors = errs;
                self.push_undo();
                self.doc = doc;
                self.path = Some(p.clone());
                self.editing = self.doc.robot.root.clone();
                self.crumbs = vec![self.editing.clone()];
                self.selection.clear();
                self.dirty = false;
                self.fit_pending = true;
                self.recompute();
                self.status = format!("Opened {}", p.display());
            }
            Err(e) => self.status = format!("Could not open: {e}"),
        }
    }

    pub fn save_to(&mut self, path: &Path) {
        match serde_json::to_string_pretty(&self.doc)
            .map_err(|e| e.to_string())
            .and_then(|t| std::fs::write(path, t).map_err(|e| e.to_string()))
        {
            Ok(()) => {
                self.path = Some(path.to_path_buf());
                self.dirty = false;
                self.status = format!("Saved {}", path.display());
            }
            Err(e) => self.status = format!("Could not save: {e}"),
        }
    }

    // ----------------------------------------------------------- drags

    /// A grid coordinate: the nearest multiple of the snap, or the
    /// value itself (rounded) when snapping is off.
    pub fn snap(&self, v: f32) -> f64 {
        if self.snap_mm > 0.0 {
            ((v as f64) / self.snap_mm).round() * self.snap_mm
        } else {
            assembly::round3(v as f64)
        }
    }

    /// Start dragging the selection on the ground plane: the starting
    /// positions, after the undo point is recorded.
    pub fn begin_move(&mut self) -> Vec<(String, [f64; 3])> {
        let starts: Vec<(String, [f64; 3])> = self
            .selected_instances()
            .iter()
            .filter(|i| !i.locked)
            .map(|i| (i.name.clone(), i.pos))
            .collect();
        if starts.is_empty() {
            self.movable();
        } else {
            self.push_undo();
        }
        starts
    }

    /// The plane drag: every start moved by `dx, dy`, snapped.
    pub fn move_by(&mut self, starts: &[(String, [f64; 3])], dx: f32, dy: f32) {
        for (name, p0) in starts {
            let nx = self.snap(p0[0] as f32 + dx);
            let ny = self.snap(p0[1] as f32 + dy);
            self.set_instance(name, |i| {
                i.pos[0] = nx;
                i.pos[1] = ny;
            });
        }
        self.recompute();
    }

    /// The shift drag: lift every dragged item by `dz` (unsnapped until
    /// the drag ends).
    pub fn lift_by(&mut self, starts: &[(String, [f64; 3])], dz: f64) {
        for (name, _) in starts {
            self.set_instance(name, |i| i.pos[2] = assembly::round3(i.pos[2] + dz));
        }
        self.recompute();
    }

    /// The plane drag let go: heights land on the grid, then the magnet.
    pub fn end_move(&mut self) {
        if self.snap_mm > 0.0 {
            for name in self.unlocked_selection() {
                let s = self.snap_mm;
                self.set_instance(&name, |i| i.pos[2] = (i.pos[2] / s).round() * s);
            }
        }
        self.recompute();
        if self.magnet {
            self.snap_selection(false);
        }
    }

    /// Start a handle drag: the starting poses, after the undo point.
    pub fn begin_handle(&mut self) -> Vec<(String, [f64; 3], [f64; 3])> {
        let starts: Vec<(String, [f64; 3], [f64; 3])> = self
            .selected_instances()
            .iter()
            .filter(|i| !i.locked)
            .map(|i| (i.name.clone(), i.pos, i.rot))
            .collect();
        if starts.is_empty() {
            self.movable();
        } else {
            self.push_undo();
        }
        starts
    }

    /// A handle drag at `delta` (mm along an arrow, degrees around a
    /// ring) from where it started; `free` skips the snapping.
    pub fn drag_handle(&mut self, handle: Handle, delta: f32, pivot: DVec3, starts: &[(String, [f64; 3], [f64; 3])], free: bool) {
        match handle {
            Handle::Axis(i) => {
                let snap = if free { 0.0 } else { self.snap_mm };
                for (name, p0, _) in starts {
                    let np = gizmo::moved(*p0, i, delta as f64, snap);
                    self.set_instance(name, |inst| inst.pos = np);
                }
            }
            Handle::Ring(i) => {
                let deg = gizmo::snap_angle(delta, !free && self.snap_mm > 0.0);
                for (name, p0, r0) in starts {
                    let (np, nr) = gizmo::rotated(*p0, *r0, pivot, i, deg);
                    self.set_instance(name, |inst| {
                        inst.pos = np;
                        inst.rot = nr;
                    });
                }
            }
        }
        self.recompute();
    }

    /// A handle drag let go: the magnet has a look.
    pub fn end_handle(&mut self) {
        self.recompute();
        if self.magnet {
            self.snap_selection(false);
        }
    }
}

/// An angle wrapped into [-180, 180).
pub fn wrap_deg(a: f64) -> f64 {
    ((a + 180.0).rem_euclid(360.0)) - 180.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle;

    /// The bundle that ships in the wheel.
    pub fn real_bundle() -> Bundle {
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../openbricks/openbricks_sim/bricks/technic_bundle.json.zlib");
        bundle::load_bundle(&p).expect("the shipped brick bundle")
    }

    fn editor() -> Editor {
        Editor::new(real_bundle(), None)
    }

    #[test]
    fn opens_on_the_example_robot() {
        let ed = editor();
        assert!(ed.is_root());
        assert_eq!(ed.crumbs, vec![ed.doc.robot.root.clone()]);
        assert!(ed.errors.is_empty(), "{:?}", ed.errors);
        assert!(ed.root_props.mass > 300.0 && ed.root_props.count > 10, "{:?}", ed.root_props.mass);
        assert_eq!(ed.leaves.len(), ed.root_props.count);
        assert!(ed.fit_pending && !ed.dirty && ed.path.is_none());
    }

    #[test]
    fn adds_a_technic_beam_and_snaps_it() {
        let mut ed = editor();
        let n = ed.children().len();
        let id = ed.ensure_ldraw_part("32278").unwrap();
        assert_eq!(id, "lego_32278");
        assert_eq!(ed.doc.parts[&id].ldraw.as_deref(), Some("32278"));
        assert_eq!(ed.ensure_ldraw_part("nope"), None);
        ed.add_instance(Some(id.clone()), None, [0.0; 3]);
        assert_eq!(ed.children().len(), n + 1);
        assert_eq!(ed.selection, vec!["lego_32278".to_string()]);
        assert!(
            ed.status.starts_with("Added lego_32278") || ed.status.starts_with("Snapped lego_32278"),
            "{}",
            ed.status
        );
        assert!(ed.dirty);
        // a second one gets a unique name
        ed.add_instance(Some(id), None, [100.0, 0.0, 0.0]);
        assert_eq!(ed.children().len(), n + 2);
        assert_ne!(ed.selection[0], "lego_32278");
        assert_eq!(ed.undo_depth(), 2 + usize::from(ed.status.starts_with("Snapped")));
    }

    #[test]
    fn a_component_cannot_contain_itself() {
        let mut ed = editor();
        let comp = ed.doc.components.keys().find(|k| **k != ed.doc.robot.root).unwrap().clone();
        ed.open_component(&comp, true);
        assert_eq!(ed.crumbs.len(), 2);
        let before = ed.children().len();
        ed.add_instance(None, Some(comp.clone()), [0.0; 3]);
        assert_eq!(ed.children().len(), before);
        assert!(ed.status.contains("inside itself"), "{}", ed.status);
        // the crumbs truncate when an earlier one is opened
        let root = ed.doc.robot.root.clone();
        ed.open_component(&root, false);
        assert_eq!(ed.crumbs, vec![root]);
        ed.open_component("no-such-component", true);
        assert!(ed.is_root());
    }

    #[test]
    fn selection_clicks_toggle_with_shift() {
        let mut ed = editor();
        let names: Vec<String> = ed.children().iter().map(|c| c.name.clone()).collect();
        ed.select(&names[0], false);
        assert_eq!(ed.selection, vec![names[0].clone()]);
        ed.select(&names[1], true);
        assert_eq!(ed.selection, vec![names[0].clone(), names[1].clone()]);
        ed.select(&names[0], true);
        assert_eq!(ed.selection, vec![names[1].clone()]);
        ed.select(&names[1], false);
        assert_eq!(ed.selection, vec![names[1].clone()]);
        ed.select(&names[2], false);
        assert_eq!(ed.selection, vec![names[2].clone()]);
    }

    #[test]
    fn removing_an_instance_clears_the_roles_that_point_at_it() {
        let mut ed = editor();
        let role_path = ed.doc.robot.roles.get("wheel_left").cloned().unwrap();
        let top = role_path.split('/').next().unwrap().to_string();
        ed.selection = vec![top.clone()];
        ed.remove_selection();
        assert!(ed.children().iter().all(|c| c.name != top));
        assert_eq!(ed.doc.robot.roles["wheel_left"], "");
        assert_eq!(ed.status, "Removed 1");
        assert!(ed.selection.is_empty());
        ed.undo();
        assert_eq!(ed.doc.robot.roles["wheel_left"], role_path);
        assert_eq!(ed.status, "Undone");
        ed.remove_selection(); // nothing selected: a no-op
        assert_eq!(ed.status, "Undone");
    }

    #[test]
    fn duplicates_sit_two_modules_over() {
        let mut ed = editor();
        let first = ed.children()[0].clone();
        ed.selection = vec![first.name.clone()];
        ed.duplicate_selection();
        assert_eq!(ed.selection.len(), 1);
        let copy = ed.selected_instances()[0].clone();
        assert_ne!(copy.name, first.name);
        assert_eq!(copy.pos[1], assembly::round3(first.pos[1] - 16.0));
        assert_eq!(ed.children()[1].name, copy.name);
        ed.selection.clear();
        let n = ed.children().len();
        ed.duplicate_selection();
        assert_eq!(ed.children().len(), n);
    }

    #[test]
    fn rotate_nudge_and_pose_wrap_and_round() {
        let mut ed = editor();
        let name = ed.children()[0].name.clone();
        ed.selection = vec![name.clone()];
        let rot0 = ed.selected_instances()[0].rot;
        ed.rotate_selection(2, 90.0);
        ed.rotate_selection(2, 90.0);
        ed.rotate_selection(2, 90.0);
        let r = ed.selected_instances()[0].rot[2];
        assert_eq!(r, wrap_deg(rot0[2] + 270.0));
        assert!((-180.0..180.0).contains(&r));
        let pos0 = ed.selected_instances()[0].pos;
        ed.nudge_selection([8.0, 0.0, 0.0]);
        ed.nudge_selection([0.0, -1.0, 0.5]);
        let p = ed.selected_instances()[0].pos;
        assert_eq!(p, [pos0[0] + 8.0, pos0[1] - 1.0, pos0[2] + 0.5]);
        assert_eq!(ed.nudge_step(false), 8.0);
        assert_eq!(ed.nudge_step(true), 1.0);
        ed.snap_mm = 0.0;
        assert_eq!(ed.nudge_step(false), 1.0);
        ed.set_pose(&name, [1.00049, 2.0, 3.0], [370.0, -190.0, 180.0]);
        let i = ed.selected_instances()[0].clone();
        assert_eq!(i.pos, [1.0, 2.0, 3.0]);
        assert_eq!(i.rot, [10.0, 170.0, -180.0]);
        assert_eq!(wrap_deg(180.0), -180.0);
        assert_eq!(wrap_deg(-180.0), -180.0);
    }

    #[test]
    fn snapping_needs_one_selected_item() {
        let mut ed = editor();
        ed.snap_selection(true);
        assert_eq!(ed.status, "Select one item to snap");
        let names: Vec<String> = ed.children().iter().take(2).map(|c| c.name.clone()).collect();
        ed.selection = names.clone();
        ed.snap_selection(true);
        assert_eq!(ed.status, "Select one item to snap");
        // a brick far from everything finds no hole
        let id = ed.ensure_ldraw_part("32278").unwrap();
        ed.magnet = false;
        ed.add_instance(Some(id), None, [5000.0, 5000.0, 5000.0]);
        ed.snap_selection(true);
        assert_eq!(ed.status, "No hole within reach");
        ed.snap_selection(false);
        assert_eq!(ed.status, "No hole within reach");
    }

    #[test]
    fn groups_and_ungroups_through_the_editor() {
        let mut ed = editor();
        let names: Vec<String> = ed.children().iter().take(2).map(|c| c.name.clone()).collect();
        ed.selection = names.clone();
        assert!(!ed.group_selection(""));
        assert_eq!(ed.status, "Give the component a name");
        assert!(!ed.group_selection("drive_unit"), "the example already has one");
        assert!(ed.status.contains("already exists"), "{}", ed.status);
        assert!(ed.group_selection("Sensor Mast"), "{}", ed.status);
        assert!(ed.doc.components.contains_key("sensor_mast"));
        assert_eq!(ed.status, "sensor_mast is now in the library");
        assert_eq!(ed.selection.len(), 1);
        assert_eq!(ed.component_of(&ed.selection[0].clone()), Some("sensor_mast".into()));
        ed.ungroup_selection();
        assert_eq!(ed.selection, names);
        assert!(ed.status.starts_with("Ungrouped"));
        assert!(ed.doc.components.contains_key("sensor_mast"));
        ed.selection.clear();
        ed.ungroup_selection();
        assert!(ed.status.starts_with("Ungrouped"));
    }

    #[test]
    fn renames_follow_the_roles_and_reject_duplicates() {
        let mut ed = editor();
        let role_path = ed.doc.robot.roles.get("wheel_left").cloned().unwrap();
        let mut segs: Vec<String> = role_path.split('/').map(String::from).collect();
        let top = segs[0].clone();
        let other = ed.children().iter().find(|c| c.name != top).unwrap().name.clone();
        ed.selection = vec![top.clone()];
        assert!(!ed.rename_selected(&other));
        assert!(ed.status.starts_with("Names must be unique"));
        assert!(!ed.rename_selected("a/b"));
        assert!(!ed.rename_selected(&top));
        assert!(ed.rename_selected("left_side"));
        segs[0] = "left_side".into();
        assert_eq!(ed.doc.robot.roles["wheel_left"], segs.join("/"));
        assert_eq!(ed.selection, vec!["left_side".to_string()]);
        ed.selection.clear();
        assert!(!ed.rename_selected("x"));
        ed.set_role("imu", "left_side".into());
        assert_eq!(ed.doc.robot.roles["imu"], "left_side");
        assert!(ed.role_options().contains(&"left_side".to_string()) || ed.role_options().iter().any(|o| o.starts_with("left_side/")));
    }

    #[test]
    fn undo_is_bounded_and_the_example_comes_back() {
        let mut ed = editor();
        let name = ed.children()[0].name.clone();
        ed.selection = vec![name];
        for _ in 0..70 {
            ed.nudge_selection([1.0, 0.0, 0.0]);
        }
        assert_eq!(ed.undo_depth(), UNDO_DEPTH);
        for _ in 0..UNDO_DEPTH {
            ed.undo();
        }
        assert_eq!(ed.status, "Undone");
        ed.undo();
        assert_eq!(ed.status, "Nothing to undo");
        ed.doc.robot.name = "changed".into();
        ed.reset_to_example();
        assert_eq!(ed.doc.robot.name, assembly::example().robot.name);
        assert!(ed.fit_pending && ed.path.is_none());
    }

    #[test]
    fn files_round_trip_and_bad_files_are_refused() {
        let mut ed = editor();
        let dir = std::env::temp_dir().join(format!("ob-editor-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robot.assembly.json");
        ed.dirty = true;
        ed.save_to(&path);
        assert!(!ed.dirty);
        assert_eq!(ed.path.as_deref(), Some(path.as_path()));
        assert!(ed.status.starts_with("Saved"));
        let mut other = editor();
        other.load_path(path.clone());
        assert!(other.status.starts_with("Opened"), "{}", other.status);
        assert_eq!(other.doc.robot.name, ed.doc.robot.name);
        assert!(!other.dirty && other.fit_pending);
        other.load_path(dir.join("missing.json"));
        assert!(other.status.starts_with("Could not open"), "{}", other.status);
        std::fs::write(
            dir.join("bad.json"),
            r#"{"format": "something-else/9", "parts": {}, "components": {"x": {}}, "robot": {"root": "x"}}"#,
        )
        .unwrap();
        other.load_path(dir.join("bad.json"));
        assert!(other.status.starts_with("Not an assembly file"), "{}", other.status);
        std::fs::write(dir.join("garbage.json"), "not json").unwrap();
        other.load_path(dir.join("garbage.json"));
        assert!(other.status.starts_with("Could not open"), "{}", other.status);
        ed.save_to(&dir.join("no-such-dir").join("x.json"));
        assert!(ed.status.starts_with("Could not save"), "{}", ed.status);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn locked_instances_stay_put_until_unlocked() {
        let mut ed = editor();
        ed.magnet = false;
        let id = ed.ensure_ldraw_part("32278").unwrap();
        ed.add_instance(Some(id.clone()), None, [0.0; 3]);
        let a = ed.selection[0].clone();
        ed.add_instance(Some(id), None, [40.0, 0.0, 0.0]);
        let b = ed.selection[0].clone();
        ed.selection = vec![a.clone()];
        ed.lock_selection(true);
        assert_eq!(ed.status, "Locked 1");
        assert!(ed.is_locked(&a) && !ed.is_locked(&b));
        // nothing moves it: keys, inspector, drags, snap
        ed.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(ed.status, "1 locked: unlock first");
        ed.rotate_selection(2, 90.0);
        ed.set_pose(&a, [1.0, 1.0, 1.0], [0.0; 3]);
        assert_eq!(ed.status, format!("{a} is locked"));
        ed.snap_selection(true);
        assert_eq!(ed.status, format!("{a} is locked"));
        assert!(ed.begin_move().is_empty());
        assert!(ed.begin_handle().is_empty());
        assert_eq!(ed.selected_instances()[0].pos, [0.0; 3]);
        assert_eq!(ed.selected_instances()[0].rot, [0.0; 3]);
        // removing a mixed selection keeps the locked one
        ed.selection = vec![a.clone(), b.clone()];
        let starts = ed.begin_move();
        assert_eq!(starts.len(), 1);
        assert_eq!(starts[0].0, b);
        ed.remove_selection();
        assert_eq!(ed.status, "Removed 1, kept 1 locked");
        assert_eq!(ed.selection, vec![a.clone()]);
        ed.remove_selection();
        assert_eq!(ed.status, "1 locked: unlock first");
        // a duplicate of a locked item is free to move
        ed.duplicate_selection();
        assert!(!ed.selected_instances()[0].locked);
        // unlock, and it moves again
        ed.selection = vec![a.clone()];
        ed.lock_selection(false);
        assert_eq!(ed.status, "Unlocked 1");
        ed.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(ed.selected_instances()[0].pos, [8.0, 0.0, 0.0]);
        ed.selection.clear();
        ed.lock_selection(true);
        assert_eq!(ed.status, "Select something to lock");
        // the flag is saved and read back, and left out when false
        let json = serde_json::to_string(&ed.doc).unwrap();
        assert!(!json.contains("\"locked\":false"));
        ed.lock_selection(true);
        ed.selection = vec![a.clone()];
        ed.lock_selection(true);
        let json = serde_json::to_string(&ed.doc).unwrap();
        let back: Document = serde_json::from_str(&json).unwrap();
        assert!(back.components[&ed.editing].children.iter().any(|c| c.name == a && c.locked));
    }

    #[test]
    fn copy_and_paste_carry_the_definitions_along() {
        let mut ed = editor();
        assert!(ed.copy_selection().is_none());
        ed.paste("not ours");
        assert_eq!(ed.status, "The clipboard holds no bricks");
        ed.paste(r#"{"format": "openbricks-clipboard/2", "from": "x", "instances": []}"#);
        assert_eq!(ed.status, "The clipboard holds no bricks");
        // copy a brick and a component instance from the robot
        let brick = ed.children().iter().find(|c| c.part.is_some()).unwrap().name.clone();
        let comp_inst = ed.children().iter().find(|c| c.component.is_some()).unwrap().clone();
        ed.selection = vec![brick.clone(), comp_inst.name.clone()];
        ed.lock_selection(true);
        let text = ed.copy_selection().unwrap();
        let clip: Clipboard = serde_json::from_str(&text).unwrap();
        assert_eq!(clip.from, ed.doc.robot.root);
        assert_eq!(clip.instances.len(), 2);
        assert!(clip.components.contains_key(comp_inst.component.as_ref().unwrap()));
        assert!(!clip.parts.is_empty());
        // pasted back where they came from: two modules over, unlocked, renamed
        let n = ed.children().len();
        assert_eq!(ed.paste(&text), 2);
        assert_eq!(ed.children().len(), n + 2);
        assert_eq!(ed.selection.len(), 2);
        let pasted = ed.selected_instances();
        assert!(pasted.iter().all(|i| !i.locked));
        assert!(pasted.iter().all(|i| i.name != brick && i.name != comp_inst.name));
        let original = ed.children().iter().find(|c| c.name == brick).unwrap().pos;
        assert_eq!(pasted[0].pos[1], assembly::round3(original[1] - 16.0));
        assert!(ed.status.starts_with("Pasted 2"), "{}", ed.status);
        // into another component: positions stay
        let cid = comp_inst.component.clone().unwrap();
        ed.selection = vec![brick.clone()];
        let brick_text = ed.copy_selection().unwrap();
        ed.open_component(&cid, true);
        let n = ed.children().len();
        assert_eq!(ed.paste(&brick_text), 1);
        assert_eq!(ed.children().len(), n + 1);
        assert_eq!(ed.children()[n].pos, original);
        assert_eq!(ed.children()[n].name, brick);
        let root = ed.doc.robot.root.clone();
        ed.open_component(&root, false);
        // into a fresh document with the same root id: the definitions come along
        let mut other = Editor::new(real_bundle(), None);
        other.doc.parts.clear();
        other.doc.components.retain(|k, _| *k == other.doc.robot.root);
        other.doc.components.get_mut(&other.doc.robot.root).unwrap().children.clear();
        other.doc.robot.roles.clear();
        other.recompute();
        assert_eq!(other.paste(&text), 2);
        assert!(other.doc.components.contains_key(&cid));
        assert!(!other.doc.parts.is_empty());
        assert_eq!(other.children()[0].name, brick);
        assert_eq!(
            other.children()[0].pos[1],
            assembly::round3(original[1] - 16.0),
            "same component id: two modules over"
        );
        // a component cannot be pasted into itself
        ed.selection = vec![comp_inst.name.clone()];
        let text = ed.copy_selection().unwrap();
        ed.open_component(&cid, true);
        assert_eq!(ed.paste(&text), 0);
        assert!(ed.status.starts_with("Nothing pasted: 1"), "{}", ed.status);
        // cut: copied, then removed (locked ones stay)
        let root = ed.doc.robot.root.clone();
        ed.open_component(&root, false);
        ed.selection = vec![brick.clone(), comp_inst.name.clone()];
        ed.lock_selection(false);
        ed.selection = vec![brick.clone()];
        let n = ed.children().len();
        let cut = ed.cut_selection().unwrap();
        assert!(cut.contains(&brick));
        assert_eq!(ed.children().len(), n - 1);
        assert!(ed.cut_selection().is_none());
    }

    #[test]
    fn plane_drags_snap_and_the_lift_lands_on_the_grid() {
        let mut ed = editor();
        ed.magnet = false;
        let id = ed.ensure_ldraw_part("32278").unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        let starts = ed.begin_move();
        assert_eq!(starts.len(), 1);
        ed.move_by(&starts, 11.0, -3.0);
        assert_eq!(ed.selected_instances()[0].pos, [8.0, 0.0, 0.0]);
        ed.lift_by(&starts, 5.2);
        assert_eq!(ed.selected_instances()[0].pos[2], 5.2);
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos, [8.0, 0.0, 8.0]);
        ed.snap_mm = 0.0;
        ed.move_by(&starts, 1.2345, 0.0);
        assert_eq!(ed.selected_instances()[0].pos[0], 1.235);
        ed.lift_by(&starts, 0.3);
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos[2], 8.3);
        assert_eq!(ed.snap(3.9), 3.9);
    }

    #[test]
    fn handle_drags_move_and_turn_about_the_pivot() {
        let mut ed = editor();
        ed.magnet = false;
        let id = ed.ensure_ldraw_part("32278").unwrap();
        ed.add_instance(Some(id.clone()), None, [0.0; 3]);
        let a = ed.selection[0].clone();
        ed.add_instance(Some(id), None, [16.0, 0.0, 0.0]);
        let b = ed.selection[0].clone();
        ed.selection = vec![a.clone(), b.clone()];
        let starts = ed.begin_handle();
        assert_eq!(starts.len(), 2);
        ed.drag_handle(Handle::Axis(2), 13.0, DVec3::ZERO, &starts, false);
        let insts = ed.selected_instances();
        assert_eq!(insts[0].pos, [0.0, 0.0, 16.0]);
        assert_eq!(insts[1].pos, [16.0, 0.0, 16.0]);
        ed.drag_handle(Handle::Axis(2), 13.0, DVec3::ZERO, &starts, true);
        assert_eq!(ed.selected_instances()[0].pos[2], 13.0);
        ed.drag_handle(Handle::Ring(2), 85.0, DVec3::ZERO, &starts, false);
        let insts = ed.selected_instances();
        assert_eq!(insts[0].rot, [0.0, 0.0, 90.0]);
        assert_eq!(insts[1].pos, [0.0, 16.0, 0.0]);
        ed.drag_handle(Handle::Ring(2), 85.0, DVec3::ZERO, &starts, true);
        assert_eq!(ed.selected_instances()[0].rot[2], 85.0);
        ed.end_handle();
        assert_eq!(ed.selected_instances().len(), 2);
    }
}
