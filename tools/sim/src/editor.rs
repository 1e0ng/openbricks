//! The Workbench's editing state, apart from any widget: the document,
//! the component being edited, the selection, undo, snapping and every
//! operation the toolbar, the keys, the inspector and the pointer
//! perform on them. `app.rs` only draws it and feeds it input, so
//! everything here runs under `cargo test`.

use crate::assembly::{self, Component, Document, Geometry, Instance, Leaf, Part, Props};
use crate::bundle::{Bundle, MeshData};
use crate::drafts;
use crate::geometry;
use crate::gizmo::{self, Handle};
use crate::overlap::{self, Overlap, Pose, Shape};
use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::rc::Rc;

pub const MODULE_MM: f64 = 8.0;
/// The drag grid: half a stud. Parts an odd number of studs wide have
/// their centres on stud centres, even ones between studs, so a 1 x 2
/// on a 2 x 2 needs a half-stud step; the toolbar offers 8, 4, 1 or off.
pub const GRID_MM: f64 = 4.0;
/// The height module: a stud's height, which every LEGO height stacks
/// on — a plate is two, a beam five, a brick six. Heights let go land on
/// it, not on the 8 mm plan grid (which would sink a plate into the one
/// below, or a brick 1.6 mm into its base).
pub const HEIGHT_MM: f64 = 1.6;
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
    /// What went wrong on the way in, listed with the errors but kept
    /// across them: a fetched part's file that would not read, or one
    /// the library ships a better record for. Validation never clears it.
    pub notes: Vec<String>,
    /// Set when the view should frame the edited component again.
    pub fit_pending: bool,
    /// Counts recomputes: anything derived from the document (thumbnails) keys on it.
    pub edits: u64,
    pub memo: HashMap<String, Props>,
    pub root_props: Props,
    pub leaves: Vec<Leaf>,
    /// Where the build's draft is kept while it is unsaved.
    pub draft_dir: PathBuf,
    pub keeper: drafts::Keeper,
    /// Each part's collision shape, built on first use.
    shapes: HashMap<String, Rc<Shape>>,
    /// The change in progress, if any: what to go back to should it
    /// make new overlaps.
    pending: Option<Pending>,
    /// Whether the build was unsaved when the last undo point was taken.
    dirty_before: bool,
    /// Whether the status line holds a refusal (cleared by the next change kept).
    refused: bool,
    /// The new overlaps the change in progress would make: the bricks to
    /// tint and the note for the status line.
    /// What the change in progress would overlap: the moved instance, the
    /// other, how, and the other's leaf (a brick inside a component by
    /// its path) for the status line.
    pub overlapping: Vec<(String, String, Overlap, String)>,
}

/// A leaf's index, collision shape, pose and world box, for the overlap test.
type Placed = (usize, Rc<Shape>, Pose, (DVec3, DVec3));

/// A pair of instances that overlap, how, and how much their boxes
/// share (mm³): the measure of whether an old overlap is made worse.
/// An overlap: the two top-level instances (the moved one first when
/// one was), how, the boxes' shared volume, and the second's leaf path.
type Found = (String, String, Overlap, f64, String);

/// A change of some instances in progress: the document to go back to
/// should it make new overlaps, the undo depth once its own point was
/// taken, whether the build was unsaved before it, and the overlaps the
/// instances had already (with how much their boxes shared), so that
/// only new overlaps — or old ones made worse — count against it, and
/// a file that already holds some can still be edited.
struct Pending {
    names: Vec<String>,
    doc: Document,
    dirty: bool,
    undo_len: usize,
    before: BTreeMap<(String, String), f64>,
}

/// Where new instances ended up when moved clear of the others.
enum Room {
    /// Where they were put: nothing was in the way.
    Clear,
    /// Moved along until nothing overlapped.
    Moved,
    /// Still overlapping this instance after 400 steps (3.2 m).
    Stuck(String),
}

/// The key the viewport and the overlap test file a part's geometry under.
pub fn mesh_key(part_id: &str, part: &Part) -> String {
    match &part.ldraw {
        Some(n) => format!("ld:{n}"),
        None => format!("part:{part_id}"),
    }
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
            snap_mm: GRID_MM,
            magnet: true,
            status: String::new(),
            errors: vec![],
            notes: vec![],
            fit_pending: true,
            edits: 0,
            memo: HashMap::new(),
            root_props: Props {
                mass: 0.0,
                com: DVec3::ZERO,
                inertia: DMat3::ZERO,
                bbox: None,
                count: 0,
            },
            leaves: vec![],
            draft_dir: drafts::dir(),
            keeper: drafts::Keeper::default(),
            shapes: HashMap::new(),
            pending: None,
            dirty_before: false,
            refused: false,
            overlapping: vec![],
        };
        ed.errors = assembly::validate(&ed.doc, &ed.bundle);
        ed.recompute();
        ed
    }

    // ---------------------------------------------------------- drafts

    /// Called every frame: a draft of the build is kept once a run of
    /// changes has settled, while there is anything unsaved.
    pub fn autosave(&mut self, now: std::time::Instant, now_ms: i64) {
        if self.dirty {
            self.keeper.changed(self.edits, now);
        }
        if self.keeper.due(now) {
            self.keep_draft(now_ms);
            self.keeper.kept(self.edits);
        }
    }

    /// The build kept as a draft, with where it belongs.
    pub fn keep_draft(&mut self, now_ms: i64) {
        let note = drafts::Note {
            path: self.path.clone(),
            kept_ms: now_ms,
        };
        if let Err(e) = serde_json::to_string_pretty(&self.doc)
            .map_err(|e| e.to_string())
            .and_then(|t| drafts::keep(&self.draft_dir, drafts::BUILD, &t, &note))
        {
            self.status = format!("Could not keep a draft: {e}");
        }
    }

    /// The draft goes: the build is saved, or a file took its place.
    pub fn drop_draft(&mut self) {
        drafts::drop(&self.draft_dir, drafts::BUILD);
        self.keeper.kept(self.edits);
    }

    /// A draft an earlier session kept comes back as the build, unsaved,
    /// belonging where it did. Nothing happens without one; a draft that
    /// is not an assembly is left and said so.
    pub fn restore_draft(&mut self, now_ms: i64) -> bool {
        let Some((text, note)) = drafts::take(&self.draft_dir, drafts::BUILD) else {
            return false;
        };
        let at = self.draft_dir.join(drafts::BUILD);
        let doc: Document = match serde_json::from_str(&text) {
            Ok(d) => d,
            Err(e) => {
                self.status = format!("Ignored an unreadable draft at {}: {e}", at.display());
                return false;
            }
        };
        let errs = assembly::validate(&doc, &self.bundle);
        if errs.iter().any(|e| e.starts_with("format") || e.starts_with("robot.root")) {
            self.status = format!("Ignored a draft that is not an assembly at {}: {}", at.display(), errs.join("; "));
            return false;
        }
        self.errors = errs;
        self.doc = doc;
        self.shapes.clear();
        self.pending = None;
        self.path = note.path.clone();
        self.editing = self.doc.robot.root.clone();
        self.crumbs = vec![self.editing.clone()];
        self.selection.clear();
        self.undo.clear();
        self.dirty = true;
        self.fit_pending = true;
        self.recompute();
        self.keeper.kept(self.edits);
        self.status = format!(
            "Restored the unsaved draft kept {}{}{}",
            drafts::ago(note.kept_ms, now_ms),
            match &note.path {
                Some(p) => format!(" of {}: Save writes it there", p.display()),
                None => ": Save as… gives it a file".to_string(),
            },
            self.overlap_report()
        );
        true
    }

    // ---------------------------------------------------------- overlap

    /// A part's mesh: the bundle's record, an imported mesh, or its shapes.
    pub fn mesh_of(&self, part: &Part) -> Option<MeshData> {
        match assembly::geometry_of(part, &self.bundle) {
            Geometry::Record(rec) => rec.mesh.decode().ok(),
            Geometry::Imported { .. } => {
                let m: crate::bundle::MeshRecord = serde_json::from_value(part.extra.get("mesh")?.clone()).ok()?;
                m.decode().ok()
            }
            Geometry::Shapes => {
                let mut out = MeshData::default();
                for s in &part.shapes {
                    let m = geometry::shape_mesh(s);
                    let base = out.positions.len() as u32;
                    out.positions.extend(m.positions);
                    out.normals.extend(m.normals);
                    out.indices.extend(m.indices.iter().map(|i| i + base));
                }
                Some(out)
            }
            Geometry::None => None,
        }
    }

    fn shape_of(&mut self, part_id: &str) -> Option<Rc<Shape>> {
        let part = self.doc.parts.get(part_id)?;
        let key = mesh_key(part_id, part);
        if let Some(s) = self.shapes.get(&key) {
            return Some(s.clone());
        }
        let shape = Rc::new(Shape::from_mesh(&self.mesh_of(part)?));
        self.shapes.insert(key, shape.clone());
        Some(shape)
    }

    /// The pairs of instances of the edited component whose bricks
    /// overlap, each with its worst overlap and how much their boxes
    /// share: every pair with one of `moved` in it (both may be: what
    /// turns together may turn into each other), or every pair when
    /// `moved` is None. Two bricks with a feature seated in the other's
    /// (a pin all the way in its hole, a plate down on its studs —
    /// within the inspector's 0.4 mm) are a joint, whatever LDraw's
    /// parts do to each other there (a friction pin's lip is wider than
    /// its hole, a frame's countersink shallower than a collar), and are
    /// never an overlap.
    fn overlaps(&mut self, moved: Option<&HashSet<String>>) -> Vec<Found> {
        let leaves = self.leaves.clone();
        let mut placed: Vec<Placed> = Vec::new();
        for (i, leaf) in leaves.iter().enumerate() {
            let Some(shape) = self.shape_of(&leaf.part_id) else { continue };
            let pose = Pose {
                pos: leaf.pos,
                rot: leaf.rot,
            };
            let bb = shape.world_bbox(&pose);
            placed.push((i, shape, pose, bb));
        }
        let mut conns: HashMap<usize, Vec<assembly::WorldConnector>> = HashMap::new();
        let mut connectors = |i: usize, doc: &Document, bundle: &Bundle| -> Vec<assembly::WorldConnector> {
            conns
                .entry(i)
                .or_insert_with(|| assembly::connectors_of_leaf(doc, bundle, &leaves[i]))
                .clone()
        };
        let pad = DVec3::splat(overlap::TOL_MM);
        let rank = |o: &Overlap| match o {
            Overlap::Inside => (2, 0.0),
            Overlap::Crossing(d) => (1, *d),
            Overlap::Coplanar(a) => (0, *a),
        };
        let mut worst: BTreeMap<(String, String), (Overlap, f64, String, String)> = BTreeMap::new();
        for (k, (i, sa, pa, (alo, ahi))) in placed.iter().enumerate() {
            let top_a = &leaves[*i].path[0];
            if moved.is_some_and(|m| !m.contains(top_a)) {
                continue;
            }
            let from = if moved.is_none() { k + 1 } else { 0 };
            for (l, (j, sb, pb, (blo, bhi))) in placed.iter().enumerate().skip(from) {
                let top_b = &leaves[*j].path[0];
                // a pair of moved ones is looked at once, from its earlier leaf
                if top_b == top_a || (moved.is_some_and(|m| m.contains(top_b)) && l <= k) {
                    continue;
                }
                if (*alo - pad).cmpgt(*bhi).any() || (*ahi + pad).cmplt(*blo).any() {
                    continue;
                }
                let (ca, cb) = (connectors(*i, &self.doc, &self.bundle), connectors(*j, &self.doc, &self.bundle));
                if assembly::seated(&ca, &cb) {
                    continue;
                }
                if let Some(o) = overlap::overlap(sa, pa, sb, pb) {
                    let shared = (ahi.min(*bhi) - alo.max(*blo)).max(DVec3::ZERO);
                    let volume = shared.x * shared.y * shared.z;
                    let (pa, pb) = (leaves[*i].path.join("/"), leaves[*j].path.join("/"));
                    let (key, la, lb) = if top_a < top_b {
                        ((top_a.clone(), top_b.clone()), pa, pb)
                    } else {
                        ((top_b.clone(), top_a.clone()), pb, pa)
                    };
                    let e = worst.entry(key).or_insert((o, volume, la.clone(), lb.clone()));
                    if rank(&o) > rank(&e.0) {
                        // the worst of the pair's leaves is the one named
                        *e = (o, e.1, la, lb);
                    }
                    e.1 = e.1.max(volume);
                }
            }
        }
        // the moved one first in each pair
        worst
            .into_iter()
            .map(|((a, b), (o, v, la, lb))| {
                if moved.is_some_and(|m| !m.contains(&a) && m.contains(&b)) {
                    (b, a, o, v, la)
                } else {
                    (a, b, o, v, lb)
                }
            })
            .collect()
    }

    fn pair(a: &str, b: &str) -> (String, String) {
        if a < b {
            (a.to_string(), b.to_string())
        } else {
            (b.to_string(), a.to_string())
        }
    }

    /// Called once a change of `names` has its undo point: the change
    /// in progress is now this one, with what the instances overlapped
    /// already.
    fn note_before(&mut self, names: &[String]) {
        let set: HashSet<String> = names.iter().cloned().collect();
        let before = self
            .overlaps(Some(&set))
            .into_iter()
            .map(|(a, b, _, v, _)| (Self::pair(&a, &b), v))
            .collect();
        self.pending = Some(Pending {
            names: names.to_vec(),
            doc: self.undo.last().cloned().unwrap_or_else(|| self.doc.clone()),
            dirty: self.dirty_before,
            undo_len: self.undo.len(),
            before,
        });
        self.overlapping.clear();
    }

    /// The overlaps the change in progress makes that its instances did
    /// not have before, or had and are made worse: an old overlap may be
    /// moved out of, not further in (the boxes' share grows by more than
    /// a twentieth and a cubic millimetre).
    fn new_overlaps(&mut self) -> Vec<(String, String, Overlap, String)> {
        let Some(p) = &self.pending else { return vec![] };
        let names: HashSet<String> = p.names.iter().cloned().collect();
        let before = p.before.clone();
        self.overlaps(Some(&names))
            .into_iter()
            .filter(|(a, b, _, v, _)| before.get(&Self::pair(a, b)).is_none_or(|old| *v > old * 1.05 + 1.0))
            .map(|(a, b, o, _, leaf)| (a, b, o, leaf))
            .collect()
    }

    /// While the change in progress goes on: what it would overlap, live.
    fn watch(&mut self) {
        self.overlapping = self.new_overlaps();
    }

    /// The change in progress is done: kept unless it made new overlaps,
    /// in which case the document goes back to before it (with the undo
    /// points the change took) and the status line names what `what`
    /// would have overlapped. True when it was taken back.
    fn settle(&mut self, what: &str) -> bool {
        let new = self.new_overlaps();
        self.overlapping.clear();
        let Some(p) = self.pending.take() else { return false };
        let Some((a, _, o, leaf)) = new.first().cloned() else {
            // a refusal's note does not outlive the next change that is kept
            if self.refused {
                self.status.clear();
                self.refused = false;
            }
            return false;
        };
        self.refused = true;
        self.undo.truncate(p.undo_len.saturating_sub(1));
        self.doc = p.doc;
        self.dirty = p.dirty;
        self.recompute();
        self.status = format!("{a} {what}: it would {}", o.phrase(&leaf));
        true
    }

    /// The change in progress abandoned (Escape during a drag): the
    /// document goes back to before it.
    pub fn cancel_change(&mut self) {
        self.overlapping.clear();
        if let Some(p) = self.pending.take() {
            self.undo.truncate(p.undo_len.saturating_sub(1));
            self.doc = p.doc;
            self.dirty = p.dirty;
            self.recompute();
            self.status = format!("{} put back", p.names.join(", "));
        }
    }

    /// What the change in progress would overlap, for the status line.
    pub fn overlap_note(&self) -> String {
        match self.overlapping.first() {
            Some((a, _, o, leaf)) => format!("{a} would {}", o.phrase(leaf)),
            None => String::new(),
        }
    }

    /// How many pairs of instances overlap in the component being edited.
    pub fn overlap_count(&mut self) -> usize {
        self.overlaps(None).len()
    }

    /// New instances `names` moved along `step` until they overlap
    /// nothing (a brick added at the origin lands beside what is there):
    /// how far that was, or what still overlaps after 400 steps.
    fn clear_of_others(&mut self, names: &[String], step: [f64; 3]) -> Room {
        self.overlapping.clear();
        let set: HashSet<String> = names.iter().cloned().collect();
        let mut moved = false;
        for _ in 0..400 {
            if self.overlaps(Some(&set)).is_empty() {
                return if moved { Room::Moved } else { Room::Clear };
            }
            moved = true;
            for name in names {
                self.set_instance(name, |i| {
                    for (p, s) in i.pos.iter_mut().zip(step) {
                        *p = assembly::round3(*p + s);
                    }
                });
            }
            self.recompute();
        }
        match self.overlaps(Some(&set)).into_iter().next() {
            Some((_, _, _, _, leaf)) => Room::Stuck(leaf),
            None => Room::Moved,
        }
    }

    /// The load report's count of overlapping pairs, when there are any.
    fn overlap_report(&mut self) -> String {
        match self.overlap_count() {
            0 => String::new(),
            1 => " · one pair of bricks overlaps".into(),
            n => format!(" · {n} pairs of bricks overlap"),
        }
    }

    // ------------------------------------------------------------ model

    pub fn recompute(&mut self) {
        self.edits += 1;
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
        self.dirty_before = self.dirty;
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
        self.overlapping.clear();
        self.pending = None;
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
            color: None,
        });
        self.selection = vec![name.clone()];
        self.recompute();
        if self.magnet {
            self.snap_selection(false);
        }
        let room = self.clear_of_others(std::slice::from_ref(&name), [MODULE_MM, 0.0, 0.0]);
        self.status = format!(
            "Added {name} to {editing}{}",
            match room {
                Room::Clear => String::new(),
                Room::Moved => ", beside what was there".into(),
                Room::Stuck(other) => format!(", still overlapping {other}: no free place along x within 3.2 m"),
            }
        );
    }

    /// A brick from the library placed at the origin, in a LEGO colour
    /// when one was picked for it.
    pub fn add_brick(&mut self, part_id: String, color: Option<u32>) {
        let before = self.children().len();
        self.add_instance(Some(part_id), None, [0.0; 3]);
        if let Some(c) = color
            && self.children().len() > before
            && let Some(name) = self.selection.first().cloned()
        {
            self.set_instance(&name, |i| i.color = Some(c));
            self.recompute();
        }
    }

    /// The document's part for an LDraw number, recorded from the bundle
    /// on first use. A part fetched by number carries its record along
    /// (mesh, box, mass properties, connectors), so the build opens on a
    /// machine whose library lacks it.
    pub fn ensure_ldraw_part(&mut self, num: &str) -> Option<String> {
        let rec = self.bundle.parts.get(num)?;
        let id = format!("lego_{num}");
        if !self.doc.parts.contains_key(&id) {
            let mut extra: serde_json::Map<String, serde_json::Value> = Default::default();
            if rec.fetched {
                let put = |extra: &mut serde_json::Map<String, serde_json::Value>, k: &str, v: serde_json::Value| {
                    extra.insert(k.to_string(), v);
                };
                put(&mut extra, "mesh", serde_json::to_value(&rec.mesh).unwrap_or_default());
                put(&mut extra, "bbox", serde_json::to_value(rec.bbox).unwrap_or_default());
                put(&mut extra, "com", serde_json::to_value(rec.com).unwrap_or_default());
                put(
                    &mut extra,
                    "inertia_per_g",
                    serde_json::to_value(rec.inertia_per_g).unwrap_or_default(),
                );
                put(&mut extra, "connectors", serde_json::to_value(&rec.connectors).unwrap_or_default());
                put(&mut extra, "volume_mm3", serde_json::to_value(rec.volume_mm3).unwrap_or_default());
            }
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
                    extra,
                },
            );
        }
        Some(id)
    }

    /// A part's record changed (fetched, or fetched again): what was
    /// derived from the old one goes, and the document is checked afresh.
    pub fn forget_part(&mut self, num: &str) {
        self.shapes.remove(&format!("ld:{num}"));
        self.errors = assembly::validate(&self.doc, &self.bundle);
        self.recompute();
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
        self.selection = made.clone();
        self.recompute();
        let _ = self.clear_of_others(&made, [0.0, -MODULE_MM, 0.0]);
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
        self.note_before(&names);
        for name in &names {
            self.set_instance(name, |i| i.rot[axis] = wrap_deg(i.rot[axis] + deg));
        }
        self.recompute();
        self.settle("stays");
    }

    pub fn nudge_selection(&mut self, d: [f64; 3]) {
        let names = self.movable();
        if names.is_empty() {
            return;
        }
        self.push_undo();
        self.note_before(&names);
        for name in &names {
            self.set_instance(name, |i| {
                for (k, dk) in d.iter().enumerate() {
                    i.pos[k] = assembly::round3(i.pos[k] + dk);
                }
            });
        }
        self.recompute();
        self.settle("stays");
    }

    /// The keyboard's nudge step: one grid step, or 1 mm with shift.
    pub fn nudge_step(&self, shift: bool) -> f64 {
        if shift { 1.0 } else { self.snap_mm.max(1.0) }
    }

    /// The magnet on `names`, within `tol_mm`: the best seat that makes
    /// no new overlap. A seat the overlap rule would refuse is passed
    /// over for the next as good (mating as many features: a pin let go
    /// nearer a taken hole than a free one lands in the free one); a
    /// lesser seat is no substitute, so when every seat as good as the
    /// best overlaps, the best is taken and the refusal names what it
    /// hits. Returns the mated path.
    fn snap_fitting(&mut self, names: &[String], tol_mm: f64) -> Option<Vec<String>> {
        let seats = assembly::group_seats(&self.doc, &self.bundle, &self.editing, names, tol_mm);
        let first = seats.first()?.clone();
        for seat in seats.iter().take_while(|s| s.mates == first.mates) {
            assembly::seat_group(&mut self.doc, &self.editing, seat);
            self.leaves = assembly::flatten(&self.doc, &self.editing);
            if self.new_overlaps().is_empty() {
                self.recompute();
                return Some(seat.path.clone());
            }
        }
        assembly::seat_group(&mut self.doc, &self.editing, &first);
        self.recompute();
        Some(first.path)
    }

    pub fn snap_selection(&mut self, announce: bool) {
        if self.selection.is_empty() {
            if announce {
                self.status = "Select an item to snap".into();
            }
            return;
        }
        // several selected move as one: their features together, against the rest
        let names = self.unlocked_selection();
        if names.is_empty() {
            if announce {
                self.status = format!("{} is locked", self.selection.join(", "));
            }
            return;
        }
        // a change of its own (the S key, the Snap button) is judged like any other; within a
        // drag or an add it belongs to that change
        let own = self.pending.is_none();
        if own {
            self.push_undo();
            self.note_before(&names);
        }
        let before = self.doc.clone();
        match self.snap_fitting(&names, assembly::SNAP_MM) {
            Some(path) => {
                if !own {
                    self.undo.push(before);
                    self.dirty = true;
                }
                self.recompute();
                if own && self.settle("stays") {
                    return;
                }
                self.status = format!("Snapped {} into {}", names.join(", "), path.join("/"));
            }
            None => {
                if own {
                    self.pending = None;
                    self.undo.pop();
                    self.dirty = self.dirty_before;
                }
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

    /// A new component of the given name, with nothing in it, joins the
    /// library and opens for building; where it was made is a crumb back.
    pub fn new_component(&mut self, name: &str) -> bool {
        let before = self.doc.clone();
        match assembly::new_component(&mut self.doc, name) {
            Ok(id) => {
                self.undo.push(before);
                self.dirty = true;
                self.open_component(&id, true);
                self.status = format!("{id} is in the library, empty: add bricks to it, then use it from the library");
                true
            }
            Err(e) => {
                self.status = e;
                false
            }
        }
    }

    /// The LEGO colour the selected bricks are placed in (an LDraw colour
    /// id the library's palette names); None gives them their category
    /// colour back. Locked bricks keep theirs, and a component instance
    /// has no colour of its own. A combo re-reports while it is open, so
    /// a run of changes to one selection with nothing else edited between
    /// is one undo point.
    pub fn set_selection_color(&mut self, color: Option<u32>) {
        let bricks: Vec<Instance> = self.selected_instances().into_iter().filter(|i| i.part.is_some()).collect();
        let names: Vec<String> = bricks.iter().filter(|i| !i.locked).map(|i| i.name.clone()).collect();
        if names.is_empty() {
            self.status = if bricks.is_empty() {
                "Select a brick to colour".into()
            } else {
                "Locked bricks keep their colour; unlock first".into()
            };
            return;
        }
        if bricks.iter().filter(|i| !i.locked).all(|i| i.color == color) {
            return;
        }
        let editing = self.editing.clone();
        let continuing = self.undo.last().is_some_and(|top| {
            let mut now = self.doc.clone();
            let was = |name: &str| {
                top.components
                    .get(&editing)
                    .and_then(|c| c.children.iter().find(|i| i.name == name))
                    .and_then(|i| i.color)
            };
            if let Some(comp) = now.components.get_mut(&editing) {
                for i in comp.children.iter_mut().filter(|i| names.contains(&i.name)) {
                    i.color = was(&i.name);
                }
            }
            now == *top
        });
        if continuing {
            self.dirty = true;
        } else {
            self.push_undo();
        }
        for name in &names {
            self.set_instance(name, |i| i.color = color);
        }
        self.recompute();
        let n = names.len();
        let what = format!("{n} brick{}", if n == 1 { "" } else { "s" });
        self.status = match color.and_then(|c| self.bundle.colors.get(&c)) {
            Some(c) => format!("{what} in {}", c.name),
            None => match color {
                Some(c) => format!("{what} in colour {c}"),
                None => format!("{what} in the category colour"),
            },
        };
    }

    /// The colours every selected brick comes in (the library's record
    /// of its part): what a colour picker may offer the selection.
    pub fn common_colors(&self) -> Vec<u32> {
        let mut common: Option<Vec<u32>> = None;
        for inst in self.selected_instances().iter().filter(|i| i.part.is_some()) {
            let mine: Vec<u32> = self
                .brick_record(inst)
                .map(|rec| rec.colors.keys().copied().collect())
                .unwrap_or_default();
            common = Some(match common {
                None => mine,
                Some(c) => c.into_iter().filter(|x| mine.contains(x)).collect(),
            });
        }
        common.unwrap_or_default()
    }

    /// The library's record of a brick instance's part, when it is one of the library's.
    pub fn brick_record(&self, inst: &Instance) -> Option<&crate::bundle::PartRecord> {
        let num = self.doc.parts.get(inst.part.as_ref()?)?.ldraw.as_ref()?;
        self.bundle.parts.get(num)
    }

    /// The LEGO element numbers that name a brick's part in the colour it is placed in.
    pub fn elements_of(&self, inst: &Instance) -> Vec<String> {
        inst.color
            .and_then(|c| self.brick_record(inst)?.colors.get(&c).cloned())
            .unwrap_or_default()
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
        let names = vec![name.to_string()];
        self.note_before(&names);
        self.set_instance(name, |i| {
            i.pos = pos.map(assembly::round3);
            i.rot = rot.map(wrap_deg);
        });
        self.recompute();
        self.settle("stays");
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
        self.shapes.clear();
        self.pending = None;
        self.path = None;
        self.editing = self.doc.robot.root.clone();
        self.crumbs = vec![self.editing.clone()];
        self.selection.clear();
        self.errors = assembly::validate(&self.doc, &self.bundle);
        self.fit_pending = true;
        self.recompute();
    }

    /// A part recorded from outside (an STL import) joins the library
    /// under a unique id and is placed in the view.
    pub fn import_part(&mut self, part: Part) -> String {
        let stem = {
            let s = assembly::slug(&part.name);
            if s.is_empty() { "part".to_string() } else { s }
        };
        let mut id = stem.clone();
        let mut k = 2;
        while self.doc.parts.contains_key(&id) {
            id = format!("{stem}_{k}");
            k += 1;
        }
        self.push_undo();
        let name = part.name.clone();
        self.doc.parts.insert(id.clone(), part);
        self.add_instance(Some(id.clone()), None, [0.0; 3]);
        self.status = format!("{name} is in the library and in the view");
        id
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
        let _ = self.clear_of_others(&made, [0.0, -MODULE_MM, 0.0]);
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
                self.shapes.clear();
                self.pending = None;
                self.path = Some(p.clone());
                self.editing = self.doc.robot.root.clone();
                self.crumbs = vec![self.editing.clone()];
                self.selection.clear();
                self.dirty = false;
                self.fit_pending = true;
                self.recompute();
                self.drop_draft();
                self.status = format!("Opened {}{}", p.display(), self.overlap_report());
            }
            Err(e) => self.status = format!("Could not open: {e}"),
        }
    }

    /// A component as a build of its own, written to `path` (the robot the
    /// file describes is that component, with the bricks and components
    /// it needs); the build open here is untouched.
    pub fn save_component(&mut self, id: &str, path: &Path) -> bool {
        let Some(sub) = assembly::subset(&self.doc, id) else {
            self.status = format!("No component {id}");
            return false;
        };
        match serde_json::to_string_pretty(&sub)
            .map_err(|e| e.to_string())
            .and_then(|t| std::fs::write(path, t).map_err(|e| e.to_string()))
        {
            Ok(()) => {
                self.status = format!("Saved {id} as {}", path.display());
                true
            }
            Err(e) => {
                self.status = format!("Could not save {id}: {e}");
                false
            }
        }
    }

    /// A saved build's components and bricks join this library, to add
    /// from; what is here already, the same, is left, a clash comes in
    /// under a new id. One undo point.
    pub fn import_build(&mut self, p: PathBuf) -> bool {
        let doc = match std::fs::read_to_string(&p)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str::<Document>(&t).map_err(|e| e.to_string()))
        {
            Ok(d) => d,
            Err(e) => {
                self.status = format!("Could not import: {e}");
                return false;
            }
        };
        let errs = assembly::validate(&doc, &self.bundle);
        if errs.iter().any(|e| e.starts_with("format") || e.starts_with("robot.root")) {
            self.status = format!("Not an assembly file: {}", errs.join("; "));
            return false;
        }
        let name = p.file_name().map(|f| f.to_string_lossy().to_string()).unwrap_or_default();
        let before = self.doc.clone();
        let got = assembly::merge_library(&mut self.doc, &doc);
        if got.components.is_empty() && got.parts.is_empty() {
            self.doc = before;
            self.status = format!("Nothing new in {name}: its components and bricks are here already");
            return false;
        }
        self.undo.push(before);
        self.dirty = true;
        self.recompute();
        let renamed = if got.renamed.is_empty() {
            String::new()
        } else {
            format!(
                " ({})",
                got.renamed
                    .iter()
                    .map(|(a, b)| format!("{a} as {b}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        };
        self.status = format!(
            "Imported {} component{} and {} brick{} from {name}{renamed}",
            got.components.len(),
            if got.components.len() == 1 { "" } else { "s" },
            got.parts.len(),
            if got.parts.len() == 1 { "" } else { "s" }
        );
        true
    }

    pub fn save_to(&mut self, path: &Path) {
        match serde_json::to_string_pretty(&self.doc)
            .map_err(|e| e.to_string())
            .and_then(|t| std::fs::write(path, t).map_err(|e| e.to_string()))
        {
            Ok(()) => {
                self.path = Some(path.to_path_buf());
                self.dirty = false;
                self.drop_draft();
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
            let names: Vec<String> = starts.iter().map(|s| s.0.clone()).collect();
            self.note_before(&names);
        }
        starts
    }

    /// The plane drag: every start moved by `dx, dy`, snapped; what is
    /// dragged near a hole or a stud grid is pulled onto it — several
    /// items as one — and lets go again as the pointer moves on (the
    /// magnet works from the start poses each frame, so nothing
    /// accumulates).
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
        if self.magnet && !starts.is_empty() {
            let names: Vec<String> = starts.iter().map(|s| s.0.clone()).collect();
            self.snap_fitting(&names, assembly::PULL_MM);
        }
        self.watch();
    }

    /// The shift drag: lift every dragged item by `dz` (unsnapped until
    /// the drag ends).
    pub fn lift_by(&mut self, starts: &[(String, [f64; 3])], dz: f64) {
        for (name, _) in starts {
            self.set_instance(name, |i| i.pos[2] = assembly::round3(i.pos[2] + dz));
        }
        self.recompute();
        self.watch();
    }

    /// The plane drag let go: heights land on the height module, then
    /// the magnet; then, if the drag made the bricks overlap others, it
    /// is taken back.
    pub fn end_move(&mut self) {
        if self.snap_mm > 0.0 {
            for name in self.unlocked_selection() {
                self.set_instance(&name, |i| i.pos[2] = assembly::round3((i.pos[2] / HEIGHT_MM).round() * HEIGHT_MM));
            }
        }
        self.recompute();
        if self.magnet {
            self.snap_selection(false);
        }
        self.settle("put back");
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
            let names: Vec<String> = starts.iter().map(|s| s.0.clone()).collect();
            self.note_before(&names);
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
        self.watch();
    }

    /// A handle drag let go: the magnet has a look; then, if the drag
    /// made the bricks overlap others, it is taken back.
    pub fn end_handle(&mut self) {
        self.recompute();
        if self.magnet {
            self.snap_selection(false);
        }
        self.settle("put back");
    }
}

/// An angle wrapped into [-180, 180).
pub fn wrap_deg(a: f64) -> f64 {
    ((a + 180.0).rem_euclid(360.0)) - 180.0
}

/// Shared by the tests of every module that needs real bricks.
#[cfg(test)]
pub mod testing {
    use super::*;
    use crate::bundle;

    /// The bundle that ships in the wheel.
    pub fn real_bundle() -> Bundle {
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../openbricks/openbricks_sim/bricks/technic_bundle.json.zlib");
        bundle::load_bundle(&p).expect("the shipped brick bundle")
    }
}

#[cfg(test)]
mod tests {
    use super::testing::real_bundle;
    use super::*;

    fn editor() -> Editor {
        Editor::new(real_bundle(), None)
    }

    /// An editor on an empty build with one brick `num` at the origin,
    /// the magnet off: its name comes back with it.
    fn solo(num: &str) -> (Editor, String) {
        let mut ed = editor();
        let root = ed.doc.robot.root.clone();
        ed.doc.components.get_mut(&root).unwrap().children.clear();
        ed.doc.robot.roles.clear();
        ed.recompute();
        ed.magnet = false;
        let id = ed.ensure_ldraw_part(num).unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        let name = ed.selection[0].clone();
        (ed, name)
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
    fn duplicates_sit_two_modules_over_or_the_next_free_two() {
        let (mut ed, first) = solo("3001");
        ed.duplicate_selection();
        assert_eq!(ed.selection.len(), 1);
        let copy = ed.selected_instances()[0].clone();
        assert_ne!(copy.name, first);
        assert_eq!(copy.pos, [0.0, -16.0, 0.0]);
        assert_eq!(ed.children()[1].name, copy.name);
        // that spot taken, the next duplicate of the first goes two modules further
        ed.selection = vec![first.clone()];
        ed.duplicate_selection();
        assert_eq!(ed.selected_instances()[0].pos, [0.0, -32.0, 0.0]);
        ed.selection.clear();
        let n = ed.children().len();
        ed.duplicate_selection();
        assert_eq!(ed.children().len(), n);
    }

    #[test]
    fn rotate_nudge_and_pose_wrap_and_round() {
        let (mut ed, name) = solo("3001");
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
        assert_eq!(ed.nudge_step(false), 4.0, "half a stud");
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
    fn snapping_needs_a_selection_and_takes_several_as_one() {
        let mut ed = editor();
        ed.snap_selection(true);
        assert_eq!(ed.status, "Select an item to snap");
        // two of the example's bricks, already seated: snapped together, they stay where they are
        let names: Vec<String> = ed.children().iter().take(2).map(|c| c.name.clone()).collect();
        let before: Vec<[f64; 3]> = ed.children().iter().take(2).map(|c| c.pos).collect();
        ed.selection = names.clone();
        ed.snap_selection(true);
        assert!(
            ed.status.starts_with(&format!("Snapped {} into", names.join(", "))),
            "{}",
            ed.status
        );
        let after: Vec<[f64; 3]> = ed.children().iter().take(2).map(|c| c.pos).collect();
        assert_eq!(after, before);
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
    fn a_new_component_opens_empty_and_undo_takes_it_back() {
        let mut ed = editor();
        let root = ed.doc.robot.root.clone();
        assert!(!ed.new_component(" "));
        assert_eq!(ed.status, "Give the component a name");
        assert!(!ed.new_component("drive_unit"), "the example already has one");
        assert!(ed.status.contains("already exists"), "{}", ed.status);
        assert!(ed.is_root() && !ed.dirty && ed.undo_depth() == 0);
        ed.fit_pending = false;
        assert!(ed.new_component("Sensor Mast"), "{}", ed.status);
        assert_eq!(ed.editing, "sensor_mast");
        assert_eq!(ed.crumbs, vec![root.clone(), "sensor_mast".to_string()]);
        assert!(ed.children().is_empty() && ed.selection.is_empty());
        assert!(ed.dirty && ed.fit_pending);
        assert_eq!(
            ed.status,
            "sensor_mast is in the library, empty: add bricks to it, then use it from the library"
        );
        assert_eq!(ed.edited_props().count, 0);
        assert!(ed.leaves.is_empty());
        // undone: the component is gone and the robot is open again
        ed.undo();
        assert!(!ed.doc.components.contains_key("sensor_mast"));
        assert!(ed.is_root());
        assert_eq!(ed.crumbs, vec![root.clone()]);
        // made again, built from a brick, then used from the robot
        assert!(ed.new_component("sensor_mast"));
        let id = ed.ensure_ldraw_part("32278").unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        assert_eq!(ed.children().len(), 1);
        assert!(ed.edited_props().mass > 0.0);
        ed.open_component(&root, false);
        assert_eq!(ed.crumbs, vec![root.clone()]);
        let n = ed.children().len();
        ed.add_instance(None, Some("sensor_mast".into()), [0.0; 3]);
        assert_eq!(ed.children().len(), n + 1);
        assert_eq!(ed.component_of(&ed.selection[0].clone()), Some("sensor_mast".into()));
        assert_eq!(assembly::usage_count(&ed.doc, "sensor_mast"), 1);
        assert!(assembly::validate(&ed.doc, &ed.bundle).is_empty());
    }

    #[test]
    fn a_brick_colour_is_one_undo_point_per_run_of_changes() {
        let mut ed = editor();
        ed.set_selection_color(Some(72));
        assert_eq!(ed.status, "Select a brick to colour");
        assert!(ed.common_colors().is_empty());
        // bricks the library knows in at least two colours (the example's first frame comes in one)
        let bricks: Vec<String> = ed
            .children()
            .iter()
            .filter(|c| c.part.is_some() && ed.brick_record(c).is_some_and(|r| r.colors.len() >= 2))
            .map(|c| c.name.clone())
            .take(2)
            .collect();
        assert_eq!(bricks.len(), 2);
        ed.selection = vec![bricks[0].clone()];
        let cols = ed.common_colors();
        assert!(cols.len() >= 2, "{cols:?}");
        let (c1, c2) = (cols[0], cols[1]);
        let name_of = |ed: &Editor, c: u32| ed.bundle.colors[&c].name.clone();
        ed.set_selection_color(None);
        assert!(!ed.dirty && ed.undo_depth() == 0, "already so: nothing recorded");
        let edits = ed.edits;
        ed.set_selection_color(Some(c1));
        assert_eq!(ed.selected_instances()[0].color, Some(c1));
        assert!(ed.dirty && ed.undo_depth() == 1 && ed.edits > edits);
        assert_eq!(ed.status, format!("1 brick in {}", name_of(&ed, c1)));
        let inst = ed.selected_instances()[0].clone();
        let expected = ed.brick_record(&inst).unwrap().colors[&c1].clone();
        assert_eq!(ed.elements_of(&inst), expected);
        assert!(!expected.is_empty());
        // a combo re-reports: one undo point for the run
        ed.set_selection_color(Some(c2));
        ed.set_selection_color(Some(c1));
        assert_eq!(ed.undo_depth(), 1);
        assert!(ed.leaves.iter().any(|l| l.color == Some(c1)));
        ed.set_selection_color(None);
        assert_eq!(ed.status, "1 brick in the category colour");
        assert_eq!(ed.undo_depth(), 1);
        assert!(ed.elements_of(&ed.selected_instances()[0]).is_empty());
        ed.undo();
        assert_eq!(ed.selected_instances()[0].color, None);
        // a colour the palette lacks still applies, named by its id
        ed.set_selection_color(Some(999_999));
        assert_eq!(ed.status, "1 brick in colour 999999");
        ed.undo();
        // several at once: a locked one keeps its colour, a component instance is passed over
        ed.selection = vec![bricks[1].clone()];
        ed.lock_selection(true);
        let comp_inst = ed.children().iter().find(|c| c.component.is_some()).unwrap().name.clone();
        ed.selection = vec![bricks[0].clone(), bricks[1].clone(), comp_inst.clone()];
        let shared = ed.common_colors();
        assert!(shared.contains(&c1) || shared.is_empty(), "{shared:?}");
        ed.set_selection_color(Some(c1));
        assert_eq!(ed.status, format!("1 brick in {}", name_of(&ed, c1)));
        let by_name = |ed: &Editor, n: &str| ed.children().iter().find(|c| c.name == n).unwrap().color;
        assert_eq!(by_name(&ed, &bricks[0]), Some(c1));
        assert_eq!(by_name(&ed, &bricks[1]), None, "locked");
        assert_eq!(by_name(&ed, &comp_inst), None, "not a brick");
        ed.selection = vec![bricks[1].clone()];
        ed.set_selection_color(Some(c1));
        assert_eq!(ed.status, "Locked bricks keep their colour; unlock first");
        ed.selection = vec![comp_inst];
        ed.set_selection_color(Some(c1));
        assert_eq!(ed.status, "Select a brick to colour");
        // the same brick recoloured again continues the run the selection change did not end;
        // another edit between two colour changes starts a new one
        ed.selection = vec![bricks[0].clone()];
        let depth = ed.undo_depth();
        ed.set_selection_color(Some(c2));
        assert_eq!(ed.undo_depth(), depth);
        ed.nudge_selection([8.0, 0.0, 0.0]);
        ed.set_selection_color(Some(c1));
        assert_eq!(ed.undo_depth(), depth + 2);
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
        let (mut ed, _) = solo("3001");
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
    fn a_component_saves_as_a_build_of_its_own_and_imports_into_a_library() {
        let mut ed = editor();
        let dir = std::env::temp_dir().join(format!("ob-comp-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let comp = ed
            .children()
            .iter()
            .find(|c| c.component.is_some())
            .unwrap()
            .component
            .clone()
            .unwrap();
        let path = dir.join(format!("{comp}.assembly.json"));
        assert!(!ed.save_component("no-such", &path));
        assert_eq!(ed.status, "No component no-such");
        ed.dirty = false;
        assert!(ed.save_component(&comp, &path), "{}", ed.status);
        assert!(ed.status.starts_with(&format!("Saved {comp} as")), "{}", ed.status);
        assert!(!ed.dirty && ed.path.is_none(), "the build here is untouched");
        assert!(!ed.save_component(&comp, &dir.join("no-such-dir").join("x.json")));
        assert!(ed.status.starts_with(&format!("Could not save {comp}")), "{}", ed.status);
        // the file is a build whose robot is the component, with only what it needs
        let mut other = editor();
        other.load_path(path.clone());
        assert!(other.status.starts_with("Opened"), "{}", other.status);
        assert_eq!(other.doc.robot.root, comp);
        assert!(other.doc.parts.len() < ed.doc.parts.len());
        assert!(other.errors.is_empty(), "{:?}", other.errors);
        // imported into a library that has it already: nothing new; into an empty one: it and its bricks
        assert!(!ed.import_build(path.clone()));
        assert!(ed.status.starts_with("Nothing new in"), "{}", ed.status);
        let mut bare = Editor::new(real_bundle(), None);
        bare.doc.parts.clear();
        bare.doc.components.retain(|k, _| *k == bare.doc.robot.root);
        bare.doc.components.get_mut(&bare.doc.robot.root).unwrap().children.clear();
        bare.doc.robot.roles.clear();
        bare.recompute();
        let depth = bare.undo_depth();
        assert!(bare.import_build(path.clone()), "{}", bare.status);
        assert!(bare.doc.components.contains_key(&comp));
        assert!(!bare.doc.parts.is_empty() && bare.dirty);
        assert_eq!(bare.undo_depth(), depth + 1);
        assert!(bare.status.starts_with("Imported 1 component"), "{}", bare.status);
        assert!(bare.status.contains(&format!("from {comp}.assembly.json")), "{}", bare.status);
        assert!(assembly::validate(&bare.doc, &bare.bundle).is_empty());
        bare.undo();
        assert!(!bare.doc.components.contains_key(&comp));
        // a clash: the library's own robot id is taken, so the file's comes in renamed
        let robot_file = dir.join("robot.assembly.json");
        ed.save_to(&robot_file);
        let mut third = editor();
        third.selection = vec!["imu".into()];
        third.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(third.selected_instances()[0].pos[0], 38.0, "{}", third.status);
        assert!(third.import_build(robot_file.clone()), "{}", third.status);
        assert!(third.status.contains(" as "), "renamed: {}", third.status);
        // not a file, not an assembly
        assert!(!third.import_build(dir.join("missing.json")));
        assert!(third.status.starts_with("Could not import"), "{}", third.status);
        std::fs::write(
            dir.join("bad.json"),
            r#"{"format": "x/9", "parts": {}, "components": {"x": {}}, "robot": {"root": "x"}}"#,
        )
        .unwrap();
        assert!(!third.import_build(dir.join("bad.json")));
        assert!(third.status.starts_with("Not an assembly file"), "{}", third.status);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_unsaved_build_is_drafted_after_a_change_settles_and_restored_next_time() {
        use std::time::{Duration, Instant};
        let dir = std::env::temp_dir().join(format!("ob-editor-drafts-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut ed = editor();
        ed.draft_dir = dir.clone();
        let t0 = Instant::now();
        // nothing unsaved: nothing kept, however long it settles
        ed.autosave(t0 + Duration::from_secs(10), 1_000);
        assert!(drafts::take(&dir, drafts::BUILD).is_none());
        // an edit (the IMU slides along the frame), then a moment: the draft is kept, with where
        // the build belongs (nowhere yet)
        let name = "imu".to_string();
        ed.selection = vec![name.clone()];
        ed.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(ed.selected_instances()[0].pos[0], 38.0, "{}", ed.status);
        ed.autosave(t0, 1_000);
        ed.autosave(t0 + Duration::from_millis(1500), 2_000);
        assert!(drafts::take(&dir, drafts::BUILD).is_none(), "not settled yet");
        ed.autosave(t0 + Duration::from_secs(3), 3_000);
        let (text, note) = drafts::take(&dir, drafts::BUILD).expect("kept");
        assert_eq!(
            note,
            drafts::Note {
                path: None,
                kept_ms: 3_000
            }
        );
        assert!(text.contains("\"components\""));
        // unchanged since: not rewritten
        std::fs::write(dir.join(drafts::BUILD), "stale").unwrap();
        ed.autosave(t0 + Duration::from_secs(9), 9_000);
        assert_eq!(drafts::take(&dir, drafts::BUILD).unwrap().0, "stale");
        // another edit: rewritten once settled, now with the file it was saved to
        let file = dir.join("robot.assembly.json");
        ed.save_to(&file);
        assert!(drafts::take(&dir, drafts::BUILD).is_none(), "saved: the draft goes");
        ed.nudge_selection([8.0, 0.0, 0.0]);
        ed.autosave(t0 + Duration::from_secs(10), 10_000);
        ed.autosave(t0 + Duration::from_secs(13), 13_000);
        let (_, note) = drafts::take(&dir, drafts::BUILD).expect("kept again");
        assert_eq!(note.path.as_deref(), Some(file.as_path()));
        // the next session restores it, unsaved, belonging to that file
        let mut next = editor();
        next.draft_dir = dir.clone();
        assert!(next.restore_draft(13_000 + 120_000));
        assert!(next.dirty && next.undo_depth() == 0);
        assert_eq!(next.path.as_deref(), Some(file.as_path()));
        assert_eq!(
            next.children().iter().find(|c| c.name == name).unwrap().pos,
            ed.children().iter().find(|c| c.name == name).unwrap().pos
        );
        assert_eq!(
            next.status,
            format!(
                "Restored the unsaved draft kept 2 min ago of {}: Save writes it there",
                file.display()
            )
        );
        // opening a file drops the draft; nothing to restore after that
        next.load_path(file.clone());
        assert!(!next.restore_draft(0));
        assert!(!next.dirty);
        // a draft that cannot be read is left, and said so
        drafts::keep(&dir, drafts::BUILD, "not json", &drafts::Note::default()).unwrap();
        assert!(!next.restore_draft(0));
        assert!(next.status.starts_with("Ignored an unreadable draft at"), "{}", next.status);
        drafts::keep(
            &dir,
            drafts::BUILD,
            r#"{"format": "x/9", "parts": {}, "components": {"x": {}}, "robot": {"root": "x"}}"#,
            &drafts::Note::default(),
        )
        .unwrap();
        assert!(!next.restore_draft(0));
        assert!(
            next.status.starts_with("Ignored a draft that is not an assembly"),
            "{}",
            next.status
        );
        // a draft that belongs nowhere says how to give it a file
        drafts::keep(
            &dir,
            drafts::BUILD,
            &serde_json::to_string(&assembly::example()).unwrap(),
            &drafts::Note::default(),
        )
        .unwrap();
        assert!(next.restore_draft(0));
        assert!(next.status.ends_with(": Save as… gives it a file"), "{}", next.status);
        // keeping into a place that cannot be written is said too
        std::fs::write(dir.join("blocker"), "").unwrap();
        ed.draft_dir = dir.join("blocker");
        ed.keep_draft(0);
        assert!(ed.status.starts_with("Could not keep a draft"), "{}", ed.status);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_example_opens_with_no_overlapping_bricks() {
        let mut ed = editor();
        assert_eq!(ed.overlap_count(), 0);
        assert!(!ed.status.contains("overlap"), "{}", ed.status);
        // nor has any of its components one within
        let ids: Vec<String> = ed.doc.components.keys().cloned().collect();
        for id in ids {
            ed.open_component(&id, false);
            assert_eq!(ed.overlap_count(), 0, "{id}");
        }
    }

    #[test]
    fn a_brick_added_where_one_is_lands_beside_it() {
        let (mut ed, a) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        let b = ed.selection[0].clone();
        assert_ne!(a, b);
        assert_eq!(ed.selected_instances()[0].pos, [32.0, 0.0, 0.0], "the first free module along x");
        assert!(ed.status.ends_with(", beside what was there"), "{}", ed.status);
        let plate = ed.ensure_ldraw_part("3022").unwrap();
        ed.add_instance(Some(plate), None, [0.0; 3]);
        assert_eq!(ed.selected_instances()[0].pos, [56.0, 0.0, 0.0], "past both bricks");
        assert_eq!(ed.overlap_count(), 0);
    }

    #[test]
    fn a_brick_dragged_into_another_is_put_back_and_says_so() {
        let (mut ed, a) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        let b = ed.selection[0].clone();
        let depth = ed.undo_depth();
        let starts = ed.begin_move();
        ed.move_by(&starts, -8.0, 0.0);
        assert_eq!(ed.selected_instances()[0].pos, [24.0, 0.0, 0.0]);
        assert_eq!(ed.overlapping.len(), 1, "{:?}", ed.overlapping);
        assert_eq!(ed.overlap_note(), format!("{b} would overlap {a}"));
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos, [32.0, 0.0, 0.0], "put back");
        assert_eq!(ed.status, format!("{b} put back: it would overlap {a}"));
        assert!(ed.overlapping.is_empty() && ed.overlap_note().is_empty());
        assert_eq!(ed.undo_depth(), depth, "the drag left no undo point");
        // dragged clear, it stays
        let starts = ed.begin_move();
        ed.move_by(&starts, 8.0, 0.0);
        assert!(ed.overlapping.is_empty());
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos, [40.0, 0.0, 0.0]);
        assert_eq!(ed.undo_depth(), depth + 1);
        // a lift into the other brick is put back too (from 40, one module over only touches)
        let starts = ed.begin_move();
        ed.move_by(&starts, -8.0, 0.0);
        assert!(ed.overlapping.is_empty(), "end to end: touching");
        ed.move_by(&starts, -16.0, 0.0);
        ed.lift_by(&starts, 4.0);
        assert_eq!(ed.overlapping.len(), 1);
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos, [40.0, 0.0, 0.0]);
        // a handle drag likewise
        let starts = ed.begin_handle();
        ed.drag_handle(Handle::Axis(0), -16.0, DVec3::ZERO, &starts, false);
        assert_eq!(ed.overlapping.len(), 1);
        ed.end_handle();
        assert_eq!(ed.selected_instances()[0].pos, [40.0, 0.0, 0.0]);
        assert!(ed.status.starts_with(&format!("{b} put back")), "{}", ed.status);
    }

    #[test]
    fn nudges_turns_and_typed_poses_into_a_neighbour_stay() {
        let (mut ed, a) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        let b = ed.selection[0].clone();
        ed.dirty = false;
        ed.nudge_selection([-8.0, 0.0, 0.0]);
        assert_eq!(ed.selected_instances()[0].pos, [32.0, 0.0, 0.0]);
        assert_eq!(ed.status, format!("{b} stays: it would overlap {a}"));
        assert!(!ed.dirty, "a refused change leaves nothing unsaved");
        ed.set_pose(&b, [24.0, 0.0, 0.0], [0.0; 3]);
        assert_eq!(ed.selected_instances()[0].pos, [32.0, 0.0, 0.0]);
        assert!(ed.status.starts_with(&format!("{b} stays")), "{}", ed.status);
        // on top of the other brick it sits; turned flat it still sits on the studs; stood on
        // end it would pass through the brick below
        ed.set_pose(&b, [0.0, 0.0, 9.6], [0.0; 3]);
        assert_eq!(ed.selected_instances()[0].pos, [0.0, 0.0, 9.6], "{}", ed.status);
        ed.rotate_selection(2, 90.0);
        assert_eq!(ed.selected_instances()[0].rot, [0.0, 0.0, 90.0], "{}", ed.status);
        ed.rotate_selection(1, 90.0);
        assert_eq!(ed.selected_instances()[0].rot, [0.0, 0.0, 90.0]);
        assert_eq!(ed.status, format!("{b} stays: it would overlap {a}"));
    }

    #[test]
    fn a_file_with_overlaps_opens_and_says_so_and_only_new_overlaps_are_refused() {
        let (mut ed, a) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id.clone()), None, [0.0; 3]);
        let b = ed.selection[0].clone();
        // a file from before the rule: b right on a
        ed.set_instance(&b, |i| i.pos = [0.0; 3]);
        ed.recompute();
        let dir = std::env::temp_dir().join(format!("ob-overlap-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("old.assembly.json");
        ed.save_to(&path);
        let mut other = editor();
        other.load_path(path.clone());
        assert!(other.status.ends_with(" · one pair of bricks overlaps"), "{}", other.status);
        assert_eq!(other.overlap_count(), 1);
        // b can still be moved while it overlaps a: that pair is not new
        other.selection = vec![b.clone()];
        other.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(other.selected_instances()[0].pos, [8.0, 0.0, 0.0], "{}", other.status);
        // a third brick lands free; b may not be moved into it
        other.magnet = false;
        other.add_instance(Some(id), None, [0.0; 3]);
        let c = other.selection[0].clone();
        assert_eq!(other.selected_instances()[0].pos, [40.0, 0.0, 0.0]);
        other.selection = vec![b.clone()];
        other.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(other.selected_instances()[0].pos, [8.0, 0.0, 0.0]);
        assert!(
            other.status.starts_with(&format!("{b} stays: it would overlap {c}")),
            "{}",
            other.status
        );
        // undo is never refused
        other.undo();
        assert_eq!(other.status, "Undone");
        let _ = a;
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_abandoned_drag_goes_back_at_once() {
        let (mut ed, _) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        let b = ed.selection[0].clone();
        ed.dirty = false;
        let depth = ed.undo_depth();
        let starts = ed.begin_move();
        ed.move_by(&starts, -16.0, 0.0);
        assert_eq!(ed.overlapping.len(), 1);
        ed.cancel_change();
        assert_eq!(ed.selected_instances()[0].pos, [32.0, 0.0, 0.0]);
        assert_eq!(ed.status, format!("{b} put back"));
        assert!(ed.overlapping.is_empty() && !ed.dirty && ed.undo_depth() == depth);
        // the release that follows has nothing to judge
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos, [32.0, 0.0, 0.0]);
        assert_eq!(ed.overlap_count(), 0);
    }

    #[test]
    fn instances_turned_together_may_not_turn_into_each_other() {
        let (mut ed, a) = solo("32278");
        let id = ed.ensure_ldraw_part("32278").unwrap();
        ed.add_instance(Some(id), None, [16.0, 0.0, 0.0]);
        let b = ed.selection[0].clone();
        ed.selection = vec![a.clone(), b.clone()];
        // side by side; each turned about its own origin they would lie across each other
        ed.rotate_selection(2, 90.0);
        assert!(ed.selected_instances().iter().all(|i| i.rot == [0.0; 3]), "{}", ed.status);
        assert!(ed.status.contains("stays: it would overlap"), "{}", ed.status);
        assert_eq!(ed.overlap_count(), 0);
    }

    #[test]
    fn a_selection_moved_together_keeps_its_gaps() {
        let (mut ed, a) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id.clone()), None, [40.0, 0.0, 0.0]);
        let b = ed.selection[0].clone();
        ed.add_instance(Some(id), None, [80.0, 0.0, 0.0]);
        let c = ed.selection[0].clone();
        ed.selection = vec![a.clone(), b.clone()];
        let starts = ed.begin_move();
        ed.move_by(&starts, 48.0, 0.0);
        assert_eq!(ed.overlapping.len(), 1, "{:?}", ed.overlapping);
        assert_eq!((&ed.overlapping[0].0, &ed.overlapping[0].1), (&b, &c));
        ed.end_move();
        let pos = |ed: &Editor, n: &str| ed.children().iter().find(|i| i.name == n).unwrap().pos;
        assert_eq!((pos(&ed, &a), pos(&ed, &b)), ([0.0; 3], [40.0, 0.0, 0.0]), "both put back");
        let starts = ed.begin_move();
        ed.move_by(&starts, 8.0, 0.0);
        assert!(ed.overlapping.is_empty(), "{:?}", ed.overlapping);
        ed.end_move();
        assert_eq!(
            (pos(&ed, &a), pos(&ed, &b)),
            ([8.0, 0.0, 0.0], [48.0, 0.0, 0.0]),
            "end to end with {c}: {}",
            ed.status
        );
    }

    #[test]
    fn an_old_overlap_may_be_moved_out_of_but_not_further_in() {
        let (mut ed, a) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        let b = ed.selection[0].clone();
        ed.set_instance(&b, |i| i.pos = [16.0, 0.0, 0.0]);
        ed.recompute();
        assert_eq!(ed.overlap_count(), 1, "half into {a}");
        ed.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(ed.selected_instances()[0].pos, [24.0, 0.0, 0.0], "out a little: {}", ed.status);
        ed.nudge_selection([-16.0, 0.0, 0.0]);
        assert_eq!(ed.selected_instances()[0].pos, [24.0, 0.0, 0.0], "further in: refused");
        assert_eq!(ed.status, format!("{b} stays: it would overlap {a}"));
        ed.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(ed.selected_instances()[0].pos, [32.0, 0.0, 0.0], "clear: {}", ed.status);
        assert_eq!(ed.overlap_count(), 0);
    }

    #[test]
    fn only_a_seated_mate_is_a_joint() {
        let (mut ed, a) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id), None, [0.0; 3]);
        let b = ed.selection[0].clone();
        // stacked a stud grid over: a joint
        ed.set_pose(&b, [8.0, 0.0, 9.6], [0.0; 3]);
        assert_eq!(ed.selected_instances()[0].pos, [8.0, 0.0, 9.6], "{}", ed.status);
        // sunk 2 mm: its tubes are still over the studs (the loose mate the magnet works with),
        // but nothing is seated, and its top slab crosses the studs
        ed.set_pose(&b, [8.0, 0.0, 7.6], [0.0; 3]);
        assert_eq!(ed.selected_instances()[0].pos, [8.0, 0.0, 9.6]);
        assert_eq!(ed.status, format!("{b} stays: it would overlap {a}"));
        // at the half-stud offset its tubes are over studs too (the loose mate), but its walls
        // would stand on the studs it does not cover: no brick sits there, sunk or not
        for z in [9.6, 7.6, 4.4, 3.6] {
            ed.set_pose(&b, [4.0, 4.0, z], [0.0; 3]);
            assert_eq!(ed.selected_instances()[0].pos, [8.0, 0.0, 9.6], "at the half-stud offset, z {z}");
            assert_eq!(ed.status, format!("{b} stays: it would overlap {a}"));
        }
        // the refusal's note goes with the next change that is kept
        ed.nudge_selection([8.0, 0.0, 0.0]);
        assert_eq!(ed.selected_instances()[0].pos, [16.0, 0.0, 9.6]);
        assert_eq!(ed.status, "");
    }

    #[test]
    fn the_snap_key_is_refused_into_a_taken_hole() {
        // a beam 5 with a pin standing in its hole at y = -16; a second pin let go beside that
        // hole snaps into it — onto the first — and is refused; beside a free hole it snaps
        let (mut ed, _) = solo("32316");
        ed.snap_mm = 0.0;
        let pin = ed.ensure_ldraw_part("2780").unwrap();
        ed.add_instance(Some(pin.clone()), None, [0.0, 40.0, 4.0]);
        let p1 = ed.selection[0].clone();
        ed.set_instance(&p1, |i| {
            i.pos = [0.0, -16.0, 4.0];
            i.rot = [0.0, 90.0, 0.0];
        });
        ed.add_instance(Some(pin), None, [0.0, 60.0, 4.0]);
        let p2 = ed.selection[0].clone();
        ed.set_instance(&p2, |i| {
            i.pos = [0.4, -15.6, 4.3];
            i.rot = [0.0, 90.0, 0.0];
        });
        ed.recompute();
        assert_eq!(ed.overlap_count(), 2, "the second pin lies across the first and the beam");
        ed.magnet = true;
        let depth = ed.undo_depth();
        ed.snap_selection(true);
        assert_eq!(ed.selected_instances()[0].pos, [0.4, -15.6, 4.3]);
        assert_eq!(ed.status, format!("{p2} stays: it would overlap {p1}"));
        assert_eq!(ed.undo_depth(), depth);
        ed.set_instance(&p2, |i| i.pos = [0.4, -7.6, 4.3]);
        ed.recompute();
        ed.snap_selection(true);
        assert_eq!(ed.selected_instances()[0].pos, [0.0, -8.0, 4.0], "{}", ed.status);
        assert!(ed.status.starts_with("Snapped"));
        assert_eq!(ed.undo_depth(), depth + 1);
    }

    #[test]
    fn the_hinge_bricks_assemble_side_by_side_pin_in_socket() {
        // 3831 (swivel base) at the origin; 3830 (swivel top) let go near it, a touch off: the
        // magnet seats its pin in the socket — both hinge axes on one line, the bricks end to
        // end in one layer — and nothing overlaps (a pin may sit anywhere along its hole, so the
        // height is the grid's; MATE_MM of slack still counts as seated)
        let (mut ed, base) = solo("3831");
        ed.magnet = true;
        let top = ed.ensure_ldraw_part("3830").unwrap();
        ed.add_instance(Some(top), None, [0.4, 0.3, 0.0]);
        let name = ed.selection[0].clone();
        ed.snap_selection(true);
        assert_eq!(ed.selected_instances()[0].pos, [0.0, 0.0, 0.0], "{}", ed.status);
        assert_eq!(ed.selected_instances()[0].rot, [0.0, 0.0, 0.0]);
        assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
        let conns = |ed: &Editor, n: &str| -> Vec<assembly::WorldConnector> {
            assembly::flatten(&ed.doc, &ed.editing)
                .iter()
                .filter(|l| l.path[0] == n)
                .flat_map(|l| assembly::connectors_of_leaf(&ed.doc, &ed.bundle, l))
                .collect()
        };
        assert!(
            assembly::seated(&conns(&ed, &name), &conns(&ed, &base)),
            "the pin sits in the socket"
        );
        // one brick higher, as a stud stack would put it, the pin hangs above the socket: no
        // mate, no overlap either
        ed.set_instance(&name, |i| i.pos = [0.0, 0.0, 9.6]);
        ed.recompute();
        assert_eq!(ed.overlap_count(), 0);
        assert!(!assembly::seated(&conns(&ed, &name), &conns(&ed, &base)));
    }

    #[test]
    fn an_assembled_hinge_moves_onto_a_base_brick_as_one() {
        // a 2 x 4 at the origin; the hinge pair assembled beside it, on the ground; both
        // selected and moved onto the 2 x 4's studs: they go, together, and nothing overlaps
        let (mut ed, brick) = solo("3001");
        let hb = ed.ensure_ldraw_part("3831").unwrap();
        let ht = ed.ensure_ldraw_part("3830").unwrap();
        ed.add_instance(Some(hb), None, [0.0, 48.0, 0.0]);
        let base = ed.selection[0].clone();
        ed.add_instance(Some(ht), None, [0.0, 48.0, 0.0]);
        let top = ed.selection[0].clone();
        ed.recompute();
        assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
        let pos = |ed: &Editor, n: &str| ed.children().iter().find(|i| i.name == n).unwrap().pos;
        assert_eq!(
            (pos(&ed, &base), pos(&ed, &top)),
            ([0.0, 48.0, 0.0], [0.0, 48.0, 0.0]),
            "{}",
            ed.status
        );
        ed.selection = vec![base.clone(), top.clone()];
        ed.magnet = true;
        // up a brick, then across onto the studs
        ed.nudge_selection([0.0, 0.0, 9.6]);
        assert_eq!(pos(&ed, &base), [0.0, 48.0, 9.6], "{}", ed.status);
        let starts = ed.begin_move();
        ed.move_by(&starts, 0.0, -48.0);
        assert!(ed.overlapping.is_empty(), "{:?}", ed.overlapping);
        let seen = ed.overlapping.clone();
        ed.end_move();
        assert_eq!(
            (pos(&ed, &base), pos(&ed, &top)),
            ([0.0, 0.0, 9.6], [0.0, 0.0, 9.6]),
            "onto {brick}: {} — during the move {seen:?}, now {:?}",
            ed.status,
            ed.overlapping
        );
        assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
        // and straight down onto the studs from above
        ed.nudge_selection([0.0, 0.0, 9.6]);
        ed.nudge_selection([0.0, 0.0, -9.6]);
        assert_eq!(pos(&ed, &top), [0.0, 0.0, 9.6], "{}", ed.status);
    }

    #[test]
    fn a_selection_moved_together_is_seated_as_one() {
        // two 2 x 4s end to end on the ground beside a third; both selected and dragged onto it:
        // the drop lands their height on the height module and the magnet seats the pair on
        // its studs as one — before 4.28.0 the magnet did that for one item only (and the drop
        // rounded heights to the 8 mm grid, 1.6 mm into the brick below), and a pair was
        // refused as an overlap
        let (mut ed, base) = solo("3001");
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id.clone()), None, [0.0, 48.0, 0.0]);
        let a = ed.selection[0].clone();
        ed.add_instance(Some(id), None, [32.0, 48.0, 0.0]);
        let b = ed.selection[0].clone();
        let pos = |ed: &Editor, n: &str| ed.children().iter().find(|i| i.name == n).unwrap().pos;
        ed.selection = vec![a.clone(), b.clone()];
        ed.magnet = true;
        ed.nudge_selection([0.0, 0.0, 9.6]);
        let starts = ed.begin_move();
        ed.move_by(&starts, 0.0, -48.0);
        ed.end_move();
        assert_eq!(pos(&ed, &a), [0.0, 0.0, 9.6], "onto {base}: {}", ed.status);
        assert_eq!(pos(&ed, &b), [32.0, 0.0, 9.6], "and its partner with it, gap kept");
        assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
        // a little off the grid, dragged as a pair: the pull seats both while the drag is on,
        // and the S key on a selection of two snaps them together
        ed.set_instance(&a, |i| i.pos = [1.0, 0.5, 9.6]);
        ed.set_instance(&b, |i| i.pos = [33.0, 0.5, 9.6]);
        ed.recompute();
        ed.snap_selection(true);
        assert_eq!((pos(&ed, &a), pos(&ed, &b)), ([0.0, 0.0, 9.6], [32.0, 0.0, 9.6]), "{}", ed.status);
        assert!(ed.status.starts_with(&format!("Snapped {a}, {b} into")), "{}", ed.status);
        ed.snap_mm = 0.0;
        let starts = ed.begin_move();
        ed.move_by(&starts, 1.5, 0.7);
        assert_eq!(
            (pos(&ed, &a), pos(&ed, &b)),
            ([0.0, 0.0, 9.6], [32.0, 0.0, 9.6]),
            "pulled on during the drag"
        );
        ed.end_move();
        // a locked member stays behind and the rest still snap; nothing selected is said so
        ed.set_instance(&b, |i| i.locked = true);
        ed.set_instance(&a, |i| i.pos = [1.0, 0.5, 9.6]);
        ed.recompute();
        ed.snap_selection(true);
        assert_eq!((pos(&ed, &a), pos(&ed, &b)), ([0.0, 0.0, 9.6], [32.0, 0.0, 9.6]), "{}", ed.status);
        assert_eq!(ed.status, format!("Snapped {a} into {base}"));
        ed.set_instance(&a, |i| i.locked = true);
        ed.snap_selection(true);
        assert_eq!(ed.status, format!("{a}, {b} is locked"));
        ed.selection.clear();
        ed.snap_selection(true);
        assert_eq!(ed.status, "Select an item to snap");
    }

    #[test]
    fn a_pair_let_go_under_a_brick_lying_across_it_keeps_its_heading() {
        // the user's build: a 2 x 4 one brick up, lying across two 1 x 6 side by side, its
        // other half on two 2 x 4s of a component standing beside the pair. The pair, dragged
        // to sit centred under it, used to be turned a quarter for the four extra studs that
        // seats — a turn the drag's pull (4 mm) never showed and the drop's snap (6 mm) took —
        // and land on the component's bricks: "lego_6 put back: it would overlap aa"
        let pos = |ed: &Editor, n: &str| ed.children().iter().find(|i| i.name == n).unwrap().pos;
        let rot = |ed: &Editor, n: &str| ed.children().iter().find(|i| i.name == n).unwrap().rot;
        let (mut ed, top) = solo("3001");
        ed.set_instance(&top, |i| {
            i.pos = [0.0, 0.0, 9.6];
            i.rot = [0.0, 0.0, 90.0];
        });
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id.clone()), None, [8.0, 8.0, 0.0]);
        let under = ed.selection[0].clone();
        ed.add_instance(Some(id), None, [8.0, 24.0, 0.0]);
        let behind = ed.selection[0].clone();
        ed.selection = vec![under.clone(), behind];
        assert!(ed.group_selection("aa"), "{}", ed.status);
        let id = ed.ensure_ldraw_part("3009").unwrap();
        ed.add_instance(Some(id.clone()), None, [16.0, -12.0, 0.0]);
        let a = ed.selection[0].clone();
        ed.add_instance(Some(id), None, [16.0, -4.0, 0.0]);
        let b = ed.selection[0].clone();
        ed.recompute();
        assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
        ed.selection = vec![a.clone(), b.clone()];
        ed.magnet = true;
        let starts = ed.begin_move();
        ed.move_by(&starts, -16.0, 0.0);
        assert!(ed.overlapping.is_empty(), "{:?}", ed.overlapping);
        ed.end_move();
        assert_eq!((pos(&ed, &a), pos(&ed, &b)), ([0.0, -12.0, 0.0], [0.0, -4.0, 0.0]), "{}", ed.status);
        assert_eq!((rot(&ed, &a), rot(&ed, &b)), ([0.0; 3], [0.0; 3]), "the magnet never turns");
        assert!(ed.status.starts_with(&format!("Snapped {a}, {b} into {top}")), "{}", ed.status);
        assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
        // nothing beside it either: the pair keeps the heading it was dragged with, on the
        // four studs it covers (the eight of the turned seat are not the magnet's to take)
        ed.select("aa", false);
        ed.remove_selection();
        ed.selection = vec![a.clone(), b.clone()];
        ed.nudge_selection([16.0, 0.0, 0.0]);
        let starts = ed.begin_move();
        ed.move_by(&starts, -16.0, 0.0);
        ed.end_move();
        assert_eq!((pos(&ed, &a), pos(&ed, &b)), ([0.0, -12.0, 0.0], [0.0, -4.0, 0.0]), "{}", ed.status);
        assert_eq!((rot(&ed, &a), rot(&ed, &b)), ([0.0; 3], [0.0; 3]));
        // a refusal names a brick inside a component by its path
        ed.magnet = false;
        ed.set_pose(&a, [0.0, -12.0, 9.6], [0.0; 3]);
        assert_eq!(pos(&ed, &a), [0.0, -12.0, 0.0]);
        assert_eq!(ed.status, format!("{a} stays: it would overlap {top}"));
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(id), None, [8.0, 8.0, 0.0]);
        let under = ed.selection[0].clone();
        ed.group_selection("bb");
        ed.select(&a, false);
        ed.set_pose(&a, [0.0, 4.0, 0.0], [0.0; 3]);
        assert_eq!(pos(&ed, &a), [0.0, -12.0, 0.0]);
        assert_eq!(ed.status, format!("{a} stays: it would overlap bb/{under}"));
    }

    #[test]
    fn the_magnet_passes_over_a_seat_the_overlap_rule_refuses() {
        // a beam 5 with a pin in its hole at y = -16; a second pin let go 3 mm from that hole
        // and 5 mm from the free one at y = -8 ranks the taken hole first (the least movement)
        // — and lands in the free one, since the first is refused; with no free seat within
        // reach it is refused as before, naming the pin it would hit
        let (mut ed, _) = solo("32316");
        ed.snap_mm = 0.0;
        let pin = ed.ensure_ldraw_part("2780").unwrap();
        ed.add_instance(Some(pin.clone()), None, [0.0, 40.0, 4.0]);
        let p1 = ed.selection[0].clone();
        ed.set_instance(&p1, |i| {
            i.pos = [0.0, -16.0, 4.0];
            i.rot = [0.0, 90.0, 0.0];
        });
        ed.add_instance(Some(pin), None, [0.0, 60.0, 4.0]);
        let p2 = ed.selection[0].clone();
        ed.set_instance(&p2, |i| {
            i.pos = [0.0, -13.0, 4.0];
            i.rot = [0.0, 90.0, 0.0];
        });
        ed.recompute();
        ed.magnet = true;
        ed.snap_selection(true);
        assert_eq!(ed.selected_instances()[0].pos, [0.0, -8.0, 4.0], "{}", ed.status);
        assert!(ed.status.starts_with("Snapped"), "{}", ed.status);
        assert_eq!(ed.overlap_count(), 0);
        ed.set_instance(&p2, |i| i.pos = [0.4, -15.6, 4.3]);
        ed.recompute();
        ed.snap_selection(true);
        assert_eq!(ed.selected_instances()[0].pos, [0.4, -15.6, 4.3]);
        assert_eq!(ed.status, format!("{p2} stays: it would overlap {p1}"));
    }

    #[test]
    fn a_brick_lands_on_an_assembled_hinge() {
        // the hinge pair assembled at the origin, on the ground; a 2 x 4 let go on top of it
        // lands on its studs. The drop puts the brick on the 8 mm grid, 1.6 mm low; the stud
        // grid (four sockets) must then win over the loose tube seat, whose three tubes used
        // to vote twice each (a tube is a stud hole and a pin hole in one place) and kept the
        // brick low, into the hinge, refused
        let (mut ed, _) = solo("3831");
        let ht = ed.ensure_ldraw_part("3830").unwrap();
        ed.add_instance(Some(ht), None, [0.0, 0.0, 0.0]);
        assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
        let id = ed.ensure_ldraw_part("3001").unwrap();
        ed.magnet = true;
        for (at, expect) in [
            ([0.0, 0.0, 9.6], [0.0, 0.0, 9.6]),
            ([0.0, 8.0, 9.6], [0.0, 8.0, 9.6]),
            ([8.0, 0.0, 9.6], [8.0, 0.0, 9.6]),
            ([0.0, 0.0, 19.2], [0.0, 0.0, 19.2]),
        ] {
            ed.add_instance(Some(id.clone()), None, [200.0, 200.0, 9.6]);
            let brick = ed.selection[0].clone();
            ed.set_instance(&brick, |inst| inst.pos = at);
            ed.recompute();
            assert_eq!(ed.overlap_count(), 0, "at {at:?}: {:?}", ed.overlapping);
            let starts = ed.begin_move();
            ed.move_by(&starts, 0.0, 0.0);
            ed.end_move();
            let pos = ed.children().iter().find(|c| c.name == brick).unwrap().pos;
            assert_eq!(pos, expect, "let go at {at:?}: {}", ed.status);
            assert!(!ed.status.contains("put back"), "{}", ed.status);
            assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
            ed.remove_selection();
        }
    }

    #[test]
    fn plates_and_bricks_let_go_at_their_own_heights_stack() {
        // a 2 x 2 plate on a 2 x 2 plate: let go at a plate's height it stays there — the drop
        // used to round it to the 8 mm grid, into the plate below, from where the magnet's
        // nearest seat was the loose tube one, refused as an overlap; near it, it lands there
        let (mut ed, base) = solo("3022");
        let plate = ed.ensure_ldraw_part("3022").unwrap();
        let brick = ed.ensure_ldraw_part("3001").unwrap();
        ed.magnet = true;
        let drop = |ed: &mut Editor, part: &str, at: [f64; 3]| -> ([f64; 3], String) {
            ed.add_instance(Some(part.to_string()), None, [200.0, 200.0, at[2]]);
            let name = ed.selection[0].clone();
            ed.set_instance(&name, |i| i.pos = at);
            ed.recompute();
            let starts = ed.begin_move();
            ed.move_by(&starts, 0.0, 0.0);
            ed.end_move();
            let pos = ed.children().iter().find(|c| c.name == name).unwrap().pos;
            let status = ed.status.clone();
            ed.remove_selection();
            (pos, status)
        };
        for z in [3.2, 2.4, 4.0, 4.8, 6.4] {
            let (pos, status) = drop(&mut ed, &plate, [0.0, 0.0, z]);
            assert_eq!(pos, [0.0, 0.0, 3.2], "a plate let go at {z} on {base}: {status}");
            assert!(!status.contains("put back"), "{status}");
        }
        // a brick on the plate (its origin is its top face: a brick's height up), and a plate
        // on a brick (a plate's height up)
        for z in [9.6, 8.8, 10.4] {
            let (pos, status) = drop(&mut ed, &brick, [0.0, 0.0, z]);
            assert_eq!(pos, [0.0, 0.0, 9.6], "a brick let go at {z} on {base}: {status}");
        }
        ed.add_instance(Some(brick.clone()), None, [48.0, 0.0, 0.0]);
        for z in [3.2, 2.4, 4.0] {
            let (pos, status) = drop(&mut ed, &plate, [48.0, 0.0, z]);
            assert_eq!(pos, [48.0, 0.0, 3.2], "a plate let go at {z} on a brick: {status}");
        }
        // heights land on the module even with nothing to seat on
        let (pos, _) = drop(&mut ed, &plate, [-96.0, -96.0, 5.0]);
        assert_eq!(pos, [-96.0, -96.0, 4.8]);
        ed.snap_mm = 0.0;
        let (pos, _) = drop(&mut ed, &plate, [-96.0, -96.0, 5.0]);
        assert_eq!(pos, [-96.0, -96.0, 5.0], "no grid, no rounding");
    }

    #[test]
    fn a_tile_lands_on_a_plates_studs() {
        // a 1 x 2 tile has no studs of its own, so it had no sockets and nothing to seat on: let
        // go over a 2 x 2 plate it stayed where it fell, half a stud off. It has a socket under
        // every place a stud fits now, and the half-stud grid lets its centre reach the plate's
        // stud row
        let (mut ed, plate) = solo("3022");
        let tile = ed.ensure_ldraw_part("3069b").unwrap();
        ed.magnet = true;
        for at in [[0.0, 4.0, 3.2], [0.8, 3.5, 3.2], [-1.0, 5.0, 4.0]] {
            ed.add_instance(Some(tile.clone()), None, [200.0, 200.0, 3.2]);
            let name = ed.selection[0].clone();
            ed.set_instance(&name, |i| i.pos = at);
            ed.recompute();
            let starts = ed.begin_move();
            ed.move_by(&starts, 0.0, 0.0);
            ed.end_move();
            let pos = ed.children().iter().find(|c| c.name == name).unwrap().pos;
            assert_eq!(pos, [0.0, 4.0, 3.2], "a tile let go at {at:?} on {plate}: {}", ed.status);
            assert_eq!(ed.overlap_count(), 0, "{:?}", ed.overlapping);
            let leaves = assembly::flatten(&ed.doc, &ed.editing);
            let mine: Vec<_> = leaves
                .iter()
                .filter(|l| l.path[0] == name)
                .flat_map(|l| assembly::connectors_of_leaf(&ed.doc, &ed.bundle, l))
                .collect();
            let theirs: Vec<_> = leaves
                .iter()
                .filter(|l| l.path[0] == plate)
                .flat_map(|l| assembly::connectors_of_leaf(&ed.doc, &ed.bundle, l))
                .collect();
            assert!(assembly::seated(&mine, &theirs), "on its studs");
            ed.remove_selection();
        }
        // a half-stud position is a place of its own: the drag grid reaches it
        ed.add_instance(Some(tile), None, [200.0, 200.0, 3.2]);
        let name = ed.selection[0].clone();
        ed.set_instance(&name, |i| i.pos = [40.0, 0.0, 3.2]);
        ed.recompute();
        let starts = ed.begin_move();
        ed.move_by(&starts, 5.0, -3.0);
        assert_eq!(ed.selected_instances()[0].pos, [44.0, -4.0, 3.2]);
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos, [44.0, -4.0, 3.2], "{}", ed.status);
    }

    #[test]
    fn a_file_that_redefines_a_part_refreshes_its_shape() {
        let (mut ed, a) = solo("3001");
        let mut part = Part {
            name: "Box".into(),
            category: "electronics".into(),
            mass_g: 1.0,
            source: "placeholder".into(),
            source_note: String::new(),
            ldraw: None,
            shapes: vec![assembly::Shape::Box {
                size: [10.0, 10.0, 10.0],
                pos: [0.0; 3],
            }],
            extra: Default::default(),
        };
        let id = ed.import_part(part.clone());
        let inst = ed.selection[0].clone();
        ed.set_pose(&inst, [48.0, 0.0, 0.0], [0.0; 3]);
        assert_eq!(ed.overlap_count(), 0, "a 10 mm box 48 mm out clears the brick");
        let dir = std::env::temp_dir().join(format!("ob-shape-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("big.assembly.json");
        part.shapes = vec![assembly::Shape::Box {
            size: [80.0, 10.0, 10.0],
            pos: [0.0; 3],
        }];
        let mut doc = ed.doc.clone();
        doc.parts.insert(id.clone(), part);
        std::fs::write(&path, serde_json::to_string(&doc).unwrap()).unwrap();
        ed.load_path(path);
        assert!(
            ed.status.ends_with(" · one pair of bricks overlaps"),
            "the 80 mm box reaches {a}: {}",
            ed.status
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_fetched_part_carries_its_record_and_opens_without_the_library() {
        // a library with one fetched part: the 2x4 brick's record under another number
        let full = real_bundle();
        let mut rec = full.parts["3001"].clone();
        rec.fetched = true;
        let mut lib = real_bundle();
        lib.parts.insert("2458".into(), rec);
        let mut ed = Editor::new(lib, None);
        let id = ed.ensure_ldraw_part("2458").unwrap();
        let part = ed.doc.parts[&id].clone();
        assert_eq!(part.ldraw.as_deref(), Some("2458"));
        for k in ["mesh", "bbox", "com", "inertia_per_g", "connectors", "volume_mm3"] {
            assert!(part.extra.contains_key(k), "{k} carried along");
        }
        // a shipped part carries nothing extra
        let id2 = ed.ensure_ldraw_part("3001").unwrap();
        assert!(ed.doc.parts[&id2].extra.is_empty());
        ed.add_instance(Some(id.clone()), None, [0.0, 0.0, 40.0]);
        let doc = ed.doc.clone();
        // opened with the shipped library alone: no error, the record's geometry in use
        let other = Editor::new(real_bundle(), Some((PathBuf::from("x.assembly.json"), doc)));
        assert!(other.errors.is_empty(), "{:?}", other.errors);
        let g = assembly::geometry_of(&other.doc.parts[&id], &other.bundle);
        assert!(matches!(g, Geometry::Imported { .. }));
        let leaf = other.leaves.iter().find(|l| l.part_id == id).unwrap();
        assert_eq!(leaf.pos.z, 40.0);
        let conns = assembly::connectors_of_leaf(&other.doc, &other.bundle, leaf);
        assert!(!conns.is_empty(), "its studs and tubes too");
        assert!(
            conns.iter().any(|c| c.kind == "stud_socket"),
            "the sockets under its studs are derived here as for the library's own: {:?}",
            conns.iter().map(|c| c.kind.as_str()).collect::<std::collections::BTreeSet<_>>()
        );
        assert_eq!(
            conns.iter().filter(|c| c.kind == "stud_socket").count(),
            assembly::connectors_of_leaf(&ed.doc, &ed.bundle, ed.leaves.iter().find(|l| l.part_id == id).unwrap())
                .iter()
                .filter(|c| c.kind == "stud_socket")
                .count(),
            "as many as on the machine that fetched it"
        );
        // and the overlap rule measures it: a second copy in the same place is refused
        let mut again = Editor::new(real_bundle(), Some((PathBuf::from("x.assembly.json"), other.doc.clone())));
        again.magnet = false;
        again.add_instance(Some(id.clone()), None, [0.0, 0.0, 40.0]);
        assert_ne!(again.selected_instances()[0].pos, [0.0, 0.0, 40.0], "{}", again.status);
        // forgetting a part checks the document afresh
        let mut third = Editor::new(real_bundle(), None);
        third.doc.parts.insert(
            "ghost".into(),
            Part {
                name: "Ghost".into(),
                category: "lego".into(),
                mass_g: 1.0,
                source: String::new(),
                source_note: String::new(),
                ldraw: Some("0000".into()),
                shapes: vec![],
                extra: Default::default(),
            },
        );
        third.forget_part("0000");
        assert!(third.errors.iter().any(|e| e.contains("0000")), "{:?}", third.errors);
        // what went wrong on the way in stays listed through every fresh check
        third.notes.push("fetched part file /x/2458.json: bundle JSON: EOF".into());
        third.forget_part("0000");
        third.reset_to_example();
        let doc = third.doc.clone();
        third.load_path(PathBuf::from("/nowhere/x.assembly.json"));
        third.restore_draft(1);
        assert_eq!(third.notes, ["fetched part file /x/2458.json: bundle JSON: EOF"]);
        assert!(!third.errors.iter().any(|e| e.contains("2458.json")), "not an error of the build");
        assert_eq!(third.doc.components.len(), doc.components.len());
    }

    #[test]
    fn locked_instances_stay_put_until_unlocked() {
        let (mut ed, a) = solo("32278");
        let id = ed.ensure_ldraw_part("32278").unwrap();
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
        // two modules over, or further along -y where that spot overlaps a neighbour
        assert!(pasted[0].pos[1] <= assembly::round3(original[1] - 16.0), "{:?}", pasted[0].pos);
        assert_eq!((pasted[0].pos[0], pasted[0].pos[2]), (original[0], original[2]));
        assert!(ed.status.starts_with("Pasted 2"), "{}", ed.status);
        // into another component: positions stay
        let cid = comp_inst.component.clone().unwrap();
        ed.selection = vec![brick.clone()];
        let brick_text = ed.copy_selection().unwrap();
        ed.open_component(&cid, true);
        let n = ed.children().len();
        assert_eq!(ed.paste(&brick_text), 1);
        assert_eq!(ed.children().len(), n + 1);
        let at = ed.children()[n].pos;
        assert_eq!((at[0], at[2]), (original[0], original[2]));
        assert!(at[1] <= original[1], "{at:?}: where it was, or along -y when that overlaps");
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
    fn imported_parts_get_unique_ids_and_a_place_in_the_view() {
        let mut ed = editor();
        ed.magnet = false;
        let part = |name: &str| Part {
            name: name.into(),
            category: "other".into(),
            mass_g: 5.0,
            source: "measured".into(),
            source_note: String::new(),
            ldraw: None,
            shapes: vec![crate::assembly::Shape::Box {
                size: [10.0, 10.0, 10.0],
                pos: [0.0; 3],
            }],
            extra: Default::default(),
        };
        let n = ed.children().len();
        assert_eq!(ed.import_part(part("Sensor Bracket")), "sensor_bracket");
        assert_eq!(ed.import_part(part("Sensor Bracket")), "sensor_bracket_2");
        assert_eq!(ed.import_part(part("???")), "part");
        assert_eq!(ed.children().len(), n + 3);
        assert_eq!(ed.selection, vec!["part".to_string()]);
        assert_eq!(ed.status, "??? is in the library and in the view");
        assert!(ed.doc.parts.contains_key("sensor_bracket_2"));
    }

    #[test]
    fn a_plate_pulls_onto_a_brick_s_stud_grid_and_is_never_turned() {
        let mut ed = editor();
        let root = ed.doc.robot.root.clone();
        ed.doc.components.get_mut(&root).unwrap().children.clear();
        ed.doc.robot.roles.clear();
        ed.recompute();
        ed.magnet = false;
        ed.snap_mm = 0.0;
        let brick = ed.ensure_ldraw_part("3001").unwrap();
        ed.add_instance(Some(brick), None, [0.0; 3]);
        let plate = ed.ensure_ldraw_part("3022").unwrap();
        // a 2 x 2 plate let go above the brick (clear of its studs), a little off and turned 5°:
        // its four sockets agree on the shift onto the studs, but turned 5° its walls stand on
        // them — the magnet used to square it up; it never turns a part now, no seat as good
        // fits (the two-stud seat beside is no substitute), and the plate is refused as let go
        ed.add_instance(Some(plate.clone()), None, [2.0, 3.0, 5.0]);
        let p1 = ed.selection[0].clone();
        ed.set_pose(&p1, [2.0, 3.0, 5.0], [0.0, 0.0, 5.0]);
        ed.magnet = true;
        ed.snap_selection(true);
        let i = ed.selected_instances()[0].clone();
        assert_eq!((i.pos, i.rot), ([2.0, 3.0, 5.0], [0.0, 0.0, 5.0]), "{}", ed.status);
        assert_eq!(ed.status, format!("{p1} stays: it would overlap lego_3001"));
        // set square, it seats on all four
        ed.set_pose(&p1, [2.0, 3.0, 5.0], [0.0; 3]);
        ed.snap_selection(true);
        assert!(ed.status.starts_with("Snapped"), "{}", ed.status);
        let i = ed.selected_instances()[0].clone();
        assert_eq!(i.pos, [0.0, 0.0, 3.2], "its underside on the brick's top face, grids in step");
        assert_eq!(i.rot, [0.0; 3]);
        let cons = assembly::connections_of(&ed.doc, &ed.bundle, &ed.editing, &p1);
        assert_eq!(
            cons.iter().filter(|(m, o, _)| m == "stud_socket" && o == "stud").count(),
            4,
            "{cons:?}"
        );
        // a second plate on the first, 30° out of square: it used to be squared up on the
        // plate's four studs; now it goes down on the one stud a socket of it can take, turned
        // 30° as it was let go
        ed.magnet = false;
        ed.add_instance(Some(plate), None, [1.0, -2.0, 8.0]);
        let p2 = ed.selection[0].clone();
        ed.set_pose(&p2, [1.0, -2.0, 8.0], [0.0, 0.0, 30.0]);
        ed.magnet = true;
        ed.snap_selection(true);
        let i = ed.selected_instances()[0].clone();
        assert_eq!((i.pos[2], i.rot), (6.4, [0.0, 0.0, 30.0]), "{}", ed.status);
        assert!(ed.status.starts_with("Snapped"), "{}", ed.status);
        // squared by hand, it lands on the plate's four studs
        ed.set_pose(&p2, [1.0, -2.0, 8.0], [0.0; 3]);
        ed.snap_selection(true);
        let i = ed.selected_instances()[0].clone();
        assert_eq!((i.pos, i.rot), ([0.0, 0.0, 6.4], [0.0, 0.0, 0.0]), "{}", ed.status);
        // dragged a little, it stays pulled; dragged clear, it lets go; let go near, it lands
        let starts = ed.begin_move();
        ed.move_by(&starts, 2.5, 1.0);
        assert_eq!(ed.selected_instances()[0].pos, [0.0, 0.0, 6.4], "pulled back onto the grid");
        ed.move_by(&starts, 30.0, 0.0);
        assert_eq!(ed.selected_instances()[0].pos, [30.0, 0.0, 6.4], "free of the magnet");
        ed.move_by(&starts, 9.0, 0.0);
        assert_eq!(ed.selected_instances()[0].pos, [8.0, 0.0, 6.4], "one stud over");
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos, [8.0, 0.0, 6.4]);
        // the magnet off: a drag is just a drag
        ed.magnet = false;
        ed.move_by(&starts, 2.5, 1.0);
        assert_eq!(ed.selected_instances()[0].pos, [2.5, 1.0, 6.4]);
    }

    #[test]
    fn a_beam_askew_on_two_pins_is_not_squared_up() {
        let mut ed = editor();
        let root = ed.doc.robot.root.clone();
        ed.doc.components.get_mut(&root).unwrap().children.clear();
        ed.doc.robot.roles.clear();
        ed.recompute();
        ed.magnet = false;
        ed.snap_mm = 0.0;
        // a beam 5 flat on the ground, two friction pins standing in its holes at y = ±8, half up
        let beam = ed.ensure_ldraw_part("32316").unwrap();
        ed.add_instance(Some(beam.clone()), None, [0.0; 3]);
        let pin = ed.ensure_ldraw_part("2780").unwrap();
        let mut pins = vec![];
        for y in [-8.0, 8.0] {
            ed.add_instance(Some(pin.clone()), None, [0.0, y, 4.0]);
            let name = ed.selection[0].clone();
            ed.set_pose(&name, [0.0, y, 4.0], [0.0, 90.0, 0.0]);
            pins.push(name);
        }
        // a second beam let go above them, off by a little and turned 10° (the pose a drag
        // passes through: set directly, as the pins would cross its holes there): the magnet
        // used to square it up on both; it never turns a part now, and the beam, 10° off with
        // one hole on its pin, has the other pin through its wall — refused
        ed.add_instance(Some(beam), None, [0.0, 0.0, 40.0]);
        let top = ed.selection[0].clone();
        ed.set_instance(&top, |i| {
            i.pos = [0.5, 1.0, 8.6];
            i.rot = [0.0, 0.0, 10.0];
        });
        ed.recompute();
        ed.magnet = true;
        ed.snap_selection(true);
        let i = ed.selected_instances()[0].clone();
        assert_eq!((i.pos, i.rot), ([0.5, 1.0, 8.6], [0.0, 0.0, 10.0]), "{}", ed.status);
        assert!(
            pins.iter().any(|p| ed.status == format!("{top} stays: it would overlap {p}")),
            "{}",
            ed.status
        );
        // squared by hand, it seats on both
        ed.set_instance(&top, |i| i.rot = [0.0; 3]);
        ed.recompute();
        ed.snap_selection(true);
        assert!(ed.status.starts_with("Snapped"), "{}", ed.status);
        let i = ed.selected_instances()[0].clone();
        assert_eq!((i.pos, i.rot), ([0.0, 0.0, 8.0], [0.0, 0.0, 0.0]));
        let cons = assembly::connections_of(&ed.doc, &ed.bundle, &ed.editing, &top);
        assert_eq!(cons.iter().filter(|(m, o, _)| m == "pin_hole" && o == "pin").count(), 2, "{cons:?}");
    }

    #[test]
    fn plane_drags_snap_and_the_lift_lands_on_the_height_module() {
        let (mut ed, _) = solo("32278");
        let starts = ed.begin_move();
        assert_eq!(starts.len(), 1);
        ed.move_by(&starts, 11.0, -3.0);
        assert_eq!(ed.selected_instances()[0].pos, [12.0, -4.0, 0.0], "half-stud steps");
        ed.lift_by(&starts, 5.2);
        assert_eq!(ed.selected_instances()[0].pos[2], 5.2);
        ed.end_move();
        assert_eq!(
            ed.selected_instances()[0].pos,
            [12.0, -4.0, 4.8],
            "a stud\'s height, not the plan grid"
        );
        ed.snap_mm = 0.0;
        ed.move_by(&starts, 1.2345, 0.0);
        assert_eq!(ed.selected_instances()[0].pos[0], 1.235);
        ed.lift_by(&starts, 0.3);
        ed.end_move();
        assert_eq!(ed.selected_instances()[0].pos[2], 5.1);
        assert_eq!(ed.snap(3.9), 3.9);
    }

    #[test]
    fn handle_drags_move_and_turn_about_the_pivot() {
        let (mut ed, a) = solo("32278");
        let id = ed.ensure_ldraw_part("32278").unwrap();
        ed.add_instance(Some(id), None, [16.0, 0.0, 0.0]);
        let b = ed.selection[0].clone();
        ed.selection = vec![a.clone(), b.clone()];
        let starts = ed.begin_handle();
        assert_eq!(starts.len(), 2);
        ed.drag_handle(Handle::Axis(2), 13.0, DVec3::ZERO, &starts, false);
        let insts = ed.selected_instances();
        assert_eq!(insts[0].pos, [0.0, 0.0, 12.0], "half-stud steps");
        assert_eq!(insts[1].pos, [16.0, 0.0, 12.0]);
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
