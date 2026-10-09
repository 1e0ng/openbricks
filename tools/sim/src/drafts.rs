//! Unsaved work kept safe: a draft of the Workbench's build and of the
//! route, written under the data directory a moment after each change
//! settles and offered back on the next launch, so a forgotten Save
//! loses nothing. A draft is the document as a file, beside a note of
//! where the work belongs (the file it was opened from or saved to, if
//! any) and when it was kept.

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// How long a change must have settled before the draft is written.
pub const SETTLE: Duration = Duration::from_secs(2);
pub const BUILD: &str = "workbench.assembly.json";
pub const ROUTE: &str = "route.json";

/// Where drafts live: `<data dir>/drafts`.
pub fn dir() -> PathBuf {
    crate::markers::data_dir().join("drafts")
}

/// Where a draft belongs and when it was kept.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Default)]
pub struct Note {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<PathBuf>,
    /// Milliseconds since the epoch.
    #[serde(default)]
    pub kept_ms: i64,
}

fn note_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(format!("{name}.note.json"))
}

/// Keep `text` as the draft `name`, with its note. Each file lands whole:
/// written aside, then moved into place.
pub fn keep(dir: &Path, name: &str, text: &str, note: &Note) -> Result<(), String> {
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let write = |path: PathBuf, body: String| -> Result<(), String> {
        let tmp = path.with_extension("part");
        std::fs::write(&tmp, body).map_err(|e| format!("{}: {e}", tmp.display()))?;
        std::fs::rename(&tmp, &path).map_err(|e| format!("{}: {e}", path.display()))
    };
    write(dir.join(name), text.to_string())?;
    write(note_path(dir, name), serde_json::to_string_pretty(note).map_err(|e| e.to_string())?)
}

/// The draft `name`, if one is kept: its text, and its note — or why the
/// note could not be read (none beside the draft, or not a note), named
/// by file, for the restore to say rather than stand a default in for
/// where the work belongs and when it was kept.
pub fn kept(dir: &Path, name: &str) -> Option<(String, Result<Note, String>)> {
    let text = std::fs::read_to_string(dir.join(name)).ok()?;
    let at = note_path(dir, name);
    let note = std::fs::read_to_string(&at)
        .map_err(|e| format!("{}: {e}", at.display()))
        .and_then(|t| serde_json::from_str(&t).map_err(|e| format!("{}: {e}", at.display())));
    Some((text, note))
}

/// The tests' view of a draft: its text and its note, a note that cannot
/// be read failing the test that asked for it.
#[cfg(test)]
pub fn take(dir: &Path, name: &str) -> Option<(String, Note)> {
    kept(dir, name).map(|(text, note)| (text, note.unwrap_or_else(|why| panic!("{why}"))))
}

/// Drop the draft `name`: the work was saved, or a file took its place.
pub fn drop(dir: &Path, name: &str) {
    let _ = std::fs::remove_file(dir.join(name));
    let _ = std::fs::remove_file(note_path(dir, name));
}

/// Now, in milliseconds since the epoch.
pub fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

/// How long ago `kept_ms` was, for a status line.
pub fn ago(kept_ms: i64, now_ms: i64) -> String {
    let s = ((now_ms - kept_ms).max(0) / 1000) as u64;
    let unit = |n: u64, one: &str, many: &str| {
        if n == 1 {
            format!("1 {one} ago")
        } else {
            format!("{n} {many} ago")
        }
    };
    match s {
        0..=59 => "moments ago".into(),
        60..=3599 => unit(s / 60, "min", "min"),
        3600..=86_399 => unit(s / 3600, "hour", "hours"),
        _ => unit(s / 86_400, "day", "days"),
    }
}

/// Watches one document for changes and says when its draft is due:
/// every new revision restarts the clock, so the draft is due once the
/// last change has settled for `SETTLE` (a drag, a run of nudges or of
/// keystrokes is kept after it ends, not part way through), and the
/// revision it was kept at is remembered so nothing is rewritten
/// unchanged.
#[derive(Debug, Default)]
pub struct Keeper {
    changed_at: Option<Instant>,
    /// The revision last seen: a different one is a change.
    seen_rev: u64,
    pub kept_rev: u64,
}

impl Keeper {
    /// The document is at `rev` now; a new revision (re)starts the clock.
    pub fn changed(&mut self, rev: u64, now: Instant) {
        if rev != self.kept_rev && rev != self.seen_rev {
            self.changed_at = Some(now);
        }
        self.seen_rev = rev;
    }

    pub fn due(&self, now: Instant) -> bool {
        self.changed_at.is_some_and(|t| now.duration_since(t) >= SETTLE)
    }

    /// The draft was kept (or dropped) at `rev`.
    pub fn kept(&mut self, rev: u64) {
        self.changed_at = None;
        self.kept_rev = rev;
        self.seen_rev = rev;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drafts_are_kept_taken_and_dropped_with_their_notes() {
        let dir = std::env::temp_dir().join(format!("ob-drafts-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(take(&dir, BUILD), None, "no directory yet");
        let note = Note {
            path: Some(PathBuf::from("/builds/robot.assembly.json")),
            kept_ms: 1_700_000_000_000,
        };
        keep(&dir, BUILD, "{\"a\": 1}", &note).unwrap();
        assert_eq!(take(&dir, BUILD), Some(("{\"a\": 1}".to_string(), note.clone())));
        assert!(!dir.join("workbench.assembly.part").exists(), "written aside, then moved");
        assert_eq!(kept(&dir, BUILD), Some(("{\"a\": 1}".to_string(), Ok(note.clone()))));
        // a draft without a note still comes back, saying the note is missing, by file: never
        // a note standing in (no file, kept at the epoch) for the restore to report
        let note_file = dir.join("workbench.assembly.json.note.json");
        std::fs::remove_file(&note_file).unwrap();
        let (text, missing) = kept(&dir, BUILD).unwrap();
        assert_eq!(text, "{\"a\": 1}");
        let why = missing.unwrap_err();
        assert!(
            why.starts_with(&note_file.display().to_string()) && why.contains("os error"),
            "{why}"
        );
        // a note that is not JSON is said so, by file
        std::fs::write(&note_file, "nope").unwrap();
        let why = kept(&dir, BUILD).unwrap().1.unwrap_err();
        assert!(
            why.starts_with(&note_file.display().to_string()) && why.contains("expected"),
            "{why}"
        );
        assert_eq!(kept(&dir, BUILD).unwrap().0, "{\"a\": 1}", "the draft comes back all the same");
        std::fs::write(&note_file, serde_json::to_string(&note).unwrap()).unwrap();
        keep(&dir, ROUTE, "{}", &Note::default()).unwrap();
        drop(&dir, BUILD);
        assert_eq!(take(&dir, BUILD), None);
        assert!(take(&dir, ROUTE).is_some(), "the other draft stays");
        drop(&dir, ROUTE);
        drop(&dir, ROUTE);
        assert!(keep(&dir.join("file-in-the-way"), BUILD, "x", &Note::default()).is_ok());
        std::fs::write(dir.join("blocker"), "").unwrap();
        assert!(
            keep(&dir.join("blocker"), BUILD, "x", &Note::default()).is_err(),
            "a file where the directory should be"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_keeper_is_due_once_a_change_has_settled() {
        let t0 = Instant::now();
        let mut k = Keeper::default();
        k.changed(0, t0);
        assert!(!k.due(t0 + SETTLE * 3), "nothing changed: revision 0 was kept");
        k.changed(1, t0);
        assert!(!k.due(t0 + Duration::from_millis(1900)));
        k.changed(2, t0 + Duration::from_millis(1900));
        assert!(!k.due(t0 + SETTLE), "a change just before the deadline postpones it");
        k.changed(2, t0 + SETTLE);
        assert!(!k.due(t0 + SETTLE), "the same revision again is no change");
        assert!(
            k.due(t0 + Duration::from_millis(1900) + SETTLE),
            "due once the last change has settled"
        );
        k.kept(2);
        assert!(!k.due(t0 + SETTLE * 4));
        k.changed(2, t0 + SETTLE * 4);
        assert!(!k.due(t0 + SETTLE * 9), "unchanged since it was kept");
        k.changed(3, t0 + SETTLE * 4);
        assert!(k.due(t0 + SETTLE * 6));
        assert_eq!(ago(1_000, 31_000), "moments ago");
        assert_eq!(ago(0, 60_000), "1 min ago");
        assert_eq!(ago(0, 150_000), "2 min ago");
        assert_eq!(ago(0, 3_600_000), "1 hour ago");
        assert_eq!(ago(0, 7_300_000), "2 hours ago");
        assert_eq!(ago(0, 86_400_000), "1 day ago");
        assert_eq!(ago(0, 200_000_000), "2 days ago");
        assert_eq!(ago(5_000, 1_000), "moments ago", "a clock set back");
        assert!(now_ms() > 1_600_000_000_000);
    }

    #[test]
    fn a_build_draft_whose_note_cannot_be_read_is_restored_and_said_so() {
        // the Workbench's restore: the build comes back; where it belongs and when it was kept
        // are said to be unknown, by file and reason, never reported from a note standing in
        let dir = std::env::temp_dir().join(format!("ob-drafts-note-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let bundle_path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../openbricks/openbricks_sim/bricks/technic_bundle.json.zlib");
        let bundle = crate::bundle::load_bundle(&bundle_path).expect("the shipped brick bundle");
        let mut ed = crate::editor::Editor::new(bundle, None);
        ed.draft_dir = dir.clone();
        let note = Note {
            path: Some(PathBuf::from("/builds/robot.assembly.json")),
            kept_ms: 1_000,
        };
        keep(&dir, BUILD, crate::assembly::EXAMPLE, &note).unwrap();
        let note_file = note_path(&dir, BUILD);
        std::fs::remove_file(&note_file).unwrap();
        assert!(ed.restore_draft(2_000_000_000));
        assert!(ed.dirty && ed.path.is_none());
        assert!(
            ed.status.starts_with("Restored the unsaved draft; its note could not be read (")
                && ed.status.contains(&note_file.display().to_string())
                && ed.status.contains("are unknown: Save as… gives it a file")
                && !ed.status.contains("days ago"),
            "{}",
            ed.status
        );
        // with its note, as before
        keep(&dir, BUILD, crate::assembly::EXAMPLE, &note).unwrap();
        assert!(ed.restore_draft(1_000 + 3_600_000));
        assert!(
            ed.status
                .starts_with("Restored the unsaved draft kept 1 hour ago of /builds/robot.assembly.json"),
            "{}",
            ed.status
        );
        assert_eq!(ed.path, note.path);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_run_of_changes_is_kept_after_its_last_one_not_part_way_through() {
        // a drag: a new revision every 100 ms for 5 s, never due while it goes on, due 2 s after
        // it ends
        let t0 = Instant::now();
        let mut k = Keeper::default();
        let mut rev = 0;
        for i in 0..=50 {
            rev += 1;
            let now = t0 + Duration::from_millis(100 * i);
            k.changed(rev, now);
            assert!(!k.due(now), "still changing at {i}");
        }
        let last = t0 + Duration::from_millis(5000);
        assert!(!k.due(last + Duration::from_millis(1999)));
        assert!(k.due(last + SETTLE));
        k.kept(rev);
        assert!(!k.due(last + SETTLE * 3));
        // seen but kept: a revision seen before the draft was kept at it is not a change after
        k.changed(rev, last + SETTLE * 3);
        assert!(!k.due(last + SETTLE * 6));
    }
}
