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

/// The draft `name`, if one is kept: its text and its note (a note that
/// cannot be read counts as none: no file, kept at the epoch).
pub fn take(dir: &Path, name: &str) -> Option<(String, Note)> {
    let text = std::fs::read_to_string(dir.join(name)).ok()?;
    let note = std::fs::read_to_string(note_path(dir, name))
        .ok()
        .and_then(|t| serde_json::from_str(&t).ok())
        .unwrap_or_default();
    Some((text, note))
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
/// the first change of a run starts the clock, the draft is due once
/// the run has settled for `SETTLE`, and the revision it was kept at
/// is remembered so nothing is rewritten unchanged.
#[derive(Debug, Default)]
pub struct Keeper {
    changed_at: Option<Instant>,
    pub kept_rev: u64,
}

impl Keeper {
    /// The document is at `rev` now; a new revision starts the clock.
    pub fn changed(&mut self, rev: u64, now: Instant) {
        if rev != self.kept_rev && self.changed_at.is_none() {
            self.changed_at = Some(now);
        }
    }

    pub fn due(&self, now: Instant) -> bool {
        self.changed_at.is_some_and(|t| now.duration_since(t) >= SETTLE)
    }

    /// The draft was kept (or dropped) at `rev`.
    pub fn kept(&mut self, rev: u64) {
        self.changed_at = None;
        self.kept_rev = rev;
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
        // a draft without a note still comes back
        std::fs::remove_file(dir.join("workbench.assembly.json.note.json")).unwrap();
        assert_eq!(take(&dir, BUILD), Some(("{\"a\": 1}".to_string(), Note::default())));
        // a note that is not JSON counts as none
        std::fs::write(dir.join("workbench.assembly.json.note.json"), "nope").unwrap();
        assert_eq!(take(&dir, BUILD).unwrap().1, Note::default());
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
        assert!(k.due(t0 + SETTLE), "the first change of a run started the clock");
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
}
