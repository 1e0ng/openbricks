//! Map markers: points the user names on a map — a corner of the mat, a
//! mission object, a line junction — kept per map on this machine so
//! they are there again whenever that map loads, and offered as snap
//! targets while placing a route's actions.
//!
//! They are not part of a route file: a route is a plan, the markers
//! are what the user knows about the map. Each map's markers live in
//! `<data dir>/markers/<map>.json`; the data dir is
//! `$OPENBRICKS_DATA_DIR`, else `$XDG_DATA_HOME/openbricks`, else
//! `~/.local/share/openbricks`.

use crate::route::Point;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

pub const FORMAT: &str = "openbricks-markers/1";

/// One named point on the map.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Marker {
    pub name: String,
    pub at: Point,
}

/// A map's markers, as stored.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Markers {
    pub format: String,
    #[serde(default)]
    pub world: String,
    #[serde(default)]
    pub markers: Vec<Marker>,
}

/// Where this machine keeps its markers (and other per-user data).
pub fn data_dir() -> PathBuf {
    data_dir_from(
        std::env::var_os("OPENBRICKS_DATA_DIR"),
        std::env::var_os("XDG_DATA_HOME"),
        std::env::var_os("HOME"),
    )
}

/// The data dir from the environment's three candidates: an explicit
/// `OPENBRICKS_DATA_DIR`, else `XDG_DATA_HOME/openbricks`, else
/// `~/.local/share/openbricks` (the working directory when there is no
/// home at all).
pub fn data_dir_from(explicit: Option<std::ffi::OsString>, xdg: Option<std::ffi::OsString>, home: Option<std::ffi::OsString>) -> PathBuf {
    if let Some(d) = explicit.filter(|d| !d.is_empty()) {
        return PathBuf::from(d);
    }
    if let Some(x) = xdg.filter(|x| !x.is_empty()) {
        return PathBuf::from(x).join("openbricks");
    }
    let home = home.map(PathBuf::from).unwrap_or_else(|| PathBuf::from("."));
    home.join(".local").join("share").join("openbricks")
}

fn file_stem(world: &str) -> String {
    let stem: String = world
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-') {
                c
            } else {
                '_'
            }
        })
        .collect();
    if stem.is_empty() { "map".into() } else { stem }
}

impl Markers {
    pub fn empty(world: &str) -> Markers {
        Markers {
            format: FORMAT.into(),
            world: world.into(),
            markers: vec![],
        }
    }

    /// The file a map's markers live in under `dir`.
    pub fn file(dir: &Path, world: &str) -> PathBuf {
        dir.join("markers").join(format!("{}.json", file_stem(world)))
    }

    /// The markers stored for a map: none when there is no file yet,
    /// an error when the file is there but not usable.
    pub fn load(dir: &Path, world: &str) -> Result<Markers, String> {
        let path = Self::file(dir, world);
        let text = match std::fs::read_to_string(&path) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Self::empty(world)),
            Err(e) => return Err(format!("could not read {}: {e}", path.display())),
        };
        let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        if v.get("format").and_then(|f| f.as_str()) != Some(FORMAT) {
            return Err(format!("{} is not a markers file (expected {FORMAT})", path.display()));
        }
        let mut m: Markers = serde_json::from_value(v).map_err(|e| format!("{}: {e}", path.display()))?;
        m.world = world.into();
        Ok(m)
    }

    /// Store the markers for their map under `dir`; returns the file.
    pub fn save(&self, dir: &Path) -> Result<PathBuf, String> {
        let path = Self::file(dir, &self.world);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|e| format!("could not create {}: {e}", parent.display()))?;
        }
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(&path, text).map_err(|e| format!("could not write {}: {e}", path.display()))?;
        Ok(path)
    }

    /// The first unused name of the form `M1`, `M2`, ...
    pub fn next_name(&self) -> String {
        (1..)
            .map(|n| format!("M{n}"))
            .find(|n| !self.markers.iter().any(|m| &m.name == n))
            .unwrap()
    }

    pub fn points(&self) -> Vec<Point> {
        self.markers.iter().map(|m| m.at).collect()
    }

    /// The marker nearest a map point within `tol_mm`, and how far it is.
    pub fn nearest(&self, p: Point, tol_mm: f64) -> Option<(usize, f64)> {
        self.markers
            .iter()
            .enumerate()
            .map(|(i, m)| (i, ((m.at[0] - p[0]).powi(2) + (m.at[1] - p[1]).powi(2)).sqrt()))
            .filter(|(_, d)| *d <= tol_mm)
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn markers_are_kept_per_map_and_come_back() {
        let dir = std::env::temp_dir().join(format!("ob-markers-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        // nothing stored yet: an empty set, no error
        let m = Markers::load(&dir, "practice-line").unwrap();
        assert_eq!(m, Markers::empty("practice-line"));
        assert_eq!(m.next_name(), "M1");
        let mut m = m;
        m.markers.push(Marker {
            name: "M1".into(),
            at: [100.0, -50.0],
        });
        m.markers.push(Marker {
            name: "corner".into(),
            at: [1100.0, 850.0],
        });
        assert_eq!(m.next_name(), "M2");
        let path = m.save(&dir).unwrap();
        assert_eq!(path, dir.join("markers").join("practice-line.json"));
        assert_eq!(Markers::load(&dir, "practice-line").unwrap(), m);
        assert_eq!(
            Markers::load(&dir, "wro-2026-senior").unwrap().markers.len(),
            0,
            "another map has its own"
        );
        // nearest within a tolerance; the points for snapping
        assert_eq!(m.nearest([104.0, -52.0], 10.0).map(|n| n.0), Some(0));
        assert_eq!(m.nearest([104.0, -52.0], 2.0), None);
        assert_eq!(m.points(), vec![[100.0, -50.0], [1100.0, 850.0]]);
        // odd map names still make a file name; a foreign or broken file is an error, not silence
        assert_eq!(Markers::file(&dir, "my map/v2").file_name().unwrap(), "my_map_v2.json");
        assert_eq!(Markers::file(&dir, "").file_name().unwrap(), "map.json");
        std::fs::write(Markers::file(&dir, "broken"), "{not json").unwrap();
        assert!(Markers::load(&dir, "broken").unwrap_err().contains("broken.json"));
        std::fs::write(Markers::file(&dir, "foreign"), r#"{"format":"something-else"}"#).unwrap();
        assert!(Markers::load(&dir, "foreign").unwrap_err().contains("not a markers file"));
        std::fs::write(Markers::file(&dir, "shape"), r#"{"format":"openbricks-markers/1","markers":5}"#).unwrap();
        assert!(Markers::load(&dir, "shape").unwrap_err().contains("shape.json"));
        std::fs::create_dir_all(Markers::file(&dir, "adir")).unwrap();
        assert!(
            Markers::load(&dir, "adir").unwrap_err().contains("could not read"),
            "a directory in the file's place"
        );
        // saving where no directory can be made fails loudly
        let blocked = dir.join("blocker");
        std::fs::write(&blocked, "x").unwrap();
        let e = Markers::empty("practice-line").save(&blocked).unwrap_err();
        assert!(e.contains("could not create") || e.contains("could not write"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_data_dir_follows_the_environment() {
        let os = |s: &str| Some(std::ffi::OsString::from(s));
        assert_eq!(data_dir_from(os("/x/ob"), os("/y"), os("/h")), PathBuf::from("/x/ob"));
        assert_eq!(
            data_dir_from(os(""), os("/y"), os("/h")),
            PathBuf::from("/y/openbricks"),
            "an empty override does not count"
        );
        assert_eq!(data_dir_from(None, os(""), os("/h")), PathBuf::from("/h/.local/share/openbricks"));
        assert_eq!(data_dir_from(None, None, None), PathBuf::from("./.local/share/openbricks"));
        // the real one has the same shape
        assert!(data_dir().to_string_lossy().contains("openbricks") || std::env::var_os("OPENBRICKS_DATA_DIR").is_some());
    }
}
