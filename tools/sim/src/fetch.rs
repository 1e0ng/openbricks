//! A part fetched by number: `python -m openbricks_sim.bricks.fetch
//! NUMBER --out FILE` run as a child and read on threads, the way the
//! run server is. It speaks JSON lines: `log` as files come down,
//! then `fetched` (the file it wrote) or `error` (ldraw.org has no
//! such part, or what went wrong). A fetcher that dies instead is
//! reported with its last words: the stdout reader waits for the
//! stderr reader before it marks the exit, so a traceback is never
//! outrun by the marker.

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;

/// How a fetch ended.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// The part is in the file at `out`: its name, how many files came
    /// down, how many colours, and a note when it has none (Rebrickable
    /// lists none for it).
    Fetched {
        name: String,
        files: u64,
        colors: u64,
        note: Option<String>,
        out: PathBuf,
    },
    Failed(String),
}

enum Line {
    Log(String),
    /// A line on stderr: a warning, or a traceback on the way out.
    Stderr(String),
    Fetched(Outcome),
    Error(String),
    Exited,
}

/// A fetch in progress.
pub struct PartFetch {
    pub number: String,
    child: Child,
    lines: Receiver<Line>,
    /// The last thing the fetcher said, for the library's row.
    pub last: String,
    /// The last thing it said on stderr: a dying fetcher's reason.
    last_err: Option<String>,
}

impl PartFetch {
    /// Start fetching `number` into `out` with `python`; `env` adds
    /// environment variables (tests point PYTHONPATH at a stand-in).
    pub fn start(python: &str, env: &[(String, String)], number: &str, out: &Path) -> Result<PartFetch, String> {
        let mut child = Command::new(python)
            .args(["-u", "-m", "openbricks_sim.bricks.fetch", number, "--out"])
            .arg(out)
            .envs(env.iter().map(|(k, v)| (k.as_str(), v.as_str())))
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| format!("could not start {python}: {e}"))?;
        let stdout = child.stdout.take().ok_or("no stdout")?;
        let stderr = child.stderr.take().ok_or("no stderr")?;
        let (tx, lines) = mpsc::channel();
        let tx_err = tx.clone();
        let errs = thread::spawn(move || {
            // read to the end whether or not anyone still listens: the pipe closes with the child
            for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                let _ = tx_err.send(Line::Stderr(line));
            }
        });
        thread::spawn(move || {
            for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                if line.trim().is_empty() {
                    continue;
                }
                let msg = match parse(&line) {
                    Ok(l) => l,
                    Err(e) => Line::Error(format!("{e}: {}", head(&line, 200))),
                };
                if tx.send(msg).is_err() {
                    return;
                }
            }
            // the child's stderr closes with its stdout: whatever it said last is in by now
            let _ = errs.join();
            let _ = tx.send(Line::Exited);
        });
        Ok(PartFetch {
            number: number.to_string(),
            child,
            lines,
            last: "starting".into(),
            last_err: None,
        })
    }

    /// What the fetcher has said since last time; the outcome once it
    /// is known (after which the fetch is over).
    pub fn poll(&mut self) -> Option<Outcome> {
        loop {
            match self.lines.try_recv() {
                Ok(Line::Log(t)) => self.last = t,
                Ok(Line::Stderr(t)) => {
                    self.last_err = Some(t.clone());
                    self.last = t;
                }
                Ok(Line::Fetched(o)) => return Some(o),
                Ok(Line::Error(t)) => return Some(Outcome::Failed(t)),
                Ok(Line::Exited) => {
                    // its reason, when it gave one on the way out; else the last thing it said
                    let code = self.child.wait().ok().and_then(|s| s.code());
                    return Some(Outcome::Failed(format!(
                        "the fetcher stopped without a word (exit code {}): {}",
                        code.map_or("none".to_string(), |c| c.to_string()),
                        self.last_err.as_deref().unwrap_or(&self.last)
                    )));
                }
                Err(TryRecvError::Empty) => return None,
                // the readers are gone without the exit marker: one of them died
                Err(TryRecvError::Disconnected) => {
                    let _ = self.child.kill();
                    let _ = self.child.wait();
                    return Some(Outcome::Failed(format!(
                        "the fetcher's output could not be read to the end: {}",
                        self.last
                    )));
                }
            }
        }
    }
}

#[cfg(test)]
impl PartFetch {
    /// What a reader that died looks like from here: a channel nobody sends on.
    fn readers_gone(&mut self) {
        self.lines = mpsc::channel().1;
    }
}

impl Drop for PartFetch {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// The first `n` characters of a line (a byte slice could cut a
/// character in two, and the child's output is anyone's).
fn head(s: &str, n: usize) -> String {
    s.chars().take(n).collect()
}

/// One line of the fetcher's: the fields an event always carries are
/// required, so a line short of one (a fetcher of another version, a
/// name that came out null) is an error naming the field, never a part
/// with no name, no files and the sim's own guess of its path. Only
/// `note` is optional: the fetcher sends it when Rebrickable lists no
/// colours.
fn parse(line: &str) -> Result<Line, String> {
    let v: serde_json::Value = serde_json::from_str(line).map_err(|e| format!("{e}"))?;
    let ev = v.get("ev").and_then(serde_json::Value::as_str).ok_or("event without ev")?;
    let s = |k: &str| {
        v.get(k)
            .and_then(serde_json::Value::as_str)
            .map(str::to_string)
            .ok_or_else(|| format!("{ev} event without {k}"))
    };
    let n = |k: &str| {
        v.get(k)
            .and_then(serde_json::Value::as_u64)
            .ok_or_else(|| format!("{ev} event without {k}"))
    };
    match ev {
        "log" => Ok(Line::Log(s("text")?)),
        "error" => Ok(Line::Error(s("text")?)),
        "fetched" => {
            let out = s("out")?;
            if out.is_empty() {
                return Err("fetched event with an empty out".into());
            }
            Ok(Line::Fetched(Outcome::Fetched {
                name: s("name")?,
                files: n("files")?,
                colors: n("colors")?,
                note: v
                    .get("note")
                    .and_then(serde_json::Value::as_str)
                    .filter(|t| !t.is_empty())
                    .map(str::to_string),
                out: PathBuf::from(out),
            }))
        }
        other => Err(format!("unknown event {other:?}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::testing::fake_fetcher;

    fn finish(f: &mut PartFetch) -> Outcome {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
        loop {
            if let Some(o) = f.poll() {
                return o;
            }
            assert!(std::time::Instant::now() < deadline, "the fetcher never finished: {}", f.last);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    #[test]
    fn a_fetch_reports_its_progress_its_file_and_its_failures() {
        let Some(fake) = fake_fetcher("fetch", r#"{"parts": {"2458": {"name": "Brick  1 x  2 with Pin"}}}"#) else {
            return;
        };
        let out = fake.dir.join("bricks").join("2458.json");
        let mut f = PartFetch::start(&fake.python, &fake.env, "2458", &out).unwrap();
        assert_eq!(f.number, "2458");
        let got = finish(&mut f);
        assert_eq!(
            got,
            Outcome::Fetched {
                name: "Brick  1 x  2 with Pin".into(),
                files: 19,
                colors: 15,
                note: None,
                out: out.clone()
            }
        );
        assert!(f.last.starts_with("fetched"), "{}", f.last);
        assert!(out.exists());
        // a part Rebrickable lists no colours for says so
        let mut f = PartFetch::start(&fake.python, &fake.env, "99050001", &fake.dir.join("bricks").join("n.json")).unwrap();
        match finish(&mut f) {
            Outcome::Fetched { colors, note, .. } => {
                assert_eq!((colors, note.as_deref()), (0, Some("Rebrickable lists no colours for 99050001")));
            }
            other => panic!("{other:?}"),
        }
        // ldraw.org has no such part
        let mut f = PartFetch::start(&fake.python, &fake.env, "4040001", &fake.dir.join("bricks").join("404.json")).unwrap();
        assert_eq!(finish(&mut f), Outcome::Failed("ldraw.org has no part 4040001".into()));
        // a fetcher that dies says so, with its last words: the traceback it wrote on the way out,
        // every time (the exit marker waits for it)
        for _ in 0..5 {
            let mut f = PartFetch::start(&fake.python, &fake.env, "99090001", &fake.dir.join("bricks").join("crash.json")).unwrap();
            match finish(&mut f) {
                Outcome::Failed(t) => {
                    assert!(t.contains("without a word") && t.contains("exit code 3"), "{t}");
                    assert!(t.ends_with("ValueError: boom"), "{t}");
                }
                other => panic!("{other:?}"),
            }
        }
        // and one that talks nonsense, however long and in whatever alphabet
        let mut f = PartFetch::start(&fake.python, &fake.env, "99080001", &fake.dir.join("bricks").join("b.json")).unwrap();
        match finish(&mut f) {
            Outcome::Failed(t) => assert!(t.contains("unknown event"), "{t}"),
            other => panic!("{other:?}"),
        }
        let mut f = PartFetch::start(&fake.python, &fake.env, "99070001", &fake.dir.join("bricks").join("c.json")).unwrap();
        match finish(&mut f) {
            Outcome::Failed(t) => {
                assert!(t.contains("expected") || t.contains("without a word"), "{t}");
                assert!(t.chars().count() < 300, "cut short: {t}");
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(head("héllo wörld", 4), "héll");
        // one dropped while it still talks (a cancel) is killed, and its readers end
        let mut f = PartFetch::start(&fake.python, &fake.env, "99030001", &fake.dir.join("bricks").join("d.json")).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(400));
        assert!(f.poll().is_none());
        assert!(f.last.starts_with("warning"), "{}", f.last);
        drop(f);
        std::thread::sleep(std::time::Duration::from_millis(200));
        // readers that died take the fetch down with them, never hanging it
        let mut f = PartFetch::start(&fake.python, &fake.env, "99060001", &fake.dir.join("bricks").join("e.json")).unwrap();
        f.readers_gone();
        match finish(&mut f) {
            Outcome::Failed(t) => assert!(t.starts_with("the fetcher's output could not be read to the end"), "{t}"),
            other => panic!("{other:?}"),
        }
        assert!(PartFetch::start("/no/such/python", &[], "1", &out).is_err());
        let _ = std::fs::remove_dir_all(&fake.dir);
    }

    #[test]
    fn a_fetched_event_short_of_a_field_is_refused_not_taken_for_a_nameless_part() {
        // a fetcher of another version (a renamed key, a name that came out null) used to be a
        // success with name "", 0 files, 0 colours and the sim's own guess of the file; the
        // stdout reader turns the error into the failure's text with the line's head
        let err = |line: &str| match parse(line) {
            Err(e) => e,
            Ok(_) => panic!("{line} parsed"),
        };
        assert_eq!(err(r#"{"ev":"fetched"}"#), "fetched event without out");
        assert_eq!(
            err(r#"{"ev":"fetched","part":"2458","files":19,"colors":15,"out":"/x/2458.json"}"#),
            "fetched event without name"
        );
        assert_eq!(
            err(r#"{"ev":"fetched","name":null,"files":19,"colors":15,"out":"/x/2458.json"}"#),
            "fetched event without name"
        );
        assert_eq!(
            err(r#"{"ev":"fetched","name":"P","files":19,"colors":15,"out":""}"#),
            "fetched event with an empty out"
        );
        assert_eq!(
            err(r#"{"ev":"fetched","name":"P","colors":15,"out":"/x/2458.json"}"#),
            "fetched event without files"
        );
        assert_eq!(
            err(r#"{"ev":"fetched","name":"P","files":"19","colors":15,"out":"/x/2458.json"}"#),
            "fetched event without files"
        );
        assert_eq!(
            err(r#"{"ev":"fetched","name":"P","files":19,"out":"/x/2458.json"}"#),
            "fetched event without colors"
        );
        assert_eq!(err(r#"{"ev":"log"}"#), "log event without text");
        assert_eq!(err(r#"{"ev":"error","message":"boom"}"#), "error event without text");
        assert_eq!(err(r#"{"text":"no event"}"#), "event without ev");
        assert_eq!(err(r#"{"ev":"weird"}"#), "unknown event \"weird\"");
        // whole, with colours at zero and no note, it is a part; the note rides when it is there
        match parse(r#"{"ev":"fetched","name":"P","files":19,"colors":0,"out":"/x/2458.json"}"#) {
            Ok(Line::Fetched(Outcome::Fetched {
                name,
                files,
                colors,
                note,
                out,
            })) => {
                assert_eq!(
                    (name.as_str(), files, colors, note, out),
                    ("P", 19, 0, None, PathBuf::from("/x/2458.json"))
                );
            }
            _ => panic!("a whole fetched event is the part"),
        }
        match parse(r#"{"ev":"fetched","name":"P","files":19,"colors":0,"note":"Rebrickable lists no colours for 2458","out":"/x"}"#) {
            Ok(Line::Fetched(Outcome::Fetched { note, .. })) => assert_eq!(note.as_deref(), Some("Rebrickable lists no colours for 2458")),
            _ => panic!("the note rides along"),
        }
        match parse(r#"{"ev":"log","text":"fetched parts/2458.dat"}"#) {
            Ok(Line::Log(t)) => assert_eq!(t, "fetched parts/2458.dat"),
            _ => panic!("a log line"),
        }
    }
}
