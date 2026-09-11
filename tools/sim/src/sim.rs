//! The Simulate side: the run server (`python -m openbricks_sim.server`)
//! as a child process, its JSON-lines protocol, and the scene it
//! describes.

use serde::Deserialize;
use serde_json::Value;
use std::collections::HashMap;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;

#[derive(Deserialize, Clone, Debug, Default)]
#[allow(dead_code)] // the whole wire format is kept, used or not
pub struct WorldEntry {
    pub alias: String,
    #[serde(default)]
    pub path: Option<String>,
    #[serde(default)]
    pub dir: Option<String>,
}

#[derive(Deserialize, Clone, Debug, Default)]
#[allow(dead_code)] // the whole wire format is kept, used or not
pub struct Geom {
    #[serde(default)]
    pub name: String,
    #[serde(rename = "type")]
    pub kind: String,
    pub body: usize,
    pub size: [f64; 3],
    pub pos: [f64; 3],
    pub quat: [f64; 4],
    pub rgba: [f64; 4],
    #[serde(default)]
    pub material: Option<String>,
    #[serde(default)]
    pub group: i64,
    #[serde(default)]
    pub mesh: Option<String>,
}

#[derive(Deserialize, Clone, Debug, Default)]
pub struct Material {
    pub rgba: [f64; 4],
    #[serde(default)]
    pub texture: Option<String>,
    #[serde(default)]
    pub texrepeat: [f64; 2],
}

/// A brick of the assembled chassis, as the server placed it on the
/// chassis body (metres, chassis frame).
#[derive(Deserialize, Clone, Debug, Default)]
#[allow(dead_code)] // the whole wire format is kept, used or not
pub struct Brick {
    pub path: String,
    pub part: String,
    #[serde(default)]
    pub ldraw: Option<String>,
    pub pos_m: [f64; 3],
    pub quat: [f64; 4],
    pub half_m: [f64; 3],
    #[serde(default)]
    pub category: String,
}

#[derive(Deserialize, Clone, Debug, Default)]
#[allow(dead_code)] // the whole wire format is kept, used or not
pub struct Scene {
    pub bodies: Vec<String>,
    pub geoms: Vec<Geom>,
    #[serde(default)]
    pub materials: HashMap<String, Material>,
    #[serde(default)]
    pub textures: HashMap<String, String>,
    #[serde(default)]
    pub bricks: Vec<Brick>,
    #[serde(default)]
    pub timestep_ms: u32,
}

impl Scene {
    pub fn body_id(&self, name: &str) -> Option<usize> {
        self.bodies.iter().position(|b| b == name)
    }
}

/// One body pose from a frame: position (m) and quaternion (w, x, y, z).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose {
    pub pos: [f64; 3],
    pub quat: [f64; 4],
}

#[derive(Clone, Debug)]
pub enum Event {
    Hello {
        version: String,
    },
    Worlds(Vec<WorldEntry>),
    Scene(Box<Scene>),
    Frame {
        t_ms: u64,
        poses: Vec<Pose>,
    },
    Log {
        stream: String,
        text: String,
    },
    State {
        status: String,
        t_ms: u64,
        speed: f64,
        error: Option<String>,
    },
    Error(String),
    Bye,
    /// The child process ended (or its output closed).
    Exited(String),
}

/// Parse one protocol line.
pub fn parse_event(line: &str) -> Result<Event, String> {
    let v: Value = serde_json::from_str(line).map_err(|e| format!("{e}"))?;
    let ev = v.get("ev").and_then(Value::as_str).unwrap_or("");
    let s = |k: &str| v.get(k).and_then(Value::as_str).unwrap_or("").to_string();
    match ev {
        "hello" => Ok(Event::Hello { version: s("version") }),
        "worlds" => Ok(Event::Worlds(
            serde_json::from_value(v.get("worlds").cloned().unwrap_or(Value::Array(vec![]))).map_err(|e| format!("worlds: {e}"))?,
        )),
        "scene" => Ok(Event::Scene(Box::new(
            serde_json::from_value(v.clone()).map_err(|e| format!("scene: {e}"))?,
        ))),
        "frame" => {
            let t_ms = v.get("t_ms").and_then(Value::as_u64).unwrap_or(0);
            let bodies: Vec<Vec<f64>> =
                serde_json::from_value(v.get("bodies").cloned().unwrap_or(Value::Array(vec![]))).map_err(|e| format!("frame: {e}"))?;
            let poses = bodies
                .into_iter()
                .map(|b| {
                    if b.len() == 7 {
                        Pose {
                            pos: [b[0], b[1], b[2]],
                            quat: [b[3], b[4], b[5], b[6]],
                        }
                    } else {
                        Pose::default()
                    }
                })
                .collect();
            Ok(Event::Frame { t_ms, poses })
        }
        "log" => Ok(Event::Log {
            stream: s("stream"),
            text: s("text"),
        }),
        "state" => Ok(Event::State {
            status: s("status"),
            t_ms: v.get("t_ms").and_then(Value::as_u64).unwrap_or(0),
            speed: v.get("speed").and_then(Value::as_f64).unwrap_or(1.0),
            error: v.get("error").and_then(Value::as_str).map(str::to_string),
        }),
        "error" => Ok(Event::Error(s("text"))),
        "bye" => Ok(Event::Bye),
        other => Err(format!("unknown event {other:?}")),
    }
}

/// The server as a child process with a reader thread.
pub struct SimProcess {
    child: Child,
    stdin: std::process::ChildStdin,
    events: Receiver<Event>,
}

impl SimProcess {
    /// Spawn the run server; `env` adds environment variables (tests
    /// point PYTHONPATH at a stand-in server).
    pub fn spawn_with_env(python: &str, env: &[(String, String)]) -> Result<SimProcess, String> {
        let mut child = Command::new(python)
            .args(["-u", "-m", "openbricks_sim.server"])
            .envs(env.iter().map(|(k, v)| (k.as_str(), v.as_str())))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| format!("could not start {python}: {e}"))?;
        let stdin = child.stdin.take().ok_or("no stdin")?;
        let stdout = child.stdout.take().ok_or("no stdout")?;
        let stderr = child.stderr.take().ok_or("no stderr")?;
        let (tx, events) = mpsc::channel();
        let tx_err = tx.clone();
        thread::spawn(move || {
            let reader = BufReader::new(stdout);
            for line in reader.lines() {
                match line {
                    Ok(l) => {
                        if l.trim().is_empty() {
                            continue;
                        }
                        let ev = parse_event(&l).unwrap_or_else(|e| Event::Error(format!("{e}: {}", &l[..l.len().min(200)])));
                        if tx.send(ev).is_err() {
                            break;
                        }
                    }
                    Err(_) => break,
                }
            }
            let _ = tx.send(Event::Exited("the run server closed its output".into()));
        });
        thread::spawn(move || {
            let reader = BufReader::new(stderr);
            for line in reader.lines().map_while(Result::ok) {
                if tx_err
                    .send(Event::Log {
                        stream: "server".into(),
                        text: line,
                    })
                    .is_err()
                {
                    break;
                }
            }
        });
        Ok(SimProcess { child, stdin, events })
    }

    pub fn send(&mut self, cmd: &Value) -> Result<(), String> {
        let mut line = serde_json::to_string(cmd).map_err(|e| format!("{e}"))?;
        line.push('\n');
        self.stdin
            .write_all(line.as_bytes())
            .and_then(|_| self.stdin.flush())
            .map_err(|e| format!("the run server is gone: {e}"))
    }

    pub fn try_recv(&mut self) -> Option<Event> {
        match self.events.try_recv() {
            Ok(ev) => Some(ev),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => None,
        }
    }

    pub fn kill(&mut self) {
        let _ = self.send(&serde_json::json!({"cmd": "quit"}));
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

impl Drop for SimProcess {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// A stand-in run server for tests: a Python package on a private
/// PYTHONPATH that speaks the protocol with canned answers.
#[cfg(test)]
pub mod testing {
    use std::path::PathBuf;
    use std::process::Command;

    const FAKE_SERVER: &str = r#"
import sys, json
def send(**kw):
    print(json.dumps(kw), flush=True)
send(ev="hello", version="fake")
W = [{"alias": "practice-line", "path": "/w/practice_line/world.xml", "dir": "/w/practice_line"},
     {"alias": "wro-2026-senior", "path": "/w/senior/world.xml", "dir": "/w/senior"},
     {"alias": "empty", "path": "/w/empty.xml", "dir": "/w"}]
SCENE = {"ev": "scene", "bodies": ["world", "chassis"],
         "geoms": [{"name": "floor", "type": "plane", "body": 0, "size": [1.2, 0.9, 0.1], "pos": [0, 0, 0],
                    "quat": [1, 0, 0, 0], "rgba": [1, 1, 1, 1], "material": None, "group": 0, "mesh": None}],
         "materials": {}, "textures": {}, "bricks": [], "timestep_ms": 1}
status = "idle"
for line in sys.stdin:
    c = json.loads(line)
    cmd = c["cmd"]
    if cmd == "worlds":
        send(ev="worlds", worlds=W)
        continue
    if cmd == "load":
        status = "loaded"
        send(**SCENE)
        send(ev="frame", t_ms=0, bodies=[[0, 0, 0, 1, 0, 0, 0], [0, 0, 0.05, 1, 0, 0, 0]])
        send(ev="log", stream="server", text="loaded %s with %s" % (c.get("world"), c.get("assembly")))
    elif cmd == "run":
        status = "running"
        send(ev="log", stream="stdout", text="hello from " + c["script"])
        send(ev="frame", t_ms=10, bodies=[[0, 0, 0, 1, 0, 0, 0], [0.01, 0, 0.05, 1, 0, 0, 0]])
    elif cmd == "pause":
        status = "paused"
    elif cmd == "resume":
        status = "running"
    elif cmd == "stop":
        status = "stopped"
    elif cmd == "speed":
        send(ev="state", status=status, t_ms=0, speed=c["factor"], error=None)
        continue
    elif cmd == "quit":
        send(ev="bye")
        break
    else:
        send(ev="error", text="no such command " + cmd)
        continue
    send(ev="state", status=status, t_ms=10 if status == "running" else 0, speed=1.0, error=None)
"#;

    /// The interpreter to run it with, the environment that makes
    /// `openbricks_sim.server` resolve to the stand-in, and the private
    /// directory (remove it when done). None without a `python3`.
    pub struct FakeServer {
        pub python: String,
        pub env: Vec<(String, String)>,
        pub dir: PathBuf,
    }

    pub fn fake_server(tag: &str) -> Option<FakeServer> {
        let python = ["python3", "python"]
            .into_iter()
            .find(|p| Command::new(p).arg("--version").output().is_ok())?;
        let dir = std::env::temp_dir().join(format!("ob-fake-{}-{tag}", std::process::id()));
        std::fs::create_dir_all(dir.join("openbricks_sim")).unwrap();
        std::fs::write(dir.join("openbricks_sim/__init__.py"), "").unwrap();
        std::fs::write(dir.join("openbricks_sim/server.py"), FAKE_SERVER).unwrap();
        let env = vec![("PYTHONPATH".to_string(), dir.to_string_lossy().to_string())];
        Some(FakeServer {
            python: python.to_string(),
            env,
            dir,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_every_event_kind() {
        match parse_event(r#"{"ev":"hello","version":"4.3.0","pid":1}"#).unwrap() {
            Event::Hello { version } => assert_eq!(version, "4.3.0"),
            e => panic!("{e:?}"),
        }
        match parse_event(r#"{"ev":"worlds","worlds":[{"alias":"empty","path":null,"dir":null},{"alias":"practice-line","path":"/w/world.xml","dir":"/w"}]}"#).unwrap() {
            Event::Worlds(w) => {
                assert_eq!(w.len(), 2);
                assert_eq!(w[1].dir.as_deref(), Some("/w"));
            }
            e => panic!("{e:?}"),
        }
        let scene = r#"{"ev":"scene","bodies":["world","chassis"],"geoms":[{"name":"floor","type":"plane","body":0,"size":[1,1,0.1],"pos":[0,0,0],"quat":[1,0,0,0],"rgba":[1,1,1,1],"material":"mat","group":0,"mesh":null}],"materials":{"mat":{"rgba":[1,1,1,1],"texture":"mat_tex","texrepeat":[1,1]}},"textures":{"mat_tex":"/w/mat.png"},"bricks":[{"path":"frame","part":"lego_64178","ldraw":"64178","pos_m":[0,0,0],"quat":[1,0,0,0],"half_m":[0.02,0.04,0.004],"category":"lego"}],"timestep_ms":1}"#;
        match parse_event(scene).unwrap() {
            Event::Scene(s) => {
                assert_eq!(s.body_id("chassis"), Some(1));
                assert_eq!(s.geoms[0].kind, "plane");
                assert_eq!(s.materials["mat"].texture.as_deref(), Some("mat_tex"));
                assert_eq!(s.bricks[0].ldraw.as_deref(), Some("64178"));
            }
            e => panic!("{e:?}"),
        }
        match parse_event(r#"{"ev":"frame","t_ms":42,"bodies":[[0,0,0,1,0,0,0],[1,2,3,0.7,0,0.7,0]]}"#).unwrap() {
            Event::Frame { t_ms, poses } => {
                assert_eq!(t_ms, 42);
                assert_eq!(poses[1].pos, [1.0, 2.0, 3.0]);
            }
            e => panic!("{e:?}"),
        }
        match parse_event(r#"{"ev":"log","stream":"stdout","text":"hi"}"#).unwrap() {
            Event::Log { text, .. } => assert_eq!(text, "hi"),
            e => panic!("{e:?}"),
        }
        match parse_event(r#"{"ev":"state","status":"paused","t_ms":10,"speed":2.0,"error":null}"#).unwrap() {
            Event::State { status, speed, error, .. } => {
                assert_eq!(status, "paused");
                assert_eq!(speed, 2.0);
                assert!(error.is_none());
            }
            e => panic!("{e:?}"),
        }
        assert!(matches!(parse_event(r#"{"ev":"error","text":"x"}"#).unwrap(), Event::Error(t) if t == "x"));
        assert!(matches!(parse_event(r#"{"ev":"bye"}"#).unwrap(), Event::Bye));
        assert!(parse_event(r#"{"ev":"what"}"#).is_err());
        assert!(parse_event("not json").is_err());
    }

    #[test]
    fn talks_to_a_fake_server() {
        // a Python one-liner that speaks the protocol is enough to prove the plumbing
        let python = if Command::new("python3").arg("--version").output().is_ok() {
            "python3"
        } else {
            "python"
        };
        if Command::new(python).arg("--version").output().is_err() {
            return;
        }
        let dir = std::env::temp_dir().join(format!("ob-sim-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("openbricks_sim")).unwrap();
        std::fs::write(dir.join("openbricks_sim/__init__.py"), "").unwrap();
        std::fs::write(
            dir.join("openbricks_sim/server.py"),
            "import sys, json\nprint(json.dumps({'ev':'hello','version':'t'}), flush=True)\nfor line in sys.stdin:\n    c = json.loads(line)\n    if c['cmd'] == 'quit':\n        print(json.dumps({'ev':'bye'}), flush=True); break\n    print(json.dumps({'ev':'log','stream':'stdout','text':'got ' + c['cmd']}), flush=True)\n",
        )
        .unwrap();
        // PYTHONPATH puts the fake package first
        let mut child = Command::new(python)
            .args(["-u", "-m", "openbricks_sim.server"])
            .env("PYTHONPATH", &dir)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let stdin = child.stdin.take().unwrap();
        let stdout = child.stdout.take().unwrap();
        let stderr = child.stderr.take().unwrap();
        let (tx, events) = mpsc::channel();
        let tx2 = tx.clone();
        thread::spawn(move || {
            for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                let _ = tx.send(parse_event(&line).unwrap());
            }
            let _ = tx.send(Event::Exited("closed".into()));
        });
        thread::spawn(move || {
            for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                let _ = tx2.send(Event::Log {
                    stream: "server".into(),
                    text: line,
                });
            }
        });
        let mut p = SimProcess { child, stdin, events };
        p.send(&serde_json::json!({"cmd": "worlds"})).unwrap();
        p.send(&serde_json::json!({"cmd": "quit"})).unwrap();
        let mut got = Vec::new();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
        while std::time::Instant::now() < deadline {
            if let Some(ev) = p.try_recv() {
                let done = matches!(ev, Event::Exited(_));
                got.push(ev);
                if done {
                    break;
                }
            } else {
                thread::sleep(std::time::Duration::from_millis(10));
            }
        }
        assert!(matches!(got.first(), Some(Event::Hello { .. })), "{got:?}");
        assert!(
            got.iter().any(|e| matches!(e, Event::Log { text, .. } if text == "got worlds")),
            "{got:?}"
        );
        assert!(got.iter().any(|e| matches!(e, Event::Bye)), "{got:?}");
        p.kill();
        let _ = std::fs::remove_dir_all(&dir);
    }
}
