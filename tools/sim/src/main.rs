//! openbricks-sim: the native Assembly Workbench and simulation front
//! end. `openbricks sim` launches it with the shipped brick library:
//!
//!     openbricks-sim --bricks technic_bundle.json.zlib [--bricks more.json] [robot.assembly.json]

mod app;
mod assembly;
mod bundle;
mod drafts;
mod edges;
mod editor;
mod fetch;
mod geometry;
mod gizmo;
mod markers;
mod overlap;
mod route;
mod sim;
mod simulate;
mod stl;
mod viewport;

use std::path::PathBuf;

#[derive(Debug)]
struct Args {
    bricks: Vec<PathBuf>,
    file: Option<PathBuf>,
    python: Option<String>,
}

/// The command line: the arguments, or None for a request for help
/// (which is no error).
fn parse_args(argv: &[String]) -> Result<Option<Args>, String> {
    let mut args = Args {
        bricks: vec![],
        file: None,
        python: None,
    };
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--bricks" => {
                i += 1;
                let p = argv.get(i).ok_or("--bricks needs a file")?;
                args.bricks.push(PathBuf::from(p));
            }
            "--python" => {
                i += 1;
                args.python = Some(argv.get(i).ok_or("--python needs an interpreter path")?.clone());
            }
            "-h" | "--help" => return Ok(None),
            s if s.starts_with('-') => return Err(format!("unknown option {s}")),
            s => {
                if args.file.is_some() {
                    return Err("only one assembly file can be opened".into());
                }
                args.file = Some(PathBuf::from(s));
            }
        }
        i += 1;
    }
    Ok(Some(args))
}

const USAGE: &str = "usage: openbricks-sim [--python PYTHON] [--bricks BUNDLE]... [robot.assembly.json]";

/// The brick library: every bundle named, merged. A bundle that will
/// not load is an error: the library would be missing what the user
/// (or `openbricks sim`, with the shipped bundle) asked for.
fn load_bundles(paths: &[PathBuf]) -> Result<bundle::Bundle, String> {
    let mut bundle = bundle::Bundle {
        format: String::new(),
        source: String::new(),
        parts: Default::default(),
        missing: vec![],
        sets: Default::default(),
        colors: Default::default(),
    };
    for p in paths {
        bundle.merge(bundle::load_bundle(p)?);
    }
    Ok(bundle)
}

/// The assembly file named on the command line, read and checked: one
/// that cannot be read, is not JSON or is not an assembly is an error
/// naming it, so nothing else opens in its place.
fn open_named(p: PathBuf, bundle: &bundle::Bundle) -> Result<(PathBuf, assembly::Document), String> {
    let text = std::fs::read_to_string(&p).map_err(|e| format!("could not open {}: {e}", p.display()))?;
    let doc: assembly::Document =
        serde_json::from_str(&text).map_err(|e| format!("could not open {}: not an assembly file ({e})", p.display()))?;
    let errs = assembly::validate(&doc, bundle);
    if assembly::not_an_assembly(&errs) {
        return Err(format!("could not open {}: not an assembly file: {}", p.display(), errs.join("; ")));
    }
    Ok((p, doc))
}

fn main() -> eframe::Result {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let args = match parse_args(&argv) {
        Ok(Some(a)) => a,
        Ok(None) => {
            println!("{USAGE}");
            std::process::exit(0);
        }
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    // what the user named must open, or the launch stops with the reason: a window on the
    // example, or on a draft of some other session, in place of the file asked for would hide it
    let mut bundle = match load_bundles(&args.bricks) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(2);
        }
    };
    let mut notes = Vec::new();
    // the parts fetched by number, kept under the data directory (the library's own win)
    for note in bundle::merge_user_bricks(&mut bundle, &markers::data_dir().join("bricks")) {
        eprintln!("warning: {note}");
        notes.push(note);
    }
    let doc = match args.file.map(|p| open_named(p, &bundle)).transpose() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(2);
        }
    };
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1440.0, 900.0])
            .with_title("Openbricks Sim"),
        ..Default::default()
    };
    eframe::run_native(
        "openbricks-sim",
        options,
        Box::new(move |cc| Ok(Box::new(app::App::new(cc, bundle, doc, args.python, notes)))),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn args_parse() {
        let a = parse_args(&[
            "--bricks".into(),
            "a.zlib".into(),
            "--python".into(),
            "/usr/bin/python3".into(),
            "--bricks".into(),
            "b.json".into(),
            "r.json".into(),
        ])
        .unwrap()
        .unwrap();
        assert_eq!(a.bricks.len(), 2);
        assert_eq!(a.python.as_deref(), Some("/usr/bin/python3"));
        assert!(parse_args(&["--python".into()]).is_err());
        assert_eq!(a.file.unwrap().to_string_lossy(), "r.json");
        assert!(parse_args(&["--bricks".into()]).is_err());
        assert!(parse_args(&["--nope".into()]).is_err());
        assert!(parse_args(&["a".into(), "b".into()]).is_err());
    }

    #[test]
    fn help_is_asked_for_not_an_error() {
        // help is its own outcome (usage on stdout, exit 0), apart from a bad command line
        assert!(matches!(parse_args(&["--help".into()]), Ok(None)));
        assert!(matches!(parse_args(&["-h".into()]), Ok(None)));
        assert!(matches!(parse_args(&[]), Ok(Some(Args { file: None, .. }))));
        assert_eq!(parse_args(&["--nope".into()]).unwrap_err(), "unknown option --nope");
        assert!(USAGE.starts_with("usage: openbricks-sim"));
    }

    #[test]
    fn a_named_file_that_will_not_open_is_refused_by_name() {
        let dir = std::env::temp_dir().join(format!("ob-main-open-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let bundle = load_bundles(&[]).unwrap();
        let missing = dir.join("robto.assembly.json");
        let e = open_named(missing.clone(), &bundle).unwrap_err();
        assert!(e.starts_with(&format!("could not open {}", missing.display())), "{e}");
        let text = dir.join("notes.json");
        std::fs::write(&text, "{\"a\": 1}").unwrap();
        let e = open_named(text.clone(), &bundle).unwrap_err();
        assert!(
            e.starts_with(&format!("could not open {}: not an assembly file", text.display())),
            "{e}"
        );
        let wrong = dir.join("map.assembly.json");
        std::fs::write(
            &wrong,
            r#"{"format": "x/9", "parts": {}, "components": {"x": {}}, "robot": {"root": "y"}}"#,
        )
        .unwrap();
        let e = open_named(wrong.clone(), &bundle).unwrap_err();
        assert!(
            e.contains("not an assembly file: format must be") && e.contains("robot.root"),
            "{e}"
        );
        // a good one opens, under its path
        let good = dir.join("robot.assembly.json");
        std::fs::write(&good, serde_json::to_string(&assembly::example()).unwrap()).unwrap();
        let (p, doc) = open_named(good.clone(), &bundle).unwrap();
        assert_eq!((p, doc.robot.root.as_str()), (good, assembly::example().robot.root.as_str()));
        // a bundle that will not load stops the library, by name
        let e = load_bundles(&[dir.join("none.json")]).unwrap_err();
        assert!(e.contains("none.json"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
