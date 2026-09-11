//! openbricks-sim: the native Assembly Workbench and simulation front
//! end. `openbricks sim` launches it with the shipped brick library:
//!
//!     openbricks-sim --bricks technic_bundle.json.zlib [--bricks more.json] [robot.assembly.json]

mod app;
mod assembly;
mod bundle;
mod geometry;
mod gizmo;
mod sim;
mod simulate;
mod viewport;

use std::path::PathBuf;

#[derive(Debug)]
struct Args {
    bricks: Vec<PathBuf>,
    file: Option<PathBuf>,
    python: Option<String>,
}

fn parse_args(argv: &[String]) -> Result<Args, String> {
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
            "-h" | "--help" => return Err("usage: openbricks-sim [--python PYTHON] [--bricks BUNDLE]... [robot.assembly.json]".into()),
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
    Ok(args)
}

fn main() -> eframe::Result {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let args = match parse_args(&argv) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    let mut bundle = bundle::Bundle {
        format: String::new(),
        source: String::new(),
        parts: Default::default(),
        missing: vec![],
    };
    for p in &args.bricks {
        match bundle::load_bundle(p) {
            Ok(b) => bundle.merge(b),
            Err(e) => eprintln!("warning: {e}"),
        }
    }
    let doc = args.file.and_then(|p| {
        match std::fs::read_to_string(&p)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str::<assembly::Document>(&t).map_err(|e| e.to_string()))
        {
            Ok(d) => Some((p, d)),
            Err(e) => {
                eprintln!("warning: could not open {}: {e}", p.display());
                None
            }
        }
    });
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
        Box::new(move |cc| Ok(Box::new(app::App::new(cc, bundle, doc, args.python)))),
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
        .unwrap();
        assert_eq!(a.bricks.len(), 2);
        assert_eq!(a.python.as_deref(), Some("/usr/bin/python3"));
        assert!(parse_args(&["--python".into()]).is_err());
        assert_eq!(a.file.unwrap().to_string_lossy(), "r.json");
        assert!(parse_args(&["--bricks".into()]).is_err());
        assert!(parse_args(&["--nope".into()]).is_err());
        assert!(parse_args(&["a".into(), "b".into()]).is_err());
        assert!(parse_args(&["--help".into()]).unwrap_err().starts_with("usage"));
    }
}
