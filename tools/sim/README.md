# openbricks-sim

The sim: the native desktop application behind `openbricks sim` — the
Assembly Workbench (LEGO Technic bricks with exact LDraw geometry, STL
parts, components, the robot as the top component, mass properties
computed at every level) and, next, the Simulate tab.

```console
$ cargo build --release
$ OPENBRICKS_SIM_BIN=target/release/openbricks-sim openbricks sim
```

`openbricks sim` passes the shipped brick bundle with `--bricks`; the
binary can also be run directly:

```console
$ target/release/openbricks-sim --bricks ../openbricks/openbricks_sim/bricks/technic_bundle.json.zlib robot.assembly.json
```

CI runs `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings`
and `cargo test`, and builds release archives for macOS (arm64 and
x86_64), Linux and Windows that the release job signs and attaches to
the `v*` release; `openbricks sim` downloads the one for its platform
on first run.
