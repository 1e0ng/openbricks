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
and the tests under `cargo llvm-cov` (coverage reaches codecov as the
`sim-rs` flag), and builds release archives for macOS (arm64 and
x86_64), Linux and Windows that the release job signs and attaches to
the `v*` release; `openbricks sim` downloads the one for its platform
on first run.

The tests need a GPU adapter for the offscreen renders (CI's Linux leg
installs mesa's lavapipe; a developer machine uses its own GPU) and
drive the whole window headless through `egui_kittest`. Set
`OPENBRICKS_SIM_PYTHON` to an interpreter that has `openbricks[sim]`
installed to run the end-to-end test against the real MuJoCo runtime:

```console
$ OPENBRICKS_SIM_PYTHON=$(which python) cargo test --release
```
