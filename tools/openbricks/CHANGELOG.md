# openbricks — host-tooling changelog

Versions the unified `openbricks` PyPI package (CLI + MuJoCo sim).
Firmware versions are tracked separately on the `v*` tag namespace.

## 0.10.7 — fix: auto-relaunch under mjpython on macOS for the GUI viewer

`openbricks sim preview --world wro-2026-elementary` on macOS
crashed with a Python stack trace ending in:

    RuntimeError: `launch_passive` requires that the Python script
    be run under `mjpython` on macOS

MuJoCo's interactive viewer needs to control the main thread for
OpenGL on macOS — Python's REPL doesn't give it that, hence the
`mjpython` wrapper that ships alongside `mujoco`. Without an opt-in,
every macOS user hit the crash.

The fix is a small re-exec helper. When the sim CLI on macOS is
asked to open the GUI viewer (`preview` without `--headless`, or
`run --viewer`), it locates the venv's bundled `mjpython` and
`os.execv`s into it before any model loading happens. Inside
mjpython, the viewer call works. No-op on Linux / Windows / when
already under mjpython / when mjpython isn't in the venv.

Workarounds that worked previously:

```
openbricks sim preview --world wro-2026-elementary --headless --duration 5
~/.local/pipx/venvs/openbricks/bin/mjpython -m openbricks_sim preview --world wro-2026-elementary
```

…still work, of course. After 0.10.7, the bare invocation also
works on macOS.

Tests in `test_sim_cli.py::RelaunchUnderMjpythonTests` pin: no-op on
Linux, no-op when already under mjpython, no-op when mjpython is
missing from the venv (let upstream raise), and re-exec strips the
leading `sim` keyword from `sys.argv` before forwarding (the CLI
sees `sim preview ...` because `openbricks` dispatched it; the sim's
own argv parser sees `preview ...`).

## 0.10.6 — fix: bundle world XMLs into the wheel

Every wheel published 0.10.3 → 0.10.5 shipped without the WRO and
practice-world MJCFs. End users hit
`WorldLoadError: world file not found: wro-2026-elementary` even
though `openbricks sim --help` listed the alias.

Root cause: the `worlds/` directory lived at
`tools/openbricks/worlds/` — outside the `openbricks_sim` Python
package — so `[tool.setuptools.packages.find]` (which only catches
`*.py` files inside the package) didn't pick it up. `MANIFEST.in`'s
`recursive-include worlds *.xml` seeded the *sdist* but not the
wheel. CI's smoke-tests passed because they ran from an editable
install (`pip install -e ".[sim,dev]"`), where the worlds exist
next to the package on the developer's disk.

The fix:

  * `worlds/` moved inside the package: now at
    `tools/openbricks/openbricks_sim/worlds/` (10 files: 5 worlds
    × `world.xml` + per-world `README.md` + the 3 WRO `mat.png`
    textures).
  * `[tool.setuptools.package-data]` added to `pyproject.toml` so
    the XMLs / PNGs / MDs ride along into the wheel.
  * `_resolve_world()` in both `cli.py` and `robot.py` now uses
    `Path(__file__).parent` (the installed package root) instead
    of `parent.parent` (the now-non-existent sibling sit).
  * `MANIFEST.in` no longer needs the `recursive-include worlds`
    line — package-data covers both sdist and wheel.

Regression test in `tests/test_wheel_bundles_worlds.py` builds a
fresh wheel and asserts `world.xml` for every alias plus `mat.png`
for the textured WRO worlds. Same gating shape as
`test_sdist_build.py`.

If you installed 0.10.3, 0.10.4, or 0.10.5, `pip install --upgrade
openbricks[sim]` to 0.10.6 — the alias resolution will start
working without any code changes on your end. (Also: 0.10.5 is the
last release with this bug; we'd recommend `pip install
'openbricks[sim]==0.10.6'` to avoid resolver pickup of the broken
versions.)

## 0.10.5 — sim: practice-zones and practice-walls learning worlds

Two small worlds for users iterating on sim code without wrestling
with a full WRO mat:

  * **practice-zones** — 1.5 × 1.5 m white floor with red / green /
    blue 150 × 150 mm slabs in a triangle around the chassis spawn.
    Pair with the colour sensor and `robot.chassis_in_box(...)` for
    "drive to the X zone" missions. The Phase E1 surface dispatch
    routes the colour-sensor raycast on the slabs' solid `rgba` to
    the material-rgba path, so reads are deterministic.
  * **practice-walls** — 2 × 2 m grey floor with three perpendicular
    walls forming a corridor open on +X. Pair with the distance
    sensor (HC-SR04 / VL53L0X / VL53L1X) for obstacle-avoidance
    practice.

Both registered in the CLI + `SimRobot` world-alias maps as
`practice-zones` and `practice-walls`. Each ships with a `README.md`
documenting layout + suggested missions.

```
openbricks sim preview --world practice-zones
openbricks sim run --world practice-walls my_avoidance.py
```

CI smoke-tests both worlds headlessly on every push, alongside the
existing three WRO worlds.

The 0.10.4 release was prepared but not tagged; this version
collapses Phase E1 + practice-worlds into a single PyPI release.
The 0.10.4 changelog entry below describes the colour-sensor
texture-sampling work that 0.10.5 also includes.

## 0.10.4 — sim Phase E1: pixel-accurate colour-sensor texture sampling

`SimColorSensor` now samples the actual texture pixel at the chassis
position when driving over a textured plane (the WRO-mat use case),
instead of returning the material's flat tint. The 0.10.x sensor
saw a single colour everywhere on a printed mat; 0.10.4 reads the
real printed pattern.

Implementation is CPU-side: the sensor reads `model.tex_data`
directly, computes UV from the geom-local hit point, and indexes
the texel. No GL context, no offscreen rendering, no platform
divergence — works on macOS / Linux / Windows identically. The
phase was originally scoped as "Linux EGL headless rendering," but
EGL is only required for scenes with shadows / lighting / overlays;
for a flat printed mat the texel itself IS the answer.

Surface-colour resolution in `SimColorSensor` now dispatches on
what the ray hit:

  * Textured plane geom — sample the texture at the (u, v) of the
    world hit point (the WRO mat path).
  * Untextured material — material's `rgba` (the previous behaviour
    on solid-colour floors, kept unchanged).
  * No material — geom's own `rgba`.

Six new tests in `test_color_sensor.py::TexturedPlaneSamplingTests`
pin the four-quadrant checker mapping and `texrepeat` re-tiling.
All 246 existing host tests untouched.

Future work — full MuJoCo offscreen rendering with `MUJOCO_GL=egl`
on Linux — only becomes relevant when sim scenes grow more complex
than a printed mat (3D coloured obstacles casting shadows on the
sensor's view, translucent overlays). Not yet prioritised.

## 0.10.3 — manylinux + macOS + Windows binary wheels

`pip install openbricks` now downloads a prebuilt binary wheel
instead of recompiling `openbricks_sim._native` on every machine.
The 0.10.2 sdist-only fallback stays in place for platforms outside
the matrix below.

Wheel matrix:

| Platform | Tag | Built on |
|---|---|---|
| Linux x86_64 | `manylinux_2_28_x86_64` | `ubuntu-latest` via cibuildwheel |
| macOS Intel + Apple Silicon | `macosx_*_universal2` | `macos-latest` |
| Windows AMD64 | `win_amd64` | `windows-latest` |

Each platform ships wheels for CPython 3.9 / 3.10 / 3.11 / 3.12 / 3.13.
Skipped (no asks; revisit when requested): musllinux (Alpine),
linux aarch64, linux i686, win32, PyPy.

CI gate: the `build-openbricks-wheels` job runs on every push / PR,
so wheel-build breakage is caught at review time rather than at
release time. Same pattern as the existing `build-openbricks-sdist`
regression check (`tests/test_sdist_build.py`).

## 0.10.2 — sdist-only publish (PyPI manylinux requirement)

The 0.10.1 publish failed at the upload step:

    400 Bad Request: Binary wheel
    'openbricks-0.10.1-cp311-cp311-linux_x86_64.whl' has an
    unsupported platform tag 'linux_x86_64'.

PyPI requires Linux wheels to be tagged ``manylinux*``, built inside
the ``quay.io/pypa/manylinux2014`` Docker image (or via
``cibuildwheel``). The CI's ``python -m build`` on a stock Ubuntu
runner produces ``linux_x86_64`` wheels, which PyPI rejects.

Quick fix: publish ``--sdist`` only. Users ``pip install openbricks``
get the source distribution and compile the native extension on
first install (slower, but gcc + python headers are typical on
Linux/macOS dev machines). Multi-platform manylinux wheels are a
follow-up.

No functional changes vs. 0.10.0 / 0.10.1 — same code, just shippable.

## 0.10.1 — sdist build fix (no functional changes)

Fixes a build failure that prevented 0.10.0 from publishing:
`python -m build` runs `setup.py` inside an isolated environment
that can't reach `../../native/user_c_modules/openbricks/`. The
sdist now bundles the synced cores via `MANIFEST.in`, and
`_sync_cores()` falls through cleanly when the upstream isn't
available (sdist build context). Regression test in
`tests/test_sdist_build.py` builds a fresh sdist and asserts every
shared core is inside.

If you saw a 0.10.0 install fail, install 0.10.1 instead — it's the
same code, just shippable.

## 0.10.0 — unification

The host CLI (`openbricks-dev`) and the MuJoCo simulator
(`openbricks-sim`) are now a single `openbricks` PyPI package with
optional `[sim]` extra. One install, one console script, every
subcommand under `openbricks`.

### New install commands

```
pip install openbricks            # CLI only (lightweight)
pip install 'openbricks[sim]'     # CLI + MuJoCo physics
```

### Breaking changes

- **Console script renamed.** `openbricks-dev <subcmd>` →
  `openbricks <subcmd>`. The old binary is gone in 0.10.0.
- **Sim subcommand.** `openbricks-sim <subcmd>` → `openbricks sim
  <subcmd>`. Same arguments, dispatched via passthrough to the sim's
  CLI when the `[sim]` extra is installed.
- **PyPI distribution names.** Users on `openbricks-dev` and
  `openbricks-sim` should `pip uninstall openbricks-dev openbricks-sim`
  and `pip install 'openbricks[sim]'`.
- **Release-tag namespace.** `openbricks-dev/v*` → `openbricks/v*`.
  Firmware tags `v*` are unchanged.

### What did NOT change

- The Python module names: `openbricks_dev` and `openbricks_sim`
  stay as they are. The firmware-side `openbricks` package on the
  hub (drivers, robotics, native cores) is also imported on the host
  by the sim's driver shim — collapsing the host CLI's module name
  to `openbricks` would shadow it.
- Subcommand argument grammars: `flash` / `list` / `run` / `upload` /
  `stop` / `log` / `sim {preview,run}` accept the same flags as before.
- The codecov flag names (`openbricks-dev`, `openbricks-sim`) — the
  CI workflow uploads coverage under both flags from the unified job
  so the dashboard split stays meaningful.

### Migration guide

```bash
# Uninstall the legacy split packages.
pip uninstall openbricks-dev openbricks-sim

# Install the unified package. Drop the [sim] if you only need flash + run.
pip install 'openbricks[sim]'

# Update your shell aliases / scripts:
#   openbricks-dev flash …  →  openbricks flash …
#   openbricks-sim run …    →  openbricks sim run …
```

The legacy `openbricks-dev` PyPI project is frozen at 0.9.x — its
last release will carry a deprecation notice in the description. No
new versions will be published under that name.
