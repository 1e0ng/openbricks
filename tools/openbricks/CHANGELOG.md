# openbricks — host-tooling changelog

Versions the unified `openbricks` PyPI package (CLI + MuJoCo sim).
Firmware versions are tracked separately on the `v*` tag namespace.

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
