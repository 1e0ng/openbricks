# openbricks — host-tooling changelog

Versions the unified `openbricks` PyPI package (CLI + MuJoCo sim).
Firmware versions are tracked separately on the `v*` tag namespace.

## 0.10.13 — `openbricks flash` uses esptool v5 commands when available

`openbricks flash` invoked `esptool.py write_flash` and `erase_flash`
— the esptool v4 forms. esptool v5 (out for ~12 months) renamed
the binary to `esptool` and the commands to kebab-case
(`write-flash`, `erase-flash`); the legacy forms still work but
print deprecation warnings on every flash:

```
Warning: DEPRECATED: 'esptool.py' is deprecated. Please use 'esptool' instead.
Warning: Deprecated: Command 'write_flash' is deprecated. Use 'write-flash' instead.
```

`flash.py` now runtime-detects which esptool form is on PATH and
picks command spelling accordingly: `esptool` + kebab-case if v5
is installed, falling back to `esptool.py` + underscore on v4.
Output is clean of deprecation warnings on v5 installs.

Why not bump the dependency floor? esptool 5+ requires Python ≥ 3.10
and openbricks supports ≥ 3.9; pinning v5 would break wheel builds
for Python 3.9 users. Runtime detection lets both paths coexist.

## 0.10.12 — Junior + Senior randomization tightened to mat-truth

Closes the two TODOs left after 0.10.10's per-round randomization
introduction (PR #105). Both are user-visible behaviour changes
for `openbricks sim --world wro-2026-junior` and
`--world wro-2026-senior`.

### Junior — slot coordinates now extracted from the mat

The four "black squares at the lower end of the field" used as
randomization slots had estimated coordinates (`y=-0.45`, round
x). PR #109 replaces them with positions extracted from the
high-res Junior mat artwork via `scripts/extract-wro-slot-coords.py`
(same flow used for Elementary in 0.10.10):

```
world (-0.1052, -0.5055) m   <- slot_1
world (+0.0267, -0.5055) m   <- slot_2
world (+0.1586, -0.5055) m   <- slot_3
world (+0.2904, -0.5055) m   <- slot_4
```

~0.13 m spacing — the same pattern as Elementary's note-start
squares.

### Senior — randomization now covers all 4 cement colours

0.10.10 wired Senior randomization for the yellow cement group
only (10 elements). 0.10.12 expands to all 4 colours
(yellow + blue + green + white = 40 elements). The rules require
each colour permutes within its own storage area independently,
so internally the spec moved from a single `_RandomizationSpec`
per world to a tuple of specs (one per colour group), driven by
one shared seeded RNG so `seed=N` still pins the entire
40-element layout.

`_SPECS` type changed from `Dict[str, _RandomizationSpec]` to
`Dict[str, Tuple[_RandomizationSpec, ...]]`. The public
`randomize(...)` signature is unchanged; the return dict simply
grows from 10 → 40 entries for Senior.

### Compatibility

No public API changes. Junior layouts produced by
`randomize(seed=N)` differ from 0.10.11 because the slot
coordinates moved; Senior layouts differ because three new
colour groups were added. Tests pinning specific (x, y) values
will need updating — the layout dict structure is the same.

## 0.10.11 — fix unusable wheels + WRO 2026 prop library complete

**Critical fix.** Wheels 0.10.7 → 0.10.10 on PyPI shipped with
**zero** `props/*.ldr` files in the package data. Every WRO world
on `pip install openbricks` raised:

```
WorldLoadError: lego_prop 'clef' references missing .ldr file
'.../openbricks_sim/worlds/wro_2026_elementary_robot_rockstars/props/clef.ldr'
```

…regardless of which `--world` you asked for, because every WRO
world references at least one `<lego_prop ldr="props/*.ldr"/>`
placeholder. Same shape of bug as the missing-worlds bug fixed
in 0.10.6; the F2 phase (PR #98 onwards) introduced `.ldr` files
without a matching `package-data` entry and the regression
slipped past until packaging-test coverage caught it during the
F4.1 wheel-build verification.

Fix: extend `[tool.setuptools.package-data]` to include
`worlds/*/props/*.ldr` and `worlds/*/*.stl`, with two new
regression tests in `test_wheel_bundles_worlds` to pin both.
**Anyone on 0.10.7 → 0.10.10 should upgrade to 0.10.11.**

### What's new (was previously stuck on PyPI)

The complete WRO 2026 RoboMission prop library now actually
loads. Every prop in all three age-group worlds is built from
LDraw assemblies (one `<lego_prop>` per visible LEGO model in the
official Building Instructions PDFs):

* **Elementary "Robot Rockstars"** — clef, two cables, three
  instruments + microphone, six notes, amplifier + two speakers
  (PRs #98 → #102).
* **Junior "Heritage Heroes"** — visitors, artefacts, towers,
  dirt, barrier, parrot (PR #103).
* **Senior "Mosaic Masters"** — 72 prop instances total: 3 tools,
  24 mosaic tiles (6 × 4 colours), 40 cement elements
  (10 × 4 colours), 4 barriers (2 colour schemes), plus the
  3D-printed mosaic frame mesh from WRO's own STL (PRs #104,
  #106, #107).

`scripts/extract-wro-slot-coords.py` produces randomization slot
coordinates for Junior + Senior the same way Elementary's were
extracted in 0.10.10 (pixel-inspection of the high-res mat).
F3.J wires Junior randomization (5 artefacts × 4 slots,
choose-N-of-M); F3.S wires Senior (10 cement positions per colour
within their respective target areas).

### Compatibility

No API changes. Worlds that worked on 0.10.6 still work; worlds
that *would* have worked on 0.10.7 → 0.10.10 if the wheel weren't
broken now actually do.

## 0.10.10 — Phase F3: per-round randomization (Elementary)

Per the WRO General Rules glossary ("Robot Round" definition):
"Before the round starts with the first team but after all robots
are placed on the robot parking, randomizations to game fields (if
any) are done." That's the part of the challenge that forces
robots to perceive the field rather than rely on hardcoded
positions. 0.10.10 brings the same to openbricks-sim:

```
openbricks sim preview --world wro-2026-elementary --seed 42
openbricks sim run     --world wro-2026-elementary --seed 7  main.py
```

Each integer seed produces a deterministic permutation; same seed
twice gives the same layout. Without `--seed`, randomization is
non-deterministic per run.

The Elementary spec (the only one wired in 0.10.10): the four
notes `black`, `white`, `yellow`, `blue` are permuted across four
fixed light-green start squares at the upper end of the mat. Per
the Game Rules PDF p7: "Four of the notes (black, white, yellow,
blue) are randomly placed on the four light-green squares at the
upper end of the game field. The positions of the green and red
note are not random." So 4! = 24 distinct layouts; the red and
green notes stay at fixed positions.

The four slot coordinates were extracted from the actual mat
artwork (the high-res `mat.png` shipped in 0.10.8), not estimated.
`scripts/extract-wro-slot-coords.py` finds connected components
of pixels matching a target colour in a strip of the mat and
prints centroids in mat-local world coordinates. Re-run when
WRO updates the mat artwork. For the Elementary mat with target
RGB (144, 208, 112) and tolerance 32, the script identifies four
clusters of identical size (35,532 px each, ≈32×32 mm) at:

```
world (+0.0499, +0.4881) m   <- slot_1
world (+0.1818, +0.4881) m   <- slot_2
world (+0.5775, +0.4881) m   <- slot_3
world (+0.7094, +0.4881) m   <- slot_4
```

Implementation surface in `openbricks_sim.randomization`:

```python
from openbricks_sim import randomization
layout = randomization.randomize(model, data,
                                 world="wro-2026-elementary",
                                 seed=N)
# {"note_black": "slot_2", "note_white": "slot_4", ...}
```

For each note it:
- locates the body's freejoint and writes `(x, y)` directly into
  `data.qpos` at the joint's qpos address;
- preserves the body's resting Z from `model.body_pos` (the value
  authored in the world XML), so the mixed-shape notes
  (sphere/box/cylinder of differing heights) all sit on the mat
  correctly after randomization;
- zeroes the freejoint's `qvel` so the body doesn't carry any
  pre-randomization motion;
- calls `mj_forward` once at the end to refresh `xpos` /
  `cam_xpos` / sensor reads.

Test pins in `test_randomization.py` (7 tests):
- determinism (same seed → same layout);
- 4! = 24 distinct layouts achievable (200 seeds, 24 unique
  results);
- every randomized note lands at one of the 4 spec slots;
- no two notes ever share a slot;
- the fixed `note_red` / `note_green` don't move;
- per-note Z is preserved through randomization;
- unknown world raises `KeyError` (Junior + Senior today; spec
  KeyError is how the CLI knows to skip the print).

Junior and Senior randomization specs land in follow-up PRs once
their Game Rules PDFs are read into specs. The mechanism is
ready; the `_SPECS` dict just needs another entry.

## 0.10.9 — fix: drop mjpython, use blocking `mujoco.viewer.launch`

User-reported on 0.10.8: `openbricks sim preview --world
wro-2026-elementary` on macOS 26.4.1 (Tahoe) with mujoco 3.8 hangs
in mjpython's `_mjpython_init` with the kernel spamming "Task
policy set failed: 4 ((os/kern) invalid argument)" at ~4 Hz. The
hang reproduces even when invoking `mjpython -m openbricks_sim
preview ...` directly, with no openbricks helper involved.

Root cause: macOS recently tightened `task_policy_set()`
permissions for QoS class promotion, and the version of mjpython
bundled with mujoco 3.8 hasn't caught up. mjpython's GLFW
main-thread handoff never completes, so the viewer thread sits
on `threading.wait()` forever.

The fix sidesteps mjpython entirely. MuJoCo also exposes
`mujoco.viewer.launch(model, data)` — the blocking variant —
which runs MuJoCo's own time loop on the calling thread instead
of returning control to Python. The blocking variant doesn't need
the main-thread handoff that `launch_passive` requires on macOS,
so it works on this user's macOS 26 with plain Python (no
mjpython wrapper). User confirmed `mujoco.viewer.launch(model,
data)` opens a window cleanly on their box.

The 0.10.7 mjpython auto-relaunch helper, the 0.10.7 viewer
manual-stepping loop in `cmd_preview`, and the corresponding
manual-stepping loop in `SimRobot.run_viewer` are all removed.
`cmd_preview` now just calls `mujoco.viewer.launch(model, data)`.
`SimRobot.run_viewer` does the same. The `until` callback param
on `run_viewer` is kept for backwards-compat in the signature
but is now ignored — blocking `launch` doesn't return control
until the user closes the window, so a Python-side predicate
couldn't end it early. Scripts that need predicate-driven
termination should use `run_for` / `run_until` instead.

Tests removed: the four `RelaunchUnderMjpythonTests` from
`test_sim_cli.py` are gone (they covered the now-deleted helper).
Tests on `cmd_preview --headless` and on `run --viewer` still
gate the surrounding behaviour; the actual GUI-launch is
manually exercised on macOS / Linux / Windows.

Net code change is a simplification: ~70 lines deleted, ~5 added.
The 0.10.7 + 0.10.9 (closed PR #94) work was in the wrong
direction; this is the right fix.

## 0.10.8 — Phase F1: high-fidelity WRO 2026 mat textures (150 dpi)

The three WRO 2026 mats (Elementary, Junior, Senior) were shipped
as 2048×991 PNGs — about 22 dpi for a 2362×1143 mm mat. That left
zone boundaries soft and made the Phase E1 colour-sensor
texture-sampling pipeline read averaged colours instead of crisp
printed pixels.

0.10.8 replaces them with 13949×6750 PNGs rasterized at 150 dpi
(0.169 mm/pixel — ~7× finer than the TCS34725's physical sampling
spot, so the chassis colour sensor reads actual printed pixels).
Source is the official "Game Mat Printing File" PDF that WRO
publishes — vector PDF, 2362×1143 mm at 1:1, rendered with
`pdftoppm -r 150`.

Wheel size goes from ~3 MB to ~17 MB because the three mats add
8.6 + 4.1 + 5.1 = ~18 MB of PNG. That's the cost of pixel-level
fidelity. If you don't need the WRO worlds at all and want a
slim install, the future direction is on-demand asset download
via `openbricks sim assets fetch <world>` — not in this release;
defer until someone asks.

The mats are not committed as the source PDFs (license + size).
`scripts/regen-wro-mat-textures.sh` re-fetches the PDFs from
`wro-association.org` and re-rasterizes — run it whenever WRO
publishes a mat-fix mid-season, or when adjusting DPI.

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
