---
myst:
  html_meta:
    description: "Run the same openbricks robot code without hardware in a MuJoCo-backed simulator, including WRO competition worlds and a live 3D viewer."
---

# Simulator

The `[sim]` extra ships a MuJoCo-backed physics simulator, so you can
develop robot programs without a hub on the desk:

```console
$ pipx install 'openbricks[sim]'
$ openbricks sim run examples/full_robot.py --viewer
```

The sim runs the **same script you'd push to the hub** — a driver shim
maps the `openbricks` API onto simulated motors and sensors, so
`ST3032Motor`, `DriveBase`, color sensors, and distance sensors behave
like their hardware counterparts.

## Commands

```console
$ openbricks sim workbench [robot.assembly.json] [--bricks more.json] [--port N] [--no-browser]
```

Opens the [Assembly Workbench](#the-assembly-workbench) in your
browser: build the robot from LEGO Technic bricks with their exact
geometry, import your own STL parts, and read the computed mass
properties.

```console
$ openbricks sim preview [--world WORLD] [--x X] [--y Y] [--headless] [--duration S] [--seed N]
```

Loads the named world (an alias or a path to an MJCF file), splices in
the default chassis, and opens the MuJoCo viewer so you can inspect the
scene. `--headless` steps the physics for `--duration` seconds without
opening a window — useful as a smoke test.

```console
$ openbricks sim run SCRIPT [--world WORLD] [--chassis FILE] [--x X] [--y Y] [--yaw DEG] [--viewer] [--no-shim] [--seed N]
```

Loads the world plus the chassis and executes `SCRIPT` against the
simulated robot. `--viewer` opens the interactive MuJoCo window;
without it the sim runs headless (CI-friendly). `--seed` makes
randomized worlds reproducible.

Run `openbricks sim --help` for the full, always-current option list.

## The Assembly Workbench

`openbricks sim workbench` serves a single page on localhost and opens
it. The page is a 3D editor for the robot as a tree of components:

- **Bricks** are recorded once, with their geometry, mass and
  provenance (`measured`, `datasheet`, `vendor` or `placeholder`).
  The library that ships in the wheel holds a curated set of popular
  LEGO Technic parts converted from the [LDraw parts
  library](https://www.ldraw.org) (CC BY 2.0 / 4.0): beams in every
  common length, bent and L beams, frames, Technic bricks and plates,
  pins, axles, bushes, connectors, gears, a few rims and tyres, and
  fairing panels, with BrickLink catalogue weights where known.
  Servos, boards and wheels are recorded as boxes, cylinders and
  spheres, and any part you have as a mesh comes in through
  **Import a part from an STL file** (binary or ASCII; mm, cm, inch
  or m; a weighed mass or a density such as PLA 1.24 g/cm³).
- **Components** are lists of bricks and other components, each
  placed by a position and a roll / pitch / yaw. Drag bricks from the
  library into the view, move and rotate them with the gizmos, select
  what you built and *Group* it: the new component joins the library
  and can be dropped anywhere, as many times as you like. Double-click
  an instance to edit its definition in place; every use follows.
- **Connections.** Pins, axles and studs are real features of the
  LDraw parts, and 4.8 mm bores are recognised as pin holes on every
  mesh, imported STL files included. Let go of a part near a hole and
  it snaps: the pin axis aligns to the hole, a pin half centres in its
  module, an axle keeps its position along the hole. The inspector
  lists what each part is mated to.
- **Mass properties** are never typed in above the brick level.
  Volume, centre of mass and the inertia tensor of every LDraw and STL
  part come from its closed mesh, so a recorded weight becomes a full
  inertia tensor; components and the robot roll their children up
  with the parallel-axis theorem. Weight divided by exact volume is
  shown as a density on every part, which catches a wrong weight or a
  wrong part at a glance (ABS is about 1.05 g/cm³).
- **Roles** name the parts the simulator binds: the two drive wheels,
  the caster, the reflectance arrays, the colour sensor, the range
  sensor and the IMU. From them the page derives the flat
  `ChassisSpec` fields (`what the simulator receives`) with the axle
  midpoint as the origin, so a build can be run today with
  `openbricks sim run --chassis`.

The file the page reads and writes, `robot.assembly.json`, stores
recorded facts only: bricks, poses, roles, spawn pose. Everything
computed is recomputed on load. Open one with `openbricks sim
workbench robot.assembly.json`; the browser also keeps your last
draft between visits.

## The brick library

```console
$ openbricks bricks fetch [--dest DIR] [--force]
$ openbricks bricks convert NUMBER [NUMBER ...] [--out FILE] [--weights FILE] [--ldraw DIR]
$ openbricks sim workbench --bricks FILE
```

The wheel ships the curated Technic set; the whole LDraw library
(every LEGO part ever catalogued, 145 MB to download, about 600 MB
unpacked) is one command away. `bricks fetch` unpacks it into
`~/.cache/openbricks/ldraw` (or `$OPENBRICKS_LDRAW_DIR`), and `bricks
convert` turns any part numbers — the LEGO design ids printed on the
parts, `3648` for the 24-tooth gear — into a bundle file that
`openbricks sim workbench --bricks` adds to the library. Converted
parts without a weight carry a volume estimate at 1.05 g/cm³ and are
flagged until you weigh them; pass `--weights` with a JSON of
`{"3648": {"g": 1.62}}` to record real ones.

LEGO® and Technic are trademarks of the LEGO Group, which does not
sponsor or endorse openbricks. The geometry is the LDraw community's
work; the bundle carries its attribution.

## Describing your robot

The default chassis is a 60 mm-wheel, 150 mm-axle box with every
down-facing sensor 60 mm ahead of the axle. A real robot differs, and
those differences decide whether a mission script's numbers work:
`--chassis FILE` loads a JSON object of `ChassisSpec` fields (metres,
kilograms, degrees) that describe the robot the script was written
for. Fields not given keep the defaults.

```json
{
  "wheel_radius": 0.0432,   "axle_length": 0.135,
  "body_length": 0.16,      "body_width": 0.12,
  "line_sensor_x": 0.06,
  "line_sensor_2_x": -0.03,  "line_sensor_2_y": 0.0,
  "color_sensor_x": 0.06,   "color_sensor_y": 0.184,
  "pos_x": -0.547,          "pos_y": -0.15,        "yaw_deg": 90
}
```

- `wheel_radius` / `axle_length` size the chassis at load time. The
  `DriveBase(wheel_diameter_mm=…, axle_track_mm=…)` in the script
  resizes it again at adoption, so the script's geometry always wins
  — set them here so a `preview` shows the same robot.
- `line_sensor_x` places the first reflectance-array site
  (`chassis_line`) ahead of the axle; `line_sensor_2_x` / `_y` place
  the second (`chassis_line2`, default 30 mm behind the axle on the
  centre line, the same height). Reflectance arrays bind these sites
  in **construction order** within one run: the first `QTRArray` /
  `QTRLineSensor` / `QTRChannel` the script constructs reads
  `chassis_line`, the second reads `chassis_line2`, and a third raises
  `RuntimeError` (two sites is the chassis's limit). The counter
  resets when the shim is installed for a run, so every `sim run`
  starts with both sites free. `color_sensor_x` / `_y` / `_z` place the centre
  colour camera (`chassis_cam_down`, the no-mux `TCS34725`) in the
  chassis frame (the floor is at `-(wheel_radius + 0.005)`);
  `color_sensor_yaw` / `_pitch` aim it (default straight down; a
  sensor on the robot's left flank reading bricks beside the line is
  `yaw 90, pitch 0` at brick height); `color_sensor_fov` is the cone
  it integrates (degrees, 0 = one ray) and `color_sensor_range` how
  far it sees. The left/right down pair rides 18 mm either side of
  (`color_sensor_x`, `color_sensor_y`).
- `pos_x` / `pos_y` / `yaw_deg` are the spawn pose; `--x` / `--y` /
  `--yaw` on the command line override them one at a time. `yaw_deg`
  is counter-clockwise from +X seen from above (0 = facing +X).

## What the shim simulates

| Firmware class | Sim binding |
|---|---|
| `ST3032Motor` / `ST3215Motor` | The first two servo ids become the chassis wheels, the third and fourth kinematic task shafts (a gripper motor that turns but pushes nothing). A `DriveBase` always gets the physical wheels for the pair it adopts, whatever order the script constructed its motors in, and re-constructing a motor for a servo id yields the same motor — both firmware rules. |
| `DriveBase` | The firmware engine over an emulated `st_bus`; `use_gyro(True)` reads the chassis's true yaw. |
| `ICM45686` / `BNO055` | Ground-truth chassis heading; the ICM's bias estimator reports calibrated at once. |
| `TCS34725` | The firmware driver class over a synthesised raw read: the centre camera (no mux) or the left/right pair (mux channels 1 / 0) casts along its own axis — optionally a cone, with a range — and the first geom hit (a mat texel, a LEGO brick's material) gives the reflectance; `rgb()` / `ambient()` are the driver's channel-over-clear arithmetic, so white reads about (85, 85, 85) and a blue brick has the largest `b`, as on the robot. |
| `QTRLineSensor` / `QTRArray` / `QTRChannel` | The firmware driver over a reflectance model: one element per array position (`QTRLineSensor(channels=8)` gives the eight-channel front layout, exactly as on the hub), spread left-to-right from the site the array bound at construction — the first array a run constructs reads `chassis_line`, the second `chassis_line2`, a third raises `RuntimeError` — each element averaging the floor over a 3 mm spot so an edge reads as a gradient, which is what makes `50 - reading[i].ambient()` proportional. `load_calibration("/qtr_front.cal")` and `calibrate()` need no file — the sim's reflectance is born normalised. |
| Distance sensors | A forward ray from the `chassis_dist` site. |

Nothing above has a load: task motors don't grip, and a prop is only
pushed when the chassis body drives into it.

## Notes

- The sim needs the `[sim]` extra (`mujoco`, `numpy`). Without it,
  `openbricks sim …` prints an install hint instead of crashing.
- The wheel carries the firmware package (`openbricks.drivers.*`,
  `openbricks.parameters`, …) since 3.6.0, so a plain
  `pipx install 'openbricks[sim]'` runs hub-style scripts; earlier
  releases needed a repo checkout for that.
- Firmware-only users never need the simulator — it's strictly
  host-side tooling.
