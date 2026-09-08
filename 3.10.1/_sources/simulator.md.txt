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
  "color_sensor_x": 0.06,   "color_sensor_y": 0.184,
  "pos_x": -0.547,          "pos_y": -0.15,        "yaw_deg": 90
}
```

- `wheel_radius` / `axle_length` size the chassis at load time. The
  `DriveBase(wheel_diameter_mm=…, axle_track_mm=…)` in the script
  resizes it again at adoption, so the script's geometry always wins
  — set them here so a `preview` shows the same robot.
- `line_sensor_x` places the reflectance-array site (`chassis_line`)
  ahead of the axle. `color_sensor_x` / `_y` / `_z` place the centre
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
| `QTRLineSensor` / `QTRArray` / `QTRChannel` | The firmware driver over a reflectance model: one element per array position, spread left-to-right from the `chassis_line` site, each averaging the floor over a 3 mm spot so an edge reads as a gradient (the basis of `edge_error`). `load_calibration("/qtr.cal")` and `calibrate()` need no file — the sim's reflectance is born normalised. |
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
