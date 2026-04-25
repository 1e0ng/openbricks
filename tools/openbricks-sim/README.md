# openbricks-sim

MuJoCo-backed simulator for openbricks firmware. The eventual goal: the user's `main.py` runs unchanged on a real hub or inside a physics sim — driver classes get monkey-patched at sim-entry time so motor/encoder/IMU/colour-sensor calls hit MuJoCo bodies instead of GPIO.

Backbone: [MuJoCo](https://github.com/google-deepmind/mujoco) (Apache-2.0, native Python, 1 kHz physics easy, native sensor primitives, stable wheel contacts). Picked over pybullet (wheel-contact chatter) and Webots (32 ms step floor incompatible with the 1 kHz firmware control loop).

## Status — Phase C in progress (sim runtime layer)

* **Phase A** ✅ chassis + world preview, MuJoCo backbone wired up.
* **Phase B** ✅ shared cores: trajectory, observer, motor_process, servo, drivebase. Sim hot-path math is byte-identical to the firmware (same `*_core.c` files compiled into both targets).
* **Phase C** 🚧 runtime adapters + driver shim. `openbricks_sim.runtime.SimMotor` + `SimDriveBase` drive the default chassis; `openbricks_sim.robot.SimRobot` bundles them with the world; `openbricks-sim run main.py` executes scripts with `robot` / `drivebase` / `left` / `right` pre-bound. **C3 (just shipped):** `openbricks_sim.shim` installs no-op `machine` + replacement `_openbricks_native` modules and patches `time.sleep_ms`, so a script that imports `openbricks.drivers.jgb37_520.JGB37Motor` and `openbricks.robotics.drivebase.DriveBase` runs unchanged inside the sim — see `examples/wander_hardware_style.py`. Next: `SimIMU` + `SimColorSensor` (C4).

```
pipx install openbricks-sim   # once on PyPI
# or, from a repo checkout:
pip install -e tools/openbricks-sim

openbricks-sim preview --world wro-2026-elementary --x 1.0 --y -0.42

# Or execute a script against the sim:
openbricks-sim run examples/wander.py --world wro-2026-elementary --viewer

# Or — with the driver shim — run unmodified firmware code:
openbricks-sim run examples/wander_hardware_style.py --world wro-2026-elementary
```

Opens a MuJoCo viewer window with a default differential-drive chassis spawned at the start area of the 2026 WRO Elementary mat. Physics steps at 1 kHz. Use the viewer's mouse controls to orbit, drive with arrow keys or programmatically from Phase C.

Available world aliases:

| Alias | File |
|---|---|
| `empty` | standalone chassis on a checker floor (default) |
| `wro-2026-elementary` | `worlds/wro_2026_elementary_robot_rockstars/world.xml` |
| `wro-2026-junior` | `worlds/wro_2026_junior_heritage_heroes/world.xml` |
| `wro-2026-senior` | `worlds/wro_2026_senior_mosaic_masters/world.xml` |

You can also pass a full path to any MJCF world.

## Default chassis

2-wheel differential drive, rear caster, ~0.5 kg total, 60 mm wheels, 150 mm axle. Exposes:

- `chassis_motor_l`, `chassis_motor_r` — torque actuators (Phase C driver shims will drive these)
- `chassis_enc_l`, `chassis_enc_r` — wheel-angle sensors (shim → encoder counts)
- `chassis_encvel_l`, `chassis_encvel_r` — wheel angular velocity
- `chassis_accel`, `chassis_gyro` — IMU equivalents (shim → BNO055)
- `chassis_cam_down` — downward camera for the TCS34725 colour sensor shim

Override via `openbricks_sim.chassis.ChassisSpec(...)` when constructing; defaults are tuned to a small educational robot.

## Tests

```
cd tools/openbricks-sim
pip install -e .
python -m unittest discover -s tests -t .
```

Covers chassis MJCF shape, physics settling, motor → encoder round-trip, and chassis injection into all three WRO worlds.

## Roadmap (5–7 months)

| Phase | Scope | Status |
|---|---|---|
| A | MuJoCo scaffold, default-chassis MJCF, `preview` command | ✅ |
| B | Port `_openbricks_native` (motor_process, observer, trajectory, servo, drivebase) C → CPython extension. Sim hot path ≡ firmware hot path | ✅ |
| C | Sim runtime (`SimMotor` / `SimDriveBase`) bridging cores ↔ MuJoCo, driver-shim monkey-patch, `openbricks-sim run main.py` | 🚧 |
| D | Camera sampling for the colour sensor, raycast for distance sensors, worlds library | — |
| E | CI integration (headless rendering via EGL), examples, docs, PyPI publish as `openbricks-sim` | — |
