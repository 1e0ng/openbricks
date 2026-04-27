# openbricks-sim

MuJoCo-backed simulator for openbricks firmware. The eventual goal: the user's `main.py` runs unchanged on a real hub or inside a physics sim — driver classes get monkey-patched at sim-entry time so motor/encoder/IMU/colour-sensor calls hit MuJoCo bodies instead of GPIO.

Backbone: [MuJoCo](https://github.com/google-deepmind/mujoco) (Apache-2.0, native Python, 1 kHz physics easy, native sensor primitives, stable wheel contacts). Picked over pybullet (wheel-contact chatter) and Webots (32 ms step floor incompatible with the 1 kHz firmware control loop).

## Status — Phase D in progress (sensors + scenarios)

* **Phase A** ✅ chassis + world preview, MuJoCo backbone wired up.
* **Phase B** ✅ shared cores: trajectory, observer, motor_process, servo, drivebase. Sim hot-path math is byte-identical to the firmware (same `*_core.c` files compiled into both targets).
* **Phase C** ✅ runtime adapters + driver shim. `SimMotor` + `SimDriveBase` + `SimIMU` + `SimColorSensor` cover the default chassis; `SimRobot` bundles them with the world; `openbricks-sim run main.py` executes scripts with `robot` / `drivebase` / `left` / `right` pre-bound. The driver shim installs no-op `machine` + replacement `_openbricks_native`, patches `time.sleep_ms`, and monkey-patches the pure-Python driver classes — `JGB37Motor` / `DriveBase` / `BNO055` / `TCS34725` imports all resolve.
* **Phase D** 🚧 distance sensors + scenarios. `SimDistanceSensor` raycasts forward from the chassis `chassis_dist` site; `ShimHCSR04` / `ShimVL53L0X` / `ShimVL53L1X` all wrap it — `from openbricks.drivers.hcsr04 import HCSR04` (or `vl53l0x` / `vl53l1x`) works unchanged in the sim. **Just shipped:** `robot.reset()` and `robot.set_pose(x_mm, y_mm, yaw_deg)` for iterating on a mission script without restarting the sim process — see `examples/repeat_mission.py`. See also `examples/wall_avoid.py`. Next: scoring helpers, worlds library expansion.

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
- `chassis_dist` site — forward-facing range-sensor origin (HC-SR04 / VL53L0X shim raycasts from here)

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
| C | Sim runtime (`SimMotor` / `SimDriveBase` / `SimIMU` / `SimColorSensor`) bridging cores ↔ MuJoCo, driver-shim monkey-patch, `openbricks-sim run main.py` | ✅ |
| D | Distance sensors (raycast HC-SR04 / VL53L0X), worlds library, scenario reset / scoring | 🚧 |
| E | CI integration (headless rendering via EGL), examples, docs, PyPI publish as `openbricks-sim` | — |
