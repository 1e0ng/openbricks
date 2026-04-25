# openbricks-sim (planning)

Future home of the openbricks MuJoCo-backed simulator. The user's `main.py` is meant to run unchanged: driver classes get monkey-patched at sim-entry time so motor/encoder/IMU/colour-sensor calls hit MuJoCo bodies instead of GPIO.

Backbone: [MuJoCo](https://github.com/google-deepmind/mujoco) (Apache-2.0, native Python, 1 kHz physics easy, native sensor primitives, stable wheel contacts). Picked over pybullet (wheel-contact chatter) and Webots (32 ms step floor incompatible with the 1 kHz firmware control loop).

## Status

Pre-implementation. The following arrived ahead of the package itself because they're useful as standalone reference assets:

- `worlds/wro_2026_elementary_robot_rockstars/` — 2026 WRO Elementary "Robot Rockstars" (max score 255)
- `worlds/wro_2026_junior_heritage_heroes/` — 2026 WRO Junior "Heritage Heroes" (max score 230)
- `worlds/wro_2026_senior_mosaic_masters/` — 2026 WRO Senior "Mosaic Masters" (max score 233)

Each world is loadable directly in `python -m mujoco.viewer` today; all three will plug into the sim runner once it exists.

## Roadmap (5–7 months)

| Phase | Scope |
|---|---|
| A | MuJoCo scaffold, default-chassis MJCF, 1 kHz step proof-of-life |
| B | Port `_openbricks_native` (motor_process, observer, trajectory, servo, drivebase) C → CPython extension. Sim hot path ≡ firmware hot path |
| C | `openbricks-sim` package: driver shims that monkey-patch `openbricks.drivers.*` to MuJoCo-backed equivalents at runtime |
| D | Camera sampling for the colour sensor, raycast for distance sensors, world library (empty floor, line-follow mat, the WRO worlds in this directory) |
| E | CI integration (headless rendering via EGL), examples, docs, PyPI publish as `openbricks-sim` |
