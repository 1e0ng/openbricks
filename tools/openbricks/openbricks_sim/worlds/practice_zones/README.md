# practice-zones

A small learning world: 1.5 × 1.5 m white floor with three coloured
target zones arranged in a triangle around the chassis spawn.

| Zone | World position (m) | Colour |
|---|---|---|
| Red   | (0.40, 0.00) | (230, 25, 25) |
| Green | (0.25, 0.30) | (25, 180, 25) |
| Blue  | (0.25, -0.30) | (25, 75, 230) |

Each zone is a 150 × 150 × 2 mm slab raised 1 mm off the floor so the
downward `chassis_cam_down` raycast lands on its solid rgba (the
SimColorSensor works at the geom-material level, not the
texture-pixel level).

Use this world to:

- Tune the colour sensor's threshold without an opaque WRO mat.
- Iterate on a "drive to the X zone" mission. Pair with
  `robot.chassis_in_box(x_min, y_min, x_max, y_max)` for scoring.
- Sanity-check the slip-immune drivebase (`use_gyro(True)`) by
  driving across coloured slabs (different friction coefficients
  not yet modelled, but the geometry's correct).

```
openbricks sim preview --world practice-zones
```
