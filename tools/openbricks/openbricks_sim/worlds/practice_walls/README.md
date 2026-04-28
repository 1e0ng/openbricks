# practice-walls

A small obstacle-avoidance learning world: 2 × 2 m grey floor with
three perpendicular walls forming a partial corridor that's open on
+X.

| Wall | Pose (m) |
|---|---|
| Back  | x=-0.5, y in [-0.6, 0.6] |
| Left  | y= 0.7, x in [-0.5, 0.5] |
| Right | y=-0.7, x in [-0.5, 0.5] |

Walls are 30 mm thick, 150 mm tall — visible to the chassis
forward-facing distance sensor (`chassis_dist` raycast).

Spawn faces +X (the open exit). A naive forward-drive script will
clear the corridor and exit cleanly; a wall-avoidance script using
the HC-SR04 / VL53L0X / VL53L1X distance sensor can navigate around
the back wall first.

Use this world to:

- Iterate on a wall-avoidance script (`if sensor.distance_mm() < N:
  turn`).
- Tune the distance sensor's threshold + the drivebase's stopping
  distance.
- Practice mission scoring: did the robot exit through the open
  side? `robot.chassis_pose()[0] > 600` is one definition.

```
openbricks sim preview --world practice-walls
```
