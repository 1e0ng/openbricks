---
myst:
  html_meta:
    description: "Example programs for openbricks robots: drivebase squares, gyro-corrected driving, line following, color sorting, and distance-based wall avoidance."
---

# Examples

The repo ships runnable example programs in
[`examples/`](https://github.com/1e0ng/openbricks/tree/main/examples).
Push any of them to a hub with `openbricks run -n <name> <script>` or
try them in the simulator with `openbricks sim run <script>`.

Two representative ones are reproduced below.

## Drive a square (ST-3032 drivebase)

```{eval-rst}
.. literalinclude:: ../examples/st3032_drivebase_square.py
   :language: python
```

## Rounded square (DriveBase.curve)

`curve(radius, angle)` follows the Pybricks contract — positional
order, parameter names, and sign semantics: positive `angle` arcs
right (clockwise), a negative `radius` drives the arc backward, and
`curve(0, angle)` degrades to a turn in place. The forward and turn
profiles run with proportionally scaled speed *and* acceleration, so
the path is a true circle even through the ramps, and the outer
wheel is automatically capped at the `straight_speed` setting.
(One deviation: `then` defaults to `"coast"` like every openbricks
move; pass `then="hold"` for the Pybricks end state.)

```{eval-rst}
.. literalinclude:: ../examples/st3032_drivebase_curve.py
   :language: python
```

## Square up on a line (two color sensors)

```{eval-rst}
.. literalinclude:: ../examples/line_align.py
   :language: python
```

## Full robot (drivebase + IMU + color sensor array)

```{eval-rst}
.. literalinclude:: ../examples/full_robot.py
   :language: python
```
