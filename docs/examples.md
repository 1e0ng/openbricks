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

## Gyro-corrected square (ICM-45686)

With an ICM-45686 attached, `use_gyro(True)` moves heading control
off the encoders and onto measured body rotation — corrected every
millisecond inside the firmware's 1 kHz control tick, with no Python
in the loop. This script drives the same square twice, encoders-only
and gyro-corrected, and prints each pass's heading drift so the
difference is a number, not an impression. On the reference bench the
gyro pass returns within ~0.6° over all four turns; wheel slip that
would bend the encoder pass simply gets steered back out.

```{eval-rst}
.. literalinclude:: ../examples/icm45686_square.py
   :language: python
```

Wiring for the IMU is four SPI pins plus power — see
{doc}`/hardware`. The first still half-second learns the gyro bias;
`save_calibration()` persists it so later boots skip the wait.

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

## Square up on a line (QTR sensor bar)

The classic align move on the `QTRLineSensor` window: each half of
the ten-element bar acts as one virtual corner sensor, in two
passes. Seek: drive slowly toward the line — the wheel whose half
reaches it first stops while the other keeps rolling, pivoting the
chassis square. Edge: servo each wheel proportionally — the
follower's KP discipline — until its half reads ambient of about
50, the elements straddling the black/white boundary, parked
right ON the line's edge.
Calibrate once with `examples/qtr_calibrate.py` first; mount the
bar ahead of the wheels.

```{eval-rst}
.. literalinclude:: ../examples/qtr_align.py
   :language: python
```

## Square up on a line (two color sensors)

The same maneuver with a corner-mounted color sensor per side, for
rigs without the QTR bar.

```{eval-rst}
.. literalinclude:: ../examples/line_align.py
   :language: python
```

## Full robot (drivebase + IMU + color sensor array)

```{eval-rst}
.. literalinclude:: ../examples/full_robot.py
   :language: python
```
