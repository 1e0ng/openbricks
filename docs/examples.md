---
myst:
  html_meta:
    description: "Example programs for openbricks robots: drivebase squares, gyro-corrected driving, line following (edge and center modes), color sensing (direct or via a TCA9548A mux), color sorting, and distance-based wall avoidance."
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
(One deviation: `then` defaults to `Stop.COAST` like every openbricks
move; pass `then=Stop.HOLD` for the Pybricks end state.)

```{eval-rst}
.. literalinclude:: ../examples/st3032_drivebase_curve.py
   :language: python
```

## Colour sensor, direct to the ESP32 (one TCS34725)

Mode 1 from the {doc}`hardware guide </hardware>`: a single TCS34725
on GPIO 15/16, no multiplexer. The driver is handed the bus itself.

```{eval-rst}
.. literalinclude:: ../examples/read_color.py
   :language: python
```

## Colour sensor array via a TCA9548A (two TCS34725s)

Mode 2: the TCS34725's address is fixed at `0x29`, so two or more go
through a TCA9548A multiplexer, one per channel. `mux[n]` behaves
like an I2C bus, so the driver call is the same as above — only the
bus argument changes. Each loop combines `ambient()` and `rgb()` to
name the colour under every sensor.

```{eval-rst}
.. literalinclude:: ../examples/color_array.py
   :language: python
```

## Line following (QTR sensor bar, center mode)

The QTRLineSensor's `LineMode.CENTER` mode steers on the weighted centroid
of all ten elements, so `edge_error()` is proportional across the
whole 56 mm window. The same control law ships pinned to
`LineMode.LEFT` and `LineMode.RIGHT` in `examples/qtr_line_follow_left.py` /
`_right.py` — see {doc}`/hardware` for what each mode holds.

```{eval-rst}
.. literalinclude:: ../examples/qtr_line_follow_center.py
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

## Full robot (ST-3032 drivebase + IMU + colour sensor + arm)

Everything from the reference build on one bus map — the wheels and
the optional ST-3215 arm share the serial bus (IDs 1, 2 and 3), the
IMU is on SPI, the colour sensor on I2C, and the QTR bank (GPIO 1-10)
is left untouched.

```{eval-rst}
.. literalinclude:: ../examples/full_robot.py
   :language: python
```

## Servo dynamometer (two ST-3032s, coupled)

Measures one servo's real no-load speed, stall-torque estimate and
gear play against a second servo used as the load — the per-unit
numbers to compare across your spares. Procedure and how to read the
summary: {doc}`servo-health`.

```{eval-rst}
.. literalinclude:: ../examples/st3032_dyno.py
   :language: python
```

## Servo soak test (one ST-3032)

An hour of swings with voltage, temperature, current and gear play
logged along the way, and the drift summarised at the end — the
pre-competition check for a servo you are not sure about.

```{eval-rst}
.. literalinclude:: ../examples/st3032_soak_test.py
   :language: python
```
