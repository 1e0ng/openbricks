---
myst:
  html_meta:
    description: "How to measure and calibrate wheel diameter and axle track for an openbricks DriveBase — two short test drives instead of guesswork."
---

# Measuring wheel diameter & axle track

`DriveBase` converts your commands from millimeters and body-degrees
into wheel rotations using exactly two numbers:

```python
db = DriveBase(left, right, wheel_diameter_mm=88, axle_track_mm=138)
```

* **`wheel_diameter_mm`** — how far one wheel rotation moves the robot.
  Every `straight()` distance scales linearly with it.
* **`axle_track_mm`** — the distance between the two wheels' *contact
  points* with the floor. Every `turn()` angle scales linearly with it.

Getting these two right is worth more than any controller tuning: a 2 %
diameter error is 20 mm of error on a 1 m drive, and a 2 % track error
is ~7° of error on a full spin. A ruler gets you close; the two test
drives below get you to a few tenths of a percent.

Calibrate **wheel diameter first, then axle track** — the turn
calculation uses the wheel diameter, so a diameter error contaminates
the track measurement.

## Step 1 — wheel diameter, from a straight drive

Start from the nominal value (caliper across the tire, or the
manufacturer's spec). Then:

1. Put a strip of masking tape on the floor and align a marked point of
   the robot (e.g. the axle center) with its edge.
2. Run a long straight — the longer the drive, the better the
   resolution:

   ```python
   db.settings(straight_speed=150)
   db.straight(1000)
   ```

3. Measure the distance actually traveled, from tape edge to the same
   marked point, in millimeters.
4. Scale the diameter by how far the robot *really* went:

   ```text
   new_diameter = old_diameter × measured_mm / 1000
   ```

   Traveled 1023 mm with `wheel_diameter_mm=88`? Then
   `88 × 1023 / 1000 = 90.0` is your real diameter.

Repeat once with the new value: the measured distance should now land
within a few millimeters of the command. Soft tires compress under the
robot's weight, so the *effective* diameter is often ~1-3 % smaller
than the caliper says — the test drive measures reality, load included.

## Step 2 — axle track, from an in-place spin

With the wheel diameter calibrated:

1. Align the robot against a straightedge (a wall or ruler touching
   both wheels works well) and note its exact heading.
2. Command several full spins in place — more turns amplify the error
   so it's easier to measure. **Keep the gyro off for this test** (the
   default): the point is to measure what the *encoders* produce.

   ```python
   db.settings(turn_rate=120)
   db.turn(3600)        # ten full clockwise spins
   ```

3. Measure the final heading error `err_deg` against your straightedge:
   positive if the robot rotated *past* the start alignment, negative
   if it stopped short. A protractor or a phone compass app is plenty —
   over ten spins, each degree of final error is only 0.1 % of track.
4. Scale the track by how far the robot really rotated:

   ```text
   new_track = old_track × (3600 + err_deg) / 3600
   ```

   Overshot by 18° with `axle_track_mm=138`? Then
   `138 × 3618 / 3600 = 138.7` is your real track.

Note the direction: if the robot turns **too far**, the real track is
**larger** than configured (each wheel-degree of travel produces less
body rotation than the math assumed), so the correction *increases*
the configured value.

The contact *points* matter, not the wheel centers: wide, soft tires
effectively touch the ground inboard of their centerline, so the real
track is usually a few millimeters less than what you measure
center-to-center with a ruler.

## Checking the result

Drive a square and see how close the robot returns to its start pose:

```python
for _ in range(4):
    db.straight(300)
    db.turn(90)
```

With both values calibrated, the return error on a 300 mm square is
typically under 1 cm and a few degrees. Do the calibration on the same
surface the robot will compete on — carpet, foam mats, and wood all
load the tires differently.

## What about the gyro?

`use_gyro(True)` (with an `imu=` attached) makes turns terminate on
*measured* body rotation, so heading no longer depends on the axle
track being exact — wheel slip included. Calibrate the track anyway:
the controller still uses it to shape the commanded wheel speeds, and
encoder-only operation (gyro off, or no IMU on the robot) depends on
it entirely.
