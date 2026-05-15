# SPDX-License-Identifier: MIT
"""Trace ST3032Motor.run_angle on both motors at full speed to see
whether the ~+7° asymmetry observed on ST-3215 hardware (right -90°
and multi-chunk +720° both missed by exactly one ~85-raw post-brake
settle) reproduces on the smaller, faster ST-3032 servo.

Runs four ``run_angle`` calls with ``debug=True`` so we see:

  * the anchor read + write,
  * the per-poll position trajectory in position mode,
  * the moment the inner loop exits (cur, err at that instant),
  * the position right before and right after the mode-switch +
    brake in the finally block,
  * 50/200/500 ms settle samples afterward.

Cases (chosen to bracket the ST-3215 bug):
  1. right +90° — passed on ST-3215 ([6c])
  2. right -90° — failed by +7° on ST-3215 ([6d])
  3. left  +360° — 2-chunk multi-rev (smaller than [6e])
  4. left  +720° — 5-chunk, failed by +7° on ST-3215 ([6e])

Compare 1 vs 2 to test the direction-dependence hypothesis on the
right motor. Compare 3 vs 4 to see whether per-chunk error scales
with chunk count.
"""

from openbricks.drivers.st3032 import ST3032Motor


SPEED_DPS = 300   # ST-3032 no-load ~300 dps at 12 V

left  = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
right = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)

print("\n========== [1] right +90° (was [6c], passed on ST-3215) ==========")
right.run_angle(SPEED_DPS, +90, debug=True)

print("\n========== [2] right -90° (was [6d], failed +7° on ST-3215) ==========")
right.run_angle(SPEED_DPS, -90, debug=True)

print("\n========== [3] left +360° (2-chunk) ==========")
left.run_angle(SPEED_DPS, +360, debug=True)

print("\n========== [4] left +720° (was [6e], failed +7° on ST-3215) ==========")
left.run_angle(SPEED_DPS, +720, debug=True)

print("\n--- done ---")
