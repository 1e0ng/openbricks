# SPDX-License-Identifier: MIT
"""Trace ST3215Motor.run_angle on both motors to diagnose the
[6c] right-motor overshoot.

Runs left run_angle(60, +90) and right run_angle(60, +90) with
``debug=True`` so we see:

  * the anchor read + write,
  * the per-poll position trajectory in position mode,
  * the moment the inner loop exits (cur, err at that instant),
  * the position right before and right after the mode-switch +
    brake in the finally block,
  * 50/200/500 ms settle samples afterward.

Compare left vs right line-for-line. Where the overshoot happens
(during position mode or after mode switch) pins which hypothesis
is right.
"""

from openbricks.drivers.st3215 import ST3215Motor


left  = ST3215Motor(servo_id=1, uart_id=1, tx=14, rx=6)
right = ST3215Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)

print("\n========== LEFT (id=1, no invert) ==========")
left.run_angle(60, 90, debug=True)

print("\n========== RIGHT (id=2, invert=True) ==========")
right.run_angle(60, 90, debug=True)

print("\n--- done ---")
