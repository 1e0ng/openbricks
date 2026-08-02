# SPDX-License-Identifier: MIT
"""
Step-mode bench test (1.46.0): run_angle / hold on ADOPTED motors.

Since 1.46.0, motors adopted by a serial-bus ``DriveBase`` execute
``run_angle`` / ``hold`` as per-slot position moves on the hard tick
(st_move_core), and the drivebase YIELDS the wheels whenever it
doesn't own a move. This script walks every new behavior with pauses
for physical checks. Put a piece of tape on each wheel first — the
run_angle steps command exactly full turns, so the tape shows the
landing error directly.

Bench hardware (edit if yours differs): 2x ST-3032, left id=2
inverted / right id=1, UART1 tx=14 rx=6, 88 mm wheels, 138 mm track.
Wheels-up is fine (and best for the hold test).

Run:
    openbricks run -n ls examples/st3032_step_mode_test.py
"""

import time

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

LEFT_ID, RIGHT_ID = 2, 1
UART_ID, TX, RX   = 1, 14, 6
WHEEL_MM, AXLE_MM = 88, 138


def pause(seconds, msg):
    print("  >>> %s (%ds)" % (msg, seconds))
    time.sleep(seconds)


def report(label, motor, expected_deg, start_deg):
    moved = motor.angle() - start_deg
    print("  [%s] moved %+.1f deg (commanded %+d) -> err %+.1f"
          % (label, moved, expected_deg, moved - expected_deg))


def main():
    left  = ST3032Motor(servo_id=LEFT_ID,  uart_id=UART_ID, tx=TX, rx=RX,
                        invert=True)
    right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX)
    db = DriveBase(left, right, wheel_diameter_mm=WHEEL_MM,
                   axle_track_mm=AXLE_MM)
    db.settings(straight_speed=80, turn_rate=80)

    print("--- 1: yield at construction ---")
    pause(5, "wheels should spin FREELY by hand (pre-1.46.0: stiff)")

    print("--- 2: drivebase move, then yield after stop ---")
    db.straight(100)
    pause(5, "db move done + stopped: wheels free again?")

    print("--- 3: run_angle on adopted motors (one full turn) ---")
    a0 = right.angle()
    right.run_angle(120, 360)
    report("right +360", right, 360, a0)
    a0 = left.angle()
    left.run_angle(120, 360)
    report("left  +360", left, 360, a0)
    pause(3, "tape marks should be back at start (hold is active)")

    print("--- 4: hold resists ---")
    left.hold()
    right.hold()
    pause(6, "try turning both wheels by hand — they should fight back")

    print("--- 5: concurrent wait=False + done() ---")
    l0, r0 = left.angle(), right.angle()
    left.run_angle(120, -360, wait=False)
    right.run_angle(120, -360, wait=False)
    while not (left.done() and right.done()):
        time.sleep_ms(10)
    report("left  -360", left, -360, l0)
    report("right -360", right, -360, r0)

    print("--- 6: run_angle during a db move must raise ---")
    db.straight(200, wait=False)
    try:
        right.run_angle(120, 90)
        print("  [FAIL] no exception raised")
    except RuntimeError as e:
        print("  [ok] refused as expected:", e)
    while not db.done():
        time.sleep_ms(10)

    print("--- 7: db reclaims after motor moves ---")
    db.turn(90)
    print("  [ok] turn completed")

    print("--- 8: stop(then='hold') on the drivebase ---")
    db.straight(100, wait=False)
    time.sleep_ms(400)
    db.stop(then="hold")
    pause(6, "wheels should HOLD position now — try turning them")

    db.stop()   # coast out
    print("--- done. Paste this whole output back. ---")


main()
