# SPDX-License-Identifier: MIT
"""
Step-mode bench test (1.46.0): run_angle / hold on ADOPTED motors.

Since 1.46.0, motors adopted by a serial-bus ``DriveBase`` execute
``run_angle`` / ``hold`` as per-slot position moves on the hard tick
(st_move_core), and the drivebase YIELDS the wheels whenever it
doesn't own a move.

The physical checks are MEASURED, not eyeballed: each interactive
phase watches the RIGHT wheel's encoder for up to 30 s, advances the
moment it sees your hand, and prints its own verdict — so there is
no pause to race against. Wheels-up is best.

Bench hardware (edit if yours differs): 2x ST-3032, left id=2
inverted / right id=1, UART1 tx=14 rx=6, 88 mm wheels, 138 mm track.

Run:
    openbricks run -n ls examples/st3032_step_mode_test.py
"""

import time

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

LEFT_ID, RIGHT_ID = 2, 1
UART_ID, TX, RX   = 1, 14, 6
WHEEL_MM, AXLE_MM = 88, 138

WAIT_S = 30


def check_free(motor, label):
    """Wait for the user to spin the wheel by hand; a freely spinning
    wheel accumulates angle, a held one fights back to its target.
    Verdict from the encoder, not the eye."""
    print("  >>> spin the RIGHT wheel by hand (%ds window)" % WAIT_S)
    base = motor.angle()
    moved = 0.0
    for i in range(WAIT_S * 10):
        time.sleep_ms(100)
        moved = motor.angle() - base
        if abs(moved) > 60:
            break
        if i % 50 == 49:
            print("  ... waiting (%ds left)" % (WAIT_S - (i + 1) // 10))
    if abs(moved) > 60:
        print("  [%s] wheel turned %+.0f deg by hand -> FREE, PASS"
              % (label, moved))
        return True
    print("  [%s] no manual spin detected in %ds (moved %+.0f deg) "
          "-> either skipped, or the wheel is HELD: FAIL if you "
          "tried" % (label, WAIT_S, moved))
    return False


def check_hold(m, label):
    """Wait for the user to push against the hold; report how far the
    shaft was forced and where it settled. A holding servo yields a
    bounded push and pulls back; a coasting one keeps whatever angle
    the hand left it at."""
    print("  >>> try to turn the RIGHT wheel against the hold "
          "(%ds window)" % WAIT_S)
    target = m.angle()
    peak = 0.0
    disturbed_at = None
    for i in range(WAIT_S * 10):
        time.sleep_ms(100)
        dev = m.angle() - target
        if abs(dev) > abs(peak):
            peak = dev
        if disturbed_at is None and abs(dev) > 5:
            disturbed_at = i
        if disturbed_at is not None and i - disturbed_at >= 50:
            break
        if i % 50 == 49 and disturbed_at is None:
            print("  ... waiting (%ds left)" % (WAIT_S - (i + 1) // 10))
    if disturbed_at is None:
        print("  [%s] no push detected in %ds -> SKIPPED" % (label, WAIT_S))
        return False
    time.sleep_ms(800)
    settled = m.angle() - target
    verdict = "HOLDING, PASS" if abs(settled) < 15 else \
              "NOT holding (stayed %+.0f deg off): FAIL" % settled
    print("  [%s] peak push %+.0f deg, settled %+.0f deg from target "
          "-> %s" % (label, peak, settled, verdict))
    return abs(settled) < 15


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

    print("--- 1: yield at construction (wheels should be free) ---")
    check_free(right, "at construction")

    print("--- 2: drivebase move, then yield after stop ---")
    db.straight(100)
    check_free(right, "after db stop")

    print("--- 3: run_angle on adopted motors (one full turn) ---")
    a0 = right.angle()
    right.run_angle(120, 360)
    report("right +360", right, 360, a0)
    a0 = left.angle()
    left.run_angle(120, 360)
    report("left  +360", left, 360, a0)

    print("--- 4: hold resists ---")
    left.hold()
    right.hold()
    check_hold(right, "motor hold")

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
    check_hold(right, "db stop-hold")

    db.stop()
    print("--- done. Paste this whole output back. ---")


main()
