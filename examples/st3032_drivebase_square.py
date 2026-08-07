# SPDX-License-Identifier: MIT
"""
Drive a square with the ST-3032 drivebase.

Four sides of ``SIDE_MM`` straight + ``+90°`` turn each. Demonstrates
``DriveBase.straight`` / ``turn`` composing into a closed loop — if
the chassis geometry (``WHEEL_DIAMETER_MM`` / ``AXLE_TRACK_MM``) is
calibrated correctly, the robot returns to within a few cm of its
starting pose after one lap.

Uses the new ``then="coast"`` default end-state (1.6.7), so the
wheels free-wheel briefly between segments. If your bench has
significant momentum carryover and you'd rather pin the wheels
between sides, pass ``then="brake"`` to the ``straight`` / ``turn``
calls (or ``then="hold"`` for active position lock — ST-3032
supports it).

Hardware: identical to ``examples/st3032_drivebase_test.py``.

Run with:
    openbricks run -n ls examples/st3032_drivebase_square.py
"""

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase


LEFT_ID, RIGHT_ID = 2, 1
UART_ID, TX, RX   = 1, 14, 6

WHEEL_DIAMETER_MM = 88
AXLE_TRACK_MM     = 136

SIDE_MM   = 200
NUM_LAPS  = 1

STRAIGHT_SPEED_MM = 150
TURN_RATE_DPS     = 200


def main():
    print("--- ST-3032 drivebase square (%d mm sides × %d laps) ---" %
          (SIDE_MM, NUM_LAPS))

    left  = ST3032Motor(servo_id=LEFT_ID,  uart_id=UART_ID, tx=TX, rx=RX, invert=True)
    right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX)

    db = DriveBase(left, right,
                   wheel_diameter_mm=WHEEL_DIAMETER_MM,
                   axle_track_mm=AXLE_TRACK_MM)
    db.settings(straight_speed=STRAIGHT_SPEED_MM, turn_rate=TURN_RATE_DPS)

    for lap in range(NUM_LAPS):
        for side in range(4):
            print("  lap %d side %d: straight(%d) → turn(+90)" %
                  (lap + 1, side + 1, SIDE_MM))
            db.straight(SIDE_MM)
            db.turn(90)

    print("--- done ---")


main()
