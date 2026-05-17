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


LEFT_ID, RIGHT_ID = 1, 2
UART_ID, TX, RX   = 1, 14, 6

WHEEL_DIAMETER_MM = 65    # EDIT to your wheels
AXLE_TRACK_MM     = 120   # EDIT to your chassis

# Geometry of the square. SIDE_MM 200 traces a 20 cm × 20 cm square —
# fits comfortably on a small mat. Bump for a larger arena.
SIDE_MM   = 200
NUM_LAPS  = 1

# Conservative chassis speeds for a small bench robot — well below
# the ST-3032 mechanical limit (datasheet no-load 888 dps at 12 V,
# loaded working point ~600 dps; see ``docs/datasheets/feetech_sts3032.pdf``).
# Dial down further if the chassis slips or the bus drops packets.
STRAIGHT_SPEED_MM = 150
TURN_RATE_DPS     = 200


def main():
    print("--- ST-3032 drivebase square (%d mm sides × %d laps) ---" %
          (SIDE_MM, NUM_LAPS))

    left  = ST3032Motor(servo_id=LEFT_ID,  uart_id=UART_ID, tx=TX, rx=RX)
    right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX,
                        invert=True)

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
