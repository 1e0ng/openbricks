# SPDX-License-Identifier: MIT
"""Line following on the 10-channel QTR window — right mode.

Run ``examples/qtr_calibrate.py`` once first. Holds the line's
RIGHT edge under channel 4; switch to ``examples/qtr_line_follow_left.py`` for the
mirror discipline (the law is shared — MODE can also be flipped per
call mid-run). The whole window going dark ends the run.
"""

import time

from openbricks.drivers.qtr import QTRArray
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

MODE = "right"

CRUISE_DPS = 200
KP = 5.0
MAX_DPS = 400
LEFT_SETPOINT_MM = -16.0
RIGHT_SETPOINT_MM = 16.0
FLAG_COUNT = 3


def _clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def get_wheel_speeds(reading, mode):
    if reading.all_dark():
        return None
    if mode == "left":
        error = reading.left_edge_position() - LEFT_SETPOINT_MM
    elif mode == "right":
        error = reading.right_edge_position() - RIGHT_SETPOINT_MM
    else:
        raise ValueError("mode must be 'left' or 'right', got %r"
                         % (mode,))
    steer = KP * error
    return (_clamp(CRUISE_DPS + steer),
            _clamp(CRUISE_DPS - steer))


def branch_seen(reading, mode):
    if mode == "left":
        flags = reading.elements[-FLAG_COUNT:]
    else:
        flags = reading.elements[:FLAG_COUNT]
    for e in flags:
        if e.dark():
            return True
    return False

# --- end control law ---


PINS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
POSITIONS_MM = (-28.0, -20.0, -16.0, -12.0, -4.0,
                4.0, 12.0, 16.0, 20.0, 28.0)

CAL = "/qtr.cal"

qtr = QTRArray(pins=PINS, positions_mm=POSITIONS_MM)
qtr.load_calibration(CAL)

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

print("following (%s mode). Full-window dark stops the run." % MODE)
while True:
    reading = qtr.read()
    speeds = get_wheel_speeds(reading, MODE)
    if speeds is None:
        db.stop()
        print("intersection - stopped")
        break
    if branch_seen(reading, MODE):
        print("branch marker")
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(5)
