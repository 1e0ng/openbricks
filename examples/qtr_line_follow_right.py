# SPDX-License-Identifier: MIT
"""Line following on the QTRLineSensor window — right edge.

Run ``examples/qtr_calibrate.py`` once first. Holds the line's
RIGHT edge under element 7 (+16 mm, GPIO 8): that element reads
darker as the robot drifts left, whiter as it drifts right, and
the steer is proportional to its distance from half grey. Elements
8 and 9 sit on the mat beyond the edge; either going dark is a
branch marker. The whole window going dark ends the run. Mirror:
``examples/qtr_line_follow_left.py``; centroid steering on all ten
elements: ``examples/qtr_line_follow_center.py``.
"""

import time

from openbricks.drivers.qtr import QTRLineSensor
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

CRUISE_DPS = 200
KP = 5.0
MAX_DPS = 400
EDGE_INDEX = 7


def clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def get_wheel_speeds(reading):
    if reading.all_dark():
        return None
    steer = KP * (50 - reading.elements[EDGE_INDEX].ambient())
    return (clamp(CRUISE_DPS + steer),
            clamp(CRUISE_DPS - steer))


def branch_seen(reading):
    for e in reading.elements[EDGE_INDEX + 1:]:
        if e.dark():
            return True
    return False

# --- end control law ---


qtr = QTRLineSensor()
qtr.load_calibration("/qtr.cal")

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

print("following. Full-window dark stops the run.")
while True:
    reading = qtr.read()
    speeds = get_wheel_speeds(reading)
    if speeds is None:
        db.stop()
        print("intersection - stopped")
        break
    if branch_seen(reading):
        print("branch marker")
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(5)
