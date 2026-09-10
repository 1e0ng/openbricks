# SPDX-License-Identifier: MIT
"""Line following on the QTRLineSensor window — left edge.

Run ``examples/qtr_calibrate.py`` once first. Holds the line's
LEFT edge under element 2 (-16 mm, GPIO 3): that element reads
darker as the robot drifts right, whiter as it drifts left, and
the steer is proportional to its distance from half grey. Elements
0 and 1 sit on the mat beyond the edge; either going dark is a
branch marker. The whole window going dark ends the run. Mirror:
``examples/qtr_line_follow_right.py``; centroid steering on all
ten elements: ``examples/qtr_line_follow_center.py``.
"""

import time

from openbricks.drivers.qtr import QTRLineSensor
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

CRUISE_DPS = 200
KP = 5.0
MAX_DPS = 400
EDGE_INDEX = 2


def clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def get_wheel_speeds(reading):
    if reading.all_dark():
        return None
    steer = KP * (reading.elements[EDGE_INDEX].ambient() - 50)
    return (clamp(CRUISE_DPS + steer),
            clamp(CRUISE_DPS - steer))


def branch_seen(reading):
    for e in reading.elements[:EDGE_INDEX]:
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
