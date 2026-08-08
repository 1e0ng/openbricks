# SPDX-License-Identifier: MIT
"""Line following on the split 5+5 QTR rig — LEFT-edge discipline.

Run ``examples/qtr_calibrate.py`` once first. The LEFT cluster
follows the line's LEFT edge; the RIGHT cluster is the branch /
ending flag bank. Left cluster fully dark AND the right cluster
seeing dark in the same instant ends the run.
"""

import time

from openbricks.drivers.qtr import QTRArray
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

CRUISE_DPS = 200
KP = 5.0
MAX_DPS = 400


def _clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def get_wheel_speeds(reading, branch_dark):
    if branch_dark and reading.all_dark():
        return None
    steer = KP * reading.left_edge_position()
    return (_clamp(CRUISE_DPS + steer),
            _clamp(CRUISE_DPS - steer))

# --- end control law ---


LEFT_PINS = (1, 2, 3, 4, 5)
RIGHT_PINS = (6, 7, 8, 9, 10)
PITCH_MM = 4.0

LEFT_CAL = "/qtr_left.cal"
RIGHT_CAL = "/qtr_right.cal"

left_qtr = QTRArray(pins=LEFT_PINS, pitch_mm=PITCH_MM)
right_qtr = QTRArray(pins=RIGHT_PINS, pitch_mm=PITCH_MM)
left_qtr.load_calibration(LEFT_CAL)
right_qtr.load_calibration(RIGHT_CAL)

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

print("following the LEFT edge. Intersection stops the run.")
while True:
    reading = left_qtr.read()
    branch_dark = right_qtr.read().dark_count() > 0
    speeds = get_wheel_speeds(reading, branch_dark)
    if speeds is None:
        db.stop()
        print("intersection - stopped")
        break
    if branch_dark:
        print("branch marker")
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(5)
