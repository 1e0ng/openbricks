# SPDX-License-Identifier: MIT
"""Line following on a QTR reflectance array — continuous-position PD.

Run ``examples/qtr_calibrate.py`` once first. Follows the line's
LEFT edge (the white→black boundary stays under the array centre);
the whole array AND the branch flag dark in the same snapshot
stops the run.
"""

import time

from openbricks.drivers.qtr import QTRArray, QTRChannel
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

CRUISE_DPS = 100
KP = 14.0
KD = 0.6
MAX_DPS = 300


def _clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def _pd_wheel_speeds(reading, branch_dark, prev_error, dt_s):
    if branch_dark and reading.all_dark():
        return None, prev_error
    error = reading.left_edge_position()
    if error is None:
        error = prev_error if prev_error is not None else 0.0
    derivative = 0.0
    if prev_error is not None and dt_s > 0:
        derivative = (error - prev_error) / dt_s
    steer = KP * error + KD * derivative
    return (_clamp(CRUISE_DPS + steer),
            _clamp(CRUISE_DPS - steer)), error

# --- end control law ---


QTR_PINS = (1, 2, 3, 7, 8, 9, 10)
PITCH_MM = 4.0
BRANCH_PIN = 5

LINE_CAL = "/qtr_line.cal"
BRANCH_CAL = "/qtr_branch.cal"

qtr = QTRArray(pins=QTR_PINS, pitch_mm=PITCH_MM)
branch = QTRChannel(pin=BRANCH_PIN)
qtr.load_calibration(LINE_CAL)
branch.load_calibration(BRANCH_CAL)

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

print("following. Intersection stops the run.")
prev_error = None
last_ms = time.ticks_ms()
while True:
    reading = qtr.read()
    branch_dark = branch.dark()
    now_ms = time.ticks_ms()
    dt_s = time.ticks_diff(now_ms, last_ms) / 1000
    last_ms = now_ms
    speeds, prev_error = _pd_wheel_speeds(reading, branch_dark,
                                          prev_error, dt_s)
    if speeds is None:
        db.stop()
        print("intersection - stopped")
        break
    if branch_dark:
        print("branch marker")
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(10)
