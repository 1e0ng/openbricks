# SPDX-License-Identifier: MIT
"""Line following on the QTRLineSensor window — centre.

Run ``examples/qtr_calibrate.py`` once first. Holds the line's
CENTRE under the middle of the window, steering on
``reading.position()`` — the dark-weighted centroid in mm, positive
when the line sits right of centre — so the error stays
proportional across the whole 56 mm span. When no element is dark
the line has left the window and the robot turns hard toward the
side it was last seen on. The two outermost elements on each side
are the branch watch. The whole window going dark ends the run.
Edge disciplines: ``examples/qtr_line_follow_left.py`` / ``_right.py``.
"""

import time

from openbricks.drivers.qtr import QTRLineSensor
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

CRUISE_DPS = 200
KP_MM = 6.0
MAX_DPS = 400
FLAG_COUNT = 2

last_side = 1


def clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def get_wheel_speeds(reading):
    global last_side
    if reading.all_dark():
        return None
    position = reading.position()
    if position is None:
        steer = last_side * CRUISE_DPS
    else:
        if position > 0:
            last_side = 1
        elif position < 0:
            last_side = -1
        steer = KP_MM * position
    return (clamp(CRUISE_DPS + steer),
            clamp(CRUISE_DPS - steer))


def branch_seen(reading):
    for e in reading.elements[:FLAG_COUNT] + reading.elements[-FLAG_COUNT:]:
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
