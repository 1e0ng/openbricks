# SPDX-License-Identifier: MIT
"""Line following on the QTRLineSensor window — right mode.

Run ``examples/qtr_calibrate.py`` once first. Holds the line's
RIGHT edge under channel 12 (the geometry lives in the firmware's QTRLineSensor; wiring
table in docs/hardware.md). Switch to
``examples/qtr_line_follow_left.py`` for the mirror discipline,
``examples/qtr_line_follow_center.py`` to hold the line's centre
on all ten elements — or call ``qtr.set_mode(...)`` mid-run. The whole window going
dark ends the run.
"""

import time

from openbricks.drivers.qtr import QTRLineSensor
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

from openbricks.parameters import LineMode

MODE = LineMode.RIGHT

CRUISE_DPS = 200
KP = 5.0
MAX_DPS = 400
FLAG_COUNT = 3


def clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def get_wheel_speeds(reading):
    if all(e.ambient() < 50 for e in reading.elements):
        return None
    steer = KP * reading.edge_error()
    return (clamp(CRUISE_DPS + steer),
            clamp(CRUISE_DPS - steer))


def branch_seen(reading, mode):
    if mode == LineMode.LEFT:
        flags = reading.elements[-FLAG_COUNT:]
    elif mode == LineMode.RIGHT:
        flags = reading.elements[:FLAG_COUNT]
    else:
        flags = reading.elements[:FLAG_COUNT] + reading.elements[-FLAG_COUNT:]
    for e in flags:
        if e.ambient() < 50:
            return True
    return False

# --- end control law ---


qtr = QTRLineSensor()
qtr.set_mode(MODE)
qtr.load_calibration("/qtr.cal")

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

print("following (%s mode). Full-window dark stops the run." % MODE)
while True:
    reading = qtr.read()
    speeds = get_wheel_speeds(reading)
    if speeds is None:
        db.stop()
        print("intersection - stopped")
        break
    if branch_seen(reading, MODE):
        print("branch marker")
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(5)
