# SPDX-License-Identifier: MIT
"""Square up on the edge of a perpendicular line, QTRLineSensor.

The classic FLL/WRO align move, on one sensor bar instead of two
corner sensors, in two passes: drive slowly toward the line until
each half of the window reaches it (the wheel whose half arrives
first stops, the other pivots the chassis on), then back each wheel
off until its half turns white again. Both halves end sitting right
at the line's near edge, so the bar — and the chassis — is square
on the edge, not somewhere inside the line.

Run ``examples/qtr_calibrate.py`` once first. The bar must be
mounted ahead of the wheels; the farther ahead, the finer the final
heading.
"""

import time

from openbricks.drivers.qtr import QTRLineSensor
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_align.py) ---

SEEK_DPS = 100
BACK_DPS = 60
SIDE_COUNT = 5


def _side_on_line(elements):
    for e in elements:
        if e.dark():
            return True
    return False


def _sides(reading):
    return (_side_on_line(reading.elements[:SIDE_COUNT]),
            _side_on_line(reading.elements[-SIDE_COUNT:]))


def seek_speeds(reading):
    left_on, right_on = _sides(reading)
    if left_on and right_on:
        return None
    return (0 if left_on else SEEK_DPS,
            0 if right_on else SEEK_DPS)


def edge_speeds(reading):
    left_on, right_on = _sides(reading)
    if not left_on and not right_on:
        return None
    return (-BACK_DPS if left_on else 0,
            -BACK_DPS if right_on else 0)

# --- end control law ---


qtr = QTRLineSensor()
qtr.load_calibration("/qtr.cal")

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)


print("aligning on the line ...")
while True:
    speeds = seek_speeds(qtr.read())
    if speeds is None:
        break
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(5)

while True:
    speeds = edge_speeds(qtr.read())
    if speeds is None:
        break
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(5)
db.stop(then="brake")
print("aligned - square on the edge")
