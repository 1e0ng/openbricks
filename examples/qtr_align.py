# SPDX-License-Identifier: MIT
"""Square up on the edge of a perpendicular line, QTRLineSensor.

The classic FLL/WRO align move, on one sensor bar instead of two
corner sensors, in two passes: drive slowly toward the line until
each half of the window reaches it (the wheel whose half arrives
first stops, the other pivots the chassis on), then nudge each
wheel until its half reads ambient of about 50 — the elements
straddling the black/white boundary. Both halves end ON the edge,
so the bar — and the chassis — is square right at it.

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
EDGE_DPS = 60
SIDE_COUNT = 5
EDGE_LOW = 40
EDGE_HIGH = 60


def _side_on_line(elements):
    for e in elements:
        if e.dark():
            return True
    return False


def _side_ambient(elements):
    total = 0
    for e in elements:
        total += e.ambient()
    return total // len(elements)


def _edge_dps(elements):
    ambient = _side_ambient(elements)
    if ambient < EDGE_LOW:
        return -EDGE_DPS
    if ambient > EDGE_HIGH:
        return EDGE_DPS
    return 0


def seek_speeds(reading):
    left_on = _side_on_line(reading.elements[:SIDE_COUNT])
    right_on = _side_on_line(reading.elements[-SIDE_COUNT:])
    if left_on and right_on:
        return None
    return (0 if left_on else SEEK_DPS,
            0 if right_on else SEEK_DPS)


def edge_speeds(reading):
    left = _edge_dps(reading.elements[:SIDE_COUNT])
    right = _edge_dps(reading.elements[-SIDE_COUNT:])
    if left == 0 and right == 0:
        return None
    return (left, right)

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
