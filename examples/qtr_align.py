# SPDX-License-Identifier: MIT
"""Square up on the edge of a perpendicular line, QTRLineSensor.

The classic FLL/WRO align move, on one sensor bar instead of two
corner sensors: each half of the window is one virtual corner
sensor, and each wheel servos proportionally until its half's mean
ambient reads 50 — the elements straddling the black/white
boundary. The wheel whose half arrives first holds while the other
pivots the chassis on; overshoot backs up. Both halves end ON the
edge, so the bar — and the chassis — is square right at it.

Run ``examples/qtr_calibrate.py`` once first. The bar must be
mounted ahead of the wheels; the farther ahead, the finer the final
heading.
"""

import time

from openbricks.drivers.qtr import QTRLineSensor
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase
from openbricks.parameters import Stop

# --- control law (pure logic, unit-tested in tests/test_qtr_align.py) ---

KP = 1.3
EDGE_TOLERANCE = 8
SIDE_COUNT = 5


def side_ambient(elements):
    total = 0
    for e in elements:
        total += e.ambient()
    return total // len(elements)


def edge_dps(elements, target):
    error = side_ambient(elements) - target
    if abs(error) <= EDGE_TOLERANCE:
        return 0
    return int(KP * error)


def edge_wheel_speeds(reading):
    left = edge_dps(reading.elements[:SIDE_COUNT], 50)
    right = edge_dps(reading.elements[-SIDE_COUNT:], 50)
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
    speeds = edge_wheel_speeds(qtr.read())
    if speeds is None:
        break
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(5)
db.stop(then=Stop.BRAKE)
print("aligned - square on the edge")
