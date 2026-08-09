# SPDX-License-Identifier: MIT
"""Square up on a perpendicular line using the QTRLineSensor window.

The classic FLL/WRO align move, on one sensor bar instead of two
corner sensors: drive slowly toward the line; the moment one half of
the window crosses onto it, that wheel stops while the other keeps
rolling, pivoting the chassis until its half arrives too. Both
halves dark = the bar (and the chassis) is square to the line.

Run ``examples/qtr_calibrate.py`` once first. The bar must be
mounted ahead of the wheels; the farther ahead, the finer the final
heading.
"""

import time

from openbricks.drivers.qtr import QTRLineSensor
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_align.py) ---

ALIGN_DPS = 100
SIDE_COUNT = 5


def _side_on_line(elements):
    for e in elements:
        if e.dark():
            return True
    return False


def get_wheel_speeds(reading):
    left_on = _side_on_line(reading.elements[:SIDE_COUNT])
    right_on = _side_on_line(reading.elements[-SIDE_COUNT:])
    if left_on and right_on:
        return None
    return (0 if left_on else ALIGN_DPS,
            0 if right_on else ALIGN_DPS)

# --- end control law ---


CAL = "/qtr.cal"
TIMEOUT_MS = 8000
POLL_MS = 5

qtr = QTRLineSensor()
qtr.load_calibration(CAL)

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

print("aligning on the line ...")
aligned = False
for _ in range(TIMEOUT_MS // POLL_MS):
    speeds = get_wheel_speeds(qtr.read())
    if speeds is None:
        db.stop(then="brake")
        aligned = True
        break
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(POLL_MS)

if not aligned:
    db.stop()
    raise RuntimeError(
        "no line within %d ms - is a line in reach, and is %s "
        "calibrated for your mat?" % (TIMEOUT_MS, CAL))
print("aligned - square to the line")
