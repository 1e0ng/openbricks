# SPDX-License-Identifier: MIT
"""QTR calibration — sweep once, save to the hub, reuse everywhere.

    openbricks run -n <hub> examples/qtr_calibrate.py

One 10-channel window (ch 15,13,12,11,9,7,5,4,3,1). Slide the robot
so the window crosses the line several times while this runs.
Recalibrate when mounting, mat, or lighting changes.
"""

from openbricks.drivers.qtr import QTRArray

PINS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
POSITIONS_MM = (-28.0, -20.0, -16.0, -12.0, -4.0,
                4.0, 12.0, 16.0, 20.0, 28.0)

CAL = "/qtr.cal"

qtr = QTRArray(pins=PINS, positions_mm=POSITIONS_MM)

print("calibrating for 5 s — sweep the window across the line NOW")
qtr.calibrate(duration_ms=5000)
qtr.save_calibration(CAL)

print("saved %s. Per-element spans (bigger = better contrast):" % CAL)
for i in range(len(qtr._cal_min)):
    print("  [%d]: %5d .. %5d" % (i, qtr._cal_min[i], qtr._cal_max[i]))
print("done — qtr_probe.py / the qtr_line_follow_* examples will load this.")
