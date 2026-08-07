# SPDX-License-Identifier: MIT
"""QTR calibration — sweep once, save to the hub, reuse everywhere.

    openbricks run -n <hub> examples/qtr_calibrate.py

Slide the robot so the array crosses the line several times while
this runs. Recalibrate when mounting, mat, or lighting changes.
"""

from openbricks.drivers.qtr import QTRArray, QTRChannel

QTR_PINS = (1, 2, 3, 7, 8, 9, 10)
PITCH_MM = 4.0
BRANCH_PIN = 5

LINE_CAL = "/qtr_line.cal"
BRANCH_CAL = "/qtr_branch.cal"

qtr = QTRArray(pins=QTR_PINS, pitch_mm=PITCH_MM)
branch = QTRChannel(pin=BRANCH_PIN)

print("calibrating for 5 s — sweep the array (and the branch "
      "channel) across the line NOW")
qtr.calibrate(duration_ms=5000)
branch.calibrate(duration_ms=5000)

qtr.save_calibration(LINE_CAL)
branch.save_calibration(BRANCH_CAL)

print("saved %s and %s. Per-element spans (bigger = better contrast):"
      % (LINE_CAL, BRANCH_CAL))
for i in range(len(qtr._cal_min)):
    print("  line[%d]: %5d .. %5d" % (i, qtr._cal_min[i],
                                      qtr._cal_max[i]))
print("  branch : %5d .. %5d" % (branch._cal_min[0],
                                 branch._cal_max[0]))
print("done — qtr_probe.py / qtr_line_follow.py will load these.")
