# SPDX-License-Identifier: MIT
"""QTR calibration — sweep once, save to the hub, reuse everywhere.

    openbricks run -n <hub> examples/qtr_calibrate.py

Slide the robot so the QTRLineSensor window crosses the line
several times while this runs. Recalibrate when mounting, mat, or
lighting changes.
"""

from openbricks.drivers.qtr import QTRLineSensor

CAL = "/qtr.cal"

qtr = QTRLineSensor()

print("calibrating for 5 s — sweep the window across the line NOW")
qtr.calibrate(duration_ms=5000)
qtr.save_calibration(CAL)

print("saved %s. Per-element spans (bigger = better contrast):" % CAL)
for i in range(len(qtr._cal_min)):
    print("  [%d]: %5d .. %5d" % (i, qtr._cal_min[i], qtr._cal_max[i]))
print("done — qtr_probe.py / the qtr_line_follow_* examples will "
      "load this.")
