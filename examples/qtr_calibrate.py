# SPDX-License-Identifier: MIT
"""QTR calibration — sweep once, save to the hub, reuse everywhere.

    openbricks run -n <hub> examples/qtr_calibrate.py

Split rig: 5 channels on the left cluster, 5 on the right. Slide
the robot so BOTH clusters cross the line several times while this
runs. Recalibrate when mounting, mat, or lighting changes.
"""

from openbricks.drivers.qtr import QTRArray

LEFT_PINS = (1, 2, 3, 4, 5)
RIGHT_PINS = (6, 7, 8, 9, 10)
PITCH_MM = 4.0

LEFT_CAL = "/qtr_left.cal"
RIGHT_CAL = "/qtr_right.cal"

left = QTRArray(pins=LEFT_PINS, pitch_mm=PITCH_MM)
right = QTRArray(pins=RIGHT_PINS, pitch_mm=PITCH_MM)

print("calibrating left cluster for 5 s — sweep it across the line NOW")
left.calibrate(duration_ms=5000)
print("calibrating right cluster for 5 s — sweep it across the line NOW")
right.calibrate(duration_ms=5000)

left.save_calibration(LEFT_CAL)
right.save_calibration(RIGHT_CAL)

print("saved %s and %s. Per-element spans (bigger = better contrast):"
      % (LEFT_CAL, RIGHT_CAL))
for i in range(len(left._cal_min)):
    print("  left[%d] : %5d .. %5d" % (i, left._cal_min[i],
                                       left._cal_max[i]))
for i in range(len(right._cal_min)):
    print("  right[%d]: %5d .. %5d" % (i, right._cal_min[i],
                                       right._cal_max[i]))
print("done — qtr_probe.py / qtr_line_follow.py will load these.")
