# SPDX-License-Identifier: MIT
"""QTR bring-up probe for the split 5+5 rig — verify wiring before
trusting control.

Run ``examples/qtr_calibrate.py`` once first, then push the robot
across the line by hand and watch both clusters:

    openbricks run -n <hub> examples/qtr_probe.py
"""

import time

from openbricks.drivers.qtr import QTRArray

LEFT_PINS = (1, 2, 3, 4, 5)
RIGHT_PINS = (6, 7, 8, 9, 10)
PITCH_MM = 4.0

LEFT_CAL = "/qtr_left.cal"
RIGHT_CAL = "/qtr_right.cal"

left = QTRArray(pins=LEFT_PINS, pitch_mm=PITCH_MM)
right = QTRArray(pins=RIGHT_PINS, pitch_mm=PITCH_MM)
left.load_calibration(LEFT_CAL)
right.load_calibration(RIGHT_CAL)
print("loaded calibration. streaming (Ctrl-C to stop):")

_BARS = " .:-=+*#%@"


def _bars(r):
    return "".join(_BARS[min(e.value * (len(_BARS) - 1) // 1000,
                             len(_BARS) - 1)] for e in r)


while True:
    lr = left.read()
    rr = right.read()
    ledge = lr.left_edge_position()
    redge = rr.right_edge_position()
    print("[%s|%s]  ledge=%s  redge=%s  ldark=%d  rdark=%d" % (
        _bars(lr), _bars(rr),
        "----" if ledge is None else "%+6.1fmm" % ledge,
        "----" if redge is None else "%+6.1fmm" % redge,
        lr.dark_count(), rr.dark_count()))
    time.sleep_ms(100)
