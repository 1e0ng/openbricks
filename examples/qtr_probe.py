# SPDX-License-Identifier: MIT
"""QTR bring-up probe — verify wiring before trusting control.

Run ``examples/qtr_calibrate.py`` once first, then push the robot
across the line by hand and watch both edges track:

    openbricks run -n <hub> examples/qtr_probe.py
"""

import time

from openbricks.drivers.qtr import QTRArray

PINS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
POSITIONS_MM = (-28.0, -20.0, -16.0, -12.0, -4.0,
                4.0, 12.0, 16.0, 20.0, 28.0)

CAL = "/qtr.cal"

qtr = QTRArray(pins=PINS, positions_mm=POSITIONS_MM)
qtr.load_calibration(CAL)
print("loaded calibration. streaming (Ctrl-C to stop):")

_BARS = " .:-=+*#%@"

while True:
    r = qtr.read()
    bars = "".join(_BARS[min(e.value * (len(_BARS) - 1) // 1000,
                             len(_BARS) - 1)] for e in r)
    ledge = r.left_edge_position()
    redge = r.right_edge_position()
    print("[%s]  ledge=%s  redge=%s  dark=%d" % (
        bars,
        "----" if ledge is None else "%+6.1fmm" % ledge,
        "----" if redge is None else "%+6.1fmm" % redge,
        r.dark_count()))
    time.sleep_ms(100)
