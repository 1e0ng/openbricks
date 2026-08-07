# SPDX-License-Identifier: MIT
"""QTR array bring-up probe — verify wiring before trusting control.

Run ``examples/qtr_calibrate.py`` once first, then push the robot
across the line by hand and watch the bars:

    openbricks run -n <hub> examples/qtr_probe.py
"""

import time

from openbricks.drivers.qtr import QTRArray, QTRChannel

QTR_PINS = (1, 2, 3, 7, 8, 9, 10)
PITCH_MM = 4.0
BRANCH_PIN = 5

LINE_CAL = "/qtr_line.cal"
BRANCH_CAL = "/qtr_branch.cal"

qtr = QTRArray(pins=QTR_PINS, pitch_mm=PITCH_MM)
branch = QTRChannel(pin=BRANCH_PIN)
qtr.load_calibration(LINE_CAL)
branch.load_calibration(BRANCH_CAL)
print("loaded calibration. streaming (Ctrl-C to stop):")

_BARS = " .:-=+*#%@"

while True:
    r = qtr.read()
    bars = "".join(_BARS[min(e.value * (len(_BARS) - 1) // 1000,
                             len(_BARS) - 1)] for e in r)
    pos = r.position()
    left = r.leftmost_position()
    print("[%s]  pos=%s  left=%s  dark=%d  branch=%s" % (
        bars,
        "----" if pos is None else "%+6.1fmm" % pos,
        "----" if left is None else "%+6.1fmm" % left,
        r.dark_count(),
        "DARK" if branch.dark() else "-"))
    time.sleep_ms(100)
