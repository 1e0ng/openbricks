# SPDX-License-Identifier: MIT
"""QTR array bring-up probe — verify wiring before trusting control.

Run ``examples/qtr_calibrate.py`` once first (it saves the
calibration on the hub); this probe loads it and streams:

    openbricks run -n <hub> examples/qtr_probe.py

Streams calibrated readings as a bar row plus the centroid position
in mm and the dark count. Push the robot across the line by hand
and check:

  * the dark bars track the line's true position, LEFT of the robot
    = NEGATIVE mm (if the sign is flipped, your pin list is wired
    right-to-left — reverse it);
  * dark_count jumps when you hold it over a stop bar / intersection.

Bench wiring: QTRX channels 15..9 (the line cluster, adjacent =
4 mm pitch) on ADC1 GPIO 1,2,3,7,8,9,10 left-to-right and channel 1 (far
right, the branch/marker flag) on GPIO 5. CTRL tied high. See the
pin-budget note below for why exactly these.
"""

import time

from openbricks.drivers.qtr import QTRArray, QTRChannel

# Bench GPIO budget: ADC1 (GPIO 1-10) is the ONLY reliable analog
# bank on the S3, and it also hosts the start/stop button (GPIO 4)
# and the servo-bus RX (GPIO 6). The hub's BLE button defaults to
# GPIO 38 (since 1.66.3 — buttons need no ADC), which is what frees
# GPIO 5 for the branch flag. GPIO 38-42 have no ADC; GPIO 11-20
# are ADC2 = radio-shared and errata-flaky.
QTR_PINS = (1, 2, 3, 7, 8, 9, 10)  # channels 15..9, left -> right
PITCH_MM = 4.0
BRANCH_PIN = 5                     # channel 1, far right

LINE_CAL = "/qtr_line.cal"
BRANCH_CAL = "/qtr_branch.cal"

qtr = QTRArray(pins=QTR_PINS, pitch_mm=PITCH_MM)
branch = QTRChannel(pin=BRANCH_PIN)
qtr.load_calibration(LINE_CAL)          # raises with the remedy if
branch.load_calibration(BRANCH_CAL)     # qtr_calibrate.py never ran
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
