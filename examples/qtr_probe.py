# SPDX-License-Identifier: MIT
"""QTR array bring-up probe — verify wiring before trusting control.

Run me FIRST on a freshly mounted QTRX array:

    openbricks run -n <hub> examples/qtr_probe.py

Phase 1 (3 s): calibration sweep — slide/rotate the robot so the
array crosses the line a few times. A channel that never sees
contrast is named with its index (left = 0): that is the unwired /
mis-mounted one.

Phase 2: streams calibrated readings as a bar row plus the centroid
position in mm and the dark count. Push the robot across the line by
hand and check:

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
# and the servo-bus RX (GPIO 6) — so the BLE button moved to GPIO 38
# to free GPIO 5 for the branch flag. GPIO 38-42 have no ADC (fine
# for buttons, useless for the array); GPIO 11-20 are ADC2 = radio-
# shared and errata-flaky.
QTR_PINS = (1, 2, 3, 7, 8, 9, 10)  # channels 15..9, left -> right
PITCH_MM = 4.0
BRANCH_PIN = 5                     # channel 1, far right

qtr = QTRArray(pins=QTR_PINS, pitch_mm=PITCH_MM)
branch = QTRChannel(pin=BRANCH_PIN)

print("calibrating for 3 s — sweep the array across the line NOW")
qtr.calibrate(duration_ms=3000)
branch.calibrate(duration_ms=3000)
print("calibrated. streaming (Ctrl-C to stop):")

_BARS = " .:-=+*#%@"

while True:
    r = qtr.read()
    bars = "".join(_BARS[min(v * (len(_BARS) - 1) // 1000,
                             len(_BARS) - 1)] for v in r)
    pos = qtr.position(r)
    print("[%s]  pos=%s  dark=%d  branch=%s" % (
        bars,
        "----" if pos is None else "%+6.1fmm" % pos,
        qtr.dark_count(r),
        "DARK" if branch.dark() else "-"))
    time.sleep_ms(100)
