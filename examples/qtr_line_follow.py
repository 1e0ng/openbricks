# SPDX-License-Identifier: MIT
"""Line following on a QTR reflectance array — continuous-position PD.

The array gives the line's centre in millimetres (weighted centroid
across the elements), so the controller steers on a REAL analog
error instead of the two-colour-sensor version's edge-crossings
(``line_follow.py``).

Stop rule: the WHOLE array dark AND the branch flag dark in the
same snapshot — the full crossing is under the robot. That stops
the run immediately; either signal alone never does.

Hardware (bench layout): QTRX-HD-15A — line cluster = channels
15..9 (adjacent, 4 mm pitch) on ADC1 GPIO 1,2,3,7,8,9,10 left-to-
right, branch flag = channel 1 (far right) on GPIO 5, CTRL tied
high. See the pin-budget note at the wiring constants for why. Wheels = two
ST-3032 on UART1 tx=14 rx=6 (left id 2 inverted, right id 1), 88 mm
wheels on a 136 mm track.

The branch flag rides OUTSIDE the steering centroid: a marker under
it must not yank the position. It GATES the fork policy instead:
while the flag is dark a second line is under the flag's side, the
global centroid lands between the two lines, and the law steers on
the cluster on the OPPOSITE side — the fork to follow.

Deliberately minimal: no debounce, no lost-line recovery mode — if
nothing on the array is dark the last correction is simply held.

Calibration is loaded from the file saved by
``examples/qtr_calibrate.py`` — run that once after mounting or
changing mats; this program refuses to guess without it.
"""

import time

from openbricks.drivers.qtr import QTRArray, QTRChannel
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

CRUISE_DPS = 100   # wheel speed when perfectly centred — deliberately
                   # gentle for bench/tuning runs (raise once the
                   # gains are trusted)

# PD gains on the position error in MILLIMETRES (the array reports
# the line centre directly). KP: dps of steering per mm of error.
# KD: per mm/s — raise KP until the robot wiggles on a straight,
# then raise KD until the wiggle dies. No KI: the centroid has no
# steady-state offset for it to trim (unlike the two-sensor
# ambient-difference error).
KP = 14.0
KD = 0.6

MAX_DPS = 300      # never reverse: each wheel is floored at 0 and
                   # capped here, so nothing spins fast on a slow run.

# Which SIDE of the line cluster the branch flag is mounted on. The
# flag sits over the branch line, so the fork to follow is the
# cluster on the OPPOSITE side: flag right -> leftmost cluster,
# flag left -> rightmost. Rewire the flag to the array's other end
# and this is the only line that changes.
BRANCH_SIDE = "right"


def _clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def _pd_wheel_speeds(reading, branch_dark, prev_error, dt_s):
    """One control tick over a ``qtr.read()`` snapshot:
    ``(decision, error)``.

    ``decision`` is ``None`` (intersection: stop NOW) or
    ``(left_dps, right_dps)``. Feed the returned ``error`` back in
    as ``prev_error`` on the next call (start with ``None``).
    ``dt_s`` is the MEASURED time since the previous call — the
    derivative is mm per real second, so a slow tick must not read
    as a fast error change.

    The intersection is the whole array dark AND the branch flag
    dark in the same snapshot. While only the flag is dark, a
    second line is under the flag's side and the law steers on the
    cluster on the opposite side (``BRANCH_SIDE`` names where the
    flag is wired). If nothing on the array is dark, the previous
    correction is held — deliberately no recovery mode.
    """
    if branch_dark and reading.all_dark():
        return None, prev_error
    if branch_dark and BRANCH_SIDE == "right":
        error = reading.leftmost_position()
    elif branch_dark:
        error = reading.rightmost_position()
    else:
        error = reading.position()
    if error is None:
        # Nothing dark: hold the last correction (0 on the very
        # first tick — drive straight until the line appears).
        error = prev_error if prev_error is not None else 0.0
    derivative = 0.0
    if prev_error is not None and dt_s > 0:
        derivative = (error - prev_error) / dt_s
    steer = KP * error + KD * derivative
    # Line right of centre (+error) -> steer right: left wheel up,
    # right wheel down — same convention as line_follow.py.
    return (_clamp(CRUISE_DPS + steer),
            _clamp(CRUISE_DPS - steer)), error

# --- end control law ---


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
qtr.load_calibration(LINE_CAL)
branch.load_calibration(BRANCH_CAL)

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

print("following. Intersection stops the run.")
prev_error = None
last_ms = time.ticks_ms()
while True:
    reading = qtr.read()
    branch_dark = branch.dark()
    now_ms = time.ticks_ms()
    dt_s = time.ticks_diff(now_ms, last_ms) / 1000
    last_ms = now_ms
    speeds, prev_error = _pd_wheel_speeds(reading, branch_dark,
                                          prev_error, dt_s)
    if speeds is None:
        db.stop(then="brake")
        print("intersection - stopped")
        break
    if branch_dark:
        # A second line under the flag: the law is steering on the
        # opposite-side fork right now (see _pd_wheel_speeds).
        print("branch marker")
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(10)
