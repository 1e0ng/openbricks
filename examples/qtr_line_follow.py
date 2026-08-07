# SPDX-License-Identifier: MIT
"""Line following on a QTR reflectance array — continuous-position PD.

The array gives the line's centre in millimetres (weighted centroid
across the elements), so the controller steers on a REAL analog
error instead of the two-colour-sensor version's edge-crossings
(``line_follow.py``). That is what allows higher cruise speeds: the
error is proportional all the way across the array's span, and
losing the line entirely still leaves a recovery direction
(``last_side``).

Stop logic: a full-width dark bar (an intersection / stop bar)
darkens most of the array at once — ``dark_count`` >= the threshold.
A branch stub darkens only one end and barely moves the centroid, so
it is steered through, not flinched at.

Hardware (bench layout): QTRX-HD-15A — line cluster = channels
15..9 (adjacent, 4 mm pitch) on ADC1 GPIO 1,2,3,7,8,9,10 left-to-
right, branch flag = channel 1 (far right) on GPIO 5, CTRL tied
high. See the pin-budget note at the wiring constants for why. Wheels = two
ST-3032 on UART1 tx=14 rx=6 (left id 2 inverted, right id 1), 88 mm
wheels on a 136 mm track.

The branch flag rides OUTSIDE the steering centroid: a marker under
it must not yank the position. It GATES the fork policy instead:
while the flag is dark a second line is under the right side, the
global centroid lands between the two lines, and the law steers on
the LEFTMOST cluster's own centre — the left fork. Both positions
are computed every tick, so the switch is jump-free.

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

MAX_DPS = 300          # never reverse. Cap scaled with the gentle
                       # cruise: it bounds steering AND the recovery
                       # pivot, so nothing spins fast on a slow run.

# An INTERSECTION is the WHOLE array dark AND the branch flag dark
# at the same time — the full crossing is under the robot. The
# condition must hold for INTERSECTION_TICKS CONSECUTIVE polls:
# bench 2026-08-07 recorded all 7 elements crossing the threshold
# for a single tick during a plain line crossing ([:*#%%#-]
# dark=7) — a real intersection stays under the array for many
# polls (~20 mm of travel), a transient for one or two. A branch on
# ONE side (flag alone, or a wide array reading alone) never stops:
# it routes through the fork policy below.
INTERSECTION_TICKS = 3

# A real line is a strong LOCALIZED peak; lifted off the mat (or
# over the mat edge) every element floats at a uniform ~250-330 and
# some cross the dark threshold — bench 2026-08-07 recorded
# positions steered on that mush ([:::::: ] pos=-1.8 dark=5). A
# position only counts when the brightest element beats this
# (calibrated units); otherwise it is treated as line-lost and the
# recovery path runs.
PEAK_MIN = 550

# When the line escapes the array entirely, steer hard back toward
# the side it was last seen (mm, fed to the same PD as a synthetic
# error so recovery uses the ordinary steering path).
RECOVERY_ERROR_MM = 40.0

# (previous error or None, consecutive intersection-count ticks)
PD_STATE0 = (None, 0)


def _clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def _pd_wheel_speeds(position_mm, fork_mm, peak, branch_dark,
                     all_dark, last_side, state, dt_s):
    """One control tick: ``(decision, state)``.

    ``decision`` is ``None`` (intersection: stop) or
    ``(left_dps, right_dps)``. ``state`` is ``(previous_error,
    intersection_streak)``; thread each returned state into the next
    call, starting from ``PD_STATE0``.

    ``position_mm`` is the global centroid; ``fork_mm`` is the
    cluster to steer on while ``branch_dark`` — the caller picks
    which end (see BRANCH_SIDE), the law only knows that during a
    branch the global centroid points between the two lines and the
    chosen cluster is the real one. Both + = right of centre,
    ``None`` when nothing is dark. ``all_dark`` is the whole array
    over the line: together with the flag dark, the full crossing
    is under the robot — the intersection. ``peak`` is the
    brightest calibrated reading — a weak peak is off-mat mush and
    falls to the ``last_side`` (+1/-1) recovery steer.
    """
    prev_error, streak = state
    if branch_dark and all_dark:
        streak += 1
        if streak >= INTERSECTION_TICKS:
            return None, (prev_error, streak)
    else:
        streak = 0
    if branch_dark and fork_mm is not None:
        steer_on = fork_mm
    else:
        steer_on = position_mm
    if steer_on is None or peak < PEAK_MIN:
        error = RECOVERY_ERROR_MM * (last_side if last_side else 1)
    else:
        error = steer_on
    derivative = 0.0
    if prev_error is not None and dt_s > 0:
        derivative = (error - prev_error) / dt_s
    steer = KP * error + KD * derivative
    # Line right of centre (+error) -> steer right: left wheel up,
    # right wheel down — same convention as line_follow.py.
    return (_clamp(CRUISE_DPS + steer),
            _clamp(CRUISE_DPS - steer)), (error, streak)

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
# Which SIDE of the line cluster the branch flag is mounted on. The
# flag sits over the branch line, so the fork to follow is the
# cluster on the OPPOSITE side: flag right -> leftmost cluster,
# flag left -> rightmost. Rewire the flag to the array's other end
# and this is the only line that changes.
BRANCH_SIDE = "right"

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
state = PD_STATE0
while True:
    reading = qtr.read()
    branch_dark = branch.dark()
    if BRANCH_SIDE == "right":
        fork_pos = reading.leftmost_position()
    else:
        fork_pos = reading.rightmost_position()
    speeds, state = _pd_wheel_speeds(reading.position(),
                                     fork_pos,
                                     reading.max(),
                                     branch_dark,
                                     reading.all_dark(),
                                     qtr.last_side(),
                                     state, 0.010)
    if speeds is None:
        db.stop(then="brake")
        print("intersection - stopped")
        break
    if branch_dark:
        # A second line under the flag: the law is steering on the
        # LEFT fork right now (see _pd_wheel_speeds).
        print("branch marker")
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(10)
