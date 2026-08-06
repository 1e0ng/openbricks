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
it must not yank the position. Here it only prints; hang your turn
decision off it.

Calibrate on every run: the first 3 seconds sweep the array across
the line (the program spins the chassis in place for you).
"""

import time

from openbricks.drivers.qtr import QTRArray, QTRChannel
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

# --- control law (pure logic, unit-tested in tests/test_qtr_line_follow.py) ---

CRUISE_DPS = 400   # wheel speed when perfectly centred

# PD gains on the position error in MILLIMETRES (the array reports
# the line centre directly). KP: dps of steering per mm of error.
# KD: per mm/s — raise KP until the robot wiggles on a straight,
# then raise KD until the wiggle dies. No KI: the centroid has no
# steady-state offset for it to trim (unlike the two-sensor
# ambient-difference error).
KP = 14.0
KD = 0.6

MAX_DPS = 800          # never reverse; see line_follow.py's note

# dark_count at or above this = full-width bar = intersection: stop.
# With a ~20 mm line on a 24 mm window (7 adjacent 4 mm channels),
# NORMAL following already darkens ~5 elements — only ALL of them
# dark discriminates a bar from the line itself.
INTERSECTION_COUNT = 7

# When the line escapes the array entirely, steer hard back toward
# the side it was last seen (mm, fed to the same PD as a synthetic
# error so recovery uses the ordinary steering path).
RECOVERY_ERROR_MM = 40.0

PD_STATE0 = None       # previous error, or None on the first tick


def _clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def _pd_wheel_speeds(position_mm, last_side, dark_count, state, dt_s):
    """One control tick: ``(decision, state)``.

    ``decision`` is ``None`` (intersection: stop) or
    ``(left_dps, right_dps)``. ``state`` carries the previous error;
    thread each returned state into the next call, starting from
    ``PD_STATE0``.

    ``position_mm`` is the array's centroid (+ = line right of
    centre) or ``None`` when the line is not under the array at all —
    then ``last_side`` (+1/-1) picks the recovery direction.
    """
    if dark_count >= INTERSECTION_COUNT:
        return None, state
    if position_mm is None:
        error = RECOVERY_ERROR_MM * (last_side if last_side else 1)
    else:
        error = position_mm
    derivative = 0.0
    if state is not None and dt_s > 0:
        derivative = (error - state) / dt_s
    steer = KP * error + KD * derivative
    # Line right of centre (+error) -> steer right: left wheel up,
    # right wheel down — same convention as line_follow.py.
    return (_clamp(CRUISE_DPS + steer),
            _clamp(CRUISE_DPS - steer)), error

# --- end control law ---


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

left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                         invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

# Calibration sweep: spin in place so the array crosses the line.
print("calibrating: spinning across the line for 3 s")
db.move_wheels(120, -120)
qtr.calibrate(duration_ms=3000)
branch.calibrate(duration_ms=3000)
db.stop()

print("following. Intersection stops the run.")
state = PD_STATE0
while True:
    readings = qtr.read()
    speeds, state = _pd_wheel_speeds(qtr.position(readings),
                                     qtr.last_side(),
                                     qtr.dark_count(readings),
                                     state, 0.010)
    if speeds is None:
        db.stop(then="brake")
        print("intersection - stopped")
        break
    if branch.dark():
        # Marker under the flag channel. Deliberately NOT steering
        # on it — put your route decision here.
        print("branch marker")
    db.move_wheels(speeds[0], speeds[1])
    time.sleep_ms(10)
