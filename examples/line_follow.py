# SPDX-License-Identifier: MIT
"""
Demo: follow a dark line using two colour sensors.

Mount the sensors at the front of the chassis, STRADDLING the line:
the line runs between them and each sensor looks at the mat. The
steering is PID on the difference between the two ambient readings:
proportional for the correction itself (gentle arc for a small
drift, pivot for a big one), derivative to damp the correction so
higher gains don't oscillate, and an optional integral for drift on
long arcs. That's what keeps it smooth on straights where a
bang-bang follower fishtails.
Branch lines peeling off the main line are IGNORED: a branch only
darkens ONE sensor, and steering toward it would follow the branch —
so a single dark sensor means "hold course straight until it
passes". Only BOTH sensors dark at the same time (an intersection,
or a stop bar square across the path) ends the run.

The controller is a full PID, and it's honest PID because the
sensors run at their 2.4 ms minimum integration time (the driver's
default is 24 ms — ten times the latency, at which a derivative
term mostly amplifies stale-sample noise). D damps the wiggle so KP
can be raised for sharper tracking; I (default 0, windup-clamped)
trims persistent drift on long constant-curvature arcs.

This is the mirror image of ``line_align.py``: align drives *at* a
line with the sensors expecting to hit it; follow drives *along* a
line with the sensors expecting to miss it.

Line detection is the same trick as ``line_align.py``: a dark line
on a light mat is "too dark to be the mat" — ``ambient()`` below a
threshold. Print ``sensor.ambient()`` over your own mat and line and
put ``LINE_AMBIENT`` between the two readings.

Tuning:
    * ``CRUISE_DPS`` — speed while centred. Higher = faster but
      corrections arrive a sensor-latency late; start slow.
    * ``KP`` — steering dps per unit of ambient difference. Raise
      until the robot just wiggles on a straight, then raise ``KD``
      until the wiggle dies; repeat for sharper tracking. ``KI``
      stays 0 unless long arcs show one-sided drift.
    * Sensor spacing — wider than the line, but not much wider: with
      proportional control the sensors' inner edges do the work, so
      closer spacing means earlier, gentler corrections.

Hardware (same bus layout as ``line_align.py`` / ``color_array.py``):
    * ESP32-S3 (I2C on 15/16; serial bus UART on 14/6)
    * 2x ST-3032 wheel servos, IDs 1 (left) / 2 (right)
    * TCA9548A mux, one TCS34725 per channel: 0 = left, 1 = right,
      both facing the mat at the front of the chassis
"""

from machine import I2C, Pin
from openbricks.tools import wait

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.drivers.st3215 import SyncServoGroup
from openbricks.drivers.tca9548a import TCA9548A
from openbricks.drivers.tcs34725 import TCS34725


left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
# One SYNC WRITE updates both wheels at the same packet boundary,
# instead of two serialised bus writes.
wheels = SyncServoGroup([left_motor, right_motor])

i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)
mux = TCA9548A(i2c)
# integration_ms=2.4 (one cycle, the chip minimum) drops the sensor
# latency from the 24 ms default to 2.4 ms — the enabler for the D
# term below. The ambient() PERCENT scale is unchanged (full-scale
# rescales with integration), so LINE_AMBIENT keeps its meaning;
# each reading just averages 10x less light, so expect ~1 count of
# extra noise. gain=16 keeps the signal budget healthy.
left_sensor = TCS34725(mux[1], gain=16, integration_ms=2.4)
right_sensor = TCS34725(mux[0], gain=16, integration_ms=2.4)


# --- control law (pure logic, unit-tested in tests/test_line_follow.py) ---

# Below this ambient (0..100) the surface counts as the line. Used
# for the INTERSECTION stop and for branch detection — the steering
# itself runs on the gradient above this threshold.
LINE_AMBIENT = 12

CRUISE_DPS = 200   # wheel speed when perfectly centred

# PID gains on the ambient-difference error (units: dps of steering
# per ambient-unit; KI per ambient-unit-second; KD per
# ambient-unit/second). PID is sound here BECAUSE the sensors run at
# 2.4 ms integration — on the old 24 ms default the derivative
# mostly amplified stale-sample noise.
#   KP: the workhorse. Same value as the P-only version.
#   KD: damping — lets you raise KP without oscillating. Tune by
#       raising KP until the robot wiggles on a straight, then
#       raising KD until the wiggle dies.
#   KI: trims steady-state drift on long constant-curvature arcs.
#       Leave 0 unless you see persistent one-sided offset; wind-up
#       is clamped either way.
KP = 0.3
KI = 0.0
KD = 0.02
INTEGRAL_LIMIT = 50.0   # anti-windup: |integral| cap, ambient-units*s

# Never reverse; cap well above CRUISE_DPS + max steering so the
# clamp bounds runaway values without eating the differential.
MAX_DPS = 400

# Print both readings every tick — for calibrating on a new mat.
# Leave off for real runs: each print streams over BLE and stretches
# the control tick.
DEBUG = False

# Initial PID state: (integral, previous-error-or-None). Thread the
# state returned by each _pid_wheel_speeds call into the next.
PID_STATE0 = (0.0, None)


def _clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def _pid_wheel_speeds(left_ambient, right_ambient, state, dt_s):
    """One control tick: ``(decision, state)``.

    ``decision`` is ``None`` (both sensors dark: intersection — stop)
    or ``(left_dps, right_dps)``. ``state`` carries the integral and
    the previous error; pass each call's returned state into the
    next call, starting from ``PID_STATE0``.

    A single dark sensor is a BRANCH line sweeping under that side:
    steering toward it would peel off onto the branch, so hold
    course straight until it passes. The previous-error history is
    reset for that tick so the derivative doesn't kick when normal
    steering resumes (the line may have shifted during the blind
    window).
    """
    integral, prev_error = state
    left_dark = left_ambient < LINE_AMBIENT
    right_dark = right_ambient < LINE_AMBIENT
    if left_dark and right_dark:
        return None, state
    if left_dark or right_dark:
        return (CRUISE_DPS, CRUISE_DPS), (integral, None)
    if DEBUG:
        print('left', left_ambient, 'right', right_ambient)

    error = left_ambient - right_ambient
    if dt_s > 0:
        integral += error * dt_s
        if integral > INTEGRAL_LIMIT:
            integral = INTEGRAL_LIMIT
        elif integral < -INTEGRAL_LIMIT:
            integral = -INTEGRAL_LIMIT
        derivative = 0.0 if prev_error is None else (error - prev_error) / dt_s
    else:
        derivative = 0.0
    steer = KP * error + KI * integral + KD * derivative
    return ((_clamp(CRUISE_DPS + steer), _clamp(CRUISE_DPS - steer)),
            (integral, error))

# --- end control law ---


def follow_line():
    # No print() inside the poll loop (the line_align lesson): each
    # one streams over the BLE console and stretches a 10 ms tick to
    # many times that, turning crisp corrections into wobble. The
    # bus is also only written when the decision CHANGES — re-sending
    # the same speeds every tick just steals ticks from sensing.
    import time
    poll_ms = 10
    timeout_ms = 30000

    last = None
    state = PID_STATE0
    prev_ms = time.ticks_ms()
    try:
        for _ in range(max(1, timeout_ms // poll_ms)):
            now_ms = time.ticks_ms()
            dt_s = time.ticks_diff(now_ms, prev_ms) / 1000.0
            prev_ms = now_ms
            speeds, state = _pid_wheel_speeds(
                left_sensor.ambient(),
                right_sensor.ambient(),
                state, dt_s,
            )
            if speeds is None:
                print("intersection reached — stopping.")
                return
            if speeds != last:
                wheels.set_goal_speeds(list(speeds))
                last = speeds
            wait(poll_ms)
    finally:
        # Whatever ends the loop — intersection, timeout, Ctrl-C, the
        # stop button — no wheel keeps creeping.
        left_motor.brake()
        right_motor.brake()
    raise RuntimeError(
        "no intersection within %d ms — lost the line, or is "
        "LINE_AMBIENT calibrated for your mat?" % timeout_ms)


def main():
    print("following the line until an intersection ...")
    follow_line()
    wait(500)
    left_motor.coast()
    right_motor.coast()


if __name__ == "__main__":
    main()
