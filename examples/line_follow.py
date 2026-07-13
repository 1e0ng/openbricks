# SPDX-License-Identifier: MIT
"""
Demo: follow a dark line using two colour sensors.

Mount the sensors at the front of the chassis, STRADDLING the line:
the line runs between them and each sensor looks at the mat. The
steering is PROPORTIONAL: the difference between the two ambient
readings is the error, and each wheel's speed is corrected by
``GAIN`` times that error — a gentle arc for a small drift, a pivot
for a big one, and no correction at all when centred. That's what
keeps it smooth on straights where a bang-bang follower fishtails.
Branch lines peeling off the main line are IGNORED: a branch only
darkens ONE sensor, and steering toward it would follow the branch —
so a single dark sensor means "hold course straight until it
passes". Only BOTH sensors dark at the same time (an intersection,
or a stop bar square across the path) ends the run.

Why proportional and not PID: the TCS34725 integrates light for
~24 ms per reading, so the loop sees the world at ~25 Hz with a full
sample of latency. At <=120 dps a P-controller is the right ceiling
for that bandwidth — a derivative term mostly amplifies sensor
noise, and an integral term adds windup risk for no visible gain.
If you crank the speed up, shorten the sensor integration time
first, then consider adding D.

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
    * ``GAIN`` — steering dps per unit of ambient difference. Too
      low drifts wide on curves; too high oscillates on straights.
      Bump it in steps of ~0.5 until it just starts to wiggle, then
      back off one step.
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
left_sensor = TCS34725(mux[1], gain=16)
right_sensor = TCS34725(mux[0], gain=16)


# --- control law (pure logic, unit-tested in tests/test_line_follow.py) ---

# Below this ambient (0..100) the surface counts as the line. Used
# for the INTERSECTION stop and for branch detection — the steering
# itself runs on the gradient above this threshold. Calibrate
# between a matte black line and your mat.
LINE_AMBIENT = 20

CRUISE_DPS = 200   # wheel speed when perfectly centred
# Proportional gain: extra dps of steering per unit of ambient
# difference between the sensors. Higher = snappier corrections but
# closer to oscillation; lower = lazier, drifts wide on curves.
GAIN = 0.3

# Never reverse; cap well above CRUISE_DPS + GAIN*100 so the clamp
# bounds runaway values without eating the steering differential.
MAX_DPS = 400

# Print both readings every tick — for calibrating LINE_AMBIENT /
# GAIN on a new mat. Leave off for real runs: each print streams
# over BLE and stretches the 10 ms control tick to many times that.
DEBUG = False


def _clamp(dps):
    return max(0, min(MAX_DPS, int(dps)))


def _wheel_speeds(left_ambient, right_ambient):
    """One control tick: ``(left_dps, right_dps)``, or ``None``
    meaning "intersection reached — stop".

    Three cases:
    * BOTH sensors dark — a line square across the path (an
      intersection / stop bar): stop.
    * ONE sensor dark — a branch line sweeping under that side.
      Steering toward it would peel off onto the branch, so it is
      IGNORED: hold course straight until it passes. The main line
      is still between the sensors the whole time.
    * Neither dark — normal proportional steering on the ambient
      difference: gentle arc for a small drift, stronger for a big
      one, zero correction when centred. Using the difference also
      self-cancels room-light changes hitting both sensors together.
    """
    left_dark = left_ambient < LINE_AMBIENT
    right_dark = right_ambient < LINE_AMBIENT
    if left_dark and right_dark:
        return None
    if left_dark or right_dark:
        return (CRUISE_DPS, CRUISE_DPS)
    if DEBUG:
        print('left', left_ambient, 'right', right_ambient)
    error = left_ambient - right_ambient
    return (_clamp(CRUISE_DPS + GAIN * error),
            _clamp(CRUISE_DPS - GAIN * error))

# --- end control law ---


def follow_line():
    # No print() inside the poll loop (the line_align lesson): each
    # one streams over the BLE console and stretches a 10 ms tick to
    # many times that, turning crisp corrections into wobble. The
    # bus is also only written when the decision CHANGES — re-sending
    # the same speeds every tick just steals ticks from sensing.
    poll_ms = 10
    timeout_ms = 30000

    last = None
    try:
        for _ in range(max(1, timeout_ms // poll_ms)):
            speeds = _wheel_speeds(
                left_sensor.ambient(),
                right_sensor.ambient(),
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
