# SPDX-License-Identifier: MIT
"""
Demo: follow a dark line using two colour sensors.

Mount the sensors at the front of the chassis, STRADDLING the line:
the line runs between them and each sensor looks at the mat. While
both see the mat the robot drives straight. When the robot drifts,
one sensor crosses onto the line — that side slows down, steering
the robot back until the line is centred between the sensors again.
A crossing where BOTH sensors go dark at the same time (an
intersection, or a stop bar drawn across the line) ends the run.

This is the mirror image of ``line_align.py``: align drives *at* a
line with the sensors expecting to hit it; follow drives *along* a
line with the sensors expecting to miss it.

Line detection is the same trick as ``line_align.py``: a dark line
on a light mat is "too dark to be the mat" — ``ambient()`` below a
threshold. Print ``sensor.ambient()`` over your own mat and line and
put ``LINE_AMBIENT`` between the two readings.

Tuning:
    * ``CRUISE_DPS`` — speed while centred. Higher = faster but the
      robot overshoots corrections; start slow.
    * ``TURN_DPS`` — inner-wheel speed during a correction. The
      bigger the gap to ``CRUISE_DPS``, the sharper the correction;
      too sharp and the robot fishtails on a straight line.
    * Sensor spacing — wider than the line, but not much wider:
      the gap between the sensors is the dead band where drift goes
      uncorrected.

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


left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6, invert=True)
# One SYNC WRITE updates both wheels at the same packet boundary,
# instead of two serialised bus writes.
wheels = SyncServoGroup([left_motor, right_motor])

i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)
mux = TCA9548A(i2c)
left_sensor = TCS34725(mux[1])
right_sensor = TCS34725(mux[0])


# --- control law (pure logic, unit-tested in tests/test_line_follow.py) ---

# Below this ambient (0..100) the surface counts as the line. Between
# a typical mat (~30+) and a matte black line (~5-10); calibrate on
# your own surfaces.
LINE_AMBIENT = 5

CRUISE_DPS = 100   # both sensors on the mat: full speed ahead
TURN_DPS   = 25    # the wheel on the line's side slows to this


def _wheel_speeds(left_on_line, right_on_line):
    """One control tick: ``(left_dps, right_dps)`` for the wheels, or
    ``None`` meaning "intersection reached — stop".

    The line drifting under the LEFT sensor means the robot slid
    right of centre: slow the left wheel so the robot arcs left,
    back over the line. Mirror image for the right. Both dark at
    once is a line crossing the path — the stop condition.
    """
    if left_on_line and right_on_line:
        return None
    if left_on_line:
        return (TURN_DPS, CRUISE_DPS)
    if right_on_line:
        return (CRUISE_DPS, TURN_DPS)
    return (CRUISE_DPS, CRUISE_DPS)

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
                left_sensor.ambient() < LINE_AMBIENT,
                right_sensor.ambient() < LINE_AMBIENT,
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
