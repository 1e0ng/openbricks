# SPDX-License-Identifier: MIT
"""
Demo: square the robot up on a dark line using two colour sensors.

The classic FLL/WRO "align on a line" move: drive slowly toward a
black line with one colour sensor mounted near each front corner,
ahead of the wheels. The moment a sensor crosses onto the line its
wheel brakes — the other wheel keeps rolling, pivoting the chassis
until *its* sensor reaches the line too. When both have stopped, the
sensor pair (and therefore the chassis) is parallel to the line, no
matter how crooked the approach was.

Geometry matters: the sensors must sit ahead of the axle and be
mounted symmetrically. The farther apart they are, the more accurate
the final heading.

Line detection reuses the idea from ``color_array.py``: a black line
on a light mat is simply "too dark to be the mat" — ``ambient()``
below a threshold. Print ``sensor.ambient()`` over your own mat and
line and put ``LINE_AMBIENT`` between the two readings.

Hardware (same bus layout as ``color_array.py`` / ``full_robot.py``):
    * ESP32-S3 (I2C on 15/16; serial bus UART on 14/6)
    * 2x ST-3032 wheel servos, IDs 1 (left) / 2 (right)
    * TCA9548A mux, one TCS34725 per channel: 0 = left, 1 = right,
      both facing the mat at the front of the chassis
"""

from machine import I2C, Pin
from openbricks.tools import wait

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.drivers.tca9548a import TCA9548A
from openbricks.drivers.tcs34725 import TCS34725


left_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
right_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)

i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)
mux = TCA9548A(i2c)
left_sensor = TCS34725(mux[0])
right_sensor = TCS34725(mux[1])


# Below this ambient (0..100) the surface counts as the line. Between
# a typical mat (~30+) and a matte black line (~5-10); calibrate on
# your own surfaces.
LINE_AMBIENT = 15


def align_on_line():
    # Drive forward until each sensor sees the line; brake that side.

    approach_dps=100
    poll_ms=10
    timeout_ms=8000

    left_done = False
    right_done = False
    left_motor.run_speed(approach_dps)
    right_motor.run_speed(approach_dps)
    try:
        for _ in range(max(1, timeout_ms // poll_ms)):
            if not left_done:
                left_ambient = left_sensor.ambient()
                print('left ambient:', left_ambient)
                print('left rgb:', left_sensor.rgb())

                if left_ambient < LINE_AMBIENT:
                    left_motor.brake()
                    left_done = True
            if not right_done:
                right_ambient = right_sensor.ambient()
                print('right ambient:', right_ambient)

                if right_ambient < LINE_AMBIENT:
                    right_motor.brake()
                    right_done = True

            if left_done and right_done:
                return
            wait(poll_ms)
    finally:
        # Whatever ends the loop — success, timeout, Ctrl-C — no
        # wheel keeps creeping.
        if not left_done:
            left_motor.brake()
        if not right_done:
            right_motor.brake()
    raise RuntimeError(
        "no line found within %d ms — is the line in reach, and is "
        "LINE_AMBIENT calibrated for your mat?" % timeout_ms)


def main():

    print("aligning on the line ...")
    align_on_line()
    print("aligned — square to the line.")
    wait(500)
    left_motor.coast()
    right_motor.coast()


if __name__ == "__main__":
    main()
