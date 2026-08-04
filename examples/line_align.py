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
from openbricks.robotics import DriveBase


left_motor = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6)
right_motor = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6, invert=True)
# ``db.move_wheels(left, right)`` puts both setpoints in ONE
# sync-write packet, so the wheels change together. It replaces the
# SyncServoGroup this example used to build: adopting wheels into a
# DriveBase hands their UART to the native driver, and a
# SyncServoGroup writes through the MicroPython one — two drivers on
# a single wire. The chassis dimensions matter only to
# straight()/turn(); this routine steers by wheel speeds.
db = DriveBase(left_motor, right_motor,
               wheel_diameter_mm=88, axle_track_mm=136)

i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)
mux = TCA9548A(i2c)
left_sensor = TCS34725(mux[1])
right_sensor = TCS34725(mux[0])


# Below this ambient (0..100) the surface counts as the line. Between
# a typical mat (~30+) and a matte black line (~5-10); calibrate on
# your own surfaces.
# Retuned 5 -> 20 when the driver defaults moved to gain=16 /
# integration_ms=2.4: normalized ambient() reads ~4x higher
# (gain x4, full scale /10). 20 matches line_follow.py's
# threshold under the same configuration.
LINE_AMBIENT = 20


def align_on_line():
    # Drive forward until each sensor sees the line; brake that side.
    #
    # No print() inside the poll loop: each one streams over the BLE
    # console and stretches a 10 ms tick to many times that, so the
    # wheel brakes long after its sensor crossed the line. Readings
    # are collected and reported after each wheel stops instead.

    approach_dps = 100
    poll_ms = 10
    timeout_ms = 8000

    left_done = False
    right_done = False
    left_ambient = None
    right_ambient = None
    # Per-wheel speeds, updated as each side arrives. Zeroing one
    # side stops that wheel while the other keeps creeping — same
    # independent-stop behaviour as the old per-motor brake(), but
    # every change still leaves in a single packet.
    speeds = [approach_dps, approach_dps]
    db.move_wheels(speeds[0], speeds[1])
    try:
        for _ in range(max(1, timeout_ms // poll_ms)):
            if not left_done:
                left_ambient = left_sensor.ambient()
                if left_ambient < LINE_AMBIENT:
                    speeds[0] = 0
                    db.move_wheels(speeds[0], speeds[1])
                    left_done = True
                    print('left wheel stopped (ambient=%d)' % left_ambient)
            if not right_done:
                right_ambient = right_sensor.ambient()
                if right_ambient < LINE_AMBIENT:
                    speeds[1] = 0
                    db.move_wheels(speeds[0], speeds[1])
                    right_done = True
                    print('right wheel stopped (ambient=%d)' % right_ambient)

            if left_done and right_done:
                return
            wait(poll_ms)
    finally:
        # Whatever ends the loop — success, timeout, Ctrl-C — no
        # wheel keeps creeping. One call, both wheels.
        db.stop(then="brake")
    raise RuntimeError(
        "no line found within %d ms (last ambient: left=%r right=%r) — "
        "is the line in reach, and is LINE_AMBIENT calibrated for "
        "your mat?" % (timeout_ms, left_ambient, right_ambient))


def main():

    print("aligning on the line ...")
    align_on_line()
    print("aligned — square to the line.")
    wait(500)
    left_motor.coast()
    right_motor.coast()


if __name__ == "__main__":
    main()
