# SPDX-License-Identifier: MIT
"""
NativeDriveBase gyro square — the arc's final regression gate.

Drives the SAME two-pass square as the classic gyro example
(``st3032_drivebase_gyro_test.py``): once encoder-only, once with
``use_gyro(True)`` — but on the hard-tick native controller, through
the public ``NativeDriveBase`` API. Compare both drift numbers with
the classic example's on the same floor: matching or beating them is
what makes the native path the recommended ST-3032 default.

Bench hardware (edit if yours differs): ST-3032 left id=2 inverted /
right id=1 on UART1 tx=14 rx=6; BNO055 behind the TCA9548A on
channel 3 at 0x29; 88 mm wheels, 136 mm track.

Run:
    openbricks run -n ls examples/native_drivebase_gyro_square.py
"""

from machine import I2C, Pin

from openbricks.drivers.bno055 import BNO055
from openbricks.drivers.tca9548a import TCA9548A
from openbricks.robotics import NativeDriveBase

SIDE_MM = 150
STRAIGHT_SPEED_DPS = 200      # wheel-deg/s, classic-example parity
TURN_RATE_DPS      = 150


def drive_square(db, imu, label):
    start = imu.heading()
    print("[%s] starting heading: %.1f" % (label, start))
    for _ in range(4):
        db.straight(SIDE_MM)
        db.turn(90)
    end = imu.heading()
    delta = end - start
    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0
    print("[%s] ending heading:   %.1f  (drift from start: %+.1f deg)"
          % (label, end, delta))


def main():
    i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)
    imu = BNO055(i2c=TCA9548A(i2c)[3], address=0x29)

    db = NativeDriveBase(left_id=2, right_id=1, invert_left=True,
                         wheel_diameter_mm=88, axle_track_mm=136,
                         imu=imu)
    db.settings(straight_speed=STRAIGHT_SPEED_DPS,
                turn_rate=TURN_RATE_DPS)

    print("--- pass 1: encoder-only ---")
    drive_square(db, imu, "encoder")

    print("--- pass 2: gyro-corrected ---")
    db.use_gyro(True)
    drive_square(db, imu, "gyro")

    db.stop()
    print("--- done. Compare drift numbers with the classic "
          "st3032_drivebase_gyro_test.py on the same floor. ---")


main()
