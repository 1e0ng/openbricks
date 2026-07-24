# SPDX-License-Identifier: MIT
"""
ST-3032 drivebase with gyro heading feedback (fallback-path use_gyro).

Since 1.23.0, ``DriveBase.use_gyro(True)`` works on serial-bus
servos too: the pure-Python fallback loop reads the IMU instead of
the (encoder-less) servo bus for ``straight()``'s heading-hold
correction and ``turn()``'s completion check. This script drives a
square TWICE — once with the gyro off (encoder-only, the old
behaviour), once with it on — so you can compare how close each
pass returns to its starting heading.

Hardware (defaults below match the openbricks reference bench —
EDIT all of this to your own wiring/dims if it differs):
    * 2x ST-3032 (wheel mode), daisy-chained on one URT-2. Left id=1,
      right id=2 (inverted). ESP32-S3 GPIO14 -> URT-2 TX,
      GPIO6 -> URT-2 RX, common GND. 12V into the URT-2 servo rail.
    * BNO055 IMU behind a TCA9548A mux on I2C0 (sda=15, scl=16;
      itself at 0x70) — channel 3, address 0x29 (this particular
      breakout's ADR/COM3 pin defaults HIGH instead of the driver's
      0x28 default — check your own board if construction raises
      "BNO055 not found"). If you don't have another fixed-address
      device sharing the bus, the mux isn't needed — set
      ``IMU_MUX_CHANNEL = None`` below to talk to the IMU directly.

Run with:
    openbricks run -n ls examples/st3032_drivebase_gyro_test.py
"""

import machine

from openbricks.drivers.bno055 import BNO055
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.drivers.tca9548a import TCA9548A
from openbricks.robotics import DriveBase


LEFT_ID, RIGHT_ID = 1, 2
UART_ID, TX, RX   = 1, 14, 6

I2C_SDA, I2C_SCL  = 15, 16
IMU_MUX_CHANNEL   = 3      # EDIT — set to None to skip the mux entirely
IMU_ADDRESS       = 0x29   # EDIT — 0x28 is the driver default

WHEEL_DIAMETER_MM = 88     # EDIT to your wheels
AXLE_TRACK_MM     = 136    # EDIT to your chassis

# Capped conservatively for a bench test — bump once you've confirmed
# the chassis handles it cleanly.
STRAIGHT_SPEED_MM = 80
TURN_RATE_DPS     = 80

SIDE_MM = 150


def line(msg):
    print(msg)


def drive_square(db, imu, label):
    start = imu.heading()
    line("[%s] starting heading: %.1f" % (label, start))
    for side in range(4):
        db.straight(SIDE_MM)
        db.turn(90)
    end = imu.heading()
    delta = end - start
    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0
    line("[%s] ending heading:   %.1f  (drift from start: %+.1f deg)" %
         (label, end, delta))


def main():
    i2c = machine.I2C(0, sda=I2C_SDA, scl=I2C_SCL, freq=400_000)
    if IMU_MUX_CHANNEL is not None:
        i2c = TCA9548A(i2c)[IMU_MUX_CHANNEL]
    imu = BNO055(i2c=i2c, address=IMU_ADDRESS)

    left  = ST3032Motor(servo_id=LEFT_ID,  uart_id=UART_ID, tx=TX, rx=RX)
    right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX,
                        invert=True)

    db = DriveBase(left, right,
                   wheel_diameter_mm=WHEEL_DIAMETER_MM,
                   axle_track_mm=AXLE_TRACK_MM,
                   imu=imu)
    db.settings(straight_speed=STRAIGHT_SPEED_MM, turn_rate=TURN_RATE_DPS)

    line("--- pass 1: encoder-only (use_gyro off, the old behaviour) ---")
    drive_square(db, imu, "encoder")

    line("--- pass 2: gyro-corrected (use_gyro on) ---")
    db.use_gyro(True)
    drive_square(db, imu, "gyro")
    db.use_gyro(False)

    line("--- done. Compare the two drift numbers above. ---")


main()
