# SPDX-License-Identifier: MIT
"""
ST-3032 drivebase with gyro heading feedback.

``DriveBase`` adopts serial-bus motors onto the hard-tick native
controller transparently (1.45.0): the 2-DOF loop runs in C at
~220 Hz per wheel, immune to Python stalls, and ``use_gyro(True)``
feeds the IMU heading into its absolute heading-hold frame. This
script drives a square TWICE — once with the gyro off (encoder-only),
once with it on — so you can compare how close each pass returns to
its starting heading (bench 2026-08-02: encoder-only ±1.6°, gyro
+0.5..+1.8° over a two-pass square).

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


LEFT_ID, RIGHT_ID = 2, 1
UART_ID, TX, RX   = 1, 14, 41

I2C_SDA, I2C_SCL  = 15, 16
IMU_MUX_CHANNEL   = 3
IMU_ADDRESS       = 0x29

WHEEL_DIAMETER_MM = 88
AXLE_TRACK_MM     = 138

STRAIGHT_SPEED_DPS = 200
TURN_RATE_DPS      = 150

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

    left  = ST3032Motor(servo_id=LEFT_ID,  uart_id=UART_ID, tx=TX, rx=RX, invert=True)
    right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX)

    db = DriveBase(left, right,
                   wheel_diameter_mm=WHEEL_DIAMETER_MM,
                   axle_track_mm=AXLE_TRACK_MM,
                   imu=imu)
    db.settings(straight_speed=STRAIGHT_SPEED_DPS, turn_rate=TURN_RATE_DPS)

    line("--- pass 1: encoder-only (use_gyro off, the old behaviour) ---")
    drive_square(db, imu, "encoder")

    line("--- pass 2: gyro-corrected (use_gyro on) ---")
    db.use_gyro(True)
    drive_square(db, imu, "gyro")
    db.use_gyro(False)

    line("--- done. Compare the two drift numbers above. ---")


main()
