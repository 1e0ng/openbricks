# SPDX-License-Identifier: MIT
"""
Example: a small robot that rolls forward until its colour sensor sees red,
then demos a servo wave.

Hardware — the reference build from docs/hardware.md, wired to its
GPIO map so nothing collides with the QTR bank (GPIO 1-10):
    * ESP32-S3 DevKitC-1
    * 2x ST-3032 serial bus servos (wheel mode, IDs 1 and 2) on one
        URT-2: GPIO 14 -> URT-2 RX, GPIO 41 -> URT-2 TX
    * 1x ICM-45686 IMU on SPI: SCK 12, MOSI 13, MISO 11, CS 17
    * 1x TCS34725 RGB colour sensor on I2C: SDA 15, SCL 16
    * 1x ST-3215 serial bus servo (ID 3) daisy-chained on the SAME
        bus as the wheels (optional — the servo demo at the bottom
        just prints a message if it isn't attached)
"""

import time

from machine import I2C, Pin

from openbricks.drivers.icm45686 import ICM45686
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.drivers.st3215 import ST3215
from openbricks.drivers.tcs34725 import TCS34725
from openbricks.robotics import DriveBase


I2C_SDA, I2C_SCL = 15, 16
SCK, MOSI, MISO, CS = 12, 13, 11, 17
UART_ID, TX, RX = 1, 14, 41
LEFT_ID, RIGHT_ID, ARM_ID = 1, 2, 3

WHEEL_DIAMETER_MM = 88
AXLE_TRACK_MM     = 138


i2c   = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400_000)
imu   = ICM45686(sck=SCK, mosi=MOSI, miso=MISO, cs=CS)
color = TCS34725(i2c)

left  = ST3032Motor(servo_id=LEFT_ID,  uart_id=UART_ID, tx=TX, rx=RX,
                    invert=True)
right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX)

drivebase = DriveBase(left, right,
                      wheel_diameter_mm=WHEEL_DIAMETER_MM,
                      axle_track_mm=AXLE_TRACK_MM)

try:
    arm = ST3215(servo_id=ARM_ID, uart_id=UART_ID, tx=TX, rx=RX)
    if not arm.ping():
        raise OSError("servo %d did not answer ping" % ARM_ID)
except Exception as e:
    print("no arm servo attached:", e)
    arm = None


while True:
    r, g, b = color.rgb()
    heading = imu.heading()

    print("rgb=({:3d},{:3d},{:3d})  heading={:6.1f}".format(r, g, b, heading))

    if r > g and r > b and r > 120:
        print("Red detected — stopping.")
        drivebase.stop()
        break

    drivebase.drive(speed_mm_s=150, turn_rate_dps=0)
    time.sleep_ms(50)

if arm is not None:
    arm.move_to(180, speed=500)
    time.sleep_ms(500)
    arm.move_to(0, speed=500)
