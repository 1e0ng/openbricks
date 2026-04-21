# SPDX-License-Identifier: MIT
"""
Example: a small robot that rolls forward until its colour sensor sees red,
then demos a servo wave.

Hardware:
    * ESP32 (or ESP32-S3) DevKitC
    * 2× JGB37-520 DC gear motors (with Hall-effect quadrature encoders)
        driven by a shared L298N (or TB6612FNG) H-bridge
    * 1× BNO055 9-DOF IMU on I2C
    * 1× TCS34725 RGB colour sensor on the same I2C bus
    * 1× ST-3215 serial bus servo on UART (optional — the servo demo
        at the bottom just prints a message if it isn't attached)

Edit the GPIOs at the top to match your wiring.
"""

import time

from machine import I2C, Pin, UART

from openbricks.drivers.bno055 import BNO055
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.drivers.st3215 import ST3215
from openbricks.drivers.tcs34725 import TCS34725
from openbricks.robotics import DriveBase

# ----- wiring -----

I2C_SDA  = 21   # ESP32 DevKitC-V4 defaults; use 15/16 on ESP32-S3 DevKitC-1
I2C_SCL  = 22

LEFT_IN1,  LEFT_IN2,  LEFT_PWM  = 25, 26, 27
LEFT_EA,   LEFT_EB               = 32, 33

RIGHT_IN1, RIGHT_IN2, RIGHT_PWM = 14, 12, 13
RIGHT_EA,  RIGHT_EB              = 34, 35

SERVO_TX, SERVO_RX = 17, 16
SERVO_ID = 1

WHEEL_DIAMETER_MM = 56
AXLE_TRACK_MM     = 114

# ----- init -----

i2c   = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400_000)
imu   = BNO055(i2c)
color = TCS34725(i2c)

left  = JGB37Motor(in1=LEFT_IN1,  in2=LEFT_IN2,  pwm=LEFT_PWM,
                   encoder_a=LEFT_EA,  encoder_b=LEFT_EB)
right = JGB37Motor(in1=RIGHT_IN1, in2=RIGHT_IN2, pwm=RIGHT_PWM,
                   encoder_a=RIGHT_EA, encoder_b=RIGHT_EB)

drivebase = DriveBase(left, right,
                      wheel_diameter_mm=WHEEL_DIAMETER_MM,
                      axle_track_mm=AXLE_TRACK_MM)

# Optional: a serial bus servo on a second UART.
try:
    uart = UART(1, baudrate=1_000_000, tx=SERVO_TX, rx=SERVO_RX)
    arm  = ST3215(uart, servo_id=SERVO_ID)
except Exception as e:
    print("no servo attached:", e)
    arm = None

# ----- main loop: drive until red -----

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

# Wave the arm once if present.
if arm is not None:
    arm.move_to(180, speed=500)
    time.sleep_ms(500)
    arm.move_to(0, speed=500)
