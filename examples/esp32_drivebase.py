# SPDX-License-Identifier: MIT
"""
Example: drive forward 500 mm, turn 90°, repeat.

Hardware:
    * ESP32-S3 (or classic ESP32)
    * 2x JGB37-520 motors with quadrature encoders
    * 1x L298N dual H-bridge (powers both motors)
    * Two 18650 cells or similar 7.4V battery on the L298N motor supply

Wiring (edit the GPIOs to match your board; defaults are for the
ESP32-S3 DevKitC-1 — avoid GPIO 19/20 = USB, 26-37 = flash/PSRAM,
0/3/45/46 = strapping, 4/5 = the firmware's default program / BLE
buttons, 15/16 = the I2C convention, 14/6 = the serial-bus UART
convention. The launcher polls GPIO 4 as an input, so a motor driver
toggling it would read as button presses and stop your program.
On a classic ESP32, remap to that board's pins):

    Left motor  (L298N channel A)
        IN1  -> GPIO 1
        IN2  -> GPIO 2
        ENA  -> GPIO 17   (PWM)
        ENC A-> GPIO 7
        ENC B-> GPIO 8

    Right motor (L298N channel B)
        IN3  -> GPIO 9
        IN4  -> GPIO 10
        ENB  -> GPIO 11   (PWM)
        ENC A-> GPIO 12
        ENC B-> GPIO 13
"""

from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics import DriveBase
from openbricks.tools import wait


left = JGB37Motor(
    in1=1, in2=2, pwm=17,
    encoder_a=7, encoder_b=8,
    counts_per_output_rev=1320,  # 11 CPR * 30:1 gearbox * 4 (quadrature)
)
right = JGB37Motor(
    in1=9, in2=10, pwm=11,
    encoder_a=12, encoder_b=13,
    counts_per_output_rev=1320,
    invert=True,  # mirror image of the left motor
)

db = DriveBase(left, right, wheel_diameter_mm=65, axle_track_mm=120)
db.settings(straight_speed=200, turn_rate=120)

for _ in range(4):
    db.straight(500)
    wait(200)
    db.turn(90)
    wait(200)
