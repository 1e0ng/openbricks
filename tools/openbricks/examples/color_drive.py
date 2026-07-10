# SPDX-License-Identifier: MIT
"""
Drive forward and stop on a coloured zone — using the firmware
``TCS34725`` driver unmodified.

Run with::

    openbricks-sim run examples/color_drive.py \\
        --world wro-2026-elementary --viewer

The driver shim replaces ``openbricks.drivers.tcs34725.TCS34725``
with a sim-aware class that raycasts down from the chassis camera
and returns the floor's material colour. From the script's point
of view, the same ``rgb()`` / ``ambient()`` API is in use.

Drive forward in 5 mm increments (open-loop wheel speed); poll the
sensor each chunk; stop when ``ambient()`` drops below 30 — typical
threshold for "we're over a dark zone".
"""

from openbricks.drivers.tcs34725 import TCS34725
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics.drivebase import DriveBase
from machine import I2C


i2c    = I2C(0, sda=15, scl=16, freq=400_000)   # ESP32-S3; 21/22 on classic ESP32
sensor = TCS34725(i2c=i2c, address=0x29)

m_left  = JGB37Motor(in1=1, in2=2, pwm=17,
                     encoder_a=7, encoder_b=8)
m_right = JGB37Motor(in1=9, in2=10, pwm=11,
                     encoder_a=12, encoder_b=13)
db = DriveBase(m_left, m_right,
               wheel_diameter_mm=60,
               axle_track_mm=150)
db.settings(straight_speed=120)

print("starting RGB / ambient:", sensor.rgb(), sensor.ambient())

# Up to 30 hops × 5 mm = 150 mm forward, stopping if we hit a dark
# zone. On the empty / standalone world there's nothing dark to find,
# so the loop runs to completion; on the WRO mat it stops when the
# camera passes over a coloured mission area.
for hop in range(30):
    db.straight(5)
    r, g, b = sensor.rgb()
    a = sensor.ambient()
    print(f"  hop {hop}: rgb=({r}, {g}, {b}) ambient={a}")
    if a < 30:
        print("  dark zone detected — stopping.")
        break

print("ending pose:", robot.chassis_pose())  # noqa: F821
