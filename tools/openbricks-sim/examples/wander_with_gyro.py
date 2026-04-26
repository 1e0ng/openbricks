# SPDX-License-Identifier: MIT
"""
Slip-immune wander — same firmware-style code as
``wander_hardware_style.py`` but with ``DriveBase.use_gyro(True)``
turned on so the heading feedback comes from the IMU instead of
the encoder differential.

Run with::

    openbricks-sim run examples/wander_with_gyro.py \\
        --world wro-2026-elementary --viewer

The point: on hardware, gyro feedback bypasses wheel slip — a
robot driving across a slippery patch with a Kp-only encoder loop
will veer, but the gyro path snaps back. The same mechanism runs
inside the sim against the chassis IMU site.
"""

from openbricks.drivers.bno055 import BNO055
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics.drivebase import DriveBase
from machine import I2C


# Hardware imports + constructor args are no-ops under the shim;
# the I2C handle the user code instantiates is just a stub the
# BNO055 stores. The shim's BNO055 binds straight to the chassis
# IMU site regardless.
i2c = I2C(0, scl=22, sda=21, freq=400_000)
imu = BNO055(i2c=i2c, address=0x28)

m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                     encoder_a=18, encoder_b=19,
                     counts_per_output_rev=1320)
m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                     encoder_a=20, encoder_b=21,
                     counts_per_output_rev=1320)

db = DriveBase(m_left, m_right,
               wheel_diameter_mm=60,
               axle_track_mm=150,
               imu=imu)
db.settings(straight_speed=180, turn_rate=120)
db.use_gyro(True)   # heading feedback now comes from the IMU

print("starting heading:", imu.heading())  # noqa: F821

for _ in range(4):
    db.straight(150)
    db.turn(90)

print("ending heading:", imu.heading())   # noqa: F821
print("chassis pose:  ", robot.chassis_pose())  # noqa: F821
