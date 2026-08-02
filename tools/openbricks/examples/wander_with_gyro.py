# SPDX-License-Identifier: MIT
"""
Slip-immune wander — same "drive a square" shape as
``wander_hardware_style.py`` (also firmware-style code, no shim
awareness) but with ``DriveBase.use_gyro(True)`` turned on so the
heading feedback comes from the IMU instead of the encoder
differential.

Run with::

    openbricks-sim run examples/wander_with_gyro.py \\
        --world wro-2026-elementary --viewer

The point: on hardware, gyro feedback bypasses wheel slip — a
robot driving across a slippery patch with a Kp-only encoder loop
will veer, but the gyro path snaps back. The same mechanism runs
inside the sim against the chassis IMU site.

Default motor here is ``ST3032Motor`` (serial-bus servo, wheel/
continuous-rotation mode) — the project's reference motor.
``DriveBase`` adopts serial-bus motors onto the hard-tick native
engine (1.45.0) — in the sim, onto the emulated ``st_bus`` over
MuJoCo wheels — and ``use_gyro(True)`` feeds the IMU heading into
that controller's absolute heading-hold frame for both
``straight()``'s correction and ``turn()``'s completion check.
"""

from openbricks.drivers.bno055 import BNO055
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics.drivebase import DriveBase
from machine import I2C


# Hardware imports + constructor args are no-ops under the shim;
# the I2C handle the user code instantiates is just a stub the
# BNO055 stores. The shim's BNO055 binds straight to the chassis
# IMU site regardless.
i2c = I2C(0, sda=15, scl=16, freq=400_000)   # ESP32-S3; 21/22 on classic ESP32
imu = BNO055(i2c=i2c, address=0x28)   # some breakouts' ADR pin defaults
                                       # high instead — try 0x29 if this
                                       # raises "BNO055 not found"

m_left  = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)
m_right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)

db = DriveBase(m_left, m_right,
               wheel_diameter_mm=88,
               axle_track_mm=136,
               imu=imu)
db.settings(straight_speed=180, turn_rate=120)
db.use_gyro(True)   # heading feedback now comes from the IMU

print("starting heading:", imu.heading())  # noqa: F821

for _ in range(4):
    db.straight(150)
    db.turn(90)

print("ending heading:", imu.heading())   # noqa: F821
print("chassis pose:  ", robot.chassis_pose())  # noqa: F821
