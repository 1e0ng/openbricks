# SPDX-License-Identifier: MIT
"""
Drive a square — using the *firmware* APIs.

Run with::

    openbricks-sim run examples/wander_hardware_style.py \\
        --world wro-2026-elementary --viewer

This script imports from ``openbricks.drivers.*`` and
``openbricks.robotics.*`` exactly as a real-hardware ``main.py``
would. The driver-shim layer (``openbricks_sim.shim``, installed by
the ``run`` command) routes those imports through to MuJoCo:

  * ``machine.Pin`` / ``machine.PWM`` are no-op fakes — the wiring
    arguments below are ignored, but the imports succeed.
  * ``openbricks._native.Servo`` + ``DriveBase`` bind to the default
    chassis's MuJoCo motor / encoder slots (chassis_motor_l/r).
  * ``time.sleep_ms`` is patched to step the sim, so the firmware
    drivers' busy-wait loops on ``is_done`` advance physics each
    iteration without changes.

Same source on hardware and in the sim — that's the point.
"""

from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics.drivebase import DriveBase


m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                     encoder_a=18, encoder_b=19,
                     counts_per_output_rev=1320)
m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                     encoder_a=20, encoder_b=21,
                     counts_per_output_rev=1320)

db = DriveBase(m_left, m_right,
               wheel_diameter_mm=60,
               axle_track_mm=150)
db.settings(straight_speed=180, turn_rate=120)

for _ in range(4):
    db.straight(150)   # mm
    db.turn(90)        # body degrees, positive = left

print("done. chassis pose (mm, mm, deg):", robot.chassis_pose())  # noqa: F821
