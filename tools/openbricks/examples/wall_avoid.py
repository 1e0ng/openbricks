# SPDX-License-Identifier: MIT
"""
Drive forward; stop on a wall — using the firmware HCSR04 driver.

Run with::

    openbricks-sim run examples/wall_avoid.py \\
        --world wro-2026-elementary --viewer

The driver shim replaces ``openbricks.drivers.hcsr04.HCSR04`` with a
sim-aware class that raycasts forward from the chassis ``chassis_dist``
site. From the script's POV the same firmware ``distance_mm()`` is
in use.

Polls the sensor every 50 ms (sim time, via ``time.sleep_ms`` which
the shim patches to step the runtime). Stops when distance drops
below 200 mm — typical "robot is about to hit a wall" threshold.
"""

import time

from openbricks.drivers.hcsr04 import HCSR04
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics.drivebase import DriveBase
from machine import Pin


sensor = HCSR04(trig=18, echo=19, timeout_us=30_000)

m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                     encoder_a=20, encoder_b=21)
m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                     encoder_a=22, encoder_b=23)
db = DriveBase(m_left, m_right,
               wheel_diameter_mm=60,
               axle_track_mm=150)

print("starting distance:", sensor.distance_mm())

# Drive forward at 100 mm/s open-loop; poll the sensor; stop when
# anything inside 200 mm shows up. The ``-1`` "no echo" result
# means we keep going.
m_left.run_speed(180.0)
m_right.run_speed(180.0)

for tick in range(200):    # up to 10 s of sim time
    time.sleep_ms(50)
    d = sensor.distance_mm()
    if d != -1 and d < 200:
        print(f"wall at {d} mm — stopping.")
        break
    if tick % 5 == 0:
        print(f"  {tick * 50} ms: distance = {d}")

m_left.brake()
m_right.brake()
print("ending pose:", robot.chassis_pose())  # noqa: F821
