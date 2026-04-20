# SPDX-License-Identifier: MIT
"""
Example: instantiate an entire robot from a single JSON file, then use it.

Hardware matches ``robot.json`` in this directory. Change ``robot.json`` to
reconfigure the hardware — the Python code stays the same.
"""

from openbricks.config import load_robot
from openbricks.tools import wait


robot = load_robot("examples/robot.json")

# Color-guided behavior: roll forward until the sensor sees red.
while True:
    r, g, b = robot.sensors["color"].rgb()
    heading = robot.sensors["imu"].heading()

    print("rgb=({:3d},{:3d},{:3d})  heading={:6.1f}".format(r, g, b, heading))

    if r > g and r > b and r > 120:
        print("Red detected — stopping.")
        robot.drivebase.stop()
        break

    robot.drivebase.drive(speed_mm_s=150, turn_rate_dps=0)
    wait(50)

# Demo the servo too.
if "arm" in robot.servos:
    robot.servos["arm"].move_to(180, speed=500)
    wait(500)
    robot.servos["arm"].move_to(0, speed=500)
