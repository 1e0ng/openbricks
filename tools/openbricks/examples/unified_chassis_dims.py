# SPDX-License-Identifier: MIT
"""
The same robot.py for firmware AND sim — chassis dims declared
once, in the ``DriveBase(...)`` call.

Run this script via the sim::

    openbricks-sim run examples/unified_chassis_dims.py --world empty

…and the same file flashed to ESP32 hardware will drive the same
distance. No second config file. No CLI flag for chassis size.
The ``wheel_diameter_mm`` / ``axle_track_mm`` you pass to
``DriveBase`` are the single source of truth — on firmware they
go straight to the encoder math, and on sim the shim layer picks
them up at construction time and resizes the MuJoCo chassis to
match.

Try changing ``WHEEL_DIAMETER_MM`` to 80 (or ``AXLE_TRACK_MM`` to
200) and re-running — the sim's wheels grow, and ``straight()``
still travels the right number of mm because the wheel
circumference and the encoder readout track each other.

Why this matters
----------------

Without dim sync, ``DriveBase(wheel_diameter_mm=80, ...)`` would
tell the firmware-side odometry math "80 mm wheels" while the sim
physics quietly used 60 mm wheels. The user calls
``db.straight(distance_mm=200)`` and the chassis travels 150 mm
in physics but the user's ``db.distance()`` reports 200 mm.
Silent divergence between firmware's mental model and the sim's
ground truth. This script exists to demonstrate the dim-sync that
prevents it.
"""

from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics.drivebase import DriveBase
from machine import I2C  # noqa: F401 — illustrates "real driver imports"


# Single source of truth for the chassis. Same constants firmware
# reads, and what the sim resizes itself to match.
WHEEL_DIAMETER_MM = 60
AXLE_TRACK_MM     = 150
DRIVE_DISTANCE_MM = 200


m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                     encoder_a=18, encoder_b=19)
m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                     encoder_a=20, encoder_b=21)
db = DriveBase(m_left, m_right,
               wheel_diameter_mm=WHEEL_DIAMETER_MM,
               axle_track_mm=AXLE_TRACK_MM)
db.settings(straight_speed=120)

print(f"DriveBase: {WHEEL_DIAMETER_MM} mm wheels, "
      f"{AXLE_TRACK_MM} mm axle track")
print(f"driving {DRIVE_DISTANCE_MM} mm forward...")

db.straight(DRIVE_DISTANCE_MM)   # blocks until the move is done

# Sim-only sanity check: read the actual chassis world pose. On
# firmware ``robot`` doesn't exist; on sim it's the SimRobot.
if "robot" in dir():
    x, y, yaw = robot.chassis_pose()                          # noqa: F821
    print(f"  ended at: ({x:6.1f}, {y:5.1f}) mm, yaw {yaw:5.1f}°")
