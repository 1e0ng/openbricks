# SPDX-License-Identifier: MIT
"""
Tiny example: drive a square with the default chassis.

Run with::

    openbricks-sim run examples/wander.py --world wro-2026-elementary --viewer

The CLI pre-builds a ``SimRobot`` and exposes ``robot``, ``drivebase``,
``left``, ``right`` in this script's globals — same names you'd use
on real hardware after::

    from openbricks.robotics.drivebase import DriveBase
    drivebase = DriveBase(left, right, wheel_diameter_mm=60, axle_track_mm=150)

A driver-shim layer that lets a single script target *both* targets
unmodified lands in Phase C3.
"""

# noqa: F821 — ``robot``, ``drivebase``, ``left``, ``right`` come
# from openbricks-sim's ``run`` command (see cli.cmd_run).

# Sanity-print the starting pose so a user running with --viewer can
# see the script ran.
print("starting at:", robot.chassis_pose())  # noqa: F821

# Drive a 200×200 mm square, pausing briefly at each corner.
SIDE_MM   = 200.0
TURN_DEG  = 90.0
SPEED     = 100.0
TURN_RATE = 90.0

for i in range(4):
    drivebase.straight(distance_mm=SIDE_MM, speed_mm_s=SPEED)        # noqa: F821
    robot.run_until(drivebase.is_done, timeout_s=5.0)                # noqa: F821
    drivebase.turn(angle_deg=TURN_DEG, rate_dps=TURN_RATE)           # noqa: F821
    robot.run_until(drivebase.is_done, timeout_s=5.0)                # noqa: F821

print("ended at:  ", robot.chassis_pose())  # noqa: F821
