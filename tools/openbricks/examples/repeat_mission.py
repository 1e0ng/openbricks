# SPDX-License-Identifier: MIT
"""
Reset + retry — demonstrate ``robot.reset()`` for iterating on a
mission script in one sim process.

Run with::

    openbricks-sim run examples/repeat_mission.py --no-shim --world empty

Drives the chassis forward 200 mm five times in a row, resetting
between attempts and printing the final position each time. Useful
template for tuning a mission: adjust your code, re-run the same
script — no need to relaunch the sim.
"""

# noqa: F821 — ``robot``, ``drivebase``, ``left``, ``right`` come
# from openbricks-sim's ``run`` command (see cli.cmd_run).

NUM_ATTEMPTS = 5
DISTANCE_MM  = 200.0
SPEED_MM_S   = 100.0


for attempt in range(NUM_ATTEMPTS):
    robot.reset()                                          # noqa: F821
    drivebase.straight(distance_mm=DISTANCE_MM,            # noqa: F821
                       speed_mm_s=SPEED_MM_S)
    robot.run_until(drivebase.is_done, timeout_s=5.0)      # noqa: F821
    x, y, yaw = robot.chassis_pose()                       # noqa: F821
    print(f"attempt {attempt}: pose=({x:.0f}, {y:.0f}, {yaw:.1f})")

print("done.")
