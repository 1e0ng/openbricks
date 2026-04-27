# SPDX-License-Identifier: MIT
"""
Drive into a target zone — scored.

Run with::

    openbricks-sim run examples/scored_mission.py --no-shim --world empty

Demonstrates the mission-scoring helpers:

  * ``robot.chassis_in_box(...)`` — axis-aligned membership.
  * ``robot.chassis_in_circle(...)`` — circular membership.
  * ``robot.distance_to(...)`` — Euclidean distance.
  * ``robot.heading_aligned_with(...)`` — yaw within tolerance.

Sweeps over a few cruise speeds, scoring each attempt. Useful as a
template for tuning a real WRO mission against a stable rubric.
"""

# noqa: F821 — ``robot``, ``drivebase`` come from openbricks-sim's
# ``run`` command (see cli.cmd_run).


# Target: a 100 mm × 100 mm box centred at (200, 0) — i.e. the
# robot should end up between 150 and 250 mm forward of spawn.
TARGET_X_MIN = 150.0
TARGET_X_MAX = 250.0
TARGET_Y_MIN = -50.0
TARGET_Y_MAX =  50.0
TARGET_CX    = (TARGET_X_MIN + TARGET_X_MAX) / 2.0
TARGET_CY    = (TARGET_Y_MIN + TARGET_Y_MAX) / 2.0


def score(robot):
    """Combined score: 100 if in the box AND aligned, otherwise
    a falloff based on distance from the target centre."""
    in_zone = robot.chassis_in_box(
        TARGET_X_MIN, TARGET_Y_MIN, TARGET_X_MAX, TARGET_Y_MAX)
    aligned = robot.heading_aligned_with(0.0, tolerance_deg=15.0)
    if in_zone and aligned:
        return 100
    d = robot.distance_to(TARGET_CX, TARGET_CY)
    return max(0, int(100 - d / 5))   # rough falloff: lose 1 point / 5 mm


for speed in (60.0, 80.0, 100.0, 120.0, 150.0):
    robot.reset()                                                # noqa: F821
    drivebase.straight(distance_mm=200.0, speed_mm_s=speed)      # noqa: F821
    robot.run_until(drivebase.is_done, timeout_s=5.0)            # noqa: F821
    s = score(robot)                                             # noqa: F821
    x, y, yaw = robot.chassis_pose()                             # noqa: F821
    print(f"speed={speed:5.0f} mm/s -> score={s:3d}, "
          f"pose=({x:6.1f}, {y:5.1f}, {yaw:5.1f}°)")
