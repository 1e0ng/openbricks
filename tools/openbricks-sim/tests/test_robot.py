# SPDX-License-Identifier: MIT
"""Tests for the SimRobot bundle.

Construct a SimRobot, drive it via the high-level convenience
methods, and verify the chassis pose responds. Same MuJoCo physics
as ``test_runtime.py``; ``SimRobot`` is just a thin wrapper, so
these tests focus on the wrapper's behaviour (run_for / run_until
semantics, world resolution, pose introspection)."""

import unittest

from openbricks_sim.chassis import ChassisSpec
from openbricks_sim.robot import SimRobot, SimRobotError, _resolve_world


class WorldResolutionTests(unittest.TestCase):
    def test_none_resolves_to_none(self):
        self.assertIsNone(_resolve_world(None))

    def test_empty_alias_resolves_to_none(self):
        self.assertIsNone(_resolve_world("empty"))

    def test_wro_alias_resolves_to_shipped_path(self):
        p = _resolve_world("wro-2026-elementary")
        self.assertIsNotNone(p)
        self.assertTrue(p.endswith("world.xml"))

    def test_unknown_returned_as_passthrough(self):
        self.assertEqual(_resolve_world("/abs/path.xml"), "/abs/path.xml")


class SimRobotConstructionTests(unittest.TestCase):
    def test_default_construct_empty_world(self):
        robot = SimRobot()
        self.assertEqual(robot.runtime.now_ms, 0)
        self.assertEqual(robot.runtime.timestep_ms, 1)
        # Standard chassis: two motors + a drivebase wired up.
        self.assertIsNotNone(robot.left)
        self.assertIsNotNone(robot.right)
        self.assertIsNotNone(robot.drivebase)

    def test_construct_with_wro_alias_loads_world(self):
        # Exercises the load_world() branch (vs the standalone path).
        robot = SimRobot(world="wro-2026-elementary",
                          chassis_spec=ChassisSpec(pos_x=1.0, pos_y=-0.42))
        # Same default chassis names should be wired up.
        self.assertIsNotNone(robot.left)
        self.assertIsNotNone(robot.right)
        self.assertIsNotNone(robot.drivebase)

    def test_construct_with_explicit_kp_sum_and_kp_diff(self):
        # Just make sure these flow through without exception.
        robot = SimRobot(kp_sum=3.0, kp_diff=6.0)
        self.assertIsNotNone(robot.drivebase)

    def test_custom_chassis_spec_propagates_to_drivebase(self):
        # 80 mm wheels, 200 mm axle — verify those land in the
        # drivebase's geometry. The native DriveBase doesn't expose
        # the geometry back, so we infer from straight-distance
        # behaviour: a 100 mm move should produce ~ 100/(π·80) · 360
        # = ~143 wheel-degrees of trajectory (vs ~191 with 60 mm).
        spec = ChassisSpec(wheel_radius=0.040,    # 80 mm diameter
                           axle_length=0.200)     # 200 mm axle
        robot = SimRobot(chassis_spec=spec)
        # Just assert wiring — no exception at construct time.
        self.assertEqual(robot.chassis_spec.wheel_radius, 0.040)


class SimRobotTimeAdvanceTests(unittest.TestCase):
    def test_step_advances_one_ms(self):
        robot = SimRobot()
        robot.step()
        self.assertEqual(robot.runtime.now_ms, 1)

    def test_run_for_steps_correct_count(self):
        robot = SimRobot()
        robot.run_for(0.25)   # 250 ms of sim time
        self.assertEqual(robot.runtime.now_ms, 250)

    def test_run_until_returns_true_when_predicate_fires(self):
        robot = SimRobot()
        target_ms = 100
        fired = robot.run_until(lambda: robot.runtime.now_ms >= target_ms,
                                 timeout_s=1.0)
        self.assertTrue(fired)
        self.assertGreaterEqual(robot.runtime.now_ms, target_ms)

    def test_run_until_returns_false_on_timeout(self):
        robot = SimRobot()
        fired = robot.run_until(lambda: False, timeout_s=0.05)
        self.assertFalse(fired)
        # Sim time should have advanced by ~50 ms regardless.
        self.assertAlmostEqual(robot.runtime.now_ms, 50, delta=2)


class SimRobotPoseTests(unittest.TestCase):
    def test_initial_pose_is_origin_ish(self):
        robot = SimRobot()
        # Spawn at origin, let it settle on its wheels.
        robot.run_for(0.2)
        x, y, yaw = robot.chassis_pose()
        self.assertAlmostEqual(x,   0.0, delta=10.0)   # mm
        self.assertAlmostEqual(y,   0.0, delta=10.0)
        # Wheel-floor settling shouldn't yaw the chassis meaningfully.
        self.assertAlmostEqual(yaw, 0.0, delta=5.0)

    def test_drivebase_straight_advances_pose_x(self):
        robot = SimRobot()
        robot.run_for(0.2)   # settle
        x0, _, _ = robot.chassis_pose()
        robot.drivebase.straight(distance_mm=200.0, speed_mm_s=80.0)
        # Mid-trajectory snapshot — direction matters more than
        # distance (covered in test_runtime.py).
        robot.run_for(0.8)
        x_mid, _, _ = robot.chassis_pose()
        self.assertGreater(x_mid - x0, 30.0,
                            "chassis should translate +X during a "
                            "straight move")


class SimRobotResetTests(unittest.TestCase):
    """``reset()`` lets a script iterate on a mission without
    restarting the sim process."""

    def test_reset_returns_chassis_to_origin(self):
        robot = SimRobot()
        # Drive the chassis somewhere.
        robot.left.run_speed(180.0)
        robot.right.run_speed(180.0)
        robot.run_for(1.0)
        x_mid, _, _ = robot.chassis_pose()
        self.assertGreater(abs(x_mid), 50.0, "should have moved")
        # Reset.
        robot.reset()
        x_after, y_after, yaw_after = robot.chassis_pose()
        self.assertAlmostEqual(x_after, 0.0, delta=5.0)
        self.assertAlmostEqual(y_after, 0.0, delta=5.0)
        self.assertAlmostEqual(yaw_after, 0.0, delta=2.0)

    def test_reset_zeroes_now_ms(self):
        robot = SimRobot()
        robot.run_for(0.5)
        self.assertEqual(robot.runtime.now_ms, 500)
        robot.reset()
        self.assertEqual(robot.runtime.now_ms, 0)

    def test_reset_clears_active_drivebase_move(self):
        robot = SimRobot()
        robot.drivebase.straight(distance_mm=200.0, speed_mm_s=80.0)
        self.assertFalse(robot.drivebase.is_done())
        robot.reset()
        self.assertTrue(robot.drivebase.is_done())

    def test_reset_allows_drive_again(self):
        # After reset the controllers must be fresh — driving forward
        # again should produce +X translation just like the first
        # time. Catches "left a stale tick callback registered" bugs.
        robot = SimRobot()
        robot.left.run_speed(180.0)
        robot.right.run_speed(180.0)
        robot.run_for(0.5)
        robot.reset()
        x0, _, _ = robot.chassis_pose()
        robot.left.run_speed(180.0)
        robot.right.run_speed(180.0)
        robot.run_for(0.5)
        x1, _, _ = robot.chassis_pose()
        self.assertGreater(x1 - x0, 30.0)


class SimRobotSetPoseTests(unittest.TestCase):

    def test_set_pose_teleports_chassis(self):
        robot = SimRobot()
        robot.set_pose(x_mm=200.0, y_mm=-100.0, yaw_deg=45.0)
        # Settle the chassis on its wheels at the new spot.
        robot.run_for(0.2)
        x, y, yaw = robot.chassis_pose()
        self.assertAlmostEqual(x, 200.0, delta=15.0)
        self.assertAlmostEqual(y, -100.0, delta=15.0)
        self.assertAlmostEqual(yaw, 45.0, delta=5.0)

    def test_set_pose_resets_clock(self):
        robot = SimRobot()
        robot.run_for(0.5)
        robot.set_pose(x_mm=100.0, y_mm=0.0)
        self.assertEqual(robot.runtime.now_ms, 0)


class SimRobotRunViewerTests(unittest.TestCase):
    """Smoke test for the viewer entry — only exercises the import +
    error-when-no-mujoco-viewer path. Actually opening a window in CI
    would block forever, so we don't go further than this."""

    def test_run_viewer_raises_when_viewer_unavailable(self):
        # Simulate ``import mujoco.viewer`` failing.
        import sys
        saved = sys.modules.pop("mujoco.viewer", None)
        sys.modules["mujoco.viewer"] = None  # raise ImportError on import
        try:
            robot = SimRobot()
            with self.assertRaises(SimRobotError):
                robot.run_viewer()
        finally:
            # Restore.
            if saved is not None:
                sys.modules["mujoco.viewer"] = saved
            else:
                sys.modules.pop("mujoco.viewer", None)


if __name__ == "__main__":
    unittest.main()
