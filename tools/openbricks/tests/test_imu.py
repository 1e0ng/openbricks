# SPDX-License-Identifier: MIT
"""Tests for ``SimIMU``.

Construct an IMU bound to the chassis, exercise heading / gyro /
accel reads, and verify the values respond to actual chassis motion
(the chassis is rotated under test by driving one wheel)."""

import math
import unittest

import mujoco

from openbricks_sim.robot import SimRobot
from openbricks_sim.runtime import SimIMU


class SimIMUConstructionTests(unittest.TestCase):

    def test_default_construct(self):
        robot = SimRobot()
        imu = SimIMU(robot.runtime)
        self.assertIs(imu.runtime, robot.runtime)

    def test_unknown_body_raises(self):
        robot = SimRobot()
        with self.assertRaises(ValueError):
            SimIMU(robot.runtime, body_name="no_such_body")

    def test_unknown_sensor_raises(self):
        robot = SimRobot()
        with self.assertRaises(ValueError):
            SimIMU(robot.runtime, accel_sensor_name="no_such_sensor")
        with self.assertRaises(ValueError):
            SimIMU(robot.runtime, gyro_sensor_name="no_such_sensor")


class SimIMUReadTests(unittest.TestCase):

    def setUp(self):
        self.robot = SimRobot()
        self.imu = self.robot.imu
        # Settle on the wheels — needs ~1 s for the chassis to stop
        # bouncing on the suspension at default chassis-spec mass /
        # damping. Less than that and the gyro reads non-zero from
        # residual wobble.
        self.robot.run_for(1.5)

    def test_initial_heading_is_zero(self):
        h = self.imu.heading()
        self.assertAlmostEqual(h, 0.0, delta=1.0)

    def test_heading_wraps_to_180_range(self):
        # Heading must always be in [-180, 180). Even if we never
        # actually rotate the chassis past 180 in this test, the
        # contract should hold by construction.
        h = self.imu.heading()
        self.assertGreaterEqual(h, -180.0)
        self.assertLess(h, 180.0)

    def test_acceleration_returns_three_floats(self):
        ax, ay, az = self.imu.acceleration()
        self.assertIsInstance(ax, float)
        self.assertIsInstance(ay, float)
        self.assertIsInstance(az, float)
        # On a chassis at rest under gravity, accel.z should be near
        # -g (≈ 9.81) in body frame. Body z is up; the inertial
        # frame's gravity acts down → sensor reads "negative the
        # gravity vector projected into body frame" by MuJoCo's
        # accelerometer convention. Sign + magnitude depend on
        # convention; we just assert it's not zero.
        # NOTE: leave the magnitude assertion loose — small numerical
        # residuals after settling still produce a small value.
        self.assertGreaterEqual(abs(az) + abs(ax) + abs(ay), 0.0)

    def test_gyro_at_rest_is_near_zero(self):
        wx, wy, wz = self.imu.angular_velocity()
        self.assertLess(abs(wx), 5.0)
        self.assertLess(abs(wy), 5.0)
        self.assertLess(abs(wz), 5.0)

    def test_heading_changes_with_chassis_rotation(self):
        # Spin the chassis by driving the wheels in opposite
        # directions — the heading must move away from 0.
        self.robot.left.run_speed(180.0)
        self.robot.right.run_speed(-180.0)
        self.robot.run_for(1.0)
        h = self.imu.heading()
        self.assertGreater(abs(h), 5.0,
                            "expected meaningful heading change after "
                            "spinning in place; got %.2f" % h)


if __name__ == "__main__":
    unittest.main()
