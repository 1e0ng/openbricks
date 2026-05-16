# SPDX-License-Identifier: MIT
"""Tests for the abstract interface base classes in ``openbricks.interfaces``.

These classes are intentionally bare — the methods exist as
``raise NotImplementedError`` so a missing override on a concrete
driver becomes a loud failure rather than a silent no-op. The tests
pin that contract so any future refactor (e.g. switching to an
``abc``-based interface) keeps the same behaviour.
"""

import tests._fakes  # noqa: F401

import unittest

from openbricks.interfaces import Motor, Servo, IMU, ColorSensor


class TestMotorInterface(unittest.TestCase):
    def test_run_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().run(50)

    def test_brake_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().brake()

    def test_coast_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().coast()

    def test_hold_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().hold()

    def test_angle_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().angle()

    def test_reset_angle_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().reset_angle()

    def test_run_speed_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().run_speed(100)

    def test_run_angle_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().run_angle(100, 90)


class TestServoInterface(unittest.TestCase):
    def test_move_to_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Servo().move_to(45)

    def test_angle_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Servo().angle()


class TestIMUInterface(unittest.TestCase):
    def test_heading_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            IMU().heading()

    def test_angular_velocity_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            IMU().angular_velocity()

    def test_acceleration_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            IMU().acceleration()


class TestColorSensorInterface(unittest.TestCase):
    def test_rgb_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            ColorSensor().rgb()

    def test_ambient_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            ColorSensor().ambient()


if __name__ == "__main__":
    unittest.main()
