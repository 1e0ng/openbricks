# SPDX-License-Identifier: MIT
"""Tests for the DriveBase robotics layer.

Covers both paths through ``_run_at_dps``:
    * Open-loop (L298N): ``run_speed`` raises NotImplementedError, fallback to
      ``run(power)`` with a rated-speed assumption.
    * Closed-loop: a fake encoder motor that accepts ``run_speed`` and whose
      angle is driven forward by the scheduler stand-in (``time.sleep_ms``).
"""

import tests._fakes  # noqa: F401

import math
import time
import unittest

from machine import Timer  # type: ignore[import-not-found]  # provided by fakes

from openbricks.drivers.l298n import L298NMotor
from openbricks.robotics.drivebase import DriveBase
from openbricks.interfaces import Motor
from openbricks.tools.scheduler import MotorProcess


class _FakeClosedLoopMotor(Motor):
    """Minimal closed-loop ``Motor`` for driving ``DriveBase`` in tests.

    ``run_speed`` sets an internal target; ``step(seconds)`` advances the
    simulated shaft angle at that rate. The test patches ``time.sleep_ms`` so
    every tick of the DriveBase loop advances both motors together.

    ``start`` / ``stop`` are recorded so tests can assert DriveBase manages
    the scheduler lifecycle around each blocking move.
    """

    def __init__(self, scale=1.0):
        self._angle_deg = 0.0
        self._target_dps = 0.0
        self._scale = scale
        self.start_calls = 0
        self.stop_calls = 0

    def run(self, power):
        # Not exercised in the closed-loop path; present so the interface is
        # satisfied.
        self._target_dps = power * 3.0

    def brake(self):
        self._target_dps = 0.0

    def coast(self):
        self._target_dps = 0.0

    def angle(self):
        return self._angle_deg

    def reset_angle(self, angle=0):
        self._angle_deg = float(angle)

    def run_speed(self, deg_per_s):
        self._target_dps = float(deg_per_s)

    def start(self):
        self.start_calls += 1

    def stop(self):
        self.stop_calls += 1

    def step(self, seconds):
        self._angle_deg += self._target_dps * self._scale * seconds


def _wheel_deg_for_distance(distance_mm, wheel_diameter_mm):
    return distance_mm / (math.pi * wheel_diameter_mm) * 360


class TestDriveBaseOpenLoop(unittest.TestCase):
    """Open-loop path exercises L298N motors via the rated-speed fallback."""

    def setUp(self):
        MotorProcess.reset()
        Timer.reset_for_test()

    def test_drive_straight_drives_both_wheels_forward(self):
        left = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.drive(100, 0)

        self.assertEqual(left._in1.value(), 1)
        self.assertEqual(left._in2.value(), 0)
        self.assertEqual(right._in1.value(), 1)
        self.assertEqual(right._in2.value(), 0)
        self.assertEqual(left._pwm.duty(), right._pwm.duty())
        self.assertGreater(left._pwm.duty(), 0)

    def test_drive_pure_turn_drives_wheels_opposite(self):
        left = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        # Positive turn rate means turn left: left wheel reverses, right advances.
        db.drive(0, 90)

        self.assertEqual(left._in1.value(), 0)
        self.assertEqual(left._in2.value(), 1)
        self.assertEqual(right._in1.value(), 1)
        self.assertEqual(right._in2.value(), 0)

    def test_stop_engages_brake(self):
        left = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.stop()

        self.assertEqual(left._in1.value(), 1)
        self.assertEqual(left._in2.value(), 1)
        self.assertEqual(right._in1.value(), 1)
        self.assertEqual(right._in2.value(), 1)


class TestDriveBaseClosedLoop(unittest.TestCase):
    """Closed-loop path: straight() and turn() converge on target angles."""

    def setUp(self):
        MotorProcess.reset()
        Timer.reset_for_test()

    def _patch_sleep_steps_motors(self, *motors):
        original = time.sleep_ms

        def stepped_sleep(ms):
            original(ms)
            for m in motors:
                m.step(ms / 1000.0)

        time.sleep_ms = stepped_sleep
        self.addCleanup(lambda: setattr(time, "sleep_ms", original))

    def test_straight_converges_on_target_wheel_angle(self):
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.straight(100)

        target = _wheel_deg_for_distance(100, wheel_diameter_mm=56)
        # The loop exits when the *average* wheel angle crosses the target,
        # which is a loose bound but the right one for straight-line travel.
        avg = (left.angle() + right.angle()) / 2
        self.assertGreaterEqual(avg, target)

    def test_straight_reverse_converges_on_negative_target(self):
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.straight(-100)

        target = _wheel_deg_for_distance(-100, wheel_diameter_mm=56)
        avg = (left.angle() + right.angle()) / 2
        self.assertLessEqual(avg, target)

    def test_turn_converges(self):
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.turn(90)

        # Both wheels should have swept through the expected arc, with opposite
        # signs (left reverses, right advances).
        arc_mm = math.radians(90) * (114 / 2)
        expected = arc_mm / (math.pi * 56) * 360
        self.assertLessEqual(left.angle(), -expected + 5)  # reversed
        self.assertGreaterEqual(right.angle(), expected - 5)

    def test_settings_overrides_cruise_parameters(self):
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.settings(straight_speed=400, turn_rate=360)
        self.assertEqual(db._straight_speed_dps, 400)
        self.assertEqual(db._turn_rate_dps, 360)

    def test_straight_manages_motor_scheduler_lifecycle(self):
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.straight(100)

        self.assertEqual(left.start_calls, 1)
        self.assertEqual(right.start_calls, 1)
        self.assertEqual(left.stop_calls, 1)
        self.assertEqual(right.stop_calls, 1)

    def test_turn_manages_motor_scheduler_lifecycle(self):
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.turn(90)

        self.assertEqual(left.start_calls, 1)
        self.assertEqual(right.start_calls, 1)
        self.assertEqual(left.stop_calls, 1)
        self.assertEqual(right.stop_calls, 1)


class TestDriveBaseWithJGB37(unittest.TestCase):
    """End-to-end: DriveBase + the real JGB37 driver wired to the fake
    scheduler. Validates that the M1 refactor ticks the control loop while
    DriveBase is blocked in its polling loop."""

    def setUp(self):
        MotorProcess.reset()
        Timer.reset_for_test()

    def test_straight_ticks_jgb37_control_step_during_sleep(self):
        from openbricks.drivers.jgb37_520 import JGB37Motor

        left = JGB37Motor(
            in1=1, in2=2, pwm=3,
            encoder_a=10, encoder_b=11,
        )
        right = JGB37Motor(
            in1=4, in2=5, pwm=6,
            encoder_a=12, encoder_b=13,
        )

        # Simulate both encoders incrementing in lock-step with the
        # scheduler by registering our own tick that advances them.
        def spin_encoders(_t=None):
            left._enc._count += 13
            right._enc._count += 13

        proc = MotorProcess.instance()

        # We register our encoder-spinner before DriveBase starts the motors
        # — DriveBase.straight will register each motor's _control_step too.
        proc.register(spin_encoders)

        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        db.settings(straight_speed=200, turn_rate=120)
        db.straight(50)  # modest distance so the test stays fast

        target = 50 / (math.pi * 56) * 360
        self.assertGreaterEqual((left.angle() + right.angle()) / 2, target)

        # After straight() returns, both motors should have unregistered.
        self.assertNotIn(left._control_step, proc._callbacks)
        self.assertNotIn(right._control_step, proc._callbacks)
        self.assertFalse(left._scheduled)
        self.assertFalse(right._scheduled)


if __name__ == "__main__":
    unittest.main()
