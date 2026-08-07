# SPDX-License-Identifier: MIT
"""Tests for the DriveBase robotics layer.

Covers the open-loop ``drive()``/``stop()`` path (L298N via
``_run_at_dps``'s rated-speed mapping) and the no-Python-loop
contract (1.45.0): motor pairs with neither a native servo nor a
serial-bus engine get NO ``straight()``/``turn()`` — the old
pure-Python heading-hold fallback was deleted, so those now raise
instead of silently degrading. Closed-loop behavior is covered by
``test_drivebase_native_2dof`` (encoder path), ``test_native_drivebase``
(serial engine over a fake bus), and the sim suite.
"""

import tests._fakes  # noqa: F401

import unittest

from openbricks._native import motor_process
from openbricks.drivers.l298n import L298NMotor
from openbricks.robotics.drivebase import DriveBase
from openbricks.interfaces import Motor


class _FakeClosedLoopMotor(Motor):
    """Minimal closed-loop ``Motor`` for driving ``DriveBase`` in tests.

    ``run_speed`` sets an internal target; ``step(seconds)`` advances the
    simulated shaft angle at that rate. The test patches ``time.sleep_ms`` so
    every tick of the DriveBase loop advances both motors together.
    """

    def __init__(self, scale=1.0):
        self._angle_deg = 0.0
        self._target_dps = 0.0
        self._scale = scale

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

    def step(self, seconds):
        self._angle_deg += self._target_dps * self._scale * seconds


def _reset_all():
    motor_process.reset()
    from machine import Timer
    Timer.reset_for_test()


class TestDriveBaseOpenLoop(unittest.TestCase):
    """Open-loop path exercises L298N motors via the rated-speed fallback."""

    def setUp(self):
        _reset_all()

    def test_drive_straight_drives_both_wheels_forward(self):
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.drive(100, 0)

        self.assertEqual(left._in1.value(), 1)
        self.assertEqual(left._in2.value(), 0)
        self.assertEqual(right._in1.value(), 1)
        self.assertEqual(right._in2.value(), 0)
        self.assertEqual(left._pwm.duty(), right._pwm.duty())
        self.assertGreater(left._pwm.duty(), 0)

    def test_drive_pure_turn_drives_wheels_opposite(self):
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        # Positive turn rate means turn right (CW, Pybricks
        # convention): left wheel advances, right reverses.
        db.drive(0, 90)

        self.assertEqual(left._in1.value(), 1)
        self.assertEqual(left._in2.value(), 0)
        self.assertEqual(right._in1.value(), 0)
        self.assertEqual(right._in2.value(), 1)

    def test_stop_default_coasts(self):
        # New default: ``stop()`` (and the implicit stop at the end of
        # ``straight``/``turn``) coasts both wheels — pybricks-style.
        # Coast on L298N drops both H-bridge inputs low and zeroes the
        # PWM duty.
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.stop()

        self.assertEqual(left._in1.value(), 0)
        self.assertEqual(left._in2.value(), 0)
        self.assertEqual(right._in1.value(), 0)
        self.assertEqual(right._in2.value(), 0)
        self.assertEqual(left._pwm.duty(), 0)
        self.assertEqual(right._pwm.duty(), 0)

    def test_stop_then_brake_engages_brake(self):
        # Explicit ``then="brake"`` falls back to the pre-1.6.7
        # behaviour: short both H-bridge terminals high.
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.stop(then="brake")

        self.assertEqual(left._in1.value(), 1)
        self.assertEqual(left._in2.value(), 1)
        self.assertEqual(right._in1.value(), 1)
        self.assertEqual(right._in2.value(), 1)

    def test_stop_then_hold_raises_on_open_loop_motors(self):
        # L298N can't actively hold — no encoder, no position loop.
        # ``stop(then="hold")`` must raise rather than silently fall
        # back to brake.
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        with self.assertRaises(NotImplementedError):
            db.stop(then="hold")

    def test_move_wheels_drives_each_side_independently(self):
        # Open-loop pairs can't batch (two PWM writes), but the API
        # must still work — documented in the method's docstring.
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.move_wheels(300, -300)          # spin in place

        self.assertEqual(left._in1.value(), 1)
        self.assertEqual(left._in2.value(), 0)
        self.assertEqual(right._in1.value(), 0)
        self.assertEqual(right._in2.value(), 1)
        self.assertGreater(left._pwm.duty(), 0)
        self.assertEqual(left._pwm.duty(), right._pwm.duty())

    def test_stop_then_invalid_raises_value_error(self):
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        with self.assertRaises(ValueError):
            db.stop(then="freewheel")


class _FakeIMU:
    def __init__(self, heading=0.0):
        self.heading_value = heading

    def heading(self):
        return self.heading_value


class TestDriveBaseNoPythonLoop(unittest.TestCase):
    """1.45.0 contract: there is NO pure-Python control loop. A motor
    pair with neither a native servo (``._servo``) nor a serial-bus
    adoption hook (``._adopt_into_drivebase``) — the shape the old
    fallback served — gets open-loop ``drive()`` only; moves by
    distance raise instead of silently running a degraded loop."""

    def setUp(self):
        _reset_all()

    def _db(self, imu=None):
        left  = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        return DriveBase(left, right, wheel_diameter_mm=56,
                         axle_track_mm=114, imu=imu), left, right

    def test_straight_raises(self):
        db, _, _ = self._db()
        with self.assertRaises(RuntimeError):
            db.straight(100)

    def test_turn_raises(self):
        db, _, _ = self._db()
        with self.assertRaises(RuntimeError):
            db.turn(90)

    def test_curve_raises(self):
        # Same contract as straight/turn: an arc by geometry needs
        # feedback; open-loop pairs use drive().
        db, _, _ = self._db()
        with self.assertRaises(RuntimeError):
            db.curve(150, 90)
        with self.assertRaises(ValueError):
            db.curve(150, 90, then="drift")

    def test_straight_wait_false_raises_too(self):
        # The raise sits at the arm site, so wait=False can't sneak a
        # move past the contract either.
        db, _, _ = self._db()
        with self.assertRaises(RuntimeError):
            db.straight(100, wait=False)
        # Nothing armed: done() reports idle.
        self.assertTrue(db.done())

    def test_use_gyro_raises_even_with_imu(self):
        # No engine = no heading-hold loop for the gyro to feed.
        db, _, _ = self._db(imu=_FakeIMU())
        with self.assertRaises(RuntimeError):
            db.use_gyro(True)

    def test_use_gyro_without_imu_still_raises_value_error(self):
        # The missing-imu diagnosis outranks the missing-engine one.
        db, _, _ = self._db()
        with self.assertRaises(ValueError):
            db.use_gyro(True)

    def test_drive_still_works_open_loop(self):
        # drive() is kinematic and loop-free — it must keep working on
        # any pair that accepts run_speed.
        db, left, right = self._db()
        db.drive(100, 0)
        self.assertGreater(left._target_dps, 0)
        self.assertEqual(left._target_dps, right._target_dps)
        db.stop()
        self.assertEqual(left._target_dps, 0.0)

    def test_done_true_when_nothing_pending(self):
        db, _, _ = self._db()
        self.assertTrue(db.done())


if __name__ == "__main__":
    unittest.main()
