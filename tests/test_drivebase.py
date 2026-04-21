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

from openbricks._native import motor_process
from openbricks.drivers.jgb37_520 import JGB37Motor
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


def _wheel_deg_for_distance(distance_mm, wheel_diameter_mm):
    return distance_mm / (math.pi * wheel_diameter_mm) * 360


def _reset_all():
    motor_process.reset()
    from machine import Timer
    Timer.reset_for_test()


class TestDriveBaseOpenLoop(unittest.TestCase):
    """Open-loop path exercises L298N motors via the rated-speed fallback."""

    def setUp(self):
        _reset_all()

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
        _reset_all()

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


class TestDriveBaseNative2DOF(unittest.TestCase):
    """Exit criterion for M3: the native 2-DOF controller keeps heading
    bounded under asymmetric-friction loads that the pure-Kp fallback
    can't. Uses ``JGB37Motor`` (which wraps a native Servo), driven by
    a motor_process callback that advances each motor's encoder from
    its own target_dps at the configured period."""

    def setUp(self):
        _reset_all()

    def _make_motor(self, in1, in2, pwm, ea, eb):
        return JGB37Motor(
            in1=in1, in2=in2, pwm=pwm,
            encoder_a=ea, encoder_b=eb,
            counts_per_output_rev=1320,
        )

    def _install_asymmetric_sim(self, left, right, left_scale=1.0, right_scale=1.0):
        """Install a motor_process callback that integrates each motor's
        target_dps into its encoder count, scaled by a per-side factor
        (values < 1.0 simulate friction). Uses a float accumulator so
        slow-speed ticks aren't rounded away."""
        left_acc = [0.0]
        right_acc = [0.0]
        cpr_over_360 = 1320 / 360.0

        def tick():
            dt_s = motor_process.period_ms() / 1000.0
            left_acc[0]  += left._servo.target_dps()  * left_scale  * dt_s * cpr_over_360
            right_acc[0] += right._servo.target_dps() * right_scale * dt_s * cpr_over_360
            left._enc.reset(int(left_acc[0]))
            right._enc.reset(int(right_acc[0]))

        motor_process.register(tick)

    def test_straight_keeps_heading_under_asymmetric_friction(self):
        """Left wheel has 10% more friction (advances at 0.9x its target).
        2-DOF coupling should still bring both wheels to within a few
        degrees of each other at the end of the move."""
        left  = self._make_motor(1, 2, 3, 10, 11)
        right = self._make_motor(4, 5, 6, 12, 13)
        self._install_asymmetric_sim(left, right, left_scale=0.9, right_scale=1.0)

        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        db.settings(straight_speed=200, turn_rate=180)
        db.straight(100)

        left_angle  = left.angle()
        right_angle = right.angle()
        target_wheel_deg = 100 / (math.pi * 56) * 360

        self.assertGreaterEqual(left_angle,  target_wheel_deg * 0.9)
        self.assertGreaterEqual(right_angle, target_wheel_deg * 0.9)

        heading_err = abs(left_angle - right_angle)
        self.assertLess(heading_err, target_wheel_deg * 0.05)

    def test_turn_converges_in_native_path(self):
        left  = self._make_motor(1, 2, 3, 10, 11)
        right = self._make_motor(4, 5, 6, 12, 13)
        self._install_asymmetric_sim(left, right)

        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        db.settings(turn_rate=180)
        db.turn(90)

        self.assertLess(left.angle(), 0)
        self.assertGreater(right.angle(), 0)

        arc_mm = math.radians(90) * (114 / 2)
        expected_diff = arc_mm / (math.pi * 56) * 360
        diff = (left.angle() - right.angle()) / 2.0
        self.assertLess(abs(abs(diff) - expected_diff), expected_diff * 0.1)


class _FakeIMU:
    """Minimal IMU shim — only ``.heading()``, which is the only thing
    the drivebase's gyro path reads. Tests mutate ``heading_value`` between
    ticks to simulate real-world rotation."""

    def __init__(self, heading=0.0):
        self.heading_value = heading

    def heading(self):
        return self.heading_value


class TestDriveBaseUseGyro(unittest.TestCase):
    """``DriveBase.use_gyro(True)`` swaps the heading feedback source
    from encoder-diff to the attached IMU — Pybricks-style, slip-immune."""

    def setUp(self):
        _reset_all()

    def _make_motor(self, in1, in2, pwm, ea, eb):
        return JGB37Motor(
            in1=in1, in2=in2, pwm=pwm,
            encoder_a=ea, encoder_b=eb,
            counts_per_output_rev=1320,
        )

    def test_requires_imu_attached(self):
        left  = self._make_motor(1, 2, 3, 10, 11)
        right = self._make_motor(4, 5, 6, 12, 13)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        with self.assertRaises(ValueError):
            db.use_gyro(True)

    def test_use_gyro_toggle_without_native_raises(self):
        # L298NMotor has no ``_servo`` — no native path, so use_gyro
        # should error rather than silently do nothing.
        left  = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        with self.assertRaises(RuntimeError):
            db.use_gyro(True)

    def test_gyro_heading_drives_diff_correction(self):
        """With use_gyro on, tick the native drivebase once with an IMU
        reading that differs from the captured offset. The heading-error
        term should push one wheel's target_dps above the other — the
        correction that would steer the robot back onto heading."""
        from openbricks._native import DriveBase as NativeDB
        imu = _FakeIMU(heading=0.0)

        left  = self._make_motor(1, 2, 3, 10, 11)
        right = self._make_motor(4, 5, 6, 12, 13)

        # Reach in and build the native drivebase directly — easier than
        # routing through the Python DriveBase + blocking straight() for
        # this behavioural assertion.
        ndb = NativeDB(
            left=left._servo, right=right._servo,
            wheel_diameter_mm=56, axle_track_mm=114,
            imu=imu,
        )
        # Attach both servos to motor_process.
        left.run_speed(0)
        right.run_speed(0)

        # Enable gyro and start a straight move (0 mm so the target
        # stays put — we just want the heading feedback loop active).
        ndb.use_gyro(True)
        ndb.straight(0.0, 50.0)

        # Now inject a +10° heading (robot rotated CCW). Correction is
        # CW rotation → left wheel advances, right retreats → left_dps
        # ends up above right_dps.
        imu.heading_value = 10.0
        time.sleep_ms(1)   # one tick

        self.assertGreater(left._servo.target_dps(), right._servo.target_dps())

        ndb.stop()

    def test_use_gyro_false_reverts_to_encoder_feedback(self):
        """Toggling use_gyro off makes the controller ignore the IMU."""
        from openbricks._native import DriveBase as NativeDB
        imu = _FakeIMU(heading=0.0)

        left  = self._make_motor(1, 2, 3, 10, 11)
        right = self._make_motor(4, 5, 6, 12, 13)
        ndb = NativeDB(
            left=left._servo, right=right._servo,
            wheel_diameter_mm=56, axle_track_mm=114,
            imu=imu,
        )
        left.run_speed(0)
        right.run_speed(0)

        ndb.use_gyro(False)      # default, but make it explicit
        ndb.straight(0.0, 50.0)

        # Heading reading is injected but use_gyro is off — encoder diff
        # is zero, so target_dps for left and right should stay equal.
        imu.heading_value = 10.0
        time.sleep_ms(1)

        self.assertAlmostEqual(
            left._servo.target_dps(), right._servo.target_dps(), places=3
        )

        ndb.stop()


if __name__ == "__main__":
    unittest.main()
