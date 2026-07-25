# SPDX-License-Identifier: MIT
"""Tests for ``DriveBase.use_gyro(True)`` — Pybricks-style IMU heading feedback.

Kept in its own test module (separate from ``test_drivebase.py``) so that
unrelated segfaults in the coverage-variant debug build of other
drivebase tests don't prevent these from running and contributing
``.gcda`` data to the coverage upload.
"""

import tests._fakes  # noqa: F401

import math
import time
import unittest

from machine import Timer

from openbricks._native import DriveBase as NativeDB, motor_process
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.drivers.l298n import L298NMotor
from openbricks.robotics.drivebase import DriveBase


def _reset_all():
    motor_process.reset()
    Timer.reset_for_test()


class _FakeIMU:
    """Minimal IMU shim — only ``.heading()``, which is the only thing
    the drivebase's gyro path reads. Tests mutate ``heading_value``
    between ticks to simulate real-world rotation."""

    def __init__(self, heading=0.0):
        self.heading_value = heading

    def heading(self):
        return self.heading_value


def _make_motor(in1, in2, pwm, ea, eb):
    return JGB37Motor(
        in1=in1, in2=in2, pwm=pwm,
        encoder_a=ea, encoder_b=eb,
        counts_per_output_rev=1320,
    )


class TestDriveBaseUseGyro(unittest.TestCase):
    def setUp(self):
        _reset_all()

    def test_requires_imu_attached(self):
        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        with self.assertRaises(ValueError):
            db.use_gyro(True)

    def test_use_gyro_toggle_without_native_and_without_imu_raises(self):
        # L298NMotor has no ``_servo`` — no native path — and no imu=
        # was passed either. The missing-imu check fires regardless of
        # which path the motors would otherwise take.
        left  = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        with self.assertRaises(ValueError):
            db.use_gyro(True)

    def test_gyro_heading_drives_diff_correction(self):
        """With use_gyro on, tick the native drivebase once with an IMU
        reading that differs from the captured offset. The heading-error
        term should push one wheel's target_dps above the other — the
        correction that would steer the robot back onto heading."""
        imu = _FakeIMU(heading=0.0)

        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)

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

        # Now inject a +10° heading (robot rotated CW/right — the
        # 1.24.0 Pybricks convention). Correction is CCW rotation →
        # right wheel advances, left retreats → right_dps ends up
        # above left_dps.
        imu.heading_value = 10.0
        time.sleep_ms(1)   # one tick

        self.assertGreater(right._servo.target_dps(), left._servo.target_dps())

        ndb.stop()

    def test_gyro_move_after_prior_rotation_starts_in_fresh_frame(self):
        """REGRESSION (sim IMU verification, 1.15.2): the core used to
        snapshot the move origin from the ENCODER diff even in gyro
        mode, while the tick read the move-relative OVERRIDE — any
        accumulated encoder diff became a permanent heading error the
        controller chased forever (a gyro'd turn(90) after one
        encoder turn rotated ~180 and never stopped)."""
        imu = _FakeIMU(heading=0.0)

        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)
        ndb = NativeDB(
            left=left._servo, right=right._servo,
            wheel_diameter_mm=56, axle_track_mm=114,
            imu=imu,
        )
        left.run_speed(0)
        right.run_speed(0)

        # Accumulate encoder diff: advance the two encoders apart, as
        # a prior encoder-mode rotation would have — then let the
        # servo observers actually ingest the counts (they only see
        # encoder values on motor_process ticks, and smooth jumps
        # over several of them).
        left._enc.reset(500)
        right._enc.reset(-500)
        time.sleep_ms(100)

        # Gyro move from rest, IMU steady at its baseline: the first
        # tick must see (near-)zero heading error — a large initial
        # diff correction means the encoder frame leaked in.
        ndb.use_gyro(True)
        ndb.straight(0.0, 50.0)
        imu.heading_value = 0.0
        time.sleep_ms(1)

        l = left._servo.target_dps()
        r = right._servo.target_dps()
        self.assertLess(abs(l - r), 5.0,
                        "gyro move started with a phantom heading "
                        "error (l=%.1f r=%.1f)" % (l, r))
        ndb.stop()

    def test_gyro_turn_also_starts_in_fresh_frame(self):
        """Twin of the straight() regression for ob_drivebase_turn's
        gyro branch: a gyro TURN after accumulated encoder diff must
        profile from the override frame, not the encoder frame."""
        imu = _FakeIMU(heading=0.0)

        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)
        ndb = NativeDB(
            left=left._servo, right=right._servo,
            wheel_diameter_mm=56, axle_track_mm=114,
            imu=imu,
        )
        left.run_speed(0)
        right.run_speed(0)
        left._enc.reset(500)
        right._enc.reset(-500)
        time.sleep_ms(100)

        ndb.use_gyro(True)
        ndb.turn(90.0, 90.0)
        imu.heading_value = 0.0
        time.sleep_ms(1)

        # First tick of a 90-deg turn at 90 dps: the commanded split
        # must be profile-sized (ramping from ~0), not the encoder
        # frame's phantom hundreds of dps.
        l = left._servo.target_dps()
        r = right._servo.target_dps()
        self.assertLess(abs(l) + abs(r), 120.0,
                        "gyro turn launched with a phantom frame "
                        "error (l=%.1f r=%.1f)" % (l, r))
        ndb.stop()

    def test_gyro_turn_overshoot_corrected_by_next_move_native(self):
        """ABSOLUTE FRAME (1.25.0), native path: after a completed
        +90 gyro turn, a straight that starts with the IMU reading 95
        (5 deg clockwise of the persistent target) must steer BACK —
        right wheel commanded faster than left — instead of adopting
        the overshoot as its new reference the way per-move
        re-baselining did."""
        imu = _FakeIMU(heading=0.0)

        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)
        ndb = NativeDB(
            left=left._servo, right=right._servo,
            wheel_diameter_mm=56, axle_track_mm=114,
            imu=imu,
        )
        left.run_speed(0)
        right.run_speed(0)

        ndb.use_gyro(True)
        ndb.turn(90.0, 360.0)
        time.sleep_ms(1000)   # run the turn trajectory to completion
        self.assertTrue(ndb.is_done())

        imu.heading_value = 95.0   # overshot the absolute target by 5
        ndb.straight(0.0, 50.0)
        time.sleep_ms(5)

        self.assertGreater(right._servo.target_dps(),
                           left._servo.target_dps(),
                           "turn overshoot was adopted instead of "
                           "corrected on the next move")
        ndb.stop()

    def test_use_gyro_false_reverts_to_encoder_feedback(self):
        """Toggling use_gyro off makes the controller ignore the IMU."""
        imu = _FakeIMU(heading=0.0)

        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)
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


class TestDriveBaseFallbackUseGyro(unittest.TestCase):
    """``use_gyro`` on the pure-Python fallback path (serial-bus
    servos, modeled here by ``_FakeClosedLoopMotor`` — it has no
    ``_servo`` so ``DriveBase`` picks the fallback exactly like a real
    ST-3215/ST-3032 pair does)."""

    def setUp(self):
        _reset_all()
        from tests.test_drivebase import _FakeClosedLoopMotor, _FakeStalledMotor
        self._FakeClosedLoopMotor = _FakeClosedLoopMotor
        self._FakeStalledMotor = _FakeStalledMotor

    def _patch_sleep_steps_motors(self, *motors):
        original = time.sleep_ms

        def stepped_sleep(ms):
            original(ms)
            for m in motors:
                m.step(ms / 1000.0)

        time.sleep_ms = stepped_sleep
        self.addCleanup(lambda: setattr(time, "sleep_ms", original))

    def _patch_sleep_noop(self):
        original = time.sleep_ms
        time.sleep_ms = lambda ms: None
        self.addCleanup(lambda: setattr(time, "sleep_ms", original))

    def test_fallback_use_gyro_toggle_succeeds_with_imu(self):
        # This is the actual feature: fallback motors (no ``_servo``)
        # used to hit the "requires closed-loop motors" error
        # unconditionally. With an imu= attached it must now work.
        left  = self._FakeClosedLoopMotor()
        right = self._FakeClosedLoopMotor()
        imu = _FakeIMU(heading=0.0)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114,
                       imu=imu)
        db.use_gyro(True)   # must not raise
        self.assertTrue(db._use_gyro)
        db.use_gyro(False)
        self.assertFalse(db._use_gyro)

    def test_fallback_straight_corrects_from_gyro_not_encoder(self):
        """Slip-immunity, demonstrated directly: both wheels report
        IDENTICAL encoder angles (zero differential — an encoder-only
        loop would apply zero correction), but the IMU reports the
        robot has actually rotated off heading. With use_gyro(True)
        the correction must still fire, proving it's sourced from the
        gyro and not the (perfectly-agreeing, and therefore useless
        here) encoders."""
        left  = self._FakeClosedLoopMotor()
        right = self._FakeClosedLoopMotor()
        imu = _FakeIMU(heading=0.0)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114,
                       imu=imu)
        db.use_gyro(True)

        db.straight(100, wait=False)
        self.assertEqual(left.angle(), right.angle())   # no encoder diff

        imu.heading_value = 10.0   # robot has actually yawed off course
        db.done()   # one fallback tick

        self.assertNotEqual(left._target_dps, right._target_dps,
                            "gyro heading error produced no correction")
        db.stop()

    def test_fallback_straight_use_gyro_false_ignores_heading(self):
        left  = self._FakeClosedLoopMotor()
        right = self._FakeClosedLoopMotor()
        imu = _FakeIMU(heading=0.0)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114,
                       imu=imu)
        # use_gyro left at its default (False) — heading is ignored,
        # matching pre-existing encoder-diff behaviour.
        db.straight(100, wait=False)
        imu.heading_value = 10.0
        db.done()

        self.assertEqual(left._target_dps, right._target_dps)
        db.stop()

    def test_fallback_turn_terminates_on_measured_heading_not_encoder(self):
        """The other half of slip-immunity: the wheel encoders are
        frozen (``_FakeStalledMotor`` — models a slipping/blocked
        wheel that never advances), which would trip the stall
        timeout under encoder-based termination. With use_gyro(True),
        the IMU alone decides when the turn is done."""
        left  = self._FakeStalledMotor()
        right = self._FakeStalledMotor()
        imu = _FakeIMU(heading=0.0)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114,
                       imu=imu)
        db.use_gyro(True)

        db.turn(90, wait=False)
        self.assertFalse(db.done())

        imu.heading_value = 90.0   # IMU: the body has actually reached 90°
        self.assertTrue(db.done(), "gyro-measured heading reached the "
                        "target but the turn did not terminate")

    def test_fallback_turn_overshoot_is_corrected_by_next_straight(self):
        """ABSOLUTE HEADING FRAME (Pybricks-style): bench 2026-07-25
        showed +7 deg of accumulated drift over one gyro'd square —
        each turn overshot ~1.5-2 deg past its target (poll latency +
        coast momentum) and the old per-move re-baselining forgave
        it. With the persistent target, a straight() that starts
        while the robot points past the target must steer BACK."""
        left  = self._FakeClosedLoopMotor()
        right = self._FakeClosedLoopMotor()
        imu = _FakeIMU(heading=0.0)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114,
                       imu=imu)
        db.use_gyro(True)

        # turn(90): overshoot to 95 before the poll notices.
        db.turn(90, wait=False)
        imu.heading_value = 95.0
        self.assertTrue(db.done())      # crossed the target → done

        # The next straight starts 5 deg clockwise of target. The
        # correction must slow the LEFT wheel (steer CCW, back
        # toward the target) — under per-move re-baselining the
        # commands would be equal and the error frozen in.
        db.straight(100, wait=False)
        db.done()   # one tick
        self.assertLess(left._target_dps, right._target_dps,
                        "banked turn overshoot was not corrected")
        db.stop()

    def test_fallback_second_turn_folds_in_first_turns_overshoot(self):
        left  = self._FakeClosedLoopMotor()
        right = self._FakeClosedLoopMotor()
        imu = _FakeIMU(heading=0.0)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114,
                       imu=imu)
        db.use_gyro(True)

        db.turn(90, wait=False)
        imu.heading_value = 95.0    # overshoot
        self.assertTrue(db.done())

        # Second turn(90): absolute target is 180, robot is at 95 —
        # 85 deg to go, not 90. At measured 178 the turn must still
        # be running; crossing 180 ends it.
        db.turn(90, wait=False)
        imu.heading_value = 178.0
        self.assertFalse(db.done())
        imu.heading_value = 180.5
        self.assertTrue(db.done())

    def test_fallback_turn_already_past_target_terminates_immediately(self):
        left  = self._FakeClosedLoopMotor()
        right = self._FakeClosedLoopMotor()
        imu = _FakeIMU(heading=0.0)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114,
                       imu=imu)
        db.use_gyro(True)

        db.turn(90, wait=False)
        imu.heading_value = 130.0   # shoved way past the target
        self.assertTrue(db.done(),
                        "turn already past its absolute target must "
                        "end, not rotate another lap")

    def test_fallback_gyro_survives_pm180_wrap(self):
        """Enable near the BNO055's ±180 boundary and turn across
        it: the continuous-heading accumulator must see a small
        positive delta, not a spurious -360 jump."""
        left  = self._FakeClosedLoopMotor()
        right = self._FakeClosedLoopMotor()
        imu = _FakeIMU(heading=170.0)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114,
                       imu=imu)
        db.use_gyro(True)   # reference = 170

        db.turn(30, wait=False)     # absolute finish = 200 == -160 wrapped
        imu.heading_value = -175.0  # 15 deg of CW progress, wrapped
        self.assertFalse(db.done())
        imu.heading_value = -160.0  # 30 deg of CW progress → target
        self.assertTrue(db.done())

    def test_fallback_turn_without_gyro_still_uses_encoder(self):
        left  = self._FakeClosedLoopMotor()
        right = self._FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.turn(90)   # no imu at all — must behave exactly as before

        # Positive = right/CW: left advances, right reverses.
        arc_mm = math.radians(90) * (114 / 2)
        expected = arc_mm / (math.pi * 56) * 360
        self.assertGreaterEqual(left.angle(), expected - 5)
        self.assertLessEqual(right.angle(), -expected + 5)


if __name__ == "__main__":
    unittest.main()
