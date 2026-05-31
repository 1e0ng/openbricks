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


class _FakeFlappyMotor(_FakeClosedLoopMotor):
    """``_FakeClosedLoopMotor`` with a controllable read-drop pattern,
    for exercising the serial-bus-style ``angle() -> None`` case that
    triggers the None-tolerance path in ``_straight_fallback`` /
    ``_turn_fallback``.
    """

    def __init__(self, scale=1.0):
        super().__init__(scale=scale)
        self._silent = False
        self._drop_indices = set()
        self._call_count = 0

    def go_silent(self):
        """All subsequent ``angle()`` calls return ``None``."""
        self._silent = True

    def drop_calls(self, indices):
        """0-indexed ``angle()`` call numbers that should return ``None``.
        Everything else returns the real angle."""
        self._drop_indices = set(indices)

    def angle(self):
        idx = self._call_count
        self._call_count += 1
        if self._silent or idx in self._drop_indices:
            return None
        return super().angle()


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

    def test_stop_default_coasts(self):
        # New default: ``stop()`` (and the implicit stop at the end of
        # ``straight``/``turn``) coasts both wheels — pybricks-style.
        # Coast on L298N drops both H-bridge inputs low and zeroes the
        # PWM duty.
        left = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
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
        left = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
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
        left = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        with self.assertRaises(NotImplementedError):
            db.stop(then="hold")

    def test_stop_then_invalid_raises_value_error(self):
        left = L298NMotor(in1=1, in2=2, pwm=3)
        right = L298NMotor(in1=4, in2=5, pwm=6)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        with self.assertRaises(ValueError):
            db.stop(then="freewheel")


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


class TestDriveBaseFallbackBusGlitchTolerance(unittest.TestCase):
    """Pre-1.6.8 regression: ``_straight_fallback`` / ``_turn_fallback``
    crashed with ``TypeError: unsupported types for __sub__: 'NoneType',
    'float'`` whenever ``motor.angle()`` returned ``None`` mid-loop —
    a routine occurrence for serial-bus servos (ST-3215, ST-3032)
    whose present-position read can time out under EMI or half-duplex
    collisions. The fix mirrors ``run_angle``'s inner-loop pattern:
    skip ``None`` ticks and only bail after ``_MAX_CONSECUTIVE_NONE``
    ticks in a row."""

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

    def _patch_sleep_noop(self):
        """Use when we don't want the test to spend real wall-clock
        time waiting for the 500 ms bail window."""
        original = time.sleep_ms
        time.sleep_ms = lambda ms: None
        self.addCleanup(lambda: setattr(time, "sleep_ms", original))

    def test_straight_survives_single_dropped_angle_read(self):
        # Pre-fix: this would raise TypeError on the very first inner
        # loop iteration. Post-fix: the None tick is skipped and the
        # move completes normally.
        left = _FakeFlappyMotor()
        right = _FakeFlappyMotor()
        # Anchor read = call 0; drop the first inner-loop read = call 1.
        left.drop_calls([1])
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.straight(100)

        target = _wheel_deg_for_distance(100, wheel_diameter_mm=56)
        avg = (left.angle() + right.angle()) / 2
        self.assertGreaterEqual(avg, target)

    def test_turn_survives_single_dropped_angle_read(self):
        left = _FakeFlappyMotor()
        right = _FakeFlappyMotor()
        right.drop_calls([1])
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.turn(90)  # would crash pre-fix; completes post-fix.

    def test_straight_bails_cleanly_on_permanent_bus_silence(self):
        # Anchor reads succeed, then both motors go silent. After
        # ``_MAX_CONSECUTIVE_NONE`` ticks the loop should exit cleanly
        # via ``stop()`` — no TypeError, no infinite loop.
        left = _FakeFlappyMotor()
        right = _FakeFlappyMotor()
        # Anchor reads consume call 0; drop everything from call 1 on.
        left.drop_calls(range(1, 1000))
        right.drop_calls(range(1, 1000))
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_noop()

        db.straight(100)   # must not raise.

    def test_turn_bails_cleanly_on_permanent_bus_silence(self):
        left = _FakeFlappyMotor()
        right = _FakeFlappyMotor()
        left.drop_calls(range(1, 1000))
        right.drop_calls(range(1, 1000))
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_noop()

        db.turn(90)        # must not raise.

    def test_straight_bails_cleanly_when_anchor_read_keeps_failing(self):
        # Bus is dead from the very first call. ``_read_angle_or_bail``
        # retries 5 times, all None, then returns None. The fallback
        # short-circuits via ``stop()`` rather than entering the inner
        # loop with a bogus reference.
        left = _FakeFlappyMotor()
        right = _FakeFlappyMotor()
        left.go_silent()
        right.go_silent()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_noop()

        db.straight(100)   # must not raise; must stop cleanly.

    def test_turn_bails_cleanly_when_anchor_read_keeps_failing(self):
        left = _FakeFlappyMotor()
        right = _FakeFlappyMotor()
        left.go_silent()
        right.go_silent()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_noop()

        db.turn(90)        # must not raise; must stop cleanly.


class TestDriveBaseWaitFalse(unittest.TestCase):
    """Pybricks-style ``wait=False`` + ``done()`` for concurrent
    drivebase use. Each ``done()`` call runs one tick of the
    fallback state machine — so polling cadence drives progress."""

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

    def test_done_returns_true_when_no_move_pending(self):
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self.assertTrue(db.done())

    def test_straight_wait_false_returns_immediately_then_converges_via_done(self):
        # ``straight(wait=False)`` should arm state and return without
        # blocking. The caller drives the move by polling ``done()``
        # until it returns True. Convergence behaviour should match
        # ``wait=True``.
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.straight(100, wait=False)
        # Just-armed: motors haven't moved yet.
        self.assertFalse(db.done())
        # Poll done() in a tight loop (with sleep to advance the
        # patched motor angle). Same cadence as the implicit
        # wait=True loop.
        for _ in range(5000):
            if db.done():
                break
            time.sleep_ms(10)

        target = _wheel_deg_for_distance(100, wheel_diameter_mm=56)
        avg = (left.angle() + right.angle()) / 2
        self.assertGreaterEqual(avg, target)
        self.assertTrue(db.done())   # remains True after completion

    def test_turn_wait_false_converges_via_done(self):
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.turn(90, wait=False)
        for _ in range(5000):
            if db.done():
                break
            time.sleep_ms(10)

        arc_mm = math.radians(90) * (114 / 2)
        expected = arc_mm / (math.pi * 56) * 360
        self.assertLessEqual(left.angle(), -expected + 5)
        self.assertGreaterEqual(right.angle(), expected - 5)
        self.assertTrue(db.done())

    def test_new_move_supersedes_pending_wait_false(self):
        # Pybricks "new command wins": calling straight() while a
        # wait=False turn is in flight abandons the turn and arms
        # the straight. ``done()`` reflects the new move's state.
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)

        db.turn(90, wait=False)
        self.assertFalse(db.done())
        db.straight(100, wait=False)
        # Still not done (now a fresh straight is pending).
        self.assertFalse(db.done())
        for _ in range(5000):
            if db.done():
                break
            time.sleep_ms(10)
        self.assertTrue(db.done())

    def test_stop_clears_pending_wait_false_move(self):
        # Explicit ``stop()`` aborts the pending move and ``done()``
        # then reports completion immediately.
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.straight(1000, wait=False)
        self.assertFalse(db.done())
        db.stop()
        self.assertTrue(db.done())

    def test_straight_wait_false_kicks_off_motors_immediately(self):
        # Regression for 1.6.9 hub-yt symptom "the whole function is
        # skipped": pre-1.6.10 the fallback ``_arm_straight`` only
        # snapshotted anchors and stashed _pending, leaving motors
        # idle until the first ``done()`` poll. From the caller's
        # perspective, ``straight(wait=False)`` did nothing visible.
        # Post-fix the motors are commanded to cruise speed on the
        # call itself; subsequent ``done()`` ticks apply heading-hold
        # corrections and check the target.
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.settings(straight_speed=200, turn_rate=180)
        db.straight(100, wait=False)

        # No done() polled yet — but both motors should already have
        # received a non-zero speed command.
        self.assertNotEqual(left._target_dps, 0.0)
        self.assertNotEqual(right._target_dps, 0.0)
        # Forward straight = both wheels same sign.
        self.assertGreater(left._target_dps, 0)
        self.assertGreater(right._target_dps, 0)

    def test_turn_wait_false_kicks_off_motors_immediately(self):
        # Same regression as test_straight_wait_false_kicks_off, but
        # for turn(). Turn in place = wheels run opposite signs.
        left = _FakeClosedLoopMotor()
        right = _FakeClosedLoopMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)

        db.settings(straight_speed=200, turn_rate=180)
        db.turn(90, wait=False)

        self.assertNotEqual(left._target_dps, 0.0)
        self.assertNotEqual(right._target_dps, 0.0)
        # Positive turn rate (= turn left) means left reverses,
        # right advances.
        self.assertLess(left._target_dps, 0)
        self.assertGreater(right._target_dps, 0)


if __name__ == "__main__":
    unittest.main()
