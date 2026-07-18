# SPDX-License-Identifier: MIT
"""Tests for DriveBase.settings(acceleration=...) — the tunable
trajectory acceleration added because the hardcoded wheel-deg/s²
launch was harsh enough to lift the rear of a real robot.

Kept in its own module (like test_drivebase_native_2dof) so the
motor_process singleton state is process-isolated under tests/run.py.
"""

import tests._fakes  # noqa: F401

import time
import unittest

from machine import Timer

from openbricks._native import motor_process
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics.drivebase import DriveBase


def _reset_all():
    motor_process.reset()
    Timer.reset_for_test()


def _make_motor(in1, in2, pwm, ea, eb):
    return JGB37Motor(
        in1=in1, in2=in2, pwm=pwm,
        encoder_a=ea, encoder_b=eb,
        counts_per_output_rev=1320,
    )


def _install_symmetric_sim(left, right):
    """Integrate each motor's target_dps into its encoder count every
    motor_process tick — same synthetic plant as
    test_drivebase_native_2dof's helper, minus the asymmetry knobs."""
    left_acc = [0.0]
    right_acc = [0.0]
    cpr_over_360 = 1320 / 360.0

    def tick():
        dt_s = motor_process.period_ms() / 1000.0
        left_acc[0]  += left._servo.target_dps()  * dt_s * cpr_over_360
        right_acc[0] += right._servo.target_dps() * dt_s * cpr_over_360
        left._enc.reset(int(left_acc[0]))
        right._enc.reset(int(right_acc[0]))

    motor_process.register(tick)


class _RecordingFallbackMotor:
    """Closed-loop-shaped motor with no ``_servo`` — forces the
    wrapper's fallback path (the serial-bus ST-3215/ST-3032 shape).
    Records every commanded speed; ``step()`` integrates the shaft."""

    def __init__(self):
        self._angle = 0.0
        self._dps = 0.0
        self.commands = []

    def run_speed(self, dps):
        self._dps = float(dps)
        self.commands.append(float(dps))

    def angle(self):
        return self._angle

    def coast(self):
        self._dps = 0.0

    def brake(self):
        self._dps = 0.0

    def step(self, seconds):
        self._angle += self._dps * seconds


def _timed_straight_ms(acceleration):
    """Run a 100 mm straight at 200 dps cruise and return the virtual
    elapsed ms. ``acceleration=None`` keeps the 720 deg/s² default."""
    _reset_all()
    left  = _make_motor(1, 2, 17, 7, 8)
    right = _make_motor(9, 10, 11, 12, 13)
    _install_symmetric_sim(left, right)
    db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
    db.settings(straight_speed=200)
    if acceleration is not None:
        db.settings(acceleration=acceleration)
    t0 = time.ticks_ms()
    db.straight(100)
    return time.ticks_diff(time.ticks_ms(), t0)


class TestDriveBaseAcceleration(unittest.TestCase):
    def setUp(self):
        _reset_all()

    def test_settings_rejects_non_positive_acceleration(self):
        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        with self.assertRaises(ValueError):
            db.settings(acceleration=0)
        with self.assertRaises(ValueError):
            db.settings(acceleration=-90)

    def test_native_set_accel_rejects_non_positive(self):
        # The C binding validates too — the wrapper isn't the only gate.
        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        with self.assertRaises(ValueError):
            db._native.set_accel(0.0)

    # ---- serial-bus fallback path (ST-3215 / ST-3032 shape) ----

    def _patch_sleep_steps_motors(self, *motors):
        original = time.sleep_ms

        def stepped_sleep(ms):
            original(ms)
            for m in motors:
                m.step(ms / 1000.0)

        time.sleep_ms = stepped_sleep
        self.addCleanup(lambda: setattr(time, "sleep_ms", original))

    def _fallback_db(self):
        left  = _RecordingFallbackMotor()
        right = _RecordingFallbackMotor()
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        self._patch_sleep_steps_motors(left, right)
        return db, left, right

    def test_fallback_launch_is_ramped_not_a_step(self):
        # The fallback used to command full cruise speed on the very
        # first tick — an effectively infinite acceleration that
        # pitched real (ST-3032) chassis on launch. Now the first
        # command is the crawl floor and the peak stays within cruise.
        db, left, _right = self._fallback_db()
        db.settings(straight_speed=200, acceleration=100)
        db.straight(100)
        first = abs(left.commands[0])
        peak  = max(abs(c) for c in left.commands)
        self.assertLess(first, 200 * 0.2,
                        "launch command %.1f dps is a cruise step" % first)
        # 100 mm at accel 100 is a triangular profile: peak =
        # sqrt(D*a) = sqrt(204.6*100) ~ 143 dps, below cruise.
        self.assertLess(peak, 200 * 0.9)

    def test_fallback_move_still_reaches_target(self):
        db, left, right = self._fallback_db()
        db.settings(straight_speed=200, acceleration=100)
        db.straight(100)
        import math
        target = 100 / (math.pi * 56) * 360
        avg = (left.angle() + right.angle()) / 2
        self.assertGreaterEqual(avg, target)

    def test_fallback_gentle_accel_does_not_trip_stall_budget(self):
        # accel=20 makes this ~6.4 s of profile time; the old
        # constant-velocity ×4 budget (~3 s) would have raised a
        # spurious stall RuntimeError.
        db, left, right = self._fallback_db()
        db.settings(straight_speed=200, acceleration=20)
        db.straight(100)   # must complete without RuntimeError
        self.assertGreater((left.angle() + right.angle()) / 2, 0)

    def test_fallback_turn_is_ramped_too(self):
        db, left, _right = self._fallback_db()
        db.settings(turn_rate=180, acceleration=100)
        db.turn(90)
        first = abs(left.commands[0])
        self.assertLess(first, 180 * 0.2)

    def test_fallback_default_launch_uses_default_accel(self):
        # No settings() call: the fallback ramps at the same 720
        # deg/s² default as the native path (not the old step).
        db, left, _right = self._fallback_db()
        db.straight(100)
        first = abs(left.commands[0])
        self.assertLess(first, 200 * 0.2)

    def test_lower_acceleration_slows_the_launch(self):
        # 100 mm at 200 dps: with the default 720 deg/s² the
        # trapezoid completes in ~1.3 s; at 90 deg/s² the profile goes
        # triangular and takes ~3.0 s. Virtual clock, deterministic.
        fast_ms = _timed_straight_ms(None)
        slow_ms = _timed_straight_ms(90)
        self.assertGreater(fast_ms, 0)
        self.assertGreater(slow_ms, fast_ms * 1.8)

    def test_acceleration_persists_across_moves(self):
        # settings() is sticky — a second straight() after one gentle
        # move is still gentle (no reset back to the default).
        _reset_all()
        left  = _make_motor(1, 2, 17, 7, 8)
        right = _make_motor(9, 10, 11, 12, 13)
        _install_symmetric_sim(left, right)
        db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
        db.settings(straight_speed=200, acceleration=90)
        t0 = time.ticks_ms()
        db.straight(100)
        first_ms = time.ticks_diff(time.ticks_ms(), t0)
        t0 = time.ticks_ms()
        db.straight(100)
        second_ms = time.ticks_diff(time.ticks_ms(), t0)
        # Both moves ran the gentle profile: within 20% of each other
        # and both far above the ~1.3 s the default profile would take.
        self.assertGreater(first_ms,  2000)
        self.assertGreater(second_ms, 2000)




class DefaultAccelValueTests(unittest.TestCase):
    """The 720 deg/s² default lives in SIX places — the Python
    fallback, native C drivebase, native C servo, two encoder-motor
    drivers, and the simulator — source-grep them all so they can't
    drift apart when someone retunes one. (The sim briefly diverged
    to 720 while its physics couldn't track steep ramps; issue #234's
    geometry + DC-motor-model fixes re-unified it.)"""

    _HOMES = [
        ("openbricks/robotics/drivebase.py", "self._accel_dps2 = 720.0"),
        ("native/user_c_modules/openbricks/drivebase_core.h",
         "OB_DRIVEBASE_DEFAULT_ACCEL_DPS2  720.0"),
        ("native/user_c_modules/openbricks/servo.c",
         "DEFAULT_ACCEL ((mp_float_t)720.0)"),
        ("openbricks/drivers/mg370.py", "accel_dps2=720.0"),
        ("openbricks/drivers/jgb37_520.py", "accel_dps2=720.0"),
        ("tools/openbricks/openbricks_sim/runtime.py",
         "accel: float = 720.0"),
    ]

    def test_all_homes_agree(self):
        here = __file__
        idx = here.rfind("/")
        root = (here[:idx] if idx >= 0 else ".") + "/.."
        for rel, needle in self._HOMES:
            with open(root + "/" + rel) as f:
                src = f.read()
            self.assertTrue(needle in src,
                            "%s: expected %r" % (rel, needle))


if __name__ == "__main__":
    unittest.main()
