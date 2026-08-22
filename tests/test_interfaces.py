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
from openbricks.parameters import Stop


class TestMotorInterface(unittest.TestCase):
    def test_run_is_closed_loop_delegate(self):
        # Pybricks parity: run(speed) is deg/s and delegates to
        # run_speed; on the bare interface that surfaces run_speed's
        # NotImplementedError.
        with self.assertRaises(NotImplementedError):
            Motor().run(50)
        calls = []

        class _M(Motor):
            def run_speed(self, deg_per_s):
                calls.append(deg_per_s)

        _M().run(123)
        self.assertEqual(calls, [123])

    def test_dc_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().dc(50)

    def test_brake_raises_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().brake()

    def test_stop_is_concrete_and_delegates_to_coast(self):
        # Pybricks Motor.stop() semantics: coast to rest by friction.
        # Concrete on the base class so EVERY driver gets it —
        # user code (line_follow's intersection stop) calls it.
        calls = []

        class _M(Motor):
            def coast(self):
                calls.append("coast")

        _M().stop()
        self.assertEqual(calls, ["coast"])

    def test_stop_on_bare_interface_surfaces_coast_not_implemented(self):
        with self.assertRaises(NotImplementedError):
            Motor().stop()

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

    def test_done_returns_true_by_default(self):
        # ``done()`` has a sensible base default rather than raising:
        # drivers that don't support non-blocking moves can be polled
        # safely. A wait=True call is finished before returning to the
        # caller, by definition.
        self.assertTrue(Motor().done())


class _ScriptedMotor(Motor):
    """Records primitive calls; feeds scripted angle/stalled values."""

    def __init__(self, angles=None, stalled_seq=None):
        self.calls = []
        self._angles = list(angles or [0.0])
        self._stalled = list(stalled_seq or [])

    def run_speed(self, deg_per_s):
        self.calls.append(("run_speed", deg_per_s))

    def run_angle(self, deg_per_s, target_angle, wait=True):
        self.calls.append(("run_angle", deg_per_s, target_angle, wait))

    def angle(self):
        return self._angles.pop(0) if len(self._angles) > 1 \
            else self._angles[0]

    def stalled(self):
        return self._stalled.pop(0) if len(self._stalled) > 1 \
            else self._stalled[0]

    def hold(self):
        self.calls.append(("hold",))

    def brake(self):
        self.calls.append(("brake",))

    def coast(self):
        self.calls.append(("coast",))


class TestCompositeManeuvers(unittest.TestCase):
    """Pybricks composite methods, concrete on the Motor base:
    run_time / run_target / run_until_stalled + the then dispatch."""

    def test_run_time_runs_waits_then_holds(self):
        m = _ScriptedMotor()
        m.run_time(200, 500)
        self.assertEqual(m.calls, [("run_speed", 200), ("hold",)])

    def test_run_time_then_flavours(self):
        for then, expect in ((Stop.BRAKE, ("brake",)),
                             (Stop.COAST, ("coast",))):
            m = _ScriptedMotor()
            m.run_time(100, 10, then=then)
            self.assertEqual(m.calls[-1], expect)
        m = _ScriptedMotor()
        m.run_time(100, 10, then=Stop.NONE)
        self.assertEqual(m.calls, [("run_speed", 100)])

    def test_run_time_rejects_bad_then_and_nowait(self):
        m = _ScriptedMotor()
        with self.assertRaises(TypeError):
            m.run_time(100, 10, then="drift")
        with self.assertRaises(NotImplementedError):
            m.run_time(100, 10, wait=False)

    def test_run_target_is_absolute_via_relative_delta(self):
        # At angle 100, target 250 -> run_angle(+150). Pybricks
        # run_target semantics in the reset_angle frame.
        m = _ScriptedMotor(angles=[100.0])
        m.run_target(200, 250)
        self.assertEqual(m.calls,
                         [("run_angle", 200, 150.0, True), ("hold",)])

    def test_run_target_unreadable_angle_raises(self):
        class _NoAngle(_ScriptedMotor):
            def angle(self):
                return None
        with self.assertRaises(OSError):
            _NoAngle().run_target(200, 90)

    def test_run_until_stalled_polls_then_coasts_returns_angle(self):
        m = _ScriptedMotor(angles=[42.0],
                           stalled_seq=[False, False, True])
        got = m.run_until_stalled(150)
        self.assertEqual(m.calls, [("run_speed", 150), ("coast",)])
        self.assertEqual(got, 42.0)

    def test_run_until_stalled_duty_limit_needs_driver_support(self):
        # The base hooks raise: only drivers with a torque-limiting
        # mechanism (the ST serial servos) opt in. The refusal names
        # them, and fires BEFORE any motion command.
        m = _ScriptedMotor(stalled_seq=[True])
        try:
            m.run_until_stalled(150, duty_limit=50)
            self.fail("expected NotImplementedError")
        except NotImplementedError as e:
            self.assertTrue("ST3215" in str(e), e)
        self.assertEqual(m.calls, [])

    def test_duty_limit_pop_base_hook_also_refuses(self):
        # Both base hooks raise, not just push: a driver overriding
        # push but forgetting pop must fail loudly at restore time
        # rather than leave a temporary cap in place.
        m = _ScriptedMotor(stalled_seq=[True])
        try:
            m._duty_limit_pop(987)
            self.fail("expected NotImplementedError")
        except NotImplementedError as e:
            self.assertTrue("ST3215" in str(e), e)

    def test_run_until_stalled_duty_limit_pushes_and_pops(self):
        pushes = []
        pops = []

        class _CappedMotor(_ScriptedMotor):
            def _duty_limit_push(self, duty_limit):
                pushes.append(duty_limit)
                return 987          # driver's restore token
            def _duty_limit_pop(self, restore):
                pops.append(restore)

        m = _CappedMotor(angles=[42.0], stalled_seq=[False, True])
        got = m.run_until_stalled(150, duty_limit=30)
        self.assertEqual(got, 42.0)
        self.assertEqual(pushes, [30])
        self.assertEqual(pops, [987],
                         "pop must receive push's token")

    def test_run_until_stalled_duty_limit_pops_on_error(self):
        # The cap must not outlive the call — a stop-button
        # KeyboardInterrupt or a silent-bus OSError mid-run still
        # restores the previous limit.
        pops = []

        class _BoomMotor(_ScriptedMotor):
            def _duty_limit_push(self, duty_limit):
                return 555
            def _duty_limit_pop(self, restore):
                pops.append(restore)
            def stalled(self):
                raise OSError("bus silent")

        m = _BoomMotor()
        with self.assertRaises(OSError):
            m.run_until_stalled(150, duty_limit=30)
        self.assertEqual(pops, [555])

    def test_speed_load_stalled_default_not_implemented(self):
        for call in (lambda: Motor().speed(),
                     lambda: Motor().load(),
                     lambda: Motor().stalled()):
            with self.assertRaises(NotImplementedError):
                call()


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
