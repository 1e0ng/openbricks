# SPDX-License-Identifier: MIT
"""``straight`` / ``turn`` / ``stop`` command BOTH wheels together.

The rule: a DriveBase move must never reach one motor a Python
statement before the other. Whatever the engine, the transition is
issued by ONE call into C, which touches both wheels before returning.
Dispatching per motor from Python — ``left.coast(); right.coast()`` —
leaves the second wheel driving at its last commanded speed for the
duration of the interpreter gap, and the chassis veers.

Two engines, two mechanisms, same contract:

* **Serial bus** (adopted ST-3032/3215): ``db_stop(mode)`` stages both
  wheels inside one bus-lock critical section, and the planner emits
  them in a single sync-write packet — pinned on the wire in
  ``test_st_drivebase`` (one packet carries both servo ids).
* **Encoder servos** (JGB37/MG370): ``DriveBase.stop(mode)`` /
  ``straight`` / ``turn`` attach or release both bridges inside one
  native call, with no Python bytecode in between.

What's pinned here is that structure — no per-motor Python dispatch
survives on either path — plus the symmetric end state it produces.
The timing property itself (no gap) follows from there being one call;
it isn't separately observable from a single-threaded test.
"""

import tests._fakes  # noqa: F401

import unittest

from machine import Timer

from openbricks._native import motor_process
from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics.drivebase import DriveBase


_DUTY_MAX = 1023        # servo.c DUTY_MAX (10-bit PWM)


def _make_motor(in1, in2, pwm, ea, eb):
    return JGB37Motor(in1=in1, in2=in2, pwm=pwm,
                      encoder_a=ea, encoder_b=eb,
                      counts_per_output_rev=1320)


def _install_wheel_sim(left, right):
    """Integrate each servo's commanded speed into its encoder count,
    so a BLOCKING move can actually arrive (``done`` requires arrival,
    not just profile expiry). Same harness as test_drivebase_native_2dof."""
    acc = [0.0, 0.0]
    cpr_over_360 = 1320 / 360.0

    def tick():
        dt_s = motor_process.period_ms() / 1000.0
        acc[0] += left._servo.target_dps() * dt_s * cpr_over_360
        acc[1] += right._servo.target_dps() * dt_s * cpr_over_360
        left._enc.reset(int(acc[0]))
        right._enc.reset(int(acc[1]))

    motor_process.register(tick)


class _Spy:
    """Counts Python-level calls to a motor method, forwarding to the
    real one. Closure-captured list, not an attribute — MicroPython
    closures have no ``__dict__``."""

    def __init__(self, motor, name):
        self.calls = []
        real = getattr(motor, name)
        calls = self.calls

        def wrapper(*args, **kwargs):
            calls.append(args)
            return real(*args, **kwargs)

        setattr(motor, name, wrapper)


class EncoderPathConcurrencyTests(unittest.TestCase):
    """The native (encoder-servo) engine."""

    def setUp(self):
        motor_process.reset()
        Timer.reset_for_test()
        self.left  = _make_motor(1, 2, 17, 7, 8)
        self.right = _make_motor(9, 10, 11, 12, 13)
        self.db = DriveBase(self.left, self.right,
                            wheel_diameter_mm=56, axle_track_mm=114)

    def _bridge(self, motor):
        return (motor._in1.value(), motor._in2.value(), motor._pwm.duty())

    def test_straight_subscribes_both_servos_without_python_dispatch(self):
        # Arming used to attach the servos as two separate Python
        # ``run_speed(0)`` calls, so the left wheel was already
        # closed-loop holding zero while the right was untouched.
        ls, rs = _Spy(self.left, "run_speed"), _Spy(self.right, "run_speed")
        self.db.straight(100, wait=False)
        self.assertEqual(ls.calls, [])
        self.assertEqual(rs.calls, [])
        # Both wheels live on the 1 kHz tick after the single call.
        self.assertTrue(self.left._servo.is_active())
        self.assertTrue(self.right._servo.is_active())

    def test_turn_subscribes_both_servos_without_python_dispatch(self):
        ls, rs = _Spy(self.left, "run_speed"), _Spy(self.right, "run_speed")
        self.db.turn(90, wait=False)
        self.assertEqual(ls.calls, [])
        self.assertEqual(rs.calls, [])
        self.assertTrue(self.left._servo.is_active())
        self.assertTrue(self.right._servo.is_active())

    def test_stop_coast_releases_both_bridges_in_one_native_call(self):
        lc, rc = _Spy(self.left, "coast"), _Spy(self.right, "coast")
        self.db.straight(100, wait=False)
        self.db.stop()                       # default then="coast"
        self.assertEqual(lc.calls, [])
        self.assertEqual(rc.calls, [])
        # Both detached from the tick, both bridges floating.
        self.assertFalse(self.left._servo.is_active())
        self.assertFalse(self.right._servo.is_active())
        self.assertEqual(self._bridge(self.left), (0, 0, 0))
        self.assertEqual(self._bridge(self.right), (0, 0, 0))

    def test_stop_brake_engages_both_bridges_in_one_native_call(self):
        lb, rb = _Spy(self.left, "brake"), _Spy(self.right, "brake")
        self.db.straight(100, wait=False)
        self.db.stop(then="brake")
        self.assertEqual(lb.calls, [])
        self.assertEqual(rb.calls, [])
        self.assertFalse(self.left._servo.is_active())
        self.assertFalse(self.right._servo.is_active())
        self.assertEqual(self._bridge(self.left), (1, 1, _DUTY_MAX))
        self.assertEqual(self._bridge(self.right), (1, 1, _DUTY_MAX))

    def test_blocking_move_ends_with_both_wheels_released(self):
        # The end-of-move stop rides the same path (done() -> stop),
        # so it must release both wheels the same way.
        _install_wheel_sim(self.left, self.right)
        lc, rc = _Spy(self.left, "coast"), _Spy(self.right, "coast")
        self.db.settings(straight_speed=200)
        self.db.straight(50)
        self.assertEqual(lc.calls, [])
        self.assertEqual(rc.calls, [])
        self.assertEqual(self._bridge(self.left), (0, 0, 0))
        self.assertEqual(self._bridge(self.right), (0, 0, 0))

    def test_move_wheels_commands_both_targets_in_one_native_call(self):
        # The SyncServoGroup replacement on the encoder path: both
        # targets set and both servos subscribed by one call, with no
        # per-motor Python dispatch to stagger them.
        ls, rs = _Spy(self.left, "run_speed"), _Spy(self.right, "run_speed")
        self.db.move_wheels(200, 120)
        self.assertEqual(ls.calls, [])
        self.assertEqual(rs.calls, [])
        self.assertTrue(self.left._servo.is_active())
        self.assertTrue(self.right._servo.is_active())
        self.assertAlmostEqual(self.left._servo.target_dps(), 200.0, places=3)
        self.assertAlmostEqual(self.right._servo.target_dps(), 120.0, places=3)

    def test_move_wheels_supersedes_a_coupled_move(self):
        self.db.straight(500, wait=False)
        self.db.move_wheels(-80, 80)     # spin in place instead
        self.assertAlmostEqual(self.left._servo.target_dps(), -80.0, places=3)
        self.assertAlmostEqual(self.right._servo.target_dps(), 80.0, places=3)
        self.assertTrue(self.db.done())  # the pending move is gone

    def test_move_wheels_is_estop_gated(self):
        from openbricks import estop
        estop.engage()
        try:
            self.db.move_wheels(100, 100)
            self.fail("expected KeyboardInterrupt")
        except KeyboardInterrupt:
            pass
        finally:
            estop.clear()

    def test_stop_then_hold_is_refused_on_encoder_servos(self):
        # There is no native position hold for encoder servos, so
        # ``hold`` takes the per-motor fall-through — and JGB37/MG370
        # don't implement hold() at all. It must fail loudly rather
        # than silently degrade to coast; the controller is halted
        # either way.
        self.db.straight(100, wait=False)
        raised = False
        try:
            self.db.stop(then="hold")
        except (AttributeError, NotImplementedError):
            raised = True
        self.assertTrue(raised, "stop(then='hold') should refuse here")

    def test_drive_still_clears_the_trajectory_without_an_end_state(self):
        # ``drive()`` calls the no-mode stop to drop an in-flight
        # profile before writing its own per-wheel speeds — it must
        # NOT coast the bridges out from under the speeds it is about
        # to command.
        self.db.drive(100, 0)
        self.assertTrue(self.left._servo.is_active())
        self.assertTrue(self.right._servo.is_active())
        self.assertGreater(self.left._servo.target_dps(), 0)
        self.assertGreater(self.right._servo.target_dps(), 0)


class EStopGateSurvivesTests(unittest.TestCase):
    """The e-stop gate used to ride on the ``run_speed(0)`` calls that
    arming no longer makes; it is now checked explicitly at the arm
    site, and must still fire."""

    def setUp(self):
        motor_process.reset()
        Timer.reset_for_test()
        from openbricks import estop
        estop.clear()
        self.estop = estop
        self.left  = _make_motor(1, 2, 17, 7, 8)
        self.right = _make_motor(9, 10, 11, 12, 13)
        self.db = DriveBase(self.left, self.right,
                            wheel_diameter_mm=56, axle_track_mm=114)

    def tearDown(self):
        self.estop.clear()

    def test_engaged_estop_blocks_straight(self):
        self.estop.engage()
        try:
            self.db.straight(100)
            self.fail("expected KeyboardInterrupt")
        except KeyboardInterrupt:
            pass

    def test_engaged_estop_blocks_turn(self):
        self.estop.engage()
        try:
            self.db.turn(90)
            self.fail("expected KeyboardInterrupt")
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    unittest.main()
