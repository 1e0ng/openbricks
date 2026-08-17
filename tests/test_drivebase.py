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
        # Pybricks-parity default (reinstated 2.4.0): ``stop()``
        # coasts — both H-bridge inputs low, PWM duty zero, the
        # wheels spin freely.
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
        # Explicit ``then="brake"``: both H-bridge terminals shorted
        # high, actively resisting motion at zero velocity.
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

    def test_curve_accepts_pybricks_keyword_names(self):
        # Parameter-name parity: Pybricks-style keyword calls bind.
        # (The RuntimeError is the open-loop refusal — the kwargs
        # made it through the signature.)
        db, _, _ = self._db()
        with self.assertRaises(RuntimeError):
            db.curve(radius=150, angle=90, then="hold", wait=True)

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


class _SpeedFake(_FakeClosedLoopMotor):
    """Closed-loop fake with a scripted MEASURED-speed sequence for
    stop(wait=True): each speed() call pops the next value; the last
    value repeats."""

    def __init__(self, speeds):
        super().__init__()
        self._speeds = list(speeds)

    def speed(self):
        if len(self._speeds) > 1:
            return self._speeds.pop(0)
        return self._speeds[0]


class StopWaitTests(unittest.TestCase):
    """stop(wait=True) — a deliberate extension beyond Pybricks
    (their stop/brake return immediately): block on MEASURED wheel
    speeds reaching ~0, loud on timeout, refuse without feedback."""

    def _db(self, left, right):
        return DriveBase(left, right, wheel_diameter_mm=56,
                         axle_track_mm=114)

    def test_waits_until_both_wheels_settle(self):
        # Decaying speeds; the wait must consume the fast readings
        # and only return once BOTH wheels read quiet repeatedly.
        left = _SpeedFake([500, 300, 100, 40, 8, 3, 0, 0])
        right = _SpeedFake([480, 280, 90, 30, 5, 2, 0, 0])
        db = self._db(left, right)
        db.stop(wait=True)                 # must not raise, must return
        # Both scripts fully drained past their loud values.
        self.assertEqual(left._speeds, [0])
        self.assertEqual(right._speeds, [0])

    def test_one_fast_wheel_keeps_it_waiting(self):
        left = _SpeedFake([0])
        right = _SpeedFake([500] * 40 + [0])
        db = self._db(left, right)
        db.stop(wait=True)
        self.assertEqual(right._speeds, [0])

    def test_never_settling_raises_loudly(self):
        db = self._db(_SpeedFake([500]), _SpeedFake([500]))
        with self.assertRaises(RuntimeError) as ctx:
            db.stop(wait=True)
        self.assertIn("still moving", str(ctx.exception))

    def test_bus_silence_counts_as_not_settled(self):
        # None = feedback stale; must never satisfy "stopped".
        db = self._db(_SpeedFake([None]), _SpeedFake([None]))
        with self.assertRaises(RuntimeError):
            db.stop(wait=True)

    def test_open_loop_motor_refuses(self):
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = self._db(left, right)
        with self.assertRaises(ValueError):
            db.stop(wait=True)

    def test_default_does_not_poll(self):
        # Pybricks-parity default (reinstated 2.4.0): a bare stop()
        # returns immediately — never-settling fakes prove no poll
        # loop ran.
        left = _SpeedFake([500])
        right = _SpeedFake([500])
        db = self._db(left, right)
        db.stop()                              # returns instantly
        self.assertEqual(left._speeds, [500])  # script untouched

    def test_wait_false_does_not_poll(self):
        # Pybricks-style instant return on request.
        left = _SpeedFake([500])
        right = _SpeedFake([500])
        db = self._db(left, right)
        db.stop(wait=False)                # returns immediately

    def test_open_loop_default_stop_is_instant(self):
        # Open-loop pairs share the instant default — a bare stop()
        # must neither raise nor hang.
        left = L298NMotor(in1=1, in2=2, pwm=17)
        right = L298NMotor(in1=9, in2=10, pwm=11)
        db = self._db(left, right)
        db.stop()                          # returns immediately


if __name__ == "__main__":
    unittest.main()
