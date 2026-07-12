# SPDX-License-Identifier: MIT
"""openbricks.estop — the emergency-stop latch.

The stop button's Achilles heel was that killing the program rode on
``mp_sched_keyboard_interrupt``, which raises in whatever main-thread
frame happens to execute — scheduled callbacks eat it. The latch
decouples the robot's halt from that lottery: engage kills motors
directly and makes every motion command raise synchronously in the
caller's own frame.
"""

import tests._fakes  # noqa: F401

import unittest

from openbricks import estop
from openbricks.drivers.l298n import L298NMotor
from openbricks.drivers.st3215 import (
    ST3215, ST3215Motor, SyncServoGroup, _REG_TORQUE, _BROADCAST_ID)


class _GateCase(unittest.TestCase):
    def setUp(self):
        estop.clear()

    def tearDown(self):
        estop.clear()


class LatchTests(_GateCase):
    def test_disengaged_by_default_and_check_passes(self):
        self.assertFalse(estop.is_engaged())
        estop.check()   # must not raise

    def test_engage_latches_and_check_raises(self):
        estop.engage()
        self.assertTrue(estop.is_engaged())
        try:
            estop.check()
        except KeyboardInterrupt:
            pass
        else:
            self.fail("check() must raise KeyboardInterrupt while engaged")

    def test_clear_releases(self):
        estop.engage()
        estop.clear()
        self.assertFalse(estop.is_engaged())
        estop.check()   # must not raise

    def test_engage_is_idempotent(self):
        estop.engage()
        estop.engage()
        self.assertTrue(estop.is_engaged())


class DriverGatingTests(_GateCase):
    """Every motion command raises while engaged; stopping commands
    (brake / coast) stay allowed — they're always safe."""

    def _expect_ki(self, fn):
        try:
            fn()
        except KeyboardInterrupt:
            return
        self.fail("motion command must raise KeyboardInterrupt "
                  "while the e-stop is engaged")

    def test_l298n_run_gated_but_brake_coast_allowed(self):
        m = L298NMotor(in1=1, in2=2, pwm=17)
        estop.engage()
        self._expect_ki(lambda: m.run(50))
        m.brake()   # must not raise
        m.coast()   # must not raise

    def test_st3215_motor_motion_gated(self):
        m = ST3215Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        estop.engage()
        self._expect_ki(lambda: m.run_speed(100))
        self._expect_ki(lambda: m.run(50))
        m.brake()   # allowed
        m.coast()   # allowed

    def test_st3215_position_servo_move_gated(self):
        s = ST3215(servo_id=1, uart_id=1, tx=14, rx=6)
        estop.engage()
        self._expect_ki(lambda: s.move_to(90, wait=False))

    def test_sync_group_gated(self):
        a = ST3215Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        b = ST3215Motor(servo_id=2, uart_id=1, tx=14, rx=6)
        group = SyncServoGroup([a, b])
        estop.engage()
        self._expect_ki(lambda: group.set_goal_speeds([100, 100]))

    def test_commands_work_again_after_clear(self):
        m = L298NMotor(in1=1, in2=2, pwm=17)
        estop.engage()
        estop.clear()
        m.run(50)   # must not raise


class _RecordingBus:
    def __init__(self):
        self.writes = []

    def write(self, servo_id, reg, data):
        self.writes.append((servo_id, reg, bytes(data)))


class EngageKillsMotorsTests(_GateCase):
    def test_engage_broadcasts_torque_off_on_every_bus(self):
        bus_a, bus_b = _RecordingBus(), _RecordingBus()
        saved = dict(ST3215._buses)
        ST3215._buses.clear()
        ST3215._buses["a"] = bus_a
        ST3215._buses["b"] = bus_b
        try:
            estop.engage()
        finally:
            ST3215._buses.clear()
            ST3215._buses.update(saved)
        for bus in (bus_a, bus_b):
            self.assertTrue(
                (_BROADCAST_ID, _REG_TORQUE, b"\x00") in bus.writes,
                "engage() must broadcast torque-off; got %r" % bus.writes)

    def test_engage_survives_a_wedged_bus(self):
        class _WedgedBus:
            def write(self, *_a):
                raise OSError("bus dead")
        good = _RecordingBus()
        saved = dict(ST3215._buses)
        ST3215._buses.clear()
        ST3215._buses["dead"] = _WedgedBus()
        ST3215._buses["good"] = good
        try:
            estop.engage()   # must not raise
        finally:
            ST3215._buses.clear()
            ST3215._buses.update(saved)
        self.assertTrue(
            (_BROADCAST_ID, _REG_TORQUE, b"\x00") in good.writes,
            "one wedged bus must not stop the others from stopping")


def tearDownModule():
    # CPython 3.11/3.12 quirk: a KeyboardInterrupt that escapes an
    # ``exec()`` marks the interpreter as interrupt-killed (process
    # exit code 130) even when the caller catches it — fixed in later
    # CPython, absent on MicroPython. Several tests here exec
    # programs that raise KeyboardInterrupt (the stop-button paths);
    # whether the suite's exit code survived used to depend on test
    # ORDER (a later clean exec resets the flag). One deliberate
    # clean exec at module teardown makes it deterministic.
    exec("pass")


if __name__ == "__main__":
    unittest.main()
