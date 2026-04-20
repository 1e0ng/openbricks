# SPDX-License-Identifier: MIT
"""Tests for the cooperative motor scheduler.

The scheduler is always-on, pbio-style: the first call to ``instance()``
starts the timer, and the timer runs for the life of the interpreter.
Motors just flip between "actively controlled" and "idle" by registering
or unregistering their tick callback.

Exit criterion for M1: 10 000 simulated ticks without the scheduler
losing beats or drifting off the nominal 10 ms period.
"""

import tests._fakes  # noqa: F401

import time
import unittest

from machine import Timer  # type: ignore[import-not-found]  # provided by fakes

from openbricks.tools.scheduler import MotorProcess


class _TickCounter:
    def __init__(self):
        self.count = 0

    def __call__(self):
        self.count += 1


class TestMotorProcess(unittest.TestCase):
    def setUp(self):
        MotorProcess.reset()
        Timer.reset_for_test()

    def test_instance_is_singleton(self):
        a = MotorProcess.instance()
        b = MotorProcess.instance()
        self.assertIs(a, b)

    def test_instance_auto_starts_timer(self):
        proc = MotorProcess.instance()
        self.assertTrue(proc.is_running())

    def test_register_unique(self):
        proc = MotorProcess.instance()
        cb = _TickCounter()
        proc.register(cb)
        proc.register(cb)
        proc.tick()
        self.assertEqual(cb.count, 1)

    def test_unregister_silent_if_not_registered(self):
        proc = MotorProcess.instance()
        proc.unregister(lambda: None)  # should not raise

    def test_tick_fires_all_registered(self):
        proc = MotorProcess.instance()
        a, b = _TickCounter(), _TickCounter()
        proc.register(a)
        proc.register(b)
        proc.tick()
        self.assertEqual(a.count, 1)
        self.assertEqual(b.count, 1)

    def test_tick_after_unregister_does_not_fire(self):
        proc = MotorProcess.instance()
        a = _TickCounter()
        proc.register(a)
        proc.unregister(a)
        proc.tick()
        self.assertEqual(a.count, 0)

    def test_register_then_sleep_fires_during_sleep(self):
        """No manual start()/stop() — ``instance()`` already started the
        timer. Registering a callback is all a motor needs to do to get
        ticks."""
        proc = MotorProcess.instance()
        cb = _TickCounter()
        proc.register(cb)
        time.sleep_ms(100)
        self.assertEqual(cb.count, 10)

    def test_unregister_keeps_timer_running(self):
        """Unlike the previous auto-lifecycle design, unregistering the
        last callback does not stop the scheduler. Matches pbio's
        always-on model."""
        proc = MotorProcess.instance()
        cb = _TickCounter()
        proc.register(cb)
        self.assertTrue(proc.is_running())
        proc.unregister(cb)
        self.assertTrue(proc.is_running())

    def test_start_is_idempotent(self):
        proc = MotorProcess.instance()
        # Already running from instance() auto-start.
        proc.start()  # no-op
        self.assertTrue(proc.is_running())

    def test_stop_and_restart(self):
        """``stop()`` / ``start()`` are exposed for the rare power-user
        case — pause the entire scheduler, then resume."""
        proc = MotorProcess.instance()
        proc.stop()
        self.assertFalse(proc.is_running())
        proc.start()
        self.assertTrue(proc.is_running())

    def test_configure_changes_period_while_running(self):
        proc = MotorProcess.instance()
        proc.configure(period_ms=5)
        cb = _TickCounter()
        proc.register(cb)
        time.sleep_ms(100)
        # 5 ms period, 100 ms sleep -> 20 ticks.
        self.assertEqual(cb.count, 20)

    def test_callback_may_unregister_itself(self):
        proc = MotorProcess.instance()
        cb = _TickCounter()

        def oneshot():
            cb()
            proc.unregister(oneshot)

        proc.register(oneshot)
        time.sleep_ms(100)
        self.assertEqual(cb.count, 1)

    def test_10000_tick_soak_no_drift(self):
        """Exit criterion: 10 000 periodic ticks arrive on time with no
        accumulated drift. The virtual clock's deadline walker fires timer
        callbacks *at* each nominal deadline, so a 100 000 ms sleep with a
        10 ms period must yield exactly 10 000 fires."""
        proc = MotorProcess.instance()
        cb = _TickCounter()
        proc.register(cb)
        time.sleep_ms(100_000)
        self.assertEqual(cb.count, 10_000)


if __name__ == "__main__":
    unittest.main()
