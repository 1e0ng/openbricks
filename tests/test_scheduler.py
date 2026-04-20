# SPDX-License-Identifier: MIT
"""Tests for the cooperative motor scheduler.

Exit criterion for M1: 10 000 simulated ticks without the scheduler losing
beats or drifting off the nominal 10 ms period.
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

    def test_start_fires_callbacks_during_sleep(self):
        proc = MotorProcess.instance()
        cb = _TickCounter()
        proc.register(cb)
        proc.start()
        # 10 ms tick period; sleep 100 ms should fire ~10 times.
        time.sleep_ms(100)
        proc.stop()
        self.assertEqual(cb.count, 10)

    def test_start_is_idempotent(self):
        proc = MotorProcess.instance()
        proc.start()
        self.assertTrue(proc.is_running())
        proc.start()  # second call is a no-op
        proc.stop()
        self.assertFalse(proc.is_running())

    def test_stop_before_start_is_silent(self):
        proc = MotorProcess.instance()
        proc.stop()  # should not raise

    def test_configure_changes_period(self):
        proc = MotorProcess.instance()
        proc.configure(period_ms=5)
        cb = _TickCounter()
        proc.register(cb)
        proc.start()
        time.sleep_ms(100)
        proc.stop()
        # 5 ms period, 100 ms sleep -> 20 ticks.
        self.assertEqual(cb.count, 20)

    def test_callback_may_unregister_itself(self):
        proc = MotorProcess.instance()
        cb = _TickCounter()

        def oneshot():
            cb()
            proc.unregister(oneshot)

        proc.register(oneshot)
        proc.start()
        time.sleep_ms(100)
        proc.stop()
        self.assertEqual(cb.count, 1)

    def test_10000_tick_soak_no_drift(self):
        """Exit criterion: 10 000 periodic ticks arrive on time with no
        accumulated drift. The virtual clock's deadline walker fires timer
        callbacks *at* each nominal deadline, so a 100 000 ms sleep with a
        10 ms period must yield exactly 10 000 fires."""
        proc = MotorProcess.instance()
        cb = _TickCounter()
        proc.register(cb)
        proc.start()
        time.sleep_ms(100_000)
        proc.stop()
        self.assertEqual(cb.count, 10_000)


if __name__ == "__main__":
    unittest.main()
