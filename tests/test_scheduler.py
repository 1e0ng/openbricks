# SPDX-License-Identifier: MIT
"""Tests for the cooperative motor scheduler.

Exercises ``openbricks._native.motor_process`` — the Python fake
installed by ``tests/_fakes.py`` on desktop, and the C module compiled
into the firmware on device (``native/user_c_modules/openbricks/motor_process.c``).
Both must pass the same assertions.

The scheduler is pbio-style always-on: the first subscription
(``register`` or the internal ``_register_c``) starts the timer, and it
stays running for the life of the interpreter.
"""

import tests._fakes  # noqa: F401

import time
import unittest

from machine import Timer  # type: ignore[import-not-found]

from openbricks._native import motor_process


class _TickCounter:
    def __init__(self):
        self.count = 0

    def __call__(self):
        self.count += 1


class TestMotorProcess(unittest.TestCase):
    def setUp(self):
        motor_process.reset()
        Timer.reset_for_test()

    def test_register_unique(self):
        cb = _TickCounter()
        motor_process.register(cb)
        motor_process.register(cb)
        motor_process.tick()
        self.assertEqual(cb.count, 1)

    def test_unregister_silent_if_not_registered(self):
        motor_process.unregister(lambda: None)

    def test_tick_fires_all_registered(self):
        a, b = _TickCounter(), _TickCounter()
        motor_process.register(a)
        motor_process.register(b)
        motor_process.tick()
        self.assertEqual(a.count, 1)
        self.assertEqual(b.count, 1)

    def test_tick_after_unregister_does_not_fire(self):
        a = _TickCounter()
        motor_process.register(a)
        motor_process.unregister(a)
        motor_process.tick()
        self.assertEqual(a.count, 0)

    def test_register_auto_starts_timer(self):
        """pbio-style: first subscription brings the scheduler online."""
        self.assertFalse(motor_process.is_running())
        motor_process.register(_TickCounter())
        self.assertTrue(motor_process.is_running())

    def test_register_then_sleep_fires_during_sleep(self):
        """At the default 1 ms period, sleeping 100 ms fires 100 ticks."""
        cb = _TickCounter()
        motor_process.register(cb)
        time.sleep_ms(100)
        self.assertEqual(cb.count, 100)

    def test_unregister_keeps_timer_running(self):
        """Unregistering the last callback does NOT stop the scheduler —
        pbio-style always-on. User code never needs start/stop."""
        motor_process.register(_TickCounter())
        self.assertTrue(motor_process.is_running())
        motor_process.unregister(_TickCounter())   # different cb; silent
        # Bring us back to empty.
        cb = _TickCounter()
        motor_process.register(cb)
        motor_process.unregister(cb)
        self.assertTrue(motor_process.is_running())

    def test_start_is_idempotent(self):
        motor_process.start()
        self.assertTrue(motor_process.is_running())
        motor_process.start()
        self.assertTrue(motor_process.is_running())

    def test_stop_before_start_is_silent(self):
        motor_process.stop()

    def test_configure_changes_period(self):
        motor_process.configure(period_ms=5)
        cb = _TickCounter()
        motor_process.register(cb)
        time.sleep_ms(100)
        # 5 ms period, 100 ms sleep -> 20 ticks.
        self.assertEqual(cb.count, 20)

    def test_configure_while_running_restarts_timer(self):
        motor_process.register(_TickCounter())   # auto-starts at 1 ms
        motor_process.configure(period_ms=5)
        self.assertTrue(motor_process.is_running())

    def test_callback_may_unregister_itself(self):
        cb = _TickCounter()

        def oneshot():
            cb()
            motor_process.unregister(oneshot)

        motor_process.register(oneshot)
        time.sleep_ms(100)
        self.assertEqual(cb.count, 1)

    def test_10000_tick_soak_no_drift(self):
        """Exit criterion: 10 000 periodic ticks arrive on time with no
        accumulated drift. Run at 10 ms period for test speed; the C
        version targets 1 ms in production and hits the same contract."""
        motor_process.configure(period_ms=10)
        cb = _TickCounter()
        motor_process.register(cb)
        time.sleep_ms(100_000)
        self.assertEqual(cb.count, 10_000)


if __name__ == "__main__":
    unittest.main()
