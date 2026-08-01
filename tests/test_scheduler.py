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


class WallClockTests(unittest.TestCase):
    """``tick(N)`` — the wall-clocked fire the firmware Timer path
    uses (with ``mp_hal_ticks_ms`` as N).

    Why this exists: machine.Timer callbacks ride micropython
    .schedule's droppable queue, so counting FIRES (the old clock)
    dilated controller time under scheduler starvation — a bench
    981 ms gap advanced the native clock by single-digit ms, slowing
    every trajectory mid-run. Wall-clocking advances by real elapsed
    time regardless of how many ticks were dropped.
    """

    def setUp(self):
        motor_process.reset()
        Timer.reset_for_test()

    def test_plain_tick_advances_by_period(self):
        motor_process.tick()
        self.assertEqual(motor_process.now_ms(), 1)

    def test_first_wall_tick_advances_one_period(self):
        # No previous timestamp to diff against.
        motor_process.tick(1000)
        self.assertEqual(motor_process.now_ms(), 1)

    def test_wall_ticks_advance_by_real_elapsed_time(self):
        motor_process.tick(1000)
        motor_process.tick(1050)
        self.assertEqual(motor_process.now_ms(), 51)   # 1 + 50

    def test_starvation_gap_is_counted_not_dilated(self):
        # THE bug: 20 dropped ticks used to mean 20 lost ms of clock.
        motor_process.tick(1000)
        motor_process.tick(1981)     # one fire after a 981 ms gap
        self.assertEqual(motor_process.now_ms(), 1 + 981)

    def test_same_timestamp_advances_zero(self):
        motor_process.tick(1000)
        motor_process.tick(1000)
        self.assertEqual(motor_process.now_ms(), 1)

    def test_pathological_jump_is_clamped(self):
        motor_process.tick(1000)
        motor_process.tick(10_000_000)
        self.assertEqual(motor_process.now_ms(), 1 + 1000)  # clamp

    def test_ticks_ms_wraparound_is_seamless(self):
        # mp_hal_ticks_ms wraps at 2^32 ms (~49 days of uptime);
        # unsigned subtraction must carry the clock straight through.
        motor_process.tick(2 ** 32 - 5)
        before = motor_process.now_ms()
        motor_process.tick(10)
        self.assertEqual(motor_process.now_ms(), before + 15)

    def test_plain_tick_does_not_disturb_wall_tracking(self):
        motor_process.tick(1000)
        motor_process.tick()          # deterministic interleave: +1
        motor_process.tick(1001)      # wall dt from ITS last (1000)
        self.assertEqual(motor_process.now_ms(), 1 + 1 + 1)

    def test_reset_reprimes_the_wall_tracker(self):
        motor_process.tick(1000)
        motor_process.tick(1500)
        motor_process.reset()
        motor_process.tick(9999)      # first-after-reset: +period
        self.assertEqual(motor_process.now_ms(), 1)

    def test_callbacks_fire_on_wall_ticks_too(self):
        cb = _TickCounter()
        motor_process.register(cb)
        motor_process.tick(1000)
        motor_process.tick(1050)
        self.assertEqual(cb.count, 2)

    def test_wall_clock_mode_is_off_by_default(self):
        # The Timer path must stay deterministic in test/sim
        # environments (fake Timer + fake clock); only the frozen
        # boot.py flips this on, and only on sys.platform == "esp32".
        self.assertFalse(motor_process.wall_clock())

    def test_wall_clock_mode_round_trips(self):
        motor_process.set_wall_clock(True)
        try:
            self.assertTrue(motor_process.wall_clock())
        finally:
            motor_process.set_wall_clock(False)
        self.assertFalse(motor_process.wall_clock())

    def test_wall_clock_mode_survives_reset(self):
        # run_program calls reset() before every program; losing the
        # mode there would silently re-dilate the clock for every run
        # after the first.
        motor_process.set_wall_clock(True)
        try:
            motor_process.reset()
            self.assertTrue(motor_process.wall_clock())
        finally:
            motor_process.set_wall_clock(False)

    def test_hard_tick_unavailable_off_hardware(self):
        # The hard tick is an esp32-port patch; unix and sim builds
        # must report it absent and must NOT expose the probe API —
        # code probing for it decides which dispatch world it's in.
        self.assertFalse(motor_process.hard_tick_available())
        self.assertFalse(hasattr(motor_process, "hard_tick_selftest"))
        self.assertFalse(hasattr(motor_process, "hard_tick_count"))

    def test_boot_py_enables_wall_clock_on_esp32_only(self):
        # The enable site is frozen boot.py, gated on sys.platform.
        with open("native/frozen/boot.py") as f:
            src = f.read()
        self.assertIn('sys.platform == "esp32"', src)
        self.assertIn("set_wall_clock(True)", src)


if __name__ == "__main__":
    unittest.main()
