# SPDX-License-Identifier: MIT
"""Parity tests for the CPython-side MotorProcess.

The shared core (``motor_process_core.c``) backs both the firmware's
``_openbricks_native.motor_process`` (a singleton wrapped by
``motor_process.c``) and this CPython-side ``MotorProcess`` class.
We can't register a C-callback from Python — that's a sibling-C-
module operation — so the parity tests here cover the bits user-
facing code actually exercises: the tick clock, period configuration,
and reset semantics. Future Phase B4/B5 will exercise the C-callback
side end-to-end via servo / drivebase.
"""

import unittest


class MotorProcessParityTests(unittest.TestCase):

    def setUp(self):
        from openbricks_sim import _native
        self.MotorProcess = _native.MotorProcess

    def test_default_period_is_1ms(self):
        mp = self.MotorProcess()
        self.assertEqual(mp.period_ms(), 1)
        self.assertEqual(mp.now_ms(), 0)
        self.assertEqual(mp.count_c(), 0)

    def test_init_accepts_period_ms_kwarg(self):
        mp = self.MotorProcess(period_ms=5)
        self.assertEqual(mp.period_ms(), 5)
        self.assertEqual(mp.now_ms(), 0)

    def test_tick_advances_clock_by_period(self):
        mp = self.MotorProcess(period_ms=3)
        mp.tick()
        self.assertEqual(mp.now_ms(), 3)
        mp.tick()
        self.assertEqual(mp.now_ms(), 6)
        mp.tick()
        self.assertEqual(mp.now_ms(), 9)

    def test_tick_with_default_period(self):
        mp = self.MotorProcess()
        for _ in range(1000):
            mp.tick()
        self.assertEqual(mp.now_ms(), 1000)

    def test_set_period_ms_changes_subsequent_advance(self):
        mp = self.MotorProcess()
        mp.tick()
        self.assertEqual(mp.now_ms(), 1)
        mp.set_period_ms(10)
        mp.tick()
        self.assertEqual(mp.now_ms(), 11)
        mp.set_period_ms(100)
        mp.tick()
        self.assertEqual(mp.now_ms(), 111)

    def test_reset_zeros_clock_and_resets_period(self):
        mp = self.MotorProcess(period_ms=5)
        for _ in range(10):
            mp.tick()
        self.assertEqual(mp.now_ms(), 50)
        mp.reset()
        self.assertEqual(mp.now_ms(), 0)
        self.assertEqual(mp.period_ms(), 1)
        self.assertEqual(mp.count_c(), 0)

    def test_independent_instances(self):
        # Unlike the firmware (singleton tied to a Timer), the CPython
        # side lets you spin up multiple instances. Useful for tests +
        # sim runs that want to step clocks at different rates.
        a = self.MotorProcess(period_ms=1)
        b = self.MotorProcess(period_ms=10)
        a.tick()
        b.tick()
        b.tick()
        self.assertEqual(a.now_ms(), 1)
        self.assertEqual(b.now_ms(), 20)


if __name__ == "__main__":
    unittest.main()
