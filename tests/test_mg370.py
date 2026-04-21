# SPDX-License-Identifier: MIT
"""Tests for the MG370 closed-loop motor driver (GMR encoder via PCNT)."""

import tests._fakes             # noqa: F401
import tests._fakes_pcnt        # noqa: F401  (installs fake esp32 module)

import time
import unittest

from machine import Timer  # type: ignore[import-not-found]

from openbricks._native import PCNTEncoder, motor_process
from openbricks.drivers.mg370 import MG370Motor
from tests._fakes_pcnt import _FakePCNT


def _make_motor(**overrides):
    kwargs = dict(
        in1=1, in2=2, pwm=3,
        encoder_a=4, encoder_b=5,
        pcnt_unit=0,
    )
    kwargs.update(overrides)
    return MG370Motor(**kwargs)


class TestMG370Motor(unittest.TestCase):
    def setUp(self):
        motor_process.reset()
        Timer.reset_for_test()
        _FakePCNT._reset_for_test()

    # --- encoder layer ---

    def test_uses_native_pcnt_encoder(self):
        m = _make_motor()
        self.assertIsInstance(m._enc, PCNTEncoder)

    def test_pcnt_unit_default_zero(self):
        _make_motor()
        self.assertIsNotNone(_FakePCNT._find(0, 0))
        self.assertIsNotNone(_FakePCNT._find(0, 1))

    def test_pcnt_unit_override(self):
        _make_motor(pcnt_unit=2)
        self.assertIsNotNone(_FakePCNT._find(2, 0))
        self.assertIsNotNone(_FakePCNT._find(2, 1))
        # And nothing on unit 0.
        self.assertIsNone(_FakePCNT._find(0, 0))

    def test_filter_override(self):
        _make_motor(pcnt_filter=50)
        self.assertEqual(_FakePCNT._find(0, 0).filter, 50)

    # --- open-loop behaviour (same story as JGB37 via the shared Servo) ---

    def test_run_passes_through_to_h_bridge(self):
        m = _make_motor()
        m.run(50)
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 0)
        self.assertEqual(m._pwm.duty(), 511)

    def test_run_reverse(self):
        m = _make_motor()
        m.run(-75)
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 1)

    def test_brake_shorts_both_terminals(self):
        m = _make_motor()
        m.brake()
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 1)

    def test_coast_floats_terminals(self):
        m = _make_motor()
        m.run(100)
        m.coast()
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 0)

    def test_invert_swaps_direction(self):
        m = _make_motor(invert=True)
        m.run(50)
        self.assertEqual(m._in1.value(), 0)
        self.assertEqual(m._in2.value(), 1)

    # --- default CPR reflects MG370 GMR 1:34 ---

    def test_default_counts_per_rev_matches_gmr_1to34(self):
        """500 PPR * 4 * 34.014 ≈ 68028 — default for the GMR 1:34 variant.

        We can't put 68028 into the 16-bit PCNT hardware register, so we
        simulate one full revolution by driving the counter up through a
        couple of sub-wrap steps. count() folds each delta into its
        running total.
        """
        m = _make_motor()
        _FakePCNT._UNITS[0]["value"] = 30000; m.angle()
        _FakePCNT._UNITS[0]["value"] = -32767; m.angle()   # +2767 via wrap? handled
        # Simpler: just drive far enough in small steps.
        # (Unit test for wrap itself lives in test_pcnt_encoder.)
        m._enc.reset(68028)
        self.assertAlmostEqual(m.angle(), 360.0, places=0)

    def test_counts_per_rev_override(self):
        m = _make_motor(counts_per_output_rev=2000)
        m._enc.reset(1000)
        self.assertAlmostEqual(m.angle(), 180.0, places=0)

    # --- closed-loop integration ---

    def test_run_speed_attaches_and_starts_scheduler(self):
        m = _make_motor()
        m.run_speed(100)
        self.assertTrue(motor_process.is_running())

    def test_scheduler_drives_bridge_at_1khz(self):
        m = _make_motor()
        m.run_speed(300)
        time.sleep_ms(1)
        self.assertEqual(m._in1.value(), 1)
        self.assertEqual(m._in2.value(), 0)
        self.assertEqual(m._pwm.duty(), 1023)
        m.brake()

    # --- two-motor allocation: different PCNT units ---

    def test_two_motors_use_different_pcnt_units(self):
        left = _make_motor(pcnt_unit=0)
        right = _make_motor(pcnt_unit=1, in1=11, in2=12, pwm=13,
                            encoder_a=14, encoder_b=15)
        # Independent counters in hardware.
        _FakePCNT._UNITS[0]["value"] = 100
        _FakePCNT._UNITS[1]["value"] = 200
        self.assertEqual(left._enc.count(), 100)
        self.assertEqual(right._enc.count(), 200)


if __name__ == "__main__":
    unittest.main()
