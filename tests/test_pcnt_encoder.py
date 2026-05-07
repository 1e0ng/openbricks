# SPDX-License-Identifier: MIT
"""Tests for the native ``PCNTEncoder`` C type."""

import tests._fakes            # noqa: F401
import tests._fakes_pcnt        # noqa: F401  (installs fake ``esp32`` module)

import unittest

from tests._fakes_pcnt import _FakePCNT
from openbricks._native import PCNTEncoder


def _ch0(unit=0):
    return _FakePCNT._find(unit, 0)


def _ch1(unit=0):
    return _FakePCNT._find(unit, 1)


def _set_hw(unit, v):
    _FakePCNT._UNITS[unit]["value"] = v


class PCNTEncoderConstructorTests(unittest.TestCase):
    def setUp(self):
        _FakePCNT._reset_for_test()

    def test_initial_count_is_zero(self):
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        self.assertEqual(enc.count(), 0)

    def test_builds_two_channels_on_one_unit(self):
        PCNTEncoder(pin_a=5, pin_b=6, unit=3)
        self.assertIsNotNone(_ch0(3))
        self.assertIsNotNone(_ch1(3))
        # No stray units configured.
        self.assertEqual(len(_FakePCNT._INSTANCES), 2)

    def test_channel_0_edge_on_a_direction_from_b(self):
        PCNTEncoder(pin_a=5, pin_b=6)
        ch = _ch0()
        self.assertEqual(ch.pin.pin, 5)
        self.assertEqual(ch.mode_pin.pin, 6)
        self.assertEqual(ch.rising, _FakePCNT.INCREMENT)
        self.assertEqual(ch.falling, _FakePCNT.DECREMENT)
        self.assertEqual(ch.mode_low, _FakePCNT.NORMAL)
        self.assertEqual(ch.mode_high, _FakePCNT.REVERSE)

    def test_channel_1_edge_on_b_direction_from_a_reversed(self):
        PCNTEncoder(pin_a=5, pin_b=6)
        ch = _ch1()
        self.assertEqual(ch.pin.pin, 6)
        self.assertEqual(ch.mode_pin.pin, 5)
        self.assertEqual(ch.mode_low, _FakePCNT.REVERSE)
        self.assertEqual(ch.mode_high, _FakePCNT.NORMAL)

    def test_counter_started(self):
        PCNTEncoder(pin_a=1, pin_b=2, unit=0)
        self.assertTrue(_FakePCNT._UNITS[0]["started"])

    def test_filter_passes_through_to_pcnt(self):
        PCNTEncoder(pin_a=1, pin_b=2, filter=100)
        self.assertEqual(_ch0().filter, 100)

    def test_pins_configured_as_inputs_with_pull_up(self):
        PCNTEncoder(pin_a=1, pin_b=2)
        self.assertEqual(_ch0().pin.mode, "IN")
        self.assertEqual(_ch0().pin.pull, "PULL_UP")
        self.assertEqual(_ch0().mode_pin.mode, "IN")
        self.assertEqual(_ch0().mode_pin.pull, "PULL_UP")


class PCNTEncoderCountTests(unittest.TestCase):
    def setUp(self):
        _FakePCNT._reset_for_test()

    def test_count_reflects_hardware_value(self):
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        _set_hw(0, 100)
        self.assertEqual(enc.count(), 100)

    def test_count_tracks_deltas_across_reads(self):
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        _set_hw(0, 50)
        self.assertEqual(enc.count(), 50)
        _set_hw(0, 80)
        self.assertEqual(enc.count(), 80)
        _set_hw(0, -20)
        self.assertEqual(enc.count(), -20)

    def test_count_handles_positive_wrap(self):
        # 32000 -> -32000 is actually +1534 through the +32767 -> -32767 wrap.
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        _set_hw(0, 32000)
        self.assertEqual(enc.count(), 32000)
        _set_hw(0, -32000)
        self.assertEqual(enc.count(), 32000 + 1534)

    def test_count_handles_negative_wrap(self):
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        _set_hw(0, -32000)
        self.assertEqual(enc.count(), -32000)
        _set_hw(0, 32000)
        self.assertEqual(enc.count(), -32000 - 1534)


class PCNTEncoderResetTests(unittest.TestCase):
    def setUp(self):
        _FakePCNT._reset_for_test()

    def test_reset_to_zero_clears_count_and_hardware(self):
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        _set_hw(0, 500)
        enc.count()
        enc.reset()
        self.assertEqual(enc.count(), 0)
        self.assertEqual(_FakePCNT._UNITS[0]["value"], 0)

    def test_reset_to_nonzero(self):
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        enc.reset(1000)
        self.assertEqual(enc.count(), 1000)

    def test_reset_accepts_int64_value(self):
        # 1.4.0: ``reset()`` widened to int64 so a value previously
        # returned by ``count()`` (which can exceed int32 after a long
        # run) round-trips back through reset() without OverflowError.
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        big = 5_000_000_000   # > int32 max (2.15e9)
        enc.reset(big)
        self.assertEqual(enc.count(), big)


class PCNTEncoderLimitIRQTests(unittest.TestCase):
    """1.4.2 redesign: configure min/max at ±16384 (not threshold0/1)
    so the hardware peripheral itself auto-resets the counter to 0 on
    each crossing AND fires an IRQ. The handler then accounts for the
    known limit value in ``accum`` WITHOUT calling ``pcnt.value()`` —
    avoiding the synchronous-flush-loop reentrancy that hung the chip
    in 1.4.0 and 1.4.1."""

    def setUp(self):
        _FakePCNT._reset_for_test()

    def test_min_max_configured_at_limit(self):
        PCNTEncoder(pin_a=1, pin_b=2)
        ch = _ch0()
        self.assertEqual(ch.min, -16384)
        self.assertEqual(ch.max, +16384)

    def test_threshold0_threshold1_not_used(self):
        # 1.4.0/1.4.1 used threshold0/threshold1 + ``pcnt.value()`` in
        # the handler — that was the buggy design. 1.4.2 uses min/max
        # auto-reset instead, so threshold0/threshold1 must be left
        # unset on the PCNT.
        PCNTEncoder(pin_a=1, pin_b=2)
        ch = _ch0()
        self.assertIsNone(ch.threshold0,
                          "1.4.2 redesign uses min/max auto-reset "
                          "instead of thresholds — threshold0 should "
                          "be left at the PCNT default")
        self.assertIsNone(ch.threshold1)

    def test_irq_is_armed_with_min_max_triggers(self):
        PCNTEncoder(pin_a=1, pin_b=2)
        ch = _ch0()
        self.assertIsNotNone(ch.irq_handler,
                             "PCNTEncoder must register an IRQ handler "
                             "so the auto-reset edges are added to the "
                             "software accumulator")
        # Trigger mask must include both limit events (PCNT.IRQ_MIN |
        # PCNT.IRQ_MAX = 8 | 16 = 24 in the fake).
        self.assertEqual(ch.irq_trigger,
                         _FakePCNT.IRQ_MIN | _FakePCNT.IRQ_MAX)


class PCNTEncoderInt64AccumulatorTests(unittest.TestCase):
    """1.4.0 fix: ``accum`` widened from int32 to int64 so high-PPR
    encoders running at full speed don't overflow the accumulator
    after ~6 hours. Verified by feeding the fake hardware counter
    multiple wraps' worth of edges and reading back the accumulated
    total — must be far past int32 max."""

    def setUp(self):
        _FakePCNT._reset_for_test()

    def test_accumulator_survives_past_int32_max(self):
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        # int32 max is 2_147_483_647. We accumulate well past that by
        # walking the hardware counter forward in 30k-edge steps. Each
        # step is under the half-range threshold so the wrap heuristic
        # in pcnt_update_count correctly adds them up. The synthetic
        # hardware counter wraps from +32767 down to -32767+1 to mimic
        # the real PCNT — note PCNT range is max-min = 65534 (not
        # 65535) since both endpoints are inclusive.
        target = 5_000_000_000   # > 2x int32 max
        step   = 30_000
        PCNT_RANGE = 65534       # = 32767 - (-32767)
        hw = 0
        expected = 0
        while expected < target:
            hw += step
            if hw > 32767:
                hw -= PCNT_RANGE   # mimic hardware wrap to negative side
            expected += step
            _set_hw(0, hw)
            enc.count()
        self.assertEqual(enc.count(), expected)

    def test_count_returns_full_int64_value_to_python(self):
        # Even on a 32-bit MicroPython port, ``count()`` must return a
        # Python int that holds the full 64-bit accumulator without
        # truncation.
        enc = PCNTEncoder(pin_a=1, pin_b=2)
        enc.reset(9_223_372_036_854_775_000)   # near int64 max
        self.assertEqual(enc.count(), 9_223_372_036_854_775_000)


if __name__ == "__main__":
    unittest.main()
