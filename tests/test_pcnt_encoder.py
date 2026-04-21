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


if __name__ == "__main__":
    unittest.main()
