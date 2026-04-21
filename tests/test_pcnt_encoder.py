# SPDX-License-Identifier: MIT
"""Tests for PCNT-based quadrature encoder driver."""

import tests._fakes            # noqa: F401
import tests._fakes_pcnt        # noqa: F401  (installs fake esp32 module)

import unittest

from tests._fakes_pcnt import _FakePCNT
from openbricks.drivers.pcnt_encoder import PCNTQuadratureEncoder


class PCNTQuadratureEncoderTests(unittest.TestCase):
    def setUp(self):
        _FakePCNT._reset_for_test()

    def test_initial_count_is_zero(self):
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2)
        self.assertEqual(enc.count(), 0)

    def test_configures_both_channels_on_one_unit(self):
        enc = PCNTQuadratureEncoder(pin_a=5, pin_b=6, unit=3)
        self.assertEqual(enc._pcnt.unit, 3)
        self.assertEqual(enc._pcnt.channel, 0)
        self.assertEqual(enc._pcnt_ch1.unit, 3)
        self.assertEqual(enc._pcnt_ch1.channel, 1)

    def test_channel_0_edge_on_a_direction_from_b(self):
        enc = PCNTQuadratureEncoder(pin_a=5, pin_b=6)
        self.assertEqual(enc._pcnt.pin.pin, 5)
        self.assertEqual(enc._pcnt.mode_pin.pin, 6)
        self.assertEqual(enc._pcnt.rising, _FakePCNT.INCREMENT)
        self.assertEqual(enc._pcnt.falling, _FakePCNT.DECREMENT)
        self.assertEqual(enc._pcnt.mode_low, _FakePCNT.NORMAL)
        self.assertEqual(enc._pcnt.mode_high, _FakePCNT.REVERSE)

    def test_channel_1_edge_on_b_direction_from_a_reversed(self):
        enc = PCNTQuadratureEncoder(pin_a=5, pin_b=6)
        self.assertEqual(enc._pcnt_ch1.pin.pin, 6)
        self.assertEqual(enc._pcnt_ch1.mode_pin.pin, 5)
        self.assertEqual(enc._pcnt_ch1.mode_low, _FakePCNT.REVERSE)
        self.assertEqual(enc._pcnt_ch1.mode_high, _FakePCNT.NORMAL)

    def test_counter_starts(self):
        PCNTQuadratureEncoder(pin_a=1, pin_b=2, unit=0)
        self.assertTrue(_FakePCNT._UNITS[0]["started"])

    def test_count_reflects_hardware_value(self):
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2)
        enc._pcnt.value(100)
        self.assertEqual(enc.count(), 100)

    def test_count_tracks_deltas_across_reads(self):
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2)
        enc._pcnt.value(50)
        self.assertEqual(enc.count(), 50)
        enc._pcnt.value(80)
        self.assertEqual(enc.count(), 80)
        enc._pcnt.value(-20)
        self.assertEqual(enc.count(), -20)

    def test_count_handles_positive_wrap(self):
        # Hardware steps from +32000 to -32000 — actually +1534 edges
        # through the +32767 -> -32767 wrap.
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2)
        enc._pcnt.value(32000)
        self.assertEqual(enc.count(), 32000)
        enc._pcnt.value(-32000)
        self.assertEqual(enc.count(), 32000 + 1534)

    def test_count_handles_negative_wrap(self):
        # Hardware steps from -32000 to +32000 — actually -1534 edges.
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2)
        enc._pcnt.value(-32000)
        self.assertEqual(enc.count(), -32000)
        enc._pcnt.value(32000)
        self.assertEqual(enc.count(), -32000 - 1534)

    def test_reset_to_zero(self):
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2)
        enc._pcnt.value(500)
        enc.count()
        enc.reset()
        self.assertEqual(enc.count(), 0)
        self.assertEqual(enc._pcnt.value(), 0)

    def test_reset_to_nonzero(self):
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2)
        enc.reset(1000)
        self.assertEqual(enc.count(), 1000)

    def test_filter_passes_through_to_pcnt(self):
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2, filter=100)
        self.assertEqual(enc._pcnt.filter, 100)

    def test_pins_configured_as_inputs_with_pull_up(self):
        enc = PCNTQuadratureEncoder(pin_a=1, pin_b=2)
        # The Pin fake records mode + pull at construction.
        self.assertEqual(enc._pcnt.pin.mode, "IN")
        self.assertEqual(enc._pcnt.pin.pull, "PULL_UP")
        self.assertEqual(enc._pcnt.mode_pin.mode, "IN")
        self.assertEqual(enc._pcnt.mode_pin.pull, "PULL_UP")


if __name__ == "__main__":
    unittest.main()
