# SPDX-License-Identifier: MIT
"""Tests for the TCA9548A I2C multiplexer driver."""

import tests._fakes  # noqa: F401

import unittest

from machine import I2C  # type: ignore[import-not-found]  # provided by fakes

from openbricks.drivers.tca9548a import TCA9548A


class TestTCA9548A(unittest.TestCase):
    def test_select_writes_channel_bitmask(self):
        i2c = I2C(0)
        TCA9548A(i2c).select(3)
        self.assertEqual(i2c._writes, [(0x70, bytes([0x08]))])

    def test_select_channel_0_is_bit_0(self):
        i2c = I2C(0)
        TCA9548A(i2c).select(0)
        self.assertEqual(i2c._writes, [(0x70, bytes([0x01]))])

    def test_custom_address(self):
        i2c = I2C(0)
        TCA9548A(i2c, address=0x71).select(7)
        self.assertEqual(i2c._writes, [(0x71, bytes([0x80]))])

    def test_select_out_of_range_raises(self):
        mux = TCA9548A(I2C(0))
        with self.assertRaises(ValueError):
            mux.select(8)
        with self.assertRaises(ValueError):
            mux.select(-1)

    def test_disable_writes_zero(self):
        i2c = I2C(0)
        mux = TCA9548A(i2c)
        mux.select(2)
        mux.disable()
        self.assertEqual(i2c._writes[-1], (0x70, bytes([0x00])))

    def test_redundant_select_is_skipped(self):
        i2c = I2C(0)
        mux = TCA9548A(i2c)
        mux.select(1)
        mux.select(1)
        self.assertEqual(len(i2c._writes), 1)

    def test_alternating_select_rewrites_each_time(self):
        i2c = I2C(0)
        mux = TCA9548A(i2c)
        mux.select(1)
        mux.select(2)
        mux.select(1)
        self.assertEqual(len(i2c._writes), 3)

    def test_disable_after_disable_is_skipped(self):
        i2c = I2C(0)
        mux = TCA9548A(i2c)
        mux.select(0)
        mux.disable()
        mux.disable()
        # select + first disable only; the second disable is a no-op.
        self.assertEqual(len(i2c._writes), 2)

    def test_getitem_out_of_range_raises(self):
        mux = TCA9548A(I2C(0))
        with self.assertRaises(ValueError):
            mux[8]

    def test_channel_proxy_selects_then_delegates_write(self):
        i2c = I2C(0)
        mux = TCA9548A(i2c)
        mux[4].writeto_mem(0x29, 0x00, b"\x03")
        # Channel 4 selected (bit 4 = 0x10)...
        self.assertEqual(i2c._writes, [(0x70, bytes([0x10]))])
        # ...and the memory write reached the downstream device.
        self.assertEqual(i2c._regs[0x29][0x00], b"\x03")

    def test_channel_proxy_selects_then_delegates_read(self):
        i2c = I2C(0)
        i2c._regs[0x29] = {0x12: bytes([0x44])}
        mux = TCA9548A(i2c)
        val = mux[5].readfrom_mem(0x29, 0x12, 1)
        self.assertEqual(val, bytes([0x44]))
        self.assertEqual(i2c._writes, [(0x70, bytes([0x20]))])

    def test_two_tcs34725_on_different_channels(self):
        # The real driver constructs unchanged through mux[n]; two
        # 0x29 sensors coexist because each sits on its own channel.
        from openbricks.drivers.tcs34725 import TCS34725

        i2c = I2C(0)
        # Chip-id register (CMD | ID = 0x80 | 0x12) so init succeeds.
        i2c._regs[0x29] = {0x80 | 0x12: bytes([0x44])}
        mux = TCA9548A(i2c)
        TCS34725(mux[0])
        TCS34725(mux[1])
        selects = [w[1] for w in i2c._writes if w[0] == 0x70]
        self.assertIn(bytes([0x01]), selects)  # channel 0 was selected
        self.assertIn(bytes([0x02]), selects)  # channel 1 was selected


if __name__ == "__main__":
    unittest.main()
