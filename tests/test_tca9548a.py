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


class _FailingI2C:
    """I2C stand-in whose device ops raise ENODEV while mux selects
    succeed (or the reverse). Models the bench failure: line_follow's
    traceback said only ``[Errno 19] ENODEV`` and telling 'mux dead'
    apart from 'sensor dead' took a whole diagnostic session."""

    def __init__(self, mux_ok=True, device_ok=False):
        self._mux_ok = mux_ok
        self._device_ok = device_ok
        self.writes = []

    def writeto(self, addr, buf):
        ok = self._mux_ok if addr == 0x70 else self._device_ok
        if not ok:
            raise OSError(19)
        self.writes.append((addr, bytes(buf)))

    def readfrom_mem(self, addr, reg, nbytes):
        if not self._device_ok:
            raise OSError(19)
        return bytes(nbytes)

    def writeto_mem(self, addr, reg, data):
        if not self._device_ok:
            raise OSError(19)

    def readfrom(self, addr, nbytes):
        if not self._device_ok:
            raise OSError(19)
        return bytes(nbytes)


class ErrorContextTests(unittest.TestCase):
    """ENODEV must name the channel and address that failed."""

    def _device_error(self, fn):
        try:
            fn()
        except OSError as e:
            return e
        self.fail("expected OSError")

    def test_device_failure_names_channel_and_address(self):
        ch1 = TCA9548A(_FailingI2C())[1]
        e = self._device_error(
            lambda: ch1.readfrom_mem(0x29, 0x12, 1))
        self.assertEqual(e.args[0], 19)         # errno preserved
        self.assertIn("0x29", str(e))
        self.assertIn("channel 1", str(e))

    def test_every_device_op_carries_the_context(self):
        ch3 = TCA9548A(_FailingI2C())[3]
        for fn in (lambda: ch3.readfrom_mem(0x29, 0, 1),
                   lambda: ch3.writeto_mem(0x29, 0, b"\x00"),
                   lambda: ch3.readfrom(0x29, 1),
                   lambda: ch3.writeto(0x29, b"\x00")):
            e = self._device_error(fn)
            self.assertIn("channel 3", str(e))

    def test_mux_failure_is_named_as_the_mux_not_the_device(self):
        # The other half of the ambiguity: mux itself absent.
        ch0 = TCA9548A(_FailingI2C(mux_ok=False))[0]
        e = self._device_error(lambda: ch0.readfrom(0x29, 1))
        self.assertIn("mux at 0x70", str(e))
        # (assertFalse: MP's unittest has no assertNotIn)
        self.assertFalse("no ACK from device" in str(e))

    def test_successful_ops_are_untouched(self):
        i2c = _FailingI2C(device_ok=True)
        ch0 = TCA9548A(i2c)[0]
        self.assertEqual(ch0.readfrom_mem(0x29, 0x12, 2), b"\x00\x00")


class SelectCacheInvalidationTests(unittest.TestCase):
    """A select whose write RAISED must not leave the cache claiming
    the old channel — the mux's real register state is unknown (reset,
    half-transaction), and a stale cache makes the next select skip
    its write and silently address the wrong channel."""

    def test_failed_select_invalidates_the_cache(self):
        i2c = _FailingI2C(device_ok=True)
        mux = TCA9548A(i2c)
        mux.select(2)                       # cached: channel 2
        i2c._mux_ok = False
        with self.assertRaises(OSError):
            mux.select(5)                   # fails; cache must clear
        i2c._mux_ok = True
        mux.select(2)                       # same as first select...
        # ...but must WRITE again: the failed attempt voided the cache.
        self.assertEqual(
            [w for w in i2c.writes if w[0] == 0x70],
            [(0x70, bytes([0x04])), (0x70, bytes([0x04]))])


if __name__ == "__main__":
    unittest.main()
