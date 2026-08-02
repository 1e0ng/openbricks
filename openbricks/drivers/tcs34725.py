# SPDX-License-Identifier: MIT
"""
AMS TCS34725 RGB + clear light-to-digital sensor.

The TCS34725 returns four 16-bit channels (clear, red, green, blue) over I2C
at address 0x29. There's an onboard LED that we leave under user control — some
breakout boards wire it to the LED pin on reset, others require GPIO control.

Reference: TCS34725 datasheet (AMS / ams-OSRAM), sections 2.4 and 3.

I2C command byte format (from datasheet):
    bit 7 (CMD)     = 1 (always for command byte)
    bits 6:5 (TYPE) = 01 (auto-increment) or 00 (single)
    bits 4:0 (ADDR) = register address
"""

import time

from openbricks.interfaces import ColorSensor

_ADDR   = 0x29
_CMD    = 0x80
_AUTO   = 0x20  # auto-increment when reading multi-byte

_ENABLE = 0x00
_ATIME  = 0x01
_CONTROL = 0x0F
_ID     = 0x12
_CDATAL = 0x14  # 8 bytes: C, R, G, B (each little-endian u16)

_ENABLE_PON = 0x01
_ENABLE_AEN = 0x02


class TCS34725(ColorSensor):
    """RGB + clear color sensor, fixed at I2C address 0x29.

    Implements the ``ColorSensor`` contract: ``rgbc()`` raw 16-bit
    channels, ``reflection()`` and the calibrated helpers built on
    it. Two or more on one robot need a
    :class:`~openbricks.drivers.tca9548a.TCA9548A` mux (the address
    is not configurable).
    """

    def __init__(self, i2c, address=_ADDR, integration_ms=2.4, gain=16):
        """
        Args:
            integration_ms: integration time, 2.4..614.4 ms in 2.4 ms
                steps. Default 2.4 (chip minimum): low-latency reads
                for control loops — the line-follow PID reads both
                sensors every cycle and its D term needs fresh
                samples, not long averages.
            gain: 1, 4, 16, or 60. Default 16 compensates the short
                integration window (1 cycle -> full scale 1024, so
                low gain floors dark readings to a handful of counts).
        """
        self._i2c = i2c
        self._addr = address

        chip_id = self._read_u8(_ID)
        # 0x44 is TCS34725, 0x4D is TCS34727. Accept both.
        if chip_id not in (0x44, 0x4D):
            raise OSError("TCS34725 not found at 0x%02x (id 0x%02x)" % (address, chip_id))

        # ATIME = 256 - (integration_ms / 2.4). Clamped.
        atime = 256 - int(integration_ms / 2.4)
        if atime < 0:
            atime = 0
        elif atime > 255:
            atime = 255
        self._write_u8(_ATIME, atime)
        # Full-scale ADC count for this integration time (datasheet:
        # MAX COUNT = 1024 x cycles, capped at 65535). At the default
        # 24 ms (10 cycles) the clear channel saturates at 10240 —
        # NOT 65535. Scaling ambient() against 65535 made it top out
        # around 15 and read 0 on any real surface (caught on
        # hardware by examples/line_align.py).
        cycles = 256 - atime
        self._full_scale = min(65535, 1024 * cycles)

        gain_map = {1: 0x00, 4: 0x01, 16: 0x02, 60: 0x03}
        self._write_u8(_CONTROL, gain_map.get(gain, 0x01))

        # Enable power + ADC.
        self._write_u8(_ENABLE, _ENABLE_PON)
        time.sleep_ms(3)
        self._write_u8(_ENABLE, _ENABLE_PON | _ENABLE_AEN)
        # First integration cycle: integration time + margin. int():
        # fractional integration_ms (2.4 is the chip minimum) made
        # this a float, and MicroPython's sleep_ms requires an int
        # (bit the bench the first time integration_ms=2.4 was used).
        time.sleep_ms(int(integration_ms) + 5)

    def raw(self):
        """Return the raw (clear, red, green, blue) 16-bit readings."""
        buf = self._i2c.readfrom_mem(self._addr, _CMD | _AUTO | _CDATAL, 8)
        c = buf[0] | (buf[1] << 8)
        r = buf[2] | (buf[3] << 8)
        g = buf[4] | (buf[5] << 8)
        b = buf[6] | (buf[7] << 8)
        return (c, r, g, b)

    def rgb(self):
        """Return ``(r, g, b)`` scaled to 0..255 using the clear channel.

        Dividing by the clear channel normalizes for ambient brightness, so a
        white object reports roughly (255, 255, 255) at any light level within
        the sensor's range.
        """
        c, r, g, b = self.raw()
        if c == 0:
            return (0, 0, 0)
        return (
            min(255, int(r * 255 / c)),
            min(255, int(g * 255 / c)),
            min(255, int(b * 255 / c)),
        )

    def ambient(self):
        """Return clear-channel brightness scaled to 0..100.

        100 means the clear ADC is saturated *for the configured
        integration time* — 1024 counts per 2.4 ms cycle, capped at
        65535 (datasheet "MAX COUNT"). Scale non-linearly would be
        nicer but keep it simple.
        """
        c, _r, _g, _b = self.raw()
        return min(100, int(c * 100 / self._full_scale))

    # --- low level ---
    def _read_u8(self, reg):
        return self._i2c.readfrom_mem(self._addr, _CMD | reg, 1)[0]

    def _write_u8(self, reg, value):
        self._i2c.writeto_mem(self._addr, _CMD | reg, bytes([value]))
