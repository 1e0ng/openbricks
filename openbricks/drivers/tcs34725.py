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
    def __init__(self, i2c, address=_ADDR, integration_ms=24, gain=4):
        """
        Args:
            integration_ms: integration time, 2.4..614.4 ms in 2.4 ms steps.
            gain: 1, 4, 16, or 60.
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

        gain_map = {1: 0x00, 4: 0x01, 16: 0x02, 60: 0x03}
        self._write_u8(_CONTROL, gain_map.get(gain, 0x01))

        # Enable power + ADC.
        self._write_u8(_ENABLE, _ENABLE_PON)
        time.sleep_ms(3)
        self._write_u8(_ENABLE, _ENABLE_PON | _ENABLE_AEN)
        # First integration cycle. 2.4 ms + integration time.
        time.sleep_ms(integration_ms + 5)

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
        """Return clear-channel brightness scaled to 0..100."""
        c, _r, _g, _b = self.raw()
        # 65535 is the ADC max; scale non-linearly would be nicer but keep it simple.
        return min(100, int(c * 100 / 65535))

    # --- low level ---
    def _read_u8(self, reg):
        return self._i2c.readfrom_mem(self._addr, _CMD | reg, 1)[0]

    def _write_u8(self, reg, value):
        self._i2c.writeto_mem(self._addr, _CMD | reg, bytes([value]))
