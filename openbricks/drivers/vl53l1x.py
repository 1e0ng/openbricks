# SPDX-License-Identifier: MIT
"""
ST VL53L1X laser time-of-flight distance sensor.

Successor to the VL53L0X with 4 m range (vs 2 m on the L0X) and a
different register map. I2C default address 0x29 (same as L0X — XSHUT
strapping needed if both share a bus).

The chip-ID lives at 16-bit register 0x010F and reads back 0xEACC for
a VL53L1X (or 0xEBAA for VL53L4CD, an L1X-pin-compatible variant).

Like the L0X driver this module ships the minimum register sequence
to get usable single-shot ranging on a power-on-default chip. ST's
reference API does an extensive calibration / VHV / SPAD-array
selection pass for tuned operation; that lives behind a separate
``calibrate()`` method we'll add when we have hardware to validate
against.

Measurement cycle:

  1. Write 0x40 to ``SYSTEM_INTERRUPT_CLEAR`` (16-bit reg 0x0086).
  2. Write 0x40 to ``SYSTEM_MODE_START`` (16-bit reg 0x0087) — kicks
     a single-shot ranging.
  3. Poll ``GPIO_TIO_HV_STATUS`` (16-bit reg 0x0031) bit 0 until clear.
  4. Read ``RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0``
     (16-bit reg 0x0096) as a 16-bit big-endian value — distance in mm.
  5. Write 0x01 to ``SYSTEM_INTERRUPT_CLEAR`` (16-bit reg 0x0086).

VL53L1X registers are 16-bit (vs L0X's 8-bit) — addresses go through
two byte-swaps on the I2C wire. The driver wraps that in
``_read_u8_16`` / ``_write_u8_16`` helpers.
"""

import time

from openbricks.distance import DistanceSensor


_DEFAULT_ADDR = 0x29

# 16-bit register addresses.
_REG_IDENTIFICATION_MODEL_ID  = 0x010F
_REG_SYSTEM_INTERRUPT_CLEAR   = 0x0086
_REG_SYSTEM_MODE_START        = 0x0087
_REG_GPIO_TIO_HV_STATUS       = 0x0031
_REG_RESULT_FINAL_RANGE_MM    = 0x0096

_EXPECTED_MODEL_ID = 0xEACC      # VL53L1X
_EXPECTED_MODEL_ID_ALT = 0xEBAA  # VL53L4CD pin-compatible variant


class VL53L1X(DistanceSensor):
    """ST VL53L1X laser time-of-flight distance sensor (4 m range).

    Args:
        i2c: a ``machine.I2C`` (or compatible) instance.
        address: 7-bit I2C address (default 0x29).
        timeout_ms: how long to poll for a measurement to finish.
            Default 200 ms; the chip is typically done in ~50 ms.
    """

    def __init__(self, i2c, address=_DEFAULT_ADDR, timeout_ms=200):
        self._i2c = i2c
        self._addr = address
        self._timeout_ms = int(timeout_ms)

        chip_id = self._read_u16_16(_REG_IDENTIFICATION_MODEL_ID)
        if chip_id not in (_EXPECTED_MODEL_ID, _EXPECTED_MODEL_ID_ALT):
            raise OSError(
                "VL53L1X not found at 0x%02x (id 0x%04x)" %
                (address, chip_id))

    def distance_mm(self):
        # Clear any leftover interrupt + start a single-shot range.
        self._write_u8_16(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write_u8_16(_REG_SYSTEM_MODE_START, 0x40)

        # Poll until measurement done (GPIO_TIO_HV_STATUS bit 0
        # CLEAR — opposite polarity from the L0X).
        polled_ms = 0
        while polled_ms < self._timeout_ms:
            status = self._read_u8_16(_REG_GPIO_TIO_HV_STATUS)
            if (status & 0x01) == 0:
                break
            time.sleep_ms(2)
            polled_ms += 2
        else:
            return -1

        # Read final-range register (16-bit big-endian distance in mm).
        mm = self._read_u16_16(_REG_RESULT_FINAL_RANGE_MM)

        # Clear the data-ready interrupt for the next cycle.
        self._write_u8_16(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)

        if mm <= 0:
            return -1
        return mm

    # ---- low-level I2C helpers (16-bit register addresses) ----

    def _read_u8_16(self, reg):
        # 16-bit reg → two-byte address pushed MSB-first; one-byte read.
        self._i2c.writeto(self._addr, bytes([(reg >> 8) & 0xFF, reg & 0xFF]))
        buf = self._i2c.readfrom(self._addr, 1)
        return buf[0] if buf else 0

    def _read_u16_16(self, reg):
        self._i2c.writeto(self._addr, bytes([(reg >> 8) & 0xFF, reg & 0xFF]))
        buf = self._i2c.readfrom(self._addr, 2)
        if len(buf) < 2:
            return 0
        return (buf[0] << 8) | buf[1]

    def _write_u8_16(self, reg, value):
        self._i2c.writeto(self._addr, bytes(
            [(reg >> 8) & 0xFF, reg & 0xFF, value & 0xFF]))
