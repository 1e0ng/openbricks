# SPDX-License-Identifier: MIT
"""
ST VL53L0X laser time-of-flight distance sensor.

The VL53L0X talks I2C at default address 0x29. A measurement cycle:

  1. Write 0x01 to ``SYSRANGE_START`` (register 0x00).
  2. Poll ``RESULT_INTERRUPT_STATUS`` (0x13) bit 0 until set
     (typical 33 ms at 33 Hz default rate).
  3. Read ``RESULT_RANGE_STATUS + 10`` (0x14 + 10 = 0x1E) as a
     16-bit big-endian integer — the raw distance in millimetres.
  4. Write 0x01 to ``SYSTEM_INTERRUPT_CLEAR`` (0x0B).

Out of range / blocked targets show as 8190 mm in the raw register;
we surface that as ``-1`` to match the :class:`DistanceSensor`
contract.

This driver implements the minimum register sequence to get usable
single-shot readings on a power-on-default VL53L0X. ST's reference
API ships an extensive calibration / VHV / measurement-budget setup
that improves accuracy on tuned hardware — that's a follow-up
patch when we have boards to validate against. The default-init
ranging here is what most off-the-shelf MicroPython VL53L0X drivers
use, and is good for the WRO use cases (line-of-sight, 30–1500 mm).
"""

import time

from openbricks.distance import DistanceSensor


_DEFAULT_ADDR = 0x29

# Subset of the VL53L0X register map we actually touch.
_REG_IDENTIFICATION_MODEL_ID  = 0xC0
_REG_SYSRANGE_START           = 0x00
_REG_RESULT_INTERRUPT_STATUS  = 0x13
_REG_RESULT_RANGE_STATUS      = 0x14   # +10 = 0x1E for the 16-bit mm value
_REG_SYSTEM_INTERRUPT_CLEAR   = 0x0B

_EXPECTED_MODEL_ID = 0xEE

# ``8190`` mm is the "no return / out of range" sentinel the chip
# writes to RESULT_RANGE when the laser doesn't see a usable reflector.
_NO_RETURN_SENTINEL = 8190


class VL53L0X(DistanceSensor):
    """ST VL53L0X laser time-of-flight distance sensor.

    Args:
        i2c: a ``machine.I2C`` (or compatible) instance.
        address: 7-bit I2C address (default 0x29). XSHUT-strapping
            multiple sensors onto one bus requires assigning each
            a unique address before constructing — outside this
            driver's scope.
        timeout_ms: how long to poll for a measurement to finish.
            Default 200 ms; the chip is typically done in 33–50 ms.
    """

    def __init__(self, i2c, address=_DEFAULT_ADDR, timeout_ms=200):
        self._i2c = i2c
        self._addr = address
        self._timeout_ms = int(timeout_ms)

        chip_id = self._read_u8(_REG_IDENTIFICATION_MODEL_ID)
        if chip_id != _EXPECTED_MODEL_ID:
            raise OSError(
                "VL53L0X not found at 0x%02x (id 0x%02x, expected 0x%02x)" %
                (address, chip_id, _EXPECTED_MODEL_ID))

    def distance_mm(self):
        # Kick off a single-shot measurement.
        self._write_u8(_REG_SYSRANGE_START, 0x01)

        # Poll until measurement done (bit 0 of interrupt status).
        deadline_ms = self._timeout_ms
        polled_ms = 0
        while polled_ms < deadline_ms:
            status = self._read_u8(_REG_RESULT_INTERRUPT_STATUS)
            if status & 0x07:
                break
            time.sleep_ms(2)
            polled_ms += 2
        else:
            return -1

        # Read 16-bit big-endian distance from RESULT_RANGE+10.
        buf = self._i2c.readfrom_mem(
            self._addr, _REG_RESULT_RANGE_STATUS + 10, 2)
        if len(buf) < 2:
            self._clear_interrupt()
            return -1
        mm = (buf[0] << 8) | buf[1]
        self._clear_interrupt()
        if mm <= 0 or mm >= _NO_RETURN_SENTINEL:
            return -1
        return mm

    # ---- low-level I2C helpers ----

    def _read_u8(self, reg):
        buf = self._i2c.readfrom_mem(self._addr, reg, 1)
        if len(buf) < 1:
            return 0
        return buf[0]

    def _write_u8(self, reg, value):
        self._i2c.writeto_mem(self._addr, reg, bytes([value & 0xFF]))

    def _clear_interrupt(self):
        self._write_u8(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
