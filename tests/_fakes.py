# SPDX-License-Identifier: MIT
"""
Fake MicroPython hardware primitives for desktop-CPython testing.

Importing this module installs a realistic ``machine`` module into
``sys.modules`` so driver imports succeed without any MCU. Every test file
that touches a driver should ``import tests._fakes  # noqa: F401`` at the
very top, before any ``openbricks.*`` imports.
"""

import sys
import time as _real_time
import types


# ---- MicroPython ``time`` shims ----
# MicroPython exposes ``ticks_ms`` / ``ticks_us`` / ``ticks_diff`` / ``sleep_ms``
# / ``sleep_us`` on the ``time`` module. CPython doesn't. We monkey-patch the
# stdlib module so that driver code written against the MicroPython API can
# run under the test suite without modification. Sleeps are no-ops to keep
# tests fast.

if not hasattr(_real_time, "ticks_ms"):
    # Virtual clock: sleep_ms advances it, ticks_ms reads it. This makes
    # timeout-polling loops in drivers (e.g. st3215) deterministic and fast,
    # and keeps jgb37_520's ``_measure_speed_dps`` returning 0 when no time
    # has passed, which is what the tests expect.
    _virtual_ms = [0]

    def _ticks_ms():
        return _virtual_ms[0]

    def _ticks_us():
        return _virtual_ms[0] * 1000

    def _ticks_diff(a, b):
        return a - b

    def _sleep_ms(ms):
        _virtual_ms[0] += int(ms)

    def _sleep_us(us):
        # Round up so every sleep makes measurable progress.
        _virtual_ms[0] += max(1, int(us) // 1000)

    _real_time.ticks_ms = _ticks_ms
    _real_time.ticks_us = _ticks_us
    _real_time.ticks_diff = _ticks_diff
    _real_time.sleep_ms = _sleep_ms
    _real_time.sleep_us = _sleep_us


class Pin:
    OUT = "OUT"
    IN = "IN"
    PULL_UP = "PULL_UP"
    IRQ_RISING = 1
    IRQ_FALLING = 2

    def __init__(self, pin, mode=None, pull=None, value=0):
        self.pin = pin
        self.mode = mode
        self._value = value
        self._irq_handler = None

    def value(self, v=None):
        if v is None:
            return self._value
        self._value = int(bool(v))

    def irq(self, trigger=None, handler=None):
        self._irq_handler = handler


class PWM:
    def __init__(self, pin, freq=1000, duty=0):
        self.pin = pin
        self._freq = freq
        self._duty = duty

    def duty(self, v=None):
        if v is None:
            return self._duty
        self._duty = v

    def freq(self, v=None):
        if v is None:
            return self._freq
        self._freq = v


class I2C:
    def __init__(self, bus_id, sda=None, scl=None, freq=100_000):
        self.bus_id = bus_id
        self.sda = sda
        self.scl = scl
        self.freq = freq
        # Minimal in-memory register bank: {addr: {reg: bytes}}.
        self._regs = {}

    def readfrom_mem(self, addr, reg, n):
        return self._regs.get(addr, {}).get(reg, b"\x00" * n)

    def writeto_mem(self, addr, reg, data):
        self._regs.setdefault(addr, {})[reg] = bytes(data)


class UART:
    def __init__(self, bus_id, baudrate=9600, tx=None, rx=None, timeout=0):
        self.bus_id = bus_id
        self.baudrate = baudrate
        self.tx = tx
        self.rx = rx
        self.timeout = timeout
        self._rx_buf = b""
        self._tx_log = []

    def write(self, data):
        self._tx_log.append(bytes(data))
        return len(data)

    def read(self, n=None):
        if not self._rx_buf:
            return None
        if n is None or n >= len(self._rx_buf):
            chunk, self._rx_buf = self._rx_buf, b""
            return chunk
        chunk, self._rx_buf = self._rx_buf[:n], self._rx_buf[n:]
        return chunk


# Install (overwrite any previous fake, including MagicMock stand-ins).
_machine = types.ModuleType("machine")
_machine.Pin = Pin
_machine.PWM = PWM
_machine.I2C = I2C
_machine.UART = UART
sys.modules["machine"] = _machine
