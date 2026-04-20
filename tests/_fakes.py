# SPDX-License-Identifier: MIT
"""
Fake MicroPython hardware primitives for desktop-CPython testing.

Importing this module installs a realistic ``machine`` module into
``sys.modules`` so driver imports succeed without any MCU. ``tests/conftest``
imports this module before any test collection runs, so per-test imports
don't need to repeat the gesture.

The fake ``time`` module uses a virtual clock that ``sleep_ms`` advances and
``ticks_ms`` reads. ``sleep_ms`` walks the clock forward one timer deadline
at a time, firing periodic ``Timer`` callbacks at their scheduled virtual
instant — so a scheduler-driven control loop ticks naturally when user code
sleeps, exactly as it would on an MCU where ``time.sleep_ms`` yields to
timer ISRs.
"""

import sys
import time as _real_time
import types


# ---- virtual clock ----

_virtual_ms = [0]


def _ticks_ms():
    return _virtual_ms[0]


def _ticks_us():
    return _virtual_ms[0] * 1000


def _ticks_diff(a, b):
    return a - b


def _advance_virtual_clock(ms):
    """Advance the clock by ``ms``, firing timer callbacks at their
    scheduled deadlines along the way."""
    target = _virtual_ms[0] + int(ms)
    while True:
        nf = Timer._next_fire_time()
        if nf is None or nf > target:
            _virtual_ms[0] = target
            return
        _virtual_ms[0] = nf
        Timer._fire_due(nf)


def _sleep_ms(ms):
    _advance_virtual_clock(ms)


def _sleep_us(us):
    # Round up so every sleep advances time by at least 1 ms; sub-ms detail
    # doesn't exist in the virtual clock.
    _advance_virtual_clock(max(1, int(us) // 1000))


if not hasattr(_real_time, "ticks_ms"):
    _real_time.ticks_ms = _ticks_ms
    _real_time.ticks_us = _ticks_us
    _real_time.ticks_diff = _ticks_diff
    _real_time.sleep_ms = _sleep_ms
    _real_time.sleep_us = _sleep_us


# ---- machine.* fakes ----


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


class Timer:
    """MicroPython-ish Timer.

    The real ``machine.Timer`` schedules a callback on a hardware timer ISR.
    The fake keeps a class-level list of active timers and integrates with
    the virtual clock: ``sleep_ms`` walks the clock forward through each
    timer's next deadline and fires its callback there, so periodic
    scheduler loops tick deterministically during user sleeps.
    """

    PERIODIC = 1
    ONE_SHOT = 0

    _instances = []

    def __init__(self, timer_id=-1):
        self._id = timer_id
        self._callback = None
        self._period_ms = 0
        self._mode = Timer.PERIODIC
        self._last_fire_ms = 0
        self._active = False
        Timer._instances.append(self)

    def init(self, period=0, mode=None, callback=None):
        self._period_ms = int(period)
        self._mode = mode if mode is not None else Timer.PERIODIC
        self._callback = callback
        self._last_fire_ms = _virtual_ms[0]
        self._active = True

    def deinit(self):
        self._callback = None
        self._active = False

    @classmethod
    def _next_fire_time(cls):
        nf = None
        for t in cls._instances:
            if not t._active or t._period_ms <= 0 or t._callback is None:
                continue
            nft = t._last_fire_ms + t._period_ms
            if nf is None or nft < nf:
                nf = nft
        return nf

    @classmethod
    def _fire_due(cls, until_ms):
        for t in list(cls._instances):
            if not t._active or t._period_ms <= 0 or t._callback is None:
                continue
            while t._active and t._last_fire_ms + t._period_ms <= until_ms:
                t._last_fire_ms += t._period_ms
                cb = t._callback
                if cb is not None:
                    cb(t)
                if t._mode == cls.ONE_SHOT:
                    t._active = False
                    t._callback = None
                    break

    @classmethod
    def reset_for_test(cls):
        """Clear all timer state. Call in setUp of tests that install timers."""
        cls._instances = []


# Install the fake machine module (overwriting any earlier stand-in).
_machine = types.ModuleType("machine")
_machine.Pin = Pin
_machine.PWM = PWM
_machine.I2C = I2C
_machine.UART = UART
_machine.Timer = Timer
sys.modules["machine"] = _machine
