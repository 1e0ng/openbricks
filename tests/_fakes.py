# SPDX-License-Identifier: MIT
"""
Fake MicroPython hardware primitives for desktop-CPython testing.

Importing this module (via ``tests/conftest.py``) installs two things
under ``sys.modules`` so that ``openbricks`` drivers and the native
scheduler can run under CPython without an MCU:

* A ``machine`` module with ``Pin`` / ``PWM`` / ``I2C`` / ``UART`` /
  ``Timer`` fakes and MicroPython-style ``time`` shims (``ticks_ms`` /
  ``sleep_ms`` on a virtual clock).
* An ``_openbricks_native`` module providing a ``motor_process``
  singleton — the Python behaviour spec the C implementation in
  ``native/user_c_modules/openbricks/motor_process.c`` must match.
  ``openbricks/_native.py`` re-exports from this module.

The virtual clock's ``sleep_ms`` walks deadline-by-deadline, firing
registered ``Timer`` callbacks at each tick boundary, so scheduler-driven
control integrates naturally with driver code that sleeps.
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


# ---- _openbricks_native fake ----
# Mirrors native/user_c_modules/openbricks/motor_process.c. The C
# implementation is the authoritative version for firmware; this Python
# copy is what desktop unit tests run against. They must stay in sync —
# tests in tests/test_scheduler.py exercise this class, and the on-device
# test suite (M2) will run the same assertions against the C module.


_DEFAULT_PERIOD_MS = 10


class _MotorProcess:
    """Module-level singleton: only one instance is ever exposed, via
    ``_openbricks_native.motor_process``. Users should not instantiate
    directly (the C version hides the type entirely)."""

    def __init__(self, period_ms=_DEFAULT_PERIOD_MS):
        self._period_ms = int(period_ms)
        self._callbacks = []
        self._timer = None

    # --- subscription ---

    def register(self, callback):
        if callback not in self._callbacks:
            self._callbacks.append(callback)

    def unregister(self, callback):
        try:
            self._callbacks.remove(callback)
        except ValueError:
            pass

    # --- lifecycle ---

    def start(self):
        if self._timer is not None:
            return
        self._timer = Timer(-1)
        self._timer.init(
            period=self._period_ms,
            mode=Timer.PERIODIC,
            callback=self._on_tick,
        )

    def stop(self):
        if self._timer is None:
            return
        self._timer.deinit()
        self._timer = None

    def is_running(self):
        return self._timer is not None

    def configure(self, period_ms):
        self._period_ms = int(period_ms)
        if self._timer is not None:
            self.stop()
            self.start()

    # --- tick ---

    def tick(self):
        self._on_tick(None)

    def _on_tick(self, _timer):
        # Snapshot the list so callbacks may unregister themselves.
        for cb in list(self._callbacks):
            cb()

    # --- test helper ---

    def reset(self):
        """Stop the timer, clear all subscribers, restore default period.
        Tests call this in setUp to isolate state. Not part of the C
        firmware API."""
        self.stop()
        self._callbacks = []
        self._period_ms = _DEFAULT_PERIOD_MS


# Install the native module fake.
_openbricks_native = types.ModuleType("_openbricks_native")
_openbricks_native.motor_process = _MotorProcess()
sys.modules["_openbricks_native"] = _openbricks_native
