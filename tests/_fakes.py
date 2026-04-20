# SPDX-License-Identifier: MIT
"""
Hardware + time fakes for running the openbricks test suite against the
**real** C implementation under the unix MicroPython binary.

Tests run under ``native/micropython/ports/unix/build-standard/micropython``
with the ``_openbricks_native`` user_c_module compiled in. They exercise
the C code directly — there is NO Python mirror of the native logic.
The only things this module installs are:

* ``machine`` — a ``Pin`` / ``PWM`` / ``I2C`` / ``UART`` / ``Timer`` stub
  so driver code that does ``from machine import Pin`` on a hardware-less
  test rig has something to bind against.
* A virtual clock that replaces ``time.sleep_ms`` and friends with
  deterministic, instantaneous advancement. Scheduler tests can pump
  arbitrary simulated time without waiting in real seconds; driver
  timeout loops (st3215 RX, bno055 init sleeps) stay deterministic.

Importing this module is a side-effect: it installs the fakes into
``sys.modules["machine"]`` and rewires ``time.*`` on the real ``time``
module. Test modules import it once at the top, before any ``openbricks``
or ``machine`` import.
"""

import sys
import time as _real_time


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


def advance_ms(ms):
    """Advance the virtual clock by ``ms`` and fire any Timer callbacks
    whose deadline falls in that interval. Public so tests can drive
    the scheduler deterministically on any runtime (CPython's ``time``
    is patchable, MicroPython's isn't — but ``advance_ms`` works on
    both)."""
    _advance_virtual_clock(ms)


def run_until_done(thing, timeout_ms=5000):
    """Advance the virtual clock in 1 ms steps until ``thing.is_done()``
    returns True, or ``timeout_ms`` elapses. Used by tests that drive a
    native servo or drivebase move without calling the blocking
    Python-side wrappers (which ``time.sleep_ms`` in a loop — that's
    real time on MicroPython)."""
    for _ in range(timeout_ms):
        if thing.is_done():
            return
        _advance_virtual_clock(1)
    raise AssertionError("timeout waiting for is_done() after %d ms" % timeout_ms)


# Replace the global ``time`` module in sys.modules with a fake that
# uses the virtual clock. Code that does ``import time`` *after*
# tests._fakes loads (i.e. openbricks drivers, JGB37Motor's run_angle
# wait loop, DriveBase.straight's polling loop) gets our fake —
# ``time.sleep_ms`` fires timer callbacks and advances the clock
# deterministically instead of waiting for real time.
#
# This is the key to making tests runnable under the unix MP binary,
# where attribute assignment on the frozen ``time`` module fails but
# replacing the ``sys.modules`` entry does not.


class _FakeTime:
    # Mirror the subset of MicroPython's time API our code uses.
    ticks_ms = staticmethod(_ticks_ms)
    ticks_us = staticmethod(_ticks_us)
    ticks_diff = staticmethod(_ticks_diff)
    sleep_ms = staticmethod(_sleep_ms)
    sleep_us = staticmethod(_sleep_us)
    # Pass-through to the real time module for wall-clock-only helpers
    # we don't need to virtualise.
    time = _real_time.time
    sleep = _real_time.sleep


sys.modules["time"] = _FakeTime


# ---- unittest assertion shims ----
# micropython-lib's unittest is missing ``assertGreater`` / ``assertLess``
# (only the ``Equal`` variants land there). Add them so tests don't have
# to rewrite to the less-readable ``assertGreaterEqual(x, y + 1)`` form.

def _install_unittest_shims():
    try:
        import unittest as _ut
    except ImportError:
        return
    tc = _ut.TestCase

    def assertGreater(self, x, y, msg=None):
        if not (x > y):
            raise AssertionError(msg or ("%r not greater than %r" % (x, y)))

    def assertLess(self, x, y, msg=None):
        if not (x < y):
            raise AssertionError(msg or ("%r not less than %r" % (x, y)))

    if not hasattr(tc, "assertGreater"):
        tc.assertGreater = assertGreater
    if not hasattr(tc, "assertLess"):
        tc.assertLess = assertLess


# Try installing shims now — on CPython ``unittest`` is stdlib. On MP
# the micropython-lib unittest path may not be on sys.path at this
# point, so the install is a no-op and ``tests/run.py`` re-invokes the
# shim installer after extending the path.
try:
    _install_unittest_shims()
except Exception:
    pass


# ---- machine.* hardware fakes ----


class Pin:
    OUT = "OUT"
    IN = "IN"
    PULL_UP = "PULL_UP"
    PULL_DOWN = "PULL_DOWN"
    IRQ_RISING = 1
    IRQ_FALLING = 2

    def __init__(self, pin, mode=None, pull=None, value=0):
        self.pin = pin
        self.mode = mode
        self.pull = pull
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
    """Deterministic machine.Timer stand-in.

    On ESP32 / RP2040 this is a hardware timer ISR. Here it's a
    class-level list that the virtual clock walks through: ``sleep_ms``
    advances the clock through each scheduled deadline and fires the
    callback as it crosses. That gives scheduler tests deterministic
    control over what fires when, without any real-time waiting.
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
        # Also drop ourselves from the instance list so a later
        # ``advance_ms`` doesn't accidentally keep a reference alive and
        # fire the (now-stale) callback. On firmware this is a no-op
        # because machine.Timer owns its own handle; under the fake
        # it's the only way to stop ``_fire_due`` from iterating us.
        try:
            Timer._instances.remove(self)
        except ValueError:
            pass

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
        cls._instances = []


# Install the fake machine module into sys.modules so that
# ``from machine import Pin`` on unix MP (which has no Pin / PWM / I2C
# / UART / Timer in its own machine module) returns our stubs.
# MicroPython doesn't ship a ``types`` stdlib, so we build the module
# object by cloning the ``tests._fakes`` module type and populating its
# attributes. This gives us a real module object (not a class) so
# ``mp_import_name`` and ``mp_load_attr`` behave identically to how
# they would on firmware.


class _FakeMachineModule:
    Pin = Pin
    PWM = PWM
    I2C = I2C
    UART = UART
    Timer = Timer


sys.modules["machine"] = _FakeMachineModule


# ---- ssd1306 fake ----
#
# Our firmware freezes micropython-lib's ``ssd1306`` driver (which itself
# depends on the C-level ``framebuf`` module). The unix MicroPython build
# has neither. Install a recording stub so tests can exercise the
# openbricks wrapper without dragging in framebuf.


class _FakeSSD1306_I2C:
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.width = width
        self.height = height
        self.i2c = i2c
        self.addr = addr
        self.external_vcc = external_vcc
        self.calls = []

    def text(self, s, x, y, c=1):
        self.calls.append(("text", s, x, y, c))

    def pixel(self, x, y, c=None):
        self.calls.append(("pixel", x, y, c))

    def fill(self, c):
        self.calls.append(("fill", c))

    def show(self):
        self.calls.append(("show",))

    def rect(self, x, y, w, h, c):
        self.calls.append(("rect", x, y, w, h, c))

    def contrast(self, c):
        self.calls.append(("contrast", c))


class _FakeSSD1306Module:
    SSD1306_I2C = _FakeSSD1306_I2C


sys.modules["ssd1306"] = _FakeSSD1306Module
