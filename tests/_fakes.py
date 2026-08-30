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
    # As strict as the real MicroPython time.sleep_ms, which requires
    # an int — a fake that accepted floats let a driver ship
    # ``sleep_ms(2.4 + 5)`` that crashed on hardware (tcs34725
    # integration_ms=2.4).
    if not isinstance(ms, int):
        raise TypeError("can't convert %s to int (sleep_ms wants int, "
                        "matching MicroPython)" % type(ms).__name__)
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
    # Wrap-safe deadline arithmetic. The fake clock is a plain int so
    # a sum suffices; on real MicroPython ports raw ``ticks_ms() + n``
    # misbehaves at the 2^30 ms wrap (~12 days uptime), which is why
    # the driver code calls this instead.
    ticks_add = staticmethod(lambda t, d: t + d)
    sleep_ms = staticmethod(_sleep_ms)
    sleep_us = staticmethod(_sleep_us)
    # Pass-through to the real time module for wall-clock-only helpers
    # we don't need to virtualise. ``gmtime`` feeds the epoch-offset
    # detection in ``openbricks.log`` (1970- vs 2000-based epochs);
    # ``time_ns`` feeds its per-line stamps. Under unix MP / CPython
    # both are the real 1970-based clock, exactly what firmware code
    # must handle.
    time = _real_time.time
    sleep = _real_time.sleep
    gmtime = _real_time.gmtime
    time_ns = _real_time.time_ns
    # CPython's unittest runner times each run with
    # ``time.perf_counter()`` — and it sees this fake because the
    # ``sys.modules`` swap below happens before ``unittest.runner``
    # first imports ``time``. Recent CPython point releases moved the
    # call to where every module run hits it, so without this shim
    # every CPython worker dies with AttributeError before running a
    # single test. Pass through to the real clock: it only feeds
    # duration display, so virtualising it would just misreport.
    # MicroPython's time module has no perf_counter — and its
    # unittest never calls it, so the shim only exists where the
    # real attribute does.
    if hasattr(_real_time, "perf_counter"):
        perf_counter = _real_time.perf_counter


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
        # Log of plain (register-less) writes, e.g. a TCA9548A channel
        # select: list of (addr, bytes) in call order.
        self._writes = []

    def readfrom_mem(self, addr, reg, n):
        return self._regs.get(addr, {}).get(reg, b"\x00" * n)

    def writeto_mem(self, addr, reg, data):
        self._regs.setdefault(addr, {})[reg] = bytes(data)

    def writeto(self, addr, buf, stop=True):
        self._writes.append((addr, bytes(buf)))
        return len(buf)


class UART:
    def deinit(self):
        # Release the (fake) driver — the native adoption path calls
        # this to hand the pins to the IDF driver.
        self._deinited = True

    def __init__(self, bus_id, baudrate=9600, tx=None, rx=None, timeout=0):
        self.bus_id = bus_id
        self.baudrate = baudrate
        self.tx = tx
        self.rx = rx
        self.timeout = timeout
        self._rx_buf = b""
        self._tx_log = []
        # Per-servo register state, kept from observed writes so
        # read-backs can answer truthfully: {(servo_id, reg): value}.
        # A test that wants a servo to ACK a write WITHOUT applying it
        # (the cold-boot EEPROM drop) overrides ``_apply_write``.
        self._regs = {}

    def write(self, data):
        self._tx_log.append(bytes(data))
        self._ack_scs_write(bytes(data))
        self._answer_scs_read(bytes(data))
        return len(data)

    def _ack_scs_write(self, packet):
        """Answer an SCS/STS register write with a status packet, the
        way a real servo does.

        The driver confirms every write against this reply — a write
        that vanishes is never allowed to look like one that landed
        (goal-speed 0 means FULL SPEED on this hardware, so a lost
        speed write is a runaway, not a rounding error). A fake that
        stayed silent would model a servo with every wire cut.

        Only non-broadcast WRITEs are answered: broadcasts and SYNC
        WRITEs get no reply by protocol, and READs are left to the
        tests that stage their own payloads.
        """
        if len(packet) < 6 or packet[0:2] != b"\xff\xff":
            return
        servo_id, instr = packet[2], packet[4]
        if instr != 0x03 or servo_id == 0xFE:
            return
        self._apply_write(servo_id, packet[5], packet[6:-1])
        body = bytes([servo_id, 2, 0])          # id, len, err=0
        chk = 0
        for b in body:
            chk += b
        self._rx_buf += b"\xff\xff" + body + bytes([(~chk) & 0xFF])

    def _apply_write(self, servo_id, reg, data):
        """Commit a write into the register map. The seam for the
        ACK-without-apply servo: a cold-booting controller answers
        the status packet but drops the EEPROM commit (the mode-1
        wheel that pivoted the bench robot, 2026-08-30) — tests model
        it by overriding this to skip chosen registers."""
        for off, b in enumerate(bytes(data)):
            self._regs[(servo_id, reg + off)] = b

    def _answer_scs_read(self, packet):
        """Answer a READ from the register map — but only for a
        register a write has populated, and only when no test has
        staged its own reply: staged ``_rx_buf`` bytes model the
        exact wire a test wants, and must never be interleaved
        with synthesized packets."""
        if self._rx_buf or len(packet) < 8 or packet[0:2] != b"\xff\xff":
            return
        servo_id, instr = packet[2], packet[4]
        if instr != 0x02 or servo_id == 0xFE:
            return
        reg, nbytes = packet[5], packet[6]
        payload = []
        for off in range(nbytes):
            if (servo_id, reg + off) not in self._regs:
                return          # unknown register: stay silent
            payload.append(self._regs[(servo_id, reg + off)])
        body = bytes([servo_id, nbytes + 2, 0]) + bytes(payload)
        chk = 0
        for b in body:
            chk += b
        self._rx_buf += b"\xff\xff" + body + bytes([(~chk) & 0xFF])

    def read(self, n=None):
        if not self._rx_buf:
            return None
        if n is None or n >= len(self._rx_buf):
            chunk, self._rx_buf = self._rx_buf, b""
            return chunk
        chunk, self._rx_buf = self._rx_buf[:n], self._rx_buf[n:]
        return chunk

    def any(self):
        return len(self._rx_buf)


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


class ADC:
    """machine.ADC stand-in for the QTR array driver.

    Tests script per-pin readings via the class-level ``reads`` map:
    ``ADC.reads[pin_num] = value`` (a constant) or a zero-arg callable
    for sequences. Unset pins read mid-scale.
    """

    ATTN_11DB = 3
    reads = {}

    def __init__(self, pin):
        # Accepts a fake Pin (has ._num-ish identity) or a bare int.
        self._pin = getattr(pin, "pin", pin)

    def atten(self, _db):
        pass

    def read_u16(self):
        v = self.reads.get(self._pin, 32768)
        return v() if callable(v) else v


class _FakeMachineModule:
    Pin = Pin
    PWM = PWM
    I2C = I2C
    UART = UART
    Timer = Timer
    ADC = ADC

    # ``time_pulse_us`` stand-in for the HC-SR04 driver. By default
    # returns -1 ("no echo"); tests rebind via
    # ``machine.time_pulse_us = lambda p, l, t: <value>`` directly.
    @staticmethod
    def time_pulse_us(pin, level, timeout_us):
        return -1


sys.modules["machine"] = _FakeMachineModule

# Note: the SSD1306 fake is intentionally not installed here. It lives
# in ``tests/_fakes_ssd1306.py`` so that test modules not touching the
# display (e.g. ``test_observer``, which allocates close to MP's heap
# limit for its variance test) don't pay its memory cost.
