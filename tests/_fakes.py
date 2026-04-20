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


_DEFAULT_PERIOD_MS = 1   # 1 kHz, matching motor_process.c


class _MotorProcess:
    """Module-level singleton: only one instance is ever exposed, via
    ``_openbricks_native.motor_process``. Users should not instantiate
    directly (the C version hides the type entirely)."""

    def __init__(self, period_ms=_DEFAULT_PERIOD_MS):
        self._period_ms = int(period_ms)
        self._callbacks = []       # Python callables (slow path)
        self._c_callbacks = []     # internal (fn, ctx) pairs (fast path)
        self._timer = None

    # --- Python subscription ---

    def register(self, callback):
        if callback not in self._callbacks:
            self._callbacks.append(callback)
        if self._timer is None:
            self.start()   # pbio-style: auto-start on first subscription, never auto-stop

    def unregister(self, callback):
        try:
            self._callbacks.remove(callback)
        except ValueError:
            pass

    # --- internal C-callback path (mirrors openbricks_motor_process_register_c) ---

    def _register_c(self, fn, ctx):
        key = (fn, ctx)
        if key not in self._c_callbacks:
            self._c_callbacks.append(key)
        if self._timer is None:
            self.start()

    def _unregister_c(self, fn, ctx):
        try:
            self._c_callbacks.remove((fn, ctx))
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
        # Fast path: C callbacks. (fn, ctx) pairs.
        for fn, ctx in list(self._c_callbacks):
            fn(ctx)
        # Slow path: Python callables. Snapshot so callbacks can unregister.
        for cb in list(self._callbacks):
            cb()

    # --- test helper ---

    def reset(self):
        """Stop the timer, clear all subscribers, restore default period.
        Tests call this in setUp to isolate state. Not part of the C
        firmware API."""
        self.stop()
        self._callbacks = []
        self._c_callbacks = []
        self._period_ms = _DEFAULT_PERIOD_MS


# TrapezoidalProfile — Python mirror of
# native/user_c_modules/openbricks/trajectory.c. Same math, same edge
# cases, same public API.


import math as _math


class _TrapezoidalProfile:
    def __init__(self, start, target, cruise_dps, accel_dps2):
        self._start = float(start)
        self._distance = float(target) - float(start)
        self._cruise = abs(float(cruise_dps))
        self._accel = abs(float(accel_dps2))
        self._direction = -1.0 if self._distance < 0 else 1.0
        D = abs(self._distance)

        if D == 0.0 or self._cruise == 0.0 or self._accel == 0.0:
            self._t_ramp = 0.0
            self._t_cruise = 0.0
            self._t_total = 0.0
            self._d_ramp = 0.0
            self._v_peak = 0.0
            self._triangular = False
            return

        t_ramp_full = self._cruise / self._accel
        d_ramp_full = 0.5 * self._accel * t_ramp_full * t_ramp_full

        if 2.0 * d_ramp_full <= D:
            self._triangular = False
            self._t_ramp = t_ramp_full
            self._d_ramp = d_ramp_full
            self._v_peak = self._cruise
            self._t_cruise = (D - 2.0 * d_ramp_full) / self._cruise
            self._t_total = 2.0 * self._t_ramp + self._t_cruise
        else:
            self._triangular = True
            t_peak = _math.sqrt(D / self._accel)
            self._t_ramp = t_peak
            self._t_cruise = 0.0
            self._t_total = 2.0 * t_peak
            self._v_peak = self._accel * t_peak
            self._d_ramp = 0.5 * self._accel * t_peak * t_peak

    def sample(self, t_s):
        t_s = float(t_s)
        D = abs(self._distance)
        if t_s <= 0.0:
            abs_pos, abs_vel = 0.0, 0.0
        elif t_s >= self._t_total:
            abs_pos, abs_vel = D, 0.0
        elif t_s < self._t_ramp:
            abs_vel = self._accel * t_s
            abs_pos = 0.5 * self._accel * t_s * t_s
        elif not self._triangular and t_s < self._t_ramp + self._t_cruise:
            abs_vel = self._v_peak
            abs_pos = self._d_ramp + self._v_peak * (t_s - self._t_ramp)
        else:
            decel_start = self._t_ramp if self._triangular else self._t_ramp + self._t_cruise
            td = t_s - decel_start
            abs_vel = self._v_peak - self._accel * td
            if abs_vel < 0.0:
                abs_vel = 0.0
            d_before = self._d_ramp if self._triangular else self._d_ramp + self._v_peak * self._t_cruise
            abs_pos = d_before + self._v_peak * td - 0.5 * self._accel * td * td
        return (self._start + self._direction * abs_pos,
                self._direction * abs_vel)

    def duration(self):
        return self._t_total

    def is_triangular(self):
        return self._triangular


# Observer — Python mirror of
# native/user_c_modules/openbricks/observer.c. Two-state (position,
# velocity) α-β filter. Tests exercise this fake; the C version must
# match its behaviour byte-for-byte (same gains → same outputs).


class _Observer:
    def __init__(self, alpha=0.5, beta=0.15):
        self._alpha = float(alpha)
        self._beta = float(beta)
        self._pos = 0.0
        self._vel = 0.0

    def reset(self, pos=0.0):
        self._pos = float(pos)
        self._vel = 0.0

    def update(self, measured_pos, dt):
        if dt <= 0.0:
            return (self._pos, self._vel)
        pos_pred = self._pos + self._vel * dt
        residual = float(measured_pos) - pos_pred
        self._pos = pos_pred + self._alpha * residual
        self._vel = self._vel + (self._beta / dt) * residual
        return (self._pos, self._vel)

    def position(self):
        return self._pos

    def velocity(self):
        return self._vel


# Servo — the Python fake of ``_openbricks_native.Servo``. Mirrors
# native/user_c_modules/openbricks/servo.c line-for-line so desktop tests
# lock in semantics the C version must match.

_DUTY_MAX = 1023
_POWER_CLAMP = 100.0
_RATED_DPS = 300.0
_DEFAULT_ACCEL = 720.0   # deg/s^2; matches servo.c default


class _Servo:
    def __init__(self, in1, in2, pwm, encoder,
                 counts_per_rev=1320, invert=False, kp=0.3):
        self._in1 = in1
        self._in2 = in2
        self._pwm = pwm
        self._encoder = encoder
        self._counts_per_rev = int(counts_per_rev)
        self._invert = bool(invert)
        self._kp = float(kp)
        self._target_dps = 0.0
        self._active = False
        self._last_time_ms = _real_time.ticks_ms()
        # Trajectory tracking (None = plain run_speed mode).
        self._trajectory = None
        self._traj_start_ms = 0
        self._traj_done = True
        # α-β state observer — same defaults as servo.c.
        self._observer = _Observer(alpha=0.5, beta=0.15)

    # --- hardware primitives ---

    def _drive_power(self, power):
        if power > _POWER_CLAMP:
            power = _POWER_CLAMP
        elif power < -_POWER_CLAMP:
            power = -_POWER_CLAMP
        effective = -power if self._invert else power
        if effective > 0:
            self._in1.value(1)
            self._in2.value(0)
        elif effective < 0:
            self._in1.value(0)
            self._in2.value(1)
        else:
            self._in1.value(0)
            self._in2.value(0)
        duty = int(abs(effective) * _DUTY_MAX / 100.0)
        if duty > _DUTY_MAX:
            duty = _DUTY_MAX
        elif duty < 0:
            duty = 0
        self._pwm.duty(duty)

    def _brake_bridge(self):
        self._in1.value(1)
        self._in2.value(1)
        self._pwm.duty(_DUTY_MAX)

    def _coast_bridge(self):
        self._in1.value(0)
        self._in2.value(0)
        self._pwm.duty(0)

    # --- control tick ---

    def _control_tick(self, _ctx):
        # If we're tracking a trajectory, sample it to get the current
        # velocity setpoint. Position feedback from observer/trajectory
        # lands later once the loop is tuned.
        if self._trajectory is not None:
            elapsed_s = (_real_time.ticks_ms() - self._traj_start_ms) / 1000.0
            if elapsed_s >= self._trajectory.duration():
                self._target_dps = 0.0
                self._traj_done = True
            else:
                _pos, vel = self._trajectory.sample(elapsed_s)
                self._target_dps = vel

        # Observer update — smoothed velocity estimate from encoder
        # position. Replaces the M1 finite-difference.
        count = self._encoder._count
        measured_pos = count * 360.0 / self._counts_per_rev
        now = _real_time.ticks_ms()
        dt_s = (now - self._last_time_ms) / 1000.0
        self._last_time_ms = now
        if dt_s > 0:
            self._observer.update(measured_pos, dt_s)
        measured = self._observer.velocity()

        error = self._target_dps - measured
        ff = self._target_dps / _RATED_DPS * _POWER_CLAMP
        self._drive_power(ff + self._kp * error)

    # --- attach / detach ---

    def _attach(self):
        if self._active:
            return
        # Re-baseline the observer on attach so the first tick doesn't
        # see a huge phantom residual from a stale last-known position.
        pos = self._encoder._count * 360.0 / self._counts_per_rev
        self._observer.reset(pos)
        self._last_time_ms = _real_time.ticks_ms()
        _motor_process_singleton._register_c(self._control_tick, self)
        self._active = True

    def _detach(self):
        if not self._active:
            return
        _motor_process_singleton._unregister_c(self._control_tick, self)
        self._active = False
        self._target_dps = 0.0
        self._trajectory = None
        self._traj_done = True

    # --- Python-facing methods ---

    def run_speed(self, deg_per_s):
        self._target_dps = float(deg_per_s)
        self._trajectory = None
        self._traj_done = True
        self._attach()

    def run_target(self, delta_deg, cruise_dps, accel_dps2=_DEFAULT_ACCEL):
        """Follow a trapezoidal profile by ``delta_deg`` (signed) at the
        given cruise speed + acceleration. Non-blocking; poll is_done()."""
        start = self.angle()
        self._trajectory = _TrapezoidalProfile(
            start, start + float(delta_deg),
            float(cruise_dps), float(accel_dps2),
        )
        self._traj_start_ms = _real_time.ticks_ms()
        self._traj_done = False
        self._target_dps = 0.0   # first tick will sample the profile
        self._attach()

    def is_done(self):
        return self._traj_done

    def run(self, power):
        self._detach()
        self._drive_power(float(power))

    def brake(self):
        self._detach()
        self._brake_bridge()

    def coast(self):
        self._detach()
        self._coast_bridge()

    def angle(self):
        return self._encoder._count * 360.0 / self._counts_per_rev

    def reset_angle(self, angle=0):
        new_count = int(float(angle) * self._counts_per_rev / 360.0)
        self._encoder._count = new_count
        self._observer.reset(float(angle))
        self._last_time_ms = _real_time.ticks_ms()


# DriveBase — Python mirror of
# native/user_c_modules/openbricks/drivebase.c. Runs two coupled
# trajectories (forward progress + heading), writes per-servo
# target_dps each tick. The individual servos still run their own
# feedforward+P velocity loop; the drivebase is a setpoint source.


_DB_DEFAULT_ACCEL = 720.0
_DB_DEFAULT_KP_SUM = 2.0
_DB_DEFAULT_KP_DIFF = 5.0


class _DriveBase:
    def __init__(self, left, right, wheel_diameter_mm, axle_track_mm,
                 kp_sum=None, kp_diff=None):
        if not isinstance(left, _Servo) or not isinstance(right, _Servo):
            raise TypeError("left and right must be Servo instances")
        self._left = left
        self._right = right
        self._wheel_circ = _math.pi * float(wheel_diameter_mm)
        self._axle = float(axle_track_mm)
        self._kp_sum  = _DB_DEFAULT_KP_SUM  if kp_sum  is None else float(kp_sum)
        self._kp_diff = _DB_DEFAULT_KP_DIFF if kp_diff is None else float(kp_diff)

        self._fwd = None
        self._fwd_start_ms = 0
        self._fwd_active = False
        self._fwd_hold = 0.0

        self._turn = None
        self._turn_start_ms = 0
        self._turn_active = False
        self._turn_hold = 0.0

        self._registered = False
        self._done = True

    # --- Python-facing methods ---

    def straight(self, distance_mm, speed_mm_s):
        distance_mm = float(distance_mm)
        speed_mm_s = float(speed_mm_s)
        distance_deg = distance_mm / self._wheel_circ * 360.0
        speed_dps = abs(speed_mm_s) / self._wheel_circ * 360.0

        sum_pos  = (self._left._observer.position() + self._right._observer.position()) / 2.0
        diff_pos = (self._left._observer.position() - self._right._observer.position()) / 2.0
        self._fwd = _TrapezoidalProfile(sum_pos, sum_pos + distance_deg,
                                        speed_dps, _DB_DEFAULT_ACCEL)
        self._fwd_start_ms = _real_time.ticks_ms()
        self._fwd_active = True
        # Hold heading at whatever it was when the move started.
        self._turn_hold  = diff_pos
        self._turn_active = False
        self._register()

    def turn(self, angle_deg, rate_dps):
        angle_deg = float(angle_deg)
        rate_dps = float(rate_dps)
        arc_mm = abs(angle_deg) * _math.pi / 180.0 * (self._axle / 2.0)
        wheel_deg = arc_mm / self._wheel_circ * 360.0
        # +θ body turn (left / CCW) → left_pos decreases, right_pos
        # increases → diff_pos = (left - right)/2 DECREASES.
        signed_delta = -wheel_deg if angle_deg >= 0 else wheel_deg

        rate_arc_mm_s = abs(rate_dps) * _math.pi / 180.0 * (self._axle / 2.0)
        rate_wheel_dps = rate_arc_mm_s / self._wheel_circ * 360.0

        diff_pos = (self._left._observer.position() - self._right._observer.position()) / 2.0
        sum_pos  = (self._left._observer.position() + self._right._observer.position()) / 2.0
        self._turn = _TrapezoidalProfile(diff_pos, diff_pos + signed_delta,
                                          rate_wheel_dps, _DB_DEFAULT_ACCEL)
        self._turn_start_ms = _real_time.ticks_ms()
        self._turn_active = True
        self._fwd_hold   = sum_pos
        self._fwd_active = False
        self._register()

    def stop(self):
        self._fwd_active = False
        self._turn_active = False
        self._done = True
        self._unregister()

    def is_done(self):
        return self._done

    # --- control tick ---

    def _control_tick(self, _ctx):
        # Forward profile (or hold).
        if self._fwd_active:
            elapsed = (_real_time.ticks_ms() - self._fwd_start_ms) / 1000.0
            if elapsed >= self._fwd.duration():
                fwd_target = self._fwd.sample(self._fwd.duration())[0]
                fwd_ff_vel = 0.0
                self._fwd_hold = fwd_target
                self._fwd_active = False
            else:
                fwd_target, fwd_ff_vel = self._fwd.sample(elapsed)
        else:
            fwd_target = self._fwd_hold
            fwd_ff_vel = 0.0

        # Turn profile (or hold).
        if self._turn_active:
            elapsed = (_real_time.ticks_ms() - self._turn_start_ms) / 1000.0
            if elapsed >= self._turn.duration():
                turn_target = self._turn.sample(self._turn.duration())[0]
                turn_ff_vel = 0.0
                self._turn_hold = turn_target
                self._turn_active = False
            else:
                turn_target, turn_ff_vel = self._turn.sample(elapsed)
        else:
            turn_target = self._turn_hold
            turn_ff_vel = 0.0

        if not self._fwd_active and not self._turn_active:
            self._done = True

        sum_pos  = (self._left._observer.position() + self._right._observer.position()) / 2.0
        diff_pos = (self._left._observer.position() - self._right._observer.position()) / 2.0

        sum_err  = fwd_target  - sum_pos
        diff_err = turn_target - diff_pos

        fwd_cmd  = fwd_ff_vel  + self._kp_sum  * sum_err
        diff_cmd = turn_ff_vel + self._kp_diff * diff_err

        # diff_pos = (left - right)/2; diff_cmd is desired
        # (left_vel - right_vel)/2. Positive → left speeds up.
        self._left._target_dps  = fwd_cmd + diff_cmd
        self._right._target_dps = fwd_cmd - diff_cmd

    def _register(self):
        if self._registered:
            self._done = False
            return
        _motor_process_singleton._register_c(self._control_tick, self)
        self._registered = True
        self._done = False

    def _unregister(self):
        if not self._registered:
            return
        _motor_process_singleton._unregister_c(self._control_tick, self)
        self._registered = False


# Install the native module fake.
_motor_process_singleton = _MotorProcess()

_openbricks_native = types.ModuleType("_openbricks_native")
_openbricks_native.motor_process = _motor_process_singleton
_openbricks_native.Servo = _Servo
_openbricks_native.TrapezoidalProfile = _TrapezoidalProfile
_openbricks_native.Observer = _Observer
_openbricks_native.DriveBase = _DriveBase
sys.modules["_openbricks_native"] = _openbricks_native
