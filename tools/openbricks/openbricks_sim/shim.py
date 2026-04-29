# SPDX-License-Identifier: MIT
"""
Driver shim — make firmware-targeting openbricks code run unchanged in the sim.

The firmware code path is::

    from machine import Pin, PWM
    from openbricks._native import Servo, QuadratureEncoder
    from openbricks.drivers.jgb37_520 import JGB37Motor
    from openbricks.robotics.drivebase import DriveBase

    m_left  = JGB37Motor(in1=12, in2=14, pwm=27, encoder_a=18, encoder_b=19)
    m_right = JGB37Motor(in1=13, in2=15, pwm=26, encoder_a=20, encoder_b=21)
    db = DriveBase(m_left, m_right, wheel_diameter_mm=60, axle_track_mm=150)
    db.straight(200)

None of those imports — ``machine``, ``_openbricks_native``,
``openbricks.*`` — exist on a stock CPython install. ``shim.install()``
manufactures the missing pieces so the same script runs against
MuJoCo:

  * ``machine`` — no-op fakes for ``Pin`` / ``PWM`` / ``I2C`` /
    ``UART`` / ``Timer``. Drivers construct them but never read back
    real hardware.
  * ``_openbricks_native`` — re-exports the cores already shipped in
    ``openbricks_sim._native`` (``TrapezoidalProfile``, ``Observer``)
    for read-only consumers, plus shim ``Servo`` / ``DriveBase``
    classes whose constructors match the firmware's signatures but
    bind to a MuJoCo (sensor, actuator) slot pair.
  * ``time.sleep_ms`` / ``time.sleep`` — patched to *step the sim*
    instead of blocking on real wall time. The firmware drivers
    busy-wait on ``while not is_done(): time.sleep_ms(10)`` — patching
    sleep makes that idiom advance MuJoCo physics so ``is_done()``
    actually flips.

Slot allocation is sequential: the first ``Servo(...)`` constructed
binds to ``chassis_motor_l`` / ``chassis_enc_l``, the second to the
``_r`` pair. The default chassis has exactly two motor slots; trying
to construct a third raises ``RuntimeError``.

After ``install(runtime)``, calling ``uninstall()`` restores the
original ``sys.modules`` + ``time`` state so back-to-back tests can
use a fresh runtime. The runtime itself is held by the shim only
while installed.
"""

from __future__ import annotations

import sys
import time as _real_time
import types
from pathlib import Path
from typing import Optional

from openbricks_sim import _native as _sim_native
from openbricks_sim.runtime import (SimRuntime, SimMotor, SimDriveBase,
                                     SimIMU, SimColorSensor,
                                     SimDistanceSensor)


# Module-level state — only one shim can be installed at a time, but
# its installer / uninstaller are symmetric so back-to-back tests work.
_INSTALLED: Optional["_ShimState"] = None


class _ShimState:
    """Bookkeeping for ``install`` / ``uninstall``: previous
    ``sys.modules`` entries + a backup of the patched ``time``
    attributes so ``uninstall()`` is exact."""

    def __init__(self):
        self.prev_sys_modules:    dict = {}
        self.prev_time_attrs:     dict = {}
        self.prev_sys_path:       list = []
        self.prev_driver_attrs:   dict = {}   # ("module.attr", prev_value)
        self.runtime: Optional[SimRuntime] = None
        self.motor_idx: int = 0


# ---------------------------------------------------------------------
# Slot allocation


_MOTOR_SLOTS = [
    ("chassis_enc_l", "chassis_motor_l"),
    ("chassis_enc_r", "chassis_motor_r"),
]


def _next_motor_slot():
    if _INSTALLED is None:
        raise RuntimeError("shim not installed; call install(runtime) first")
    if _INSTALLED.motor_idx >= len(_MOTOR_SLOTS):
        raise RuntimeError(
            "default chassis has only %d motor slots; constructed too "
            "many JGB37Motor / Servo objects" % len(_MOTOR_SLOTS))
    slot = _MOTOR_SLOTS[_INSTALLED.motor_idx]
    _INSTALLED.motor_idx += 1
    return slot


# ---------------------------------------------------------------------
# Hardware no-op stand-ins (machine.*)


class _NoopHardware:
    """Catch-all for Pin / PWM / I2C / UART / etc. Accepts any args
    at construction; methods called on it return None or self."""

    OUT = "OUT"; IN = "IN"
    PULL_UP = "PULL_UP"; PULL_DOWN = "PULL_DOWN"
    IRQ_RISING = 1; IRQ_FALLING = 2

    def __init__(self, *args, **kwargs):
        self._args   = args
        self._kwargs = kwargs

    # Most driver callsites use these — keep them no-ops.
    def value(self, v=None): return 0 if v is None else None
    def on(self):             pass
    def off(self):            pass
    def duty(self, v=None):   return 0 if v is None else None
    def freq(self, v=None):   return 0 if v is None else None
    def irq(self, *a, **k):   return None

    # I2C / UART method coverage so the import doesn't blow up.
    def readfrom_mem(self, *a, **k):  return b""
    def writeto_mem(self, *a, **k):   return None
    def readfrom(self, *a, **k):       return b""
    def writeto(self, *a, **k):        return None
    def write(self, *a, **k):          return 0
    def read(self, *a, **k):           return b""

    # Timer.init / deinit — drivers occasionally instantiate Timer for
    # local periodic work (rare on the user-code path, but the shim
    # stays cheap enough to cover it).
    def init(self, *a, **k):  return None
    def deinit(self, *a, **k): return None


def _make_machine_module():
    m = types.ModuleType("machine")
    m.Pin    = _NoopHardware
    m.PWM    = _NoopHardware
    m.I2C    = _NoopHardware
    m.UART   = _NoopHardware
    m.SPI    = _NoopHardware
    m.Timer  = _NoopHardware
    m.ADC    = _NoopHardware
    m.DAC    = _NoopHardware
    m.RTC    = _NoopHardware
    return m


# ---------------------------------------------------------------------
# Shim Servo + DriveBase + supporting types


class ShimServo:
    """Drop-in for ``_openbricks_native.Servo``.

    Mirrors the firmware's constructor signature
    (``in1=, in2=, pwm=, encoder=, counts_per_rev=, invert=, kp=``).
    All hardware kwargs are ignored; the shim binds to the next
    available chassis motor slot.

    Methods proxy to a wrapped :class:`SimMotor` adapter, plus a
    bypass-the-controller ``run(power)`` for open-loop callers."""

    def __init__(self, in1=None, in2=None, pwm=None, encoder=None,
                 counts_per_rev: int = 1320,
                 invert: bool = False,
                 kp: float = 0.3):
        sensor_name, actuator_name = _next_motor_slot()
        self._adapter = SimMotor(
            _INSTALLED.runtime, sensor_name, actuator_name,
            counts_per_rev=int(counts_per_rev),
            kp=float(kp),
            invert=bool(invert))

    # Closed-loop entry points — the firmware's drivers call these.
    def run_speed(self, dps):
        self._adapter.run_speed(float(dps))

    def run_target(self, delta_deg, cruise_dps, accel=720.0):
        self._adapter.run_target(float(delta_deg),
                                  float(cruise_dps),
                                  float(accel))

    def is_done(self):
        return self._adapter.is_done()

    def angle(self):
        return self._adapter.angle()

    def reset_angle(self, angle=0.0):
        self._adapter.reset_angle(float(angle))

    # Open-loop bypass + brake / coast.
    def run(self, power):
        # Mirror SimMotor.brake's "detach + write ctrl directly":
        # firmware Servo.run() detaches from the scheduler and writes
        # the bridge with a raw power value.
        self._adapter._detach()
        adapter = self._adapter
        scale   = adapter._ctrl_scale
        rt      = adapter.runtime
        p       = float(power)
        if p >  100.0: p =  100.0
        if p < -100.0: p = -100.0
        if adapter.invert:
            p = -p
        rt.data.ctrl[adapter._actuator_id] = p * scale

    def brake(self):
        self._adapter.brake()

    def coast(self):
        self._adapter.coast()


class ShimTCS34725:
    """Drop-in for ``openbricks.drivers.tcs34725.TCS34725``.

    Constructor accepts the firmware shape (``i2c=, address=,
    integration_ms=, gain=``); arguments are kept for documentation
    but ignored — the shim binds straight to the chassis
    :class:`SimColorSensor` regardless.

    ``rgb()`` and ``ambient()`` proxy directly. ``raw()`` returns a
    synthetic 4-tuple ``(c, r, g, b)`` in the 16-bit range that the
    real driver would produce, so user code that calls ``raw()``
    sees realistic-looking values.
    """

    def __init__(self, *args, **kwargs):
        if _INSTALLED is None:
            raise RuntimeError(
                "shim not installed; call install(runtime) first")
        self._cs = SimColorSensor(_INSTALLED.runtime)

    def rgb(self):
        return self._cs.rgb()

    def ambient(self):
        return self._cs.ambient()

    def raw(self):
        # Synthesise a (clear, R, G, B) 16-bit reading from the
        # raycast result. The real TCS34725 has independent C / R /
        # G / B ADCs; the sim reduces RGB to a clear-channel via
        # luminance and scales each channel to 0..65535. Plenty
        # accurate enough for tests that check "is the sensor over a
        # red zone yet".
        r8, g8, b8 = self._cs.rgb()
        c8 = self._cs.ambient() * 255 // 100
        scale = 65535 / 255
        return (int(c8 * scale), int(r8 * scale),
                int(g8 * scale), int(b8 * scale))


class _ShimDistanceSensorBase:
    """Common shim for any distance-sensor driver.

    HC-SR04, VL53L0X, VL53L1X all implement
    :class:`openbricks.distance.DistanceSensor` with the same
    one-method shape (``distance_mm()``). Their firmware classes
    differ in *constructor* (Pin pair vs I2C handle) but the
    sim doesn't care — the underlying physics question
    ("what's in front of the chassis?") is answered the same way
    by :class:`SimDistanceSensor`. Each concrete shim subclass just
    accepts whatever the firmware constructor takes.
    """

    def __init__(self, *args, **kwargs):
        if _INSTALLED is None:
            raise RuntimeError(
                "shim not installed; call install(runtime) first")
        self._ds = SimDistanceSensor(_INSTALLED.runtime)

    def distance_mm(self):
        return self._ds.distance_mm()


class ShimHCSR04(_ShimDistanceSensorBase):
    """Drop-in for ``openbricks.drivers.hcsr04.HCSR04``."""


class ShimVL53L0X(_ShimDistanceSensorBase):
    """Drop-in for ``openbricks.drivers.vl53l0x.VL53L0X``."""


class ShimVL53L1X(_ShimDistanceSensorBase):
    """Drop-in for ``openbricks.drivers.vl53l1x.VL53L1X``."""


class ShimBNO055:
    """Drop-in for ``_openbricks_native.BNO055`` — what
    ``openbricks.drivers.bno055`` re-exports.

    Constructor accepts whatever the firmware BNO055 takes
    (typically an ``i2c=`` handle and an ``address=``); the shim
    binds to the chassis IMU regardless. Methods proxy to a wrapped
    :class:`SimIMU`.
    """

    def __init__(self, *args, **kwargs):
        if _INSTALLED is None:
            raise RuntimeError(
                "shim not installed; call install(runtime) first")
        self._imu = SimIMU(_INSTALLED.runtime)

    def heading(self):
        return self._imu.heading()

    def angular_velocity(self):
        return self._imu.angular_velocity()

    def acceleration(self):
        return self._imu.acceleration()


class ShimDriveBase:
    """Drop-in for ``_openbricks_native.DriveBase``.

    Constructor signature matches the firmware:
    ``DriveBase(left=Servo, right=Servo, wheel_diameter_mm=,
                axle_track_mm=, imu=None, kp_sum=, kp_diff=)``.

    With ``use_gyro(True)`` the shim installs a per-tick callback
    that reads the IMU heading, computes the body-degree delta from
    the move-start offset, and pushes it into the native drivebase's
    ``heading_override_wheel_deg`` slot — the same slip-immune
    feedback path as firmware. The IMU just needs to expose a
    ``heading()`` method (degrees, [-180, 180)); the shim BNO055
    qualifies, and so does any user-supplied object with the same
    shape.
    """

    def __init__(self, left=None, right=None,
                 wheel_diameter_mm: float = 60.0,
                 axle_track_mm: float = 150.0,
                 imu=None,
                 kp_sum=None,
                 kp_diff=None):
        if not isinstance(left, ShimServo) or not isinstance(right, ShimServo):
            raise TypeError(
                "shim DriveBase requires shim Servo instances "
                "(got %s, %s)" % (type(left).__name__, type(right).__name__))
        # The user's robot.py is the same script the firmware runs; its
        # ``wheel_diameter_mm`` / ``axle_track_mm`` are the single source
        # of truth for chassis dims. Apply them to the sim model in
        # place so wheel encoders rotate a wheel of the right size — see
        # ``chassis.apply_drivebase_dims_to_model`` for the trade-offs.
        from openbricks_sim.chassis import apply_drivebase_dims_to_model
        apply_drivebase_dims_to_model(
            _INSTALLED.runtime.model,
            wheel_diameter_mm=float(wheel_diameter_mm),
            axle_track_mm=float(axle_track_mm))
        self._db = SimDriveBase(
            _INSTALLED.runtime, left._adapter, right._adapter,
            wheel_diameter_mm=float(wheel_diameter_mm),
            axle_track_mm=float(axle_track_mm),
            kp_sum=kp_sum,
            kp_diff=kp_diff)
        self._imu              = imu
        self._use_gyro         = False
        self._heading_offset   = 0.0
        self._imu_tick_active  = False

    # ----- IMU heading update tick ---------------------------------

    def _imu_tick(self, now_ms):
        body = float(self._imu.heading())
        delta = body - self._heading_offset
        # Wrap ±180 so a move that crosses the boundary doesn't see
        # a spurious ±360 jump.
        if delta >  180.0: delta -= 360.0
        if delta < -180.0: delta += 360.0
        self._db.set_heading_override(delta)

    def _attach_imu_tick(self):
        """Insert the IMU tick *before* the drivebase tick so the
        override is fresh by the time the controller reads it."""
        if self._imu_tick_active:
            return
        rt = self._db.runtime
        rt.remove_tick(self._db._tick)
        rt.remove_tick(self._db.left._tick)
        rt.remove_tick(self._db.right._tick)
        rt.add_tick(self._imu_tick)
        rt.add_tick(self._db._tick)
        rt.add_tick(self._db.left._tick)
        rt.add_tick(self._db.right._tick)
        # SimDriveBase + its motors all consider themselves attached
        # after this dance — preserve that flag.
        self._db._attached = True
        self._db.left._attached  = True
        self._db.right._attached = True
        self._imu_tick_active = True

    def _detach_imu_tick(self):
        if not self._imu_tick_active:
            return
        self._db.runtime.remove_tick(self._imu_tick)
        self._imu_tick_active = False

    # ----- Move setup ----------------------------------------------

    def straight(self, distance_mm, speed_mm_s):
        if self._use_gyro and self._imu is not None:
            self._heading_offset = float(self._imu.heading())
        self._db.straight(float(distance_mm), float(speed_mm_s))

    def turn(self, angle_deg, rate_dps):
        if self._use_gyro and self._imu is not None:
            self._heading_offset = float(self._imu.heading())
        self._db.turn(float(angle_deg), float(rate_dps))

    def stop(self):
        self._db.stop()

    def is_done(self):
        return self._db.is_done()

    def use_gyro(self, enable):
        if enable and self._imu is None:
            raise RuntimeError(
                "use_gyro requires imu= at construction")
        # Toggling on captures the offset so "now" is heading 0.
        if enable:
            self._heading_offset = float(self._imu.heading())
            self._db.set_use_gyro(True)
            self._attach_imu_tick()
        else:
            self._db.set_use_gyro(False)
            self._detach_imu_tick()
        self._use_gyro = enable


def _make_native_module():
    """Build the ``_openbricks_native`` replacement module."""
    m = types.ModuleType("_openbricks_native")
    # The pure-math cores already exist in openbricks_sim._native —
    # firmware code that imports TrapezoidalProfile / Observer
    # gets exactly the same classes that openbricks_sim users get.
    m.TrapezoidalProfile = _sim_native.TrapezoidalProfile
    m.Observer           = _sim_native.Observer
    # Hardware-bound types are the shim variants.
    m.Servo              = ShimServo
    m.DriveBase          = ShimDriveBase
    # Encoders + sensors that openbricks code imports but doesn't
    # actually drive on the sim path. No-op stand-ins are enough.
    m.QuadratureEncoder  = _NoopHardware
    m.PCNTEncoder        = _NoopHardware
    m.BNO055             = ShimBNO055
    # ``motor_process`` — the firmware exposes it as a singleton
    # object with .start / .stop / .tick / .is_running. The sim's
    # SimRuntime.step() is the equivalent; expose a stub that
    # delegates so user code calling motor_process.start() doesn't
    # crash. (Rarely used by user-facing code; mostly internal.)
    m.motor_process = _MotorProcessStub()
    return m


class _MotorProcessStub:
    """Minimal stand-in for the firmware's motor_process singleton.
    The runtime drives ticks itself; nothing here actually controls
    physics — it's just enough surface so ``import`` doesn't fail."""

    def start(self):     return None
    def stop(self):      return None
    def is_running(self): return True
    def configure(self, *a, **k): return None
    def now_ms(self):
        return _INSTALLED.runtime.now_ms if _INSTALLED else 0


# ---------------------------------------------------------------------
# Time patching — make sleep advance the sim


def _patched_sleep_ms(ms):
    if _INSTALLED is None:
        # Shim was uninstalled out from under us; fall back to real
        # sleep so we don't hang.
        return _real_time.sleep(max(0.0, ms / 1000.0))
    rt = _INSTALLED.runtime
    n = max(1, int(round(ms / max(1, rt.timestep_ms))))
    for _ in range(n):
        rt.step()


def _patched_sleep(seconds):
    _patched_sleep_ms(seconds * 1000.0)


def _patched_ticks_ms():
    if _INSTALLED is None:
        return int(_real_time.time() * 1000)
    return int(_INSTALLED.runtime.now_ms)


def _patched_ticks_diff(a, b):
    return a - b


def _patched_ticks_us():
    return _patched_ticks_ms() * 1000


# ---------------------------------------------------------------------
# Public API


def install(runtime: SimRuntime) -> None:
    """Install all shims for ``runtime``. Call this *before* any
    ``import openbricks.*`` from the user-script side."""
    global _INSTALLED
    if _INSTALLED is not None:
        raise RuntimeError("shim already installed; call uninstall() first")
    state = _ShimState()
    state.runtime = runtime

    # 1. machine + _openbricks_native fakes.
    for name, factory in [
            ("machine",             _make_machine_module),
            ("_openbricks_native",  _make_native_module),
    ]:
        state.prev_sys_modules[name] = sys.modules.get(name)
        sys.modules[name] = factory()

    # 2. time patches — only patch the attributes that exist (or
    # plant new ones for sleep_ms / ticks_ms which don't exist on
    # CPython). Restore exactly on uninstall.
    for attr, patched in [
            ("sleep",      _patched_sleep),
            ("sleep_ms",   _patched_sleep_ms),
            ("ticks_ms",   _patched_ticks_ms),
            ("ticks_us",   _patched_ticks_us),
            ("ticks_diff", _patched_ticks_diff),
    ]:
        state.prev_time_attrs[attr] = getattr(_real_time, attr, _MISSING)
        setattr(_real_time, attr, patched)

    # 3. Make ``import openbricks`` work from within a sim run by
    # adding the repo root to sys.path. The openbricks-sim package
    # lives at ``<repo>/tools/openbricks-sim/openbricks_sim/`` so
    # the repo root is three directories up.
    state.prev_sys_path = list(sys.path)
    repo_root = Path(__file__).resolve().parents[3]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    _INSTALLED = state

    # 4. Patch driver classes that don't go through ``_native``.
    # The TCS34725 talks I2C directly — replace the class so user
    # ``from openbricks.drivers.tcs34725 import TCS34725`` resolves
    # to the sim version. Must happen *after* sys.path is rigged so
    # the import succeeds.
    _patch_pure_python_drivers(state)


def _patch_pure_python_drivers(state: "_ShimState") -> None:
    """Replace pure-Python driver classes with sim-aware versions.

    Records the original attributes in ``state.prev_driver_attrs`` so
    ``uninstall`` can restore them exactly."""
    # Each entry: (module-import-name, attr-name, replacement). If
    # the import fails (e.g. openbricks repo not on sys.path) skip
    # silently — user script just doesn't use that driver.
    targets = [
        ("openbricks.drivers.tcs34725", "TCS34725", ShimTCS34725),
        ("openbricks.drivers.hcsr04",   "HCSR04",   ShimHCSR04),
        ("openbricks.drivers.vl53l0x",  "VL53L0X",  ShimVL53L0X),
        ("openbricks.drivers.vl53l1x",  "VL53L1X",  ShimVL53L1X),
    ]
    for mod_name, attr, replacement in targets:
        try:
            mod = __import__(mod_name, fromlist=[attr])
        except Exception:
            continue
        state.prev_driver_attrs[(mod_name, attr)] = (
            mod, attr, getattr(mod, attr))
        setattr(mod, attr, replacement)


def uninstall() -> None:
    """Roll back every change ``install()`` made. Idempotent."""
    global _INSTALLED
    if _INSTALLED is None:
        return
    state = _INSTALLED

    # 1. sys.modules — restore the previous entry, or remove if it
    # was absent before.
    for name, prev in state.prev_sys_modules.items():
        if prev is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = prev

    # 2. time attributes.
    for attr, prev in state.prev_time_attrs.items():
        if prev is _MISSING:
            try:
                delattr(_real_time, attr)
            except AttributeError:
                pass
        else:
            setattr(_real_time, attr, prev)

    # 3. sys.path
    sys.path[:] = state.prev_sys_path

    # 4. Driver classes patched in step 4 of install.
    for _key, (mod, attr, prev) in state.prev_driver_attrs.items():
        setattr(mod, attr, prev)

    _INSTALLED = None


_MISSING = object()


def is_installed() -> bool:
    return _INSTALLED is not None
