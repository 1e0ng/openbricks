# SPDX-License-Identifier: MIT
"""
Driver shim — make firmware-targeting openbricks code run unchanged in the sim.

The firmware code path is::

    from machine import Pin, PWM
    from openbricks._native import Servo, QuadratureEncoder
    from openbricks.drivers.jgb37_520 import JGB37Motor
    from openbricks.robotics.drivebase import DriveBase

    m_left  = JGB37Motor(in1=1, in2=2, pwm=17, encoder_a=7, encoder_b=8)
    m_right = JGB37Motor(in1=9, in2=10, pwm=11, encoder_a=12, encoder_b=13)
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

  * Serial-bus wheel servos — ``ST3215Motor`` / ``ST3032Motor`` are
    replaced at the class level (they talk UART directly, like the
    I2C drivers). The openbricks ``DriveBase`` adopts them through
    ``_adopt_into_drivebase`` onto an EMULATED ``st_bus``
    (``_SimStBus`` + the native extension's ``RawDriveBase``) — the
    same engine and controller code path as firmware — against
    MuJoCo wheels.

Slot allocation is sequential: the first motor constructed —
``Servo(...)`` or a serial ``ST3215Motor(...)`` / ``ST3032Motor(...)``
— binds to ``chassis_motor_l`` / ``chassis_enc_l``, the second to the
``_r`` pair. The third and fourth bind KINEMATIC task-motor slots
(no chassis body — the shaft integrates its commanded speed), so a
real robot's four-servo ``main.py`` constructs and runs; a fifth
raises ``RuntimeError``.

After ``install(runtime)``, calling ``uninstall()`` restores the
original ``sys.modules`` + ``time`` state so back-to-back tests can
use a fresh runtime. The runtime itself is held by the shim only
while installed.
"""

from __future__ import annotations

import math
import sys
import time as _real_time
import types
from pathlib import Path
from typing import Optional

from openbricks_sim import _native as _sim_native
from openbricks_sim.runtime import (SimRuntime, SimMotor, SimDriveBase,
                                     SimIMU, SimColorSensor,
                                     SimDistanceSensor)

# The firmware package (``openbricks``) is not part of the wheel: the
# sim runs against a checkout, whose root is three directories up.
# Rigged here at import (``install()`` repeats it idempotently) because
# the shim's own signatures default to ``openbricks.parameters``
# members (``then=Stop.COAST``), which must resolve at class-body time.
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from openbricks.parameters import Stop, DriveMode  # noqa: E402


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


# The first two slots are the chassis's physical wheels; the next two
# are KINEMATIC task-motor slots (None): the firmware bus has four
# native slots and the bench robot is four ST-3032s on one UART
# (2 wheels + 2 task motors), so a real robot's main.py must at least
# CONSTRUCT under the sim. Task motors have no MuJoCo body — their
# shafts integrate the commanded speed (see ShimST3215Motor).
_MOTOR_SLOTS = [
    ("chassis_enc_l", "chassis_motor_l"),
    ("chassis_enc_r", "chassis_motor_r"),
    None,
    None,
]


def _next_motor_slot(kinematic_ok=True):
    if _INSTALLED is None:
        raise RuntimeError("shim not installed; call install(runtime) first")
    if _INSTALLED.motor_idx >= len(_MOTOR_SLOTS):
        raise RuntimeError(
            "default chassis has only %d motor slots (2 wheels + 2 "
            "kinematic task motors); constructed too many motor "
            "objects" % len(_MOTOR_SLOTS))
    slot = _MOTOR_SLOTS[_INSTALLED.motor_idx]
    if slot is None and not kinematic_ok:
        # Refused WITHOUT consuming the slot: an encoder motor that
        # can't use a kinematic slot must not burn it for the serial
        # motor that can.
        raise RuntimeError(
            "default chassis has two encoder-motor slots; the third "
            "and fourth are kinematic task slots for serial servos "
            "(ST3215Motor / ST3032Motor) only")
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


class _SimYawTracker:
    """Continuous (multi-turn) chassis yaw — the sim stand-in for the
    firmware's hard-tick yaw integrator. SimIMU wraps to [-180, 180);
    the integrator does not, so unwrap here. Ground truth, no drift."""

    def __init__(self):
        self._imu = None
        self._cont = 0.0
        self._prev = None
        self._ref = 0.0

    def deg(self):
        if self._imu is None:
            from openbricks_sim.runtime import SimIMU
            self._imu = SimIMU(_INSTALLED.runtime)
        h = self._imu.heading()
        if self._prev is None:
            self._prev = h
        d = h - self._prev
        if d > 180.0:
            d -= 360.0
        elif d < -180.0:
            d += 360.0
        self._cont += d
        self._prev = h
        return self._cont - self._ref

    def reset(self):
        self.deg()
        self._ref = self._cont


_sim_yaw = None      # bound by install(), cleared by uninstall()


def _make_esp32_module():
    """Minimal ``esp32`` fake so firmware code that persists state to
    NVS (the ICM-45686's gyro-bias calibration, the hub name) runs
    unchanged in the sim. In-memory, fresh per install."""
    m = types.ModuleType("esp32")
    store = {}

    class NVS:
        def __init__(self, namespace):
            self._ns = str(namespace)

        def _key(self, key):
            return (self._ns, str(key))

        def set_i32(self, key, value):
            store[self._key(key)] = int(value)

        def get_i32(self, key):
            try:
                v = store[self._key(key)]
            except KeyError:
                raise OSError(-4354)      # ESP_ERR_NVS_NOT_FOUND
            if not isinstance(v, int):
                raise OSError(-4354)
            return v

        def set_blob(self, key, value):
            store[self._key(key)] = bytes(value)

        def get_blob(self, key, buf):
            try:
                v = store[self._key(key)]
            except KeyError:
                raise OSError(-4354)
            if not isinstance(v, (bytes, bytearray)):
                raise OSError(-4354)
            n = len(v)
            buf[:n] = v
            return n

        def commit(self):
            return None

    m.NVS = NVS
    return m


class _SimIcm45686:
    """The firmware's ``_native.icm45686`` singleton, sim edition —
    the REAL ``openbricks.drivers.icm45686.ICM45686`` class runs
    unchanged on top of it. Ground-truth chassis state stands in for
    the SPI part; the hard-yaw surfaces live on the motor_process
    stub, same split as firmware."""

    def __init__(self):
        self._configured = False
        self._reads = 0
        self._imu = None

    def config(self, **kwargs):
        if _INSTALLED is None:
            raise RuntimeError(
                "shim not installed; call install(runtime) first")
        from openbricks_sim.runtime import SimIMU
        self._imu = SimIMU(_INSTALLED.runtime)
        self._configured = True

    def read(self):
        if not self._configured:
            raise OSError("icm45686 not configured")
        self._reads += 1
        ax, ay, az = self._imu.acceleration()     # m/s^2 -> g
        gx, gy, gz = self._imu.angular_velocity()
        return (ax / 9.81, ay / 9.81, az / 9.81, gx, gy, gz)

    def stats(self):
        return (self._reads, 0, self._configured)

    def available(self):
        return True

    def selftest(self):
        # The canned-frame decode the firmware pins; same tuple.
        return (0, 258, 772, 1286, -2, 300, -5)


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
        # kinematic_ok=False: an encoder motor's whole point is the
        # physics behind it, so it can't take a kinematic task slot.
        sensor_name, actuator_name = _next_motor_slot(kinematic_ok=False)
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

    def measured_dps(self):
        # The firmware observer samples the encoder every tick even
        # while coasting; the shim detaches its tick on coast, which
        # would freeze ``observed_dps()`` at the last driven value.
        # Physics joint velocity is the always-live equivalent —
        # sign-corrected here because raw qvel bypasses the servo
        # core's invert handling.
        v = self._adapter.speed()
        return -v if self._adapter.invert else v

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


class _SimStBus:
    """The sim's emulation of the firmware ``st_bus`` surface — the
    seam that lets ``_SerialNativeEngine`` run UNCHANGED here (one
    code path, user decision 1.45.0). Wheel-level control law is the
    REAL C controller (``RawDriveBase`` = drivebase_core over bridge
    servos); this class is data plumbing, not a control loop: each
    sim tick feeds MuJoCo wheel angles in and applies the returned
    per-wheel dps setpoints to the shim wheels' velocity loops.

    Wire-level details (packets, slots-as-hardware, torque registers)
    are elided — attach/config calls succeed trivially; servo_run /
    servo_coast / servo_counts map straight onto the shim wheels."""

    _STEPS_PER_DEG = 4096 / 360.0

    def __init__(self, left_wheel, right_wheel, runtime):
        self._wheels = {0: left_wheel, 1: right_wheel}
        self._rt = runtime
        self._raw = None
        self._active = False
        # Firmware-parity arbitration (1.46.0): the drivebase owns
        # its wheels only from db_straight/db_turn until db_stop /
        # db_disable; yielded, per-slot moves and direct speed
        # commands own them.
        self._db_writing = False
        self._use_gyro = False
        self._gyro_hard = False
        self._ws_active = False
        self._ws_stop_pending = 0
        self._moves = {}
        self._slot_ids = {}
        _sim_st_buses.append(self)
        runtime.add_tick(self._tick)

    def _move(self, slot):
        if slot not in self._moves:
            from openbricks_sim._native import RawServoMove
            self._moves[slot] = RawServoMove()
        return self._moves[slot]

    # -- engine-facing surface ---------------------------------------

    def attach_uart(self, uart_id, baud, tx, rx):
        return True

    def servo_attach(self, slot, servo_id, invert, goal_acc):
        # invert intentionally ignored: the sim chassis mounts both
        # wheel hinges on one axis (see ShimST3215Motor docstring).
        #
        # Slot bookkeeping is firmware-faithful: an occupied slot
        # REFUSES, which is what makes the engine's first-free-slot
        # claim loop hand out 0 then 1. Without it both wheels
        # reported slot 0, and anything addressing the right wheel by
        # its claimed slot silently operated on the left one.
        if slot not in self._wheels or slot in self._slot_ids:
            return False
        self._slot_ids[slot] = int(servo_id)
        return True

    def servo_slot_of(self, servo_id):
        # One physical servo, one slot — a re-run reuses the claim
        # (firmware st_bus.servo_slot_of).
        for slot, sid in self._slot_ids.items():
            if sid == int(servo_id):
                return slot
        return -1

    def servo_detach(self, slot):
        self._slot_ids.pop(slot, None)

    def db_disable(self):
        self._active = False
        self._db_writing = False

    def db_config(self, slot_l, slot_r, wheel_mm, axle_mm, accel):
        from openbricks_sim._native import RawDriveBase
        self._raw = RawDriveBase(float(wheel_mm), float(axle_mm))
        self._raw.set_accel(float(accel))
        # Firmware parity: separate straight/turn accelerations,
        # both seeded from db_config, selected at arm time.
        self._accel_straight = float(accel)
        self._accel_turn = float(accel)
        self._active = True
        self._db_writing = False
        for m in self._moves.values():
            m.stop()

    def db_set_accel(self, dps2):
        self._accel_straight = float(dps2)

    def db_set_turn_accel(self, dps2):
        self._accel_turn = float(dps2)

    def servo_drive_duty(self, slot, on):
        # Firmware parity surface (st_bus.servo_drive_duty). The sim
        # wheel is an ideal plant already driven by our own model, so
        # "dumb mode" changes nothing here — but scripts that flip it
        # must run unchanged, and a bad slot must fail as loudly as
        # the firmware's ValueError.
        if not (0 <= int(slot) < 4):
            raise ValueError("servo_drive_duty: bad slot")
        self._duty_slots = getattr(self, "_duty_slots", set())
        if on:
            self._duty_slots.add(int(slot))
        else:
            self._duty_slots.discard(int(slot))

    def duty_gains(self, ff, kp, ki):
        # Accepted for parity; the sim plant needs no gain schedule.
        self._duty_gain_log = getattr(self, "_duty_gain_log", [])
        self._duty_gain_log.append((int(ff), int(kp), int(ki)))

    def _sync_bridges(self):
        # Firmware parity (st_bus.c sb_db_straight/sb_db_turn): arm
        # against LIVE odometry AND the live commanded speeds — the
        # arm reads the bridges' target_dps as the trajectory's entry
        # speed (2.0.0), so a straight() after move_wheels() blends
        # from cruise instead of cliffing.
        self._raw.sync(self._wheels[0].angle(), self._wheels[1].angle(),
                       getattr(self._wheels[0], "_target_dps", 0.0),
                       getattr(self._wheels[1], "_target_dps", 0.0))

    def db_straight(self, mm, mm_s, carry=0):
        for m in self._moves.values():
            m.stop()                      # new command wins
        self._sync_bridges()
        self._ws_active = False        # trajectory supersedes a slew
        self._ws_stop_pending = 0
        self._db_writing = True
        self._raw.set_accel(self._accel_straight)
        self._raw.straight(self._rt.now_ms, float(mm), float(mm_s),
                           bool(carry))

    def db_turn(self, deg, dps):
        for m in self._moves.values():
            m.stop()
        self._sync_bridges()
        self._ws_active = False        # trajectory supersedes a slew
        self._ws_stop_pending = 0
        self._db_writing = True
        self._raw.set_accel(self._accel_turn)
        self._raw.turn(self._rt.now_ms, float(deg), float(dps))

    def db_curve(self, radius_mm, deg, mm_s, carry=0):
        for m in self._moves.values():
            m.stop()
        self._sync_bridges()
        self._ws_active = False        # trajectory supersedes a slew
        self._ws_stop_pending = 0
        self._db_writing = True
        self._raw.set_accel(self._accel_straight)
        self._raw.curve(self._rt.now_ms, float(radius_mm), float(deg),
                        float(mm_s), bool(carry))

    def db_move_wheels(self, left_steps_per_s, right_steps_per_s):
        # Firmware parity (st_bus.c sb_db_move_wheels, ramped since
        # 1.94.0): per-wheel speeds slew at settings.acceleration —
        # proportionally, so the L:R ratio (a drive() arc's radius)
        # holds through the ramp — then the db yields with the wheels
        # holding the final speeds. Per-slot moves cancelled.
        self._raw.stop()
        for slot, w in self._wheels.items():
            self._move(slot).stop()
        # Continue from the wheels' last commanded speeds (dps).
        self._ws_cur = [getattr(self._wheels[0], "_target_dps", 0.0),
                        getattr(self._wheels[1], "_target_dps", 0.0)]
        self._ws_tgt = [left_steps_per_s / self._STEPS_PER_DEG,
                        right_steps_per_s / self._STEPS_PER_DEG]
        self._ws_last_ms = self._rt.now_ms
        self._ws_active = True
        self._ws_stop_pending = 0
        self._db_writing = True
        return True

    def _ws_tick(self, now_ms):
        dt_s = max(0.0, min(0.05, (now_ms - self._ws_last_ms) / 1000.0))
        self._ws_last_ms = now_ms
        max_step = self._accel_straight * dt_s
        diff = [t - c for t, c in zip(self._ws_tgt, self._ws_cur)]
        mag_max = max(abs(diff[0]), abs(diff[1]))
        if mag_max <= max_step:
            self._ws_cur = list(self._ws_tgt)
            self._ws_active = False
            self._db_writing = False   # ramp done: yield
            if self._ws_stop_pending == 2:
                for slot, w in self._wheels.items():
                    self._move(slot).hold_at(
                        w.angle() * self._STEPS_PER_DEG)
            self._ws_stop_pending = 0
        else:
            self._ws_cur = [c + max_step * d / mag_max
                            for c, d in zip(self._ws_cur, diff)]
        self._wheels[0].run_speed(self._ws_cur[0])
        self._wheels[1].run_speed(self._ws_cur[1])

    def db_stop(self, mode=None):
        # Firmware parity (see st_bus.c sb_db_stop): without ``mode``
        # yield only; 0 = coast (instant — freewheel has no
        # controlled deceleration); 1 = brake and 2 = hold DECELERATE
        # at settings.acceleration first (uniform-accel rule,
        # 2026-08-14), hold anchoring where the robot actually stops.
        self._raw.stop()
        if mode == 1 or mode == 2:
            for slot in self._wheels:
                self._move(slot).stop()
            self._ws_cur = [getattr(self._wheels[0], "_target_dps", 0.0),
                            getattr(self._wheels[1], "_target_dps", 0.0)]
            self._ws_tgt = [0.0, 0.0]
            self._ws_last_ms = self._rt.now_ms
            self._ws_active = True
            self._ws_stop_pending = mode
            self._db_writing = True
            return True
        self._db_writing = False          # yield to the motor layer
        for slot, w in self._wheels.items():
            w.run_speed(0)
            if mode == 0:
                self._move(slot).stop()
                w.coast()
        return True

    def db_done(self):
        return bool(self._raw.is_done())

    def db_use_gyro(self, enable):
        self._use_gyro = bool(enable)
        self._raw.set_use_gyro(bool(enable))

    def db_settle_stats(self):
        # (expiry_residual_wheel_deg, landings) — firmware parity.
        return self._raw.settle_stats()

    def db_gyro_in_use(self):
        # Firmware parity: the ICM driver's reset_heading guard.
        return bool(self._active and self._use_gyro)

    def db_reset(self):
        # Firmware parity (db_reset binding): re-zero yaw integrator,
        # frame reference, and held target together; refuse while a
        # move is active.
        if not self._active:
            raise RuntimeError("db_reset before db_config")
        if not self._raw.is_done():
            raise RuntimeError(
                "can't reset while a move is active — stop first")
        if self._use_gyro:
            if self._gyro_hard:
                _sim_yaw.reset()
                self._gyro_hard_ref = _sim_yaw.deg()
            # Frame reset via the enable transition — the same
            # "here, now is zero" the firmware binding performs.
            self._raw.set_use_gyro(False)
            self._raw.set_use_gyro(True)

    def db_gyro_source(self, mode):
        # Firmware parity: source 1 = the hard-tick yaw integrator
        # feeds the controller directly (the ICM-45686 path). The
        # engine SKIPS its Python pump for hard-source IMUs, so the
        # sim must feed heading itself — ref captured at selection,
        # exactly like the C side.
        if int(mode) == 1:
            self._gyro_hard_ref = _sim_yaw.deg()
            self._gyro_hard = True
        else:
            self._gyro_hard = False

    def db_set_heading(self, body_deg):
        self._raw.set_heading_override(float(body_deg))

    def servo_run(self, slot, steps_per_s):
        self._move(slot).stop()           # new command wins
        self._wheels[slot].run_speed(steps_per_s / self._STEPS_PER_DEG)
        return True

    def servo_coast(self, slot):
        self._move(slot).stop()
        self._wheels[slot].coast()
        return True

    def servo_counts(self, slot):
        return int(self._wheels[slot].angle() * self._STEPS_PER_DEG)

    def servo_stats(self, slot):
        # (reads_ok, reads_failed, stale). Sim wheels never go silent,
        # so this reports permanently healthy — the health CONTRACT is
        # exercised, the failure mode is a hardware one.
        return (1, 0, 0)

    def servo_write_stats(self, slot):
        # (writes_failed, config_failed). Sim writes always land, for
        # the same reason servo_stats is permanently healthy.
        return (0, 0)

    def db_fault(self):
        return 0

    def servo_feedback(self, slot):
        # (speed_steps, load_raw, fresh) — the firmware's widened
        # 6-byte read surface (1.50.0). Speed is the wheel's actual
        # velocity; the shim wheel model has no torque estimate, so
        # load reads 0 (documented sim limitation, not a silent
        # fake: fresh stays True because speed IS live).
        w = self._wheels[slot]
        return (int(w._vel_dps() * self._STEPS_PER_DEG), 0, True)

    def _slot_ready(self, slot):
        # Sim odometry is always live; only the db-ownership half of
        # the firmware gate applies.
        return slot in self._wheels and not (self._active
                                             and self._db_writing)

    def servo_move(self, slot, delta_counts, speed_cps, accel_cps2):
        if not self._slot_ready(slot):
            return False
        self._move(slot).start(
            self._rt.now_ms,
            self._wheels[slot].angle() * self._STEPS_PER_DEG,
            float(delta_counts), float(speed_cps), float(accel_cps2))
        return True

    def servo_hold(self, slot):
        if not self._slot_ready(slot):
            return False
        self._move(slot).hold_at(
            self._wheels[slot].angle() * self._STEPS_PER_DEG)
        return True

    def servo_move_done(self, slot):
        return slot in self._moves and bool(self._moves[slot].is_done())

    def reset_runtime(self):
        self._active = False
        self._db_writing = False
        self._raw = None
        for m in self._moves.values():
            m.stop()

    def torque_off_all(self):
        self._active = False
        self._db_writing = False
        for m in self._moves.values():
            m.stop()
        for w in self._wheels.values():
            w.coast()
        return True

    # -- sim step ------------------------------------------------------

    def _tick(self, now_ms):
        if self._raw is not None:
            if getattr(self, "_gyro_hard", False):
                self._raw.set_heading_override(
                    _sim_yaw.deg() - self._gyro_hard_ref)
            l = self._wheels[0].angle()
            r = self._wheels[1].angle()
            if self._active and self._db_writing:
                if self._ws_active:
                    # Wheel-speed slew replaces the trajectory tick;
                    # bridge odometry stays live for the next arm.
                    self._raw.sync(l, r)
                    self._ws_tick(now_ms)
                else:
                    lt, rt = self._raw.tick(now_ms, l, r)
                    self._wheels[0].run_speed(lt)
                    self._wheels[1].run_speed(rt)
            else:
                # Yielded (move_wheels / per-slot moves own the
                # wheels): keep the bridge odometry live anyway, like
                # the firmware's st_db_tick does on every hard tick —
                # the next straight()/turn() arms from the TRUE pose,
                # and a mid-move abort captures the TRUE pose for its
                # holds. Unsynced, a straight(50) after 2 s of
                # move_wheels drove the chassis backward 142.6 mm.
                self._raw.sync(l, r)
        for slot, m in self._moves.items():
            if m.is_active():
                cmd = m.tick(now_ms,
                             self._wheels[slot].angle()
                             * self._STEPS_PER_DEG)
                self._wheels[slot].run_speed(cmd / self._STEPS_PER_DEG)


class ShimST3215Motor:
    """Drop-in for ``openbricks.drivers.st3215.ST3215Motor`` (and the
    ``ST3032Motor`` marker subclass).

    On firmware these are serial-bus wheel servos: ``DriveBase``
    adopts them onto the hard-tick native engine. Under the sim the
    same engine runs — ``_adopt_into_drivebase`` hands it a
    ``_SimStBus`` emulating the ``st_bus`` surface — so a serial-bus
    ``main.py`` runs unchanged. Outside a drivebase, each shim motor
    binds the next chassis wheel slot and answers the wheel-mode
    Motor API (``run_speed`` + ``angle``) from MuJoCo.

    A real STS32xx runs its own internal wheel-mode velocity loop, so
    the shim implements one too: a per-tick P controller on the exact
    MuJoCo joint velocity (``data.qvel``). Deliberately *not* routed
    through :class:`SimMotor`'s count-based servo core — integer
    encoder-count quantisation at 1 kHz makes that observer's velocity
    estimate swing thousands of dps around a ~250 dps wheel, and the
    resulting bang-bang torque never accumulates the wheel rotation
    the drivebase's position check waits for. ``qvel`` is exact,
    and a plain P loop on it is flat at every gain we tested.

    Two firmware arguments are deliberately ignored as wiring
    concerns (like ``tx=`` / ``rx=`` / ``dir_pin=``):

    * ``invert=`` — compensates for mirrored physical mounting. The
      sim chassis defines both wheel hinges on the same axis, so
      +speed is already "forward" on both sides; honouring the flag
      would spin the robot in place.
    * bus identity (``servo_id`` / ``uart_id`` / ``baud``) — slot
      binding is by construction order (first constructed = left),
      same convention as :class:`ShimServo`.

    ``max_dps`` *is* honoured — the firmware driver clamps every
    speed command to it, and scripts tuned against that clamp should
    behave identically here.

    Scale caveat: wheel *rotation* tracks commands exactly, but
    millimetre travel reflects the sim model's wheel size. The
    serial path can't resize the chassis to the script's
    ``wheel_diameter_mm`` the way the native :class:`ShimDriveBase`
    path does (the openbricks wrapper never constructs a shim
    drivebase on this path), so treat sim distances as behavioural,
    not calibrated.
    """

    # Velocity-loop P gain, power-% per dps of error. Empirically
    # flat from 0.005 through 1.0 on the default chassis; 0.5 tracks
    # a 150 dps command to within ~1 dps.
    _KP_VEL = 0.5
    # run_angle approach shaping: decelerate so v² = 2·a·remaining,
    # with a crawl floor so friction can't stall short of the target.
    _DECEL_DPS2 = 720.0
    _MIN_APPROACH_DPS = 15.0

    def __init__(self, servo_id, uart_id=1, tx=17, rx=16,
                 baud=1_000_000, dir_pin=None,
                 invert=False, max_dps=600.0, **_ignored):
        slot = _next_motor_slot()
        rt = _INSTALLED.runtime
        self._rt = rt
        # Kinematic integrator state — used only when this motor got
        # a task-motor slot (no MuJoCo actuator); defined always so
        # the tick can branch on _plumb alone.
        self._kin_angle   = 0.0
        self._kin_vel     = 0.0
        self._kin_last_ms = None
        if slot is None:
            # Task-motor slot (third/fourth constructed): the default
            # chassis has two physical wheels, so this shaft
            # INTEGRATES its commanded speed instead of driving a
            # MuJoCo joint. run/run_angle/angle()/done() all behave;
            # load stays 0 and nothing pushes back — behavioural, not
            # physical, exactly like the bench robot's gripper motors
            # need for the script to run end-to-end.
            self._plumb = None
            self._dof = None
            self._actuator_id = None
        else:
            sensor_name, actuator_name = slot
            # Reuse SimMotor purely for the (sensor, actuator)
            # plumbing — ids, ctrl scale, raw angle read. Its servo
            # core is never ticked (see class docstring for why).
            self._plumb = SimMotor(rt, sensor_name, actuator_name)
            joint_id  = int(rt.model.sensor_objid[self._plumb._sensor_id])
            self._dof = int(rt.model.jnt_dofadr[joint_id])
            self._actuator_id = self._plumb._actuator_id

        self._max_dps      = float(max_dps)
        self._angle_offset = 0.0
        # Control mode for the per-tick loop:
        #   "speed" — hold self._target_dps (0.0 == active brake/hold)
        #   "angle" — decel-shaped approach to self._move["target"]
        #   "idle"  — attached but commanding zero torque (coast; the
        #             tick can't detach itself mid-iteration, so a
        #             move that ends inside the tick parks here)
        self._mode       = "idle"
        self._target_dps = 0.0
        self._move       = None
        self._attached   = False

    # ----- tick loop -------------------------------------------------

    def _attach(self):
        if not self._attached:
            # A kinematic shaft must not integrate across the time it
            # spent detached (coasted): restart the dt clock.
            self._kin_last_ms = None
            self._rt.add_tick(self._tick)
            self._attached = True

    def _detach(self):
        if self._attached:
            self._rt.remove_tick(self._tick)
            self._attached = False

    def _vel_dps(self):
        if self._plumb is None:
            return self._kin_vel
        return float(self._rt.data.qvel[self._dof]) * (180.0 / math.pi)

    def _apply_v(self, now_ms, v_cmd):
        """Drive the shaft. ``None`` = coast (zero torque). Physical
        slots run the P loop into the DC-motor model; kinematic task
        slots integrate the command directly (massless shaft — the
        honest analogue of a free gripper motor with no sim body)."""
        if self._plumb is not None:
            if v_cmd is None:
                self._rt.data.ctrl[self._actuator_id] = 0.0
                return
            power = self._KP_VEL * (v_cmd - self._vel_dps())
            if power >  100.0: power =  100.0
            if power < -100.0: power = -100.0
            # Through the shared DC-motor model — a serial servo's
            # inner loop is still a DC motor behind a controller.
            self._plumb.apply_power(power)
            return
        dt = 0.0
        if self._kin_last_ms is not None:
            dt = (now_ms - self._kin_last_ms) / 1000.0
        self._kin_last_ms = now_ms
        self._kin_vel = 0.0 if v_cmd is None else v_cmd
        self._kin_angle += self._kin_vel * dt

    def _tick(self, now_ms):
        if self._mode == "idle":
            self._apply_v(now_ms, None)
            return
        if self._mode == "angle":
            mv = self._move
            remaining = mv["target"] - self.angle()
            if remaining * mv["direction"] <= mv["tol"]:
                # Reached (or crossed) the target. Can't detach from
                # inside the tick loop; park in the end-state mode.
                self._move = None
                if mv["then"] in (Stop.BRAKE, Stop.HOLD):
                    self._mode = "speed"      # active zero velocity
                    self._target_dps = 0.0
                else:
                    self._mode = "idle"       # coast
                    self._apply_v(now_ms, None)
                    return
                v_cmd = 0.0
            else:
                v = math.sqrt(2.0 * self._DECEL_DPS2 * abs(remaining))
                if v > mv["cruise"]:
                    v = mv["cruise"]
                if v < self._MIN_APPROACH_DPS:
                    v = self._MIN_APPROACH_DPS
                v_cmd = v * mv["direction"]
        else:   # "speed"
            v_cmd = self._target_dps
        self._apply_v(now_ms, v_cmd)

    def _clamp(self, dps):
        if dps >  self._max_dps: return  self._max_dps
        if dps < -self._max_dps: return -self._max_dps
        return dps

    def _adopt_into_drivebase(self, right, wheel_diameter_mm,
                              axle_track_mm, imu=None, accel_dps2=400.0,
                              drive=DriveMode.DUTY):
        """DriveBase adoption hook, sim edition: the engine runs
        UNCHANGED against the emulated bus."""
        from openbricks.robotics.native_drivebase import _SerialNativeEngine
        if self._plumb is None or right._plumb is None:
            raise RuntimeError(
                "sim DriveBase wheels must be the first two motors "
                "constructed — the third and fourth are kinematic "
                "task-motor stand-ins with no chassis body (the sim "
                "binds motors by construction order, not servo id)")
        emu = _SimStBus(self, right, self._rt)
        return _SerialNativeEngine(
            left_id=1, right_id=2,
            wheel_diameter_mm=wheel_diameter_mm,
            axle_track_mm=axle_track_mm, imu=imu,
            accel_dps2=accel_dps2, sb=emu, drive=drive)

    # ----- Motor interface -------------------------------------------

    def run(self, power):
        p = max(-100.0, min(100.0, float(power)))
        self.run_speed(self._max_dps * p / 100.0)

    def run_speed(self, deg_per_s):
        self._mode = "speed"
        self._move = None
        self._target_dps = self._clamp(float(deg_per_s))
        self._attach()

    def brake(self):
        # Actively hold zero velocity — same net behaviour as the
        # servo's wheel-mode brake.
        self.run_speed(0.0)

    def hold(self):
        # No position PID in the shim; active zero velocity plus
        # MuJoCo joint damping is the honest analogue.
        self.run_speed(0.0)

    def coast(self):
        self._mode = "idle"
        self._move = None
        self._detach()
        if self._plumb is not None:
            self._rt.data.ctrl[self._actuator_id] = 0.0
        else:
            self._kin_vel = 0.0

    def _raw_angle(self):
        return (self._kin_angle if self._plumb is None
                else self._plumb.angle())

    def angle(self):
        return self._raw_angle() - self._angle_offset

    def reset_angle(self, angle=0):
        self._angle_offset = self._raw_angle() - float(angle)

    def speed(self):
        # Measured shaft speed in deg/s — Motor API parity with the
        # firmware driver (present-speed register / hard-tick pump).
        # The sim model's velocity IS the measurement; never silent,
        # so never None.
        return self._vel_dps()

    def ping(self):
        return True

    # ----- run_angle / done ------------------------------------------

    def run_angle(self, deg_per_s, target_angle, wait=True,
                  tolerance_deg=0.5, then=Stop.COAST, **_ignored):
        """Rotate by ``target_angle`` (relative, unbounded) at up to
        ``deg_per_s``, ending within ``tolerance_deg``. Firmware
        tuning knobs (``kp``, ``poll_ms``, ``debug``) are accepted
        and ignored — the shim's velocity loop handles tracking."""
        delta = float(target_angle)
        self._move = {
            "target":    self.angle() + delta,
            "direction": 1.0 if delta >= 0 else -1.0,
            "cruise":    abs(self._clamp(float(deg_per_s))),
            "tol":       float(tolerance_deg),
            "then":      then,
        }
        self._mode = "angle"
        self._attach()
        if not wait:
            return
        # Bounded like the firmware's stall budget (trapezoid time
        # x4 + 1 s): a physically blocked wheel (chassis against a
        # wall, jammed arm) defeats the crawl floor, and an unbounded
        # loop turned that into a CI timeout instead of a named
        # stall. Firmware reports and continues; so does the sim.
        cruise = self._move["cruise"] if self._move["cruise"] > 1 else 1
        budget_ms = int(abs(delta) / cruise * 1000) * 4 + 1000
        waited = 0
        while self._mode == "angle":
            _real_time.sleep_ms(10)
            waited += 10
            if waited > budget_ms:
                self._mode = "idle"
                self._move = None
                print("openbricks-sim: run_angle(%g deg) gave up "
                      "after %d ms — the wheel is blocked (wall, "
                      "jam?). Same report-and-continue contract as "
                      "the firmware stall detector."
                      % (delta, budget_ms))
                return False
        return True

    def done(self):
        return self._mode != "angle"


class ShimST3032Motor(ShimST3215Motor):
    """Drop-in for ``openbricks.drivers.st3032.ST3032Motor`` — the
    firmware class is a marker subclass of ``ST3215Motor``, and so is
    the shim, except for one thing: the firmware class raises the
    default ``max_dps`` from 600 (the ST-3215's clamp, wrong for this
    smaller/faster servo) to the ST-3032's actual no-load speed. A
    default-constructed shim motor must honour the same ceiling or a
    script tuned against real ST-3032 hardware quietly wouldn't reach
    its commanded speed in the sim.
    """

    # Must match ``ST3032_NO_LOAD_DPS`` in
    # ``openbricks/drivers/st3032.py`` (datasheet no-load speed,
    # 0.067 s/60° at 12 V). Not imported directly — that module
    # chain reaches ``from machine import UART, Pin`` at import time,
    # which only resolves once the shim's fake ``machine`` is
    # installed, so a plain top-of-file import here would race it.
    _ST3032_NO_LOAD_DPS = 888.0

    def __init__(self, servo_id, uart_id=1, tx=17, rx=16,
                 baud=1_000_000, dir_pin=None, invert=False,
                 max_dps=_ST3032_NO_LOAD_DPS, **_ignored):
        super().__init__(servo_id, uart_id=uart_id, tx=tx, rx=rx,
                         baud=baud, dir_pin=dir_pin, invert=invert,
                         max_dps=max_dps, **_ignored)


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

    # Mux channel -> chassis camera. Real code addresses the two
    # sensors by mux channel (``TCS34725(mux[1])``), so that is the
    # natural key: the same construction that picks a physical
    # sensor picks the camera under it. Channels with no mapping —
    # and a sensor built without a mux at all — get the centre
    # camera, which is what every existing script and test expects.
    _CHANNEL_CAMERAS = {0: "chassis_cam_down_r", 1: "chassis_cam_down_l"}

    def __init__(self, *args, **kwargs):
        if _INSTALLED is None:
            raise RuntimeError(
                "shim not installed; call install(runtime) first")
        bus = kwargs.get("i2c", args[0] if args else None)
        channel = getattr(bus, "_channel", None)
        if channel is None:
            # No mux: the single documented default camera.
            camera = "chassis_cam_down"
        elif channel in self._CHANNEL_CAMERAS:
            camera = self._CHANNEL_CAMERAS[channel]
        else:
            # An unmapped channel silently fell back to the centre
            # camera — plausible readings from the WRONG sensor (the
            # bench has a third TCS34725 on channel 2). A sensor the
            # sim cannot model must say so, not impersonate another.
            raise RuntimeError(
                "sim chassis has no camera for mux channel %r — "
                "mapped channels: %s (and no-mux uses the centre "
                "camera). Add a camera to the chassis model or move "
                "the sensor." % (channel,
                                 sorted(self._CHANNEL_CAMERAS)))
        self._channel = channel
        self._cs = SimColorSensor(_INSTALLED.runtime, camera_name=camera)

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
        self._left     = left
        self._right    = right
        self._imu      = imu
        self._use_gyro = False
        if imu is not None:
            self._db.attach_imu(imu)

    # ----- Move setup ----------------------------------------------

    def straight(self, distance_mm, speed_mm_s, carry=False):
        # Heading re-baseline per move happens inside SimDriveBase.
        self._db.straight(float(distance_mm), float(speed_mm_s),
                          bool(carry))

    def turn(self, angle_deg, rate_dps):
        self._db.turn(float(angle_deg), float(rate_dps))

    def curve(self, radius_mm, angle_deg, speed_mm_s, carry=False):
        self._db.curve(float(radius_mm), float(angle_deg),
                       float(speed_mm_s), bool(carry))

    def move_wheels(self, left_dps, right_dps):
        # Firmware parity (drivebase.c db_move_wheels): the coupled
        # controller yields, both wheels take their own speed.
        self._db.stop()
        self._left.run_speed(float(left_dps))
        self._right.run_speed(float(right_dps))

    def stop(self, mode=None):
        # Firmware parity (see drivebase.c db_stop): without ``mode``
        # halt the controller only; with it (0 = coast, 1 = brake)
        # the end state applies to BOTH wheels in this one call.
        # SimDriveBase already subscribes both motors in one call, so
        # arming needs no equivalent here.
        self._db.stop()
        if mode == 0:
            self._left.coast()
            self._right.coast()
        elif mode == 1:
            self._left.brake()
            self._right.brake()

    def is_done(self):
        return self._db.is_done()

    def set_accel(self, accel_dps2):
        # Same surface as the firmware binding, so the openbricks
        # wrapper's ``settings(acceleration=...)`` works under the sim.
        self._db.set_accel(float(accel_dps2))

    def use_gyro(self, enable):
        if enable and self._imu is None:
            raise RuntimeError(
                "use_gyro requires imu= at construction")
        # SimDriveBase owns the heading-feed tick (attach_imu was
        # called at construction); one implementation for both the
        # firmware-style shim path and the sim-native API.
        self._db.set_use_gyro(bool(enable))
        self._use_gyro = bool(enable)


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
    # The ICM-45686 driver is pure Python over these two surfaces —
    # the real class runs unchanged in the sim (no Shim* variant).
    m.icm45686           = _SimIcm45686()
    # ``motor_process`` — the firmware exposes it as a singleton
    # object with .start / .stop / .tick / .is_running. The sim's
    # SimRuntime.step() is the equivalent; expose a stub that
    # delegates so user code calling motor_process.start() doesn't
    # crash. (Rarely used by user-facing code; mostly internal.)
    m.motor_process = _MotorProcessStub()
    # NOTE: no ``st_bus`` here, ON PURPOSE — consumers select the
    # firmware serial-bus path by attribute presence (see
    # openbricks/_native.py). The reset-heading guard lives in
    # _MotorProcessStub.hard_yaw_reset instead, scanning the emulated
    # buses; a fresh install starts with none registered.
    del _sim_st_buses[:]
    return m


# Live _SimStBus instances of the CURRENT install — the sim's
# hard_yaw_reset guard scans them (the firmware has one global drive
# base; the sim can construct several).
_sim_st_buses = []


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

    # Hard-tick yaw surfaces (the ICM-45686 driver's heading source).
    # Ground-truth chassis yaw; the bias machinery reports "locked"
    # immediately — the sim has no bias to learn.
    def hard_tick_selftest(self):
        return True

    def hard_yaw_deg(self):
        return _sim_yaw.deg()

    def hard_yaw_reset(self):
        # Firmware parity (motor_process.c mp_hard_yaw_reset): the
        # refusal is enforced at this binding on both platforms.
        if any(b.db_gyro_in_use() for b in _sim_st_buses):
            raise OSError("can't reset heading while a drive base "
                          "is using the gyro - use db.reset() "
                          "instead")
        _sim_yaw.reset()

    def hard_yaw_state(self):
        return (0.0, True, True)

    def hard_yaw_seed_bias(self, bias):
        return None


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
            ("esp32",               _make_esp32_module),
            ("_openbricks_native",  _make_native_module),
    ]:
        state.prev_sys_modules[name] = sys.modules.get(name)
        sys.modules[name] = factory()
    global _sim_yaw
    _sim_yaw = _SimYawTracker()

    # 2. time patches — only patch the attributes that exist (or
    # plant new ones for sleep_ms / ticks_ms which don't exist on
    # CPython). Restore exactly on uninstall.
    for attr, patched in [
            ("sleep",      _patched_sleep),
            ("sleep_ms",   _patched_sleep_ms),
            ("ticks_ms",   _patched_ticks_ms),
            ("ticks_us",   _patched_ticks_us),
            ("ticks_diff", _patched_ticks_diff),
            # Wrap-safe deadline arithmetic (the sim clock is a plain
            # int, so a sum suffices; real ports wrap at 2^30 ms).
            ("ticks_add",  lambda t, d: t + d),
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
        ("openbricks.drivers.tcs34725", "TCS34725",    ShimTCS34725),
        ("openbricks.drivers.hcsr04",   "HCSR04",      ShimHCSR04),
        ("openbricks.drivers.vl53l0x",  "VL53L0X",     ShimVL53L0X),
        ("openbricks.drivers.vl53l1x",  "VL53L1X",     ShimVL53L1X),
        # Serial-bus wheel servos: the firmware classes drive UART
        # directly (no ``_openbricks_native`` involvement), so like
        # the I2C drivers they're replaced at the class level. The
        # openbricks DriveBase then adopts the shim motors onto the
        # emulated st_bus (_SimStBus) — the same engine code path
        # as hardware.
        ("openbricks.drivers.st3215",   "ST3215Motor", ShimST3215Motor),
        ("openbricks.drivers.st3032",   "ST3032Motor", ShimST3032Motor),
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

    # 1b. Evict every ``openbricks.*`` module imported WHILE the shim
    # was installed: their module-level ``from machine import UART``
    # bindings captured the fakes, and the cached module would keep
    # serving those to a later import in the same process — where a
    # clean CPython import of e.g. ``openbricks.drivers.st3215``
    # would fail on ``machine``. Order-dependent test poison.
    for name in [n for n in list(sys.modules)
                 if n == "openbricks" or n.startswith("openbricks.")]:
        if name not in state.prev_sys_modules:
            sys.modules.pop(name, None)

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
