# SPDX-License-Identifier: MIT
"""
_SerialNativeEngine — the serial-bus drivebase engine on the
hard-tick controller. PRIVATE: users construct ``DriveBase`` with
Motor objects and it adopts them onto this engine automatically when
the firmware native bus exists — there is exactly one drivebase
class (user decision, 1.45.0; the short-lived public NativeDriveBase
of 1.43.x is gone).

The 2-DOF coupled controller runs entirely in C on the esp_timer hard
tick (see ``st_bus``): ~220 Hz odometry per wheel, speed setpoints in
sync-write packets, immune to anything Python does — a 981 ms Python
stall that freezes the classic DriveBase's control loop does not
perturb this one. Floor-verified: closed square with 0.3 % odometry
closure at bench speeds.

UART double-ownership is solved by ADOPTION, not by a second public
class: ``ST3032Motor`` objects open ``machine.UART`` in their
constructor, so ``adopt_motors`` releases that UART (``deinit`` +
registry removal) before the native IDF driver claims the pins. The
adopted Motor objects stay usable — their wheel-mode API is rerouted
through the engine's servo slots, and since 1.46.0 ``run_angle`` /
``hold`` run as per-slot position moves on the hard tick
(st_move_core). The drivebase and per-slot moves arbitrate by
yielding: the db owns its wheels only while one of ITS moves is in
flight.

Gyro: pass an ``imu`` and call ``use_gyro(True)`` on the DriveBase —
the wait loop inside ``straight()`` / ``turn()`` reads the IMU at
~50-100 Hz and feeds the heading to the C controller
(``db_set_heading``). The outer loop lives in the wait loop on
purpose: correction matters exactly while a move is in flight, and
this avoids burning a hardware timer (all four are spoken for).

Example (the engine is invisible — this is just DriveBase)::

    from openbricks.drivers.bno055 import BNO055
    from openbricks.drivers.st3032 import ST3032Motor
    from openbricks.drivers.tca9548a import TCA9548A
    from openbricks.robotics import DriveBase
    from machine import I2C, Pin

    mux = TCA9548A(I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000))
    imu = BNO055(i2c=mux[3], address=0x29)
    left  = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)
    right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
    db = DriveBase(left, right, wheel_diameter_mm=88,
                   axle_track_mm=138, imu=imu)
    db.use_gyro(True)
    db.straight(300)
    db.turn(90)
"""

import math
import time

from openbricks import estop


_STEPS_PER_DEG = 4096 / 360.0
_LEFT_SLOT, _RIGHT_SLOT = 0, 1


def _bus():
    """The native bus module, or an informative error. A seam so the
    test suite can exercise the full class against a recording fake
    (the real module's firmware backend only exists on the hub)."""
    from openbricks import _native
    sb = getattr(_native, "st_bus", None)
    if sb is None or not hasattr(sb, "attach_uart"):
        raise RuntimeError(
            "the serial drivebase engine needs the firmware native bus "
            "(st_bus with attach_uart) — not available on this build")
    return sb


class _SerialNativeEngine:
    @classmethod
    def adopt_motors(cls, left, right, wheel_diameter_mm,
                     axle_track_mm, imu=None, accel_dps2=400.0):
        """Adopt two constructed serial-bus Motor objects: recover the
        bus params from the driver registry, RELEASE their
        machine.UART (explicit ownership handover — the double-claim
        trap is why a separate public class briefly existed), then
        run this engine and re-point the motors' wheel-mode API at
        the slots."""
        from openbricks.drivers.st3215 import ST3215
        bus = left._bus
        if right._bus is not bus:
            raise ValueError("left and right motors must share one bus")
        params = None
        for key, val in ST3215._buses.items():
            if val is bus:
                params = key
                break
        if params is None:
            raise RuntimeError("motor bus not found in the registry")
        uart_id, tx, rx, baud = params
        # Hand the UART over: MicroPython driver out, IDF driver in.
        bus._uart.deinit()
        del ST3215._buses[params]
        eng = cls(left_id=left._id, right_id=right._id,
                  wheel_diameter_mm=wheel_diameter_mm,
                  axle_track_mm=axle_track_mm, imu=imu,
                  invert_left=left._invert, invert_right=right._invert,
                  uart_id=uart_id, tx=tx, rx=rx, baud=baud,
                  accel_dps2=accel_dps2)
        left._adopt_native(eng._sb, 0)
        right._adopt_native(eng._sb, 1)
        return eng

    def __init__(self, left_id, right_id, wheel_diameter_mm,
                 axle_track_mm, imu=None,
                 invert_left=False, invert_right=False,
                 uart_id=1, tx=14, rx=6, baud=1_000_000,
                 accel_dps2=400.0, sb=None):
        # ``sb`` is the bus-surface seam: firmware injects the real
        # st_bus (default), the sim injects its emulation — the ONE
        # engine code path serves both worlds.
        self._sb = sb if sb is not None else _bus()
        self._wheel_circumference = math.pi * wheel_diameter_mm
        self._axle_track = float(axle_track_mm)
        self._imu = imu
        self._use_gyro = False
        # Continuous-heading frame (absolute target frame,
        # Pybricks-style — overshoot is corrected by the NEXT move,
        # not accumulated).
        self._gyro_cont = 0.0
        self._gyro_prev = None
        self._deadline = 0
        self._straight_speed_dps = 200
        self._turn_rate_dps = 150
        # Kept for diagnostics: a dead-motor message is only useful if
        # it says WHICH motor and where it's wired.
        self._left_id, self._right_id = left_id, right_id
        self._uart_id, self._tx, self._rx = uart_id, tx, rx

        try:
            from openbricks._native import motor_process
            motor_process.hard_tick_selftest()  # dispatcher on (idempotent)
        except (ImportError, AttributeError):
            pass    # sim / stub worlds have no hard tick to arm
        # Same-boot re-construction: a previous run's slots and
        # drivebase survive in the C singletons (openbricks run keeps
        # the interpreter alive between scripts), and servo_attach
        # rejects an in-use slot — so a second run of the same script
        # failed with "slot attach failed" until a power-cycle. Tear
        # down our own claims first; detach of an unclaimed slot is
        # silent, so a fresh boot pays nothing.
        self._sb.db_disable()
        self._sb.servo_detach(_LEFT_SLOT)
        self._sb.servo_detach(_RIGHT_SLOT)
        if not self._sb.attach_uart(uart_id, baud, tx, rx):
            raise RuntimeError("attach_uart(%d) failed" % uart_id)
        acc = int(accel_dps2 * _STEPS_PER_DEG / 100.0)
        acc = 0 if acc < 0 else (254 if acc > 254 else acc)
        if not self._sb.servo_attach(_LEFT_SLOT, left_id,
                                     bool(invert_left), acc):
            raise RuntimeError("left servo slot attach failed")
        if not self._sb.servo_attach(_RIGHT_SLOT, right_id,
                                     bool(invert_right), acc):
            raise RuntimeError("right servo slot attach failed")
        self._sb.db_config(_LEFT_SLOT, _RIGHT_SLOT,
                           float(wheel_diameter_mm), float(axle_track_mm),
                           float(accel_dps2))
        # Wiring/ID/power problems are found HERE, at construction,
        # not as mysterious non-motion later: attaching a slot only
        # claims it in C, it never asks the servo whether it exists.
        self._require_live_wheels()

    # -- motor health ----------------------------------------------------
    #
    # A serial wheel that stops answering is invisible to every layer
    # above it unless someone looks: ``servo_attach`` only claims a
    # slot, the controller happily integrates a frozen odometry
    # reading, and a fire-and-forget speed command has nothing to
    # wait for. So the engine checks explicitly, and every message
    # names the motor — side, bus id, slot, UART and pins — because
    # "nothing moved" is the least actionable error a robot can give.

    _LIVE_WHEEL_TIMEOUT_MS = 400     # ~130 read attempts per wheel

    def _wheel_desc(self, slot):
        side = "left" if slot == _LEFT_SLOT else "right"
        ids = {_LEFT_SLOT: self._left_id, _RIGHT_SLOT: self._right_id}
        return ("%s wheel (servo id %s, slot %d) on UART%s tx=%s rx=%s"
                % (side, ids[slot], slot, self._uart_id,
                   self._tx, self._rx))

    def _wheel_evidence(self, slot):
        stats = getattr(self._sb, "servo_stats", None)
        if stats is None:
            return ""
        ok, failed, stale = stats(slot)
        return (" — %d replies, %d failed reads (%d in a row)"
                % (ok, failed, stale))

    def _dead_wheel_error(self, slot, headline):
        return OSError(
            "%s: %s%s. Check the servo's power and TX/RX wiring, and "
            "that it really has that bus id — `openbricks servo-id "
            "--scan` lists the ids actually answering on the bus."
            % (headline, self._wheel_desc(slot),
               self._wheel_evidence(slot)))

    def _require_live_wheels(self):
        """Both wheels must answer at least one feedback read before
        we hand the user a drivebase. Raises naming the silent one."""
        stats = getattr(self._sb, "servo_stats", None)
        if stats is None:
            return                  # bus surface without health data
        deadline = time.ticks_ms() + self._LIVE_WHEEL_TIMEOUT_MS
        pending = [_LEFT_SLOT, _RIGHT_SLOT]
        while pending:
            pending = [s for s in pending if stats(s)[0] == 0]
            if not pending:
                return
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                raise self._dead_wheel_error(
                    pending[0], "motor is not responding on the bus")
            time.sleep_ms(10)

    def check_motors(self):
        """Raise if a wheel has gone silent since the last check.

        The C tick latches the fault and stops driving (a frozen
        odometry reading otherwise winds that wheel's command to the
        rail), so this converts the latch into a diagnosis."""
        fault = getattr(self._sb, "db_fault", None)
        if fault is None:
            return
        bits = fault()
        if not bits:
            return
        slot = _LEFT_SLOT if bits & 0x01 else _RIGHT_SLOT
        raise self._dead_wheel_error(
            slot, "motor stopped responding mid-move; the drivebase "
                  "halted to stop it running away")

    # -- configuration ---------------------------------------------------

    def settings(self, straight_speed=None, turn_rate=None):
        """Cruise parameters in WHEEL-deg/s — the same units as the
        classic ``DriveBase.settings`` for drop-in parity."""
        if straight_speed is not None:
            self._straight_speed_dps = straight_speed
        if turn_rate is not None:
            self._turn_rate_dps = turn_rate

    def use_gyro(self, enable):
        """Heading feedback from the IMU instead of the wheel
        differential. Requires ``imu`` at construction.

        Hard-source IMUs (``_hard_heading_source`` marker — the
        ICM-45686) feed the controller INSIDE the hard tick at
        1 kHz: the C tick pulls the yaw integrator directly
        (``db_gyro_source(1)``) and the Python pump is skipped
        entirely. Fused/I2C IMUs (BNO055) keep the classic pump."""
        enable = bool(enable)
        if enable and self._imu is None:
            raise ValueError("use_gyro(True) needs an imu")
        self._hard_gyro = bool(enable and getattr(
            self._imu, "_hard_heading_source", False))
        if enable and not self._use_gyro and not self._hard_gyro:
            # Fresh absolute frame: current heading is zero/target.
            self._gyro_cont = 0.0
            self._gyro_prev = self._imu.heading()
        self._use_gyro = enable
        self._sb.db_use_gyro(enable)
        gyro_source = getattr(self._sb, "db_gyro_source", None)
        if gyro_source is not None:
            # Selecting source 1 captures the frame reference in C.
            gyro_source(1 if self._hard_gyro else 0)

    # -- moves -----------------------------------------------------------

    def set_accel(self, accel_dps2):
        self._sb.db_set_accel(float(accel_dps2))

    def arm_straight(self, distance_mm):
        estop.check()
        mm_s = self._straight_speed_dps * self._wheel_circumference / 360.0
        self._sb.db_straight(float(distance_mm), float(mm_s))
        self._arm_deadline()

    def straight(self, distance_mm):
        self.arm_straight(distance_mm)
        self._wait()

    def arm_turn(self, angle_deg):
        """Body degrees, CW-positive (Pybricks convention)."""
        estop.check()
        # turn_rate is WHEEL-deg/s (settings parity with the classic
        # DriveBase); the C API takes BODY deg/s. One body-degree of
        # turn-in-place is pi*axle/360 mm of arc per wheel, and one
        # wheel-degree is circumference/360 mm — so:
        body_dps = (self._turn_rate_dps * self._wheel_circumference
                    / (math.pi * self._axle_track))
        self._sb.db_turn(float(angle_deg), float(body_dps))
        self._arm_deadline()

    def turn(self, angle_deg):
        self.arm_turn(angle_deg)
        self._wait()

    def move_wheels(self, left_wheel_speed, right_wheel_speed):
        """Independent per-wheel speeds (wheel-deg/s), both staged in
        one C critical section so they leave in a single sync-write
        packet — the drivebase-owned equivalent of a SyncServoGroup
        over the two wheels (which adoption makes unreachable: the
        motors' MicroPython UART is gone)."""
        estop.check()
        # Nothing downstream waits for these, so a silent wheel would
        # never surface — check before commanding. In a control loop
        # the failure lands on the next iteration.
        self.check_motors()
        ok = self._sb.db_move_wheels(
            int(float(left_wheel_speed) * _STEPS_PER_DEG),
            int(float(right_wheel_speed) * _STEPS_PER_DEG))
        if not ok:
            raise RuntimeError(
                "move_wheels refused: the drivebase has no slots "
                "configured")

    _STOP_MODES = {"coast": 0, "brake": 1, "hold": 2}

    def stop(self, then=None):
        """Stop the drivebase and yield the wheels.

        Without ``then`` the engine only yields (abort paths — the
        caller dispatches the wheels' end-state itself). With
        ``then`` the complete stop is staged atomically in C, so
        both wheels reach the end-state at the same bus-packet
        boundary: coast is one sync-torque write covering both
        servos, brake one sync-speed write, and hold captures both
        wheel poses at the same instant — instead of one motor at a
        time, a bus transaction apart."""
        if then is None:
            self._sb.db_stop()
            return
        ok = self._sb.db_stop(self._STOP_MODES[then])
        if then == "hold" and not ok:
            raise RuntimeError(
                "hold refused: slot odometry is not live yet")

    def done(self):
        return bool(self._sb.db_done())

    # -- internals -------------------------------------------------------

    def _gyro_pump(self):
        """One outer-loop iteration: read the IMU, unwrap across the
        +/-180 boundary into the continuous frame, feed the C
        controller. ~50-100 Hz from the wait loop. No-op on the
        hard source — the C tick feeds itself at 1 kHz."""
        if getattr(self, "_hard_gyro", False):
            return
        h = self._imu.heading()
        d = h - self._gyro_prev
        if d > 180.0:
            d -= 360.0
        elif d < -180.0:
            d += 360.0
        self._gyro_cont += d
        self._gyro_prev = h
        self._sb.db_set_heading(self._gyro_cont)

    _SETTLE_TIMEOUT_MS = 8000

    def _arm_deadline(self):
        self._deadline = time.ticks_ms() + self._SETTLE_TIMEOUT_MS

    def tick_done(self):
        """One non-blocking iteration of the drive loop: gyro pump,
        settle-timeout check, completion check. DriveBase's done()
        polls this for wait=False moves; _wait() below is the
        blocking form of the same loop."""
        estop.check()
        # A silent wheel fails HERE, in ~200 ms with the motor named,
        # rather than burning the full settle timeout and blaming
        # "stalled or gyro diverged".
        self.check_motors()
        if self._use_gyro:
            self._gyro_pump()
        if self._sb.db_done():
            return True
        if time.ticks_diff(self._deadline, time.ticks_ms()) <= 0:
            self._sb.db_stop()
            raise RuntimeError(self._settle_timeout_message())
        return False

    def _settle_timeout_message(self):
        """The move ran out of time with both wheels still talking —
        so this is mechanical. Include each wheel's traffic anyway;
        an asymmetry between them localises the problem."""
        msg = ("DriveBase move did not reach target within %d ms — "
               "wheel stalled, blocked, or gyro frame diverged"
               % self._SETTLE_TIMEOUT_MS)
        stats = getattr(self._sb, "servo_stats", None)
        if stats is None:
            return msg
        return msg + (" [left%s; right%s]"
                      % (self._wheel_evidence(_LEFT_SLOT),
                         self._wheel_evidence(_RIGHT_SLOT)))

    def _wait(self):
        deadline = self._deadline
        while True:
            estop.check()
            # BEFORE db_done: halting on a dead wheel latches the
            # controller's ``done`` flag, so testing done first would
            # let a faulted move exit the wait reporting success —
            # the silent-failure shape this whole check exists to
            # kill.
            self.check_motors()
            if self._sb.db_done():
                return
            if self._use_gyro:
                self._gyro_pump()
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                # done now requires ARRIVAL, not just profile expiry
                # (the +4.5-deg banked-overshoot fix) — so a wheel
                # that physically can't reach the target must raise
                # (classic stall-timeout contract).
                self._sb.db_stop()
                raise RuntimeError(self._settle_timeout_message())
            time.sleep_ms(10)
