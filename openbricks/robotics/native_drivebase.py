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
    left  = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41, invert=True)
    right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
    db = DriveBase(left, right, wheel_diameter_mm=88,
                   axle_track_mm=138, imu=imu)
    db.use_gyro(True)
    db.straight(300)
    db.turn(90)
"""

import math
import time

from openbricks import estop
from openbricks import parameters
from openbricks.parameters import Stop, DriveMode


_STEPS_PER_DEG = 4096 / 360.0
# Slots are claimed first-come, not reserved by role. Reserving 0/1
# for wheels meant a script that built its task motors first ran out
# of slots before the DriveBase was reached, on a robot that fits the
# hardware exactly (4 motors, 4 slots).
_ALL_SLOTS = (0, 1, 2, 3)


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
                     axle_track_mm, imu=None, accel_dps2=1500.0,
                     drive=DriveMode.DUTY):
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
        if bus is None:
            # Both wheels are ALREADY on native slots — they were
            # constructed on a UART the native driver owns, so there
            # is no MicroPython bus to hand over and no registry
            # entry to find. Take the wiring from the motor itself;
            # attach_uart is idempotent.
            uart_id, tx, rx, baud = left._uart_params
        else:
            params = None
            for key, val in ST3215._buses.items():
                if val is bus:
                    params = key
                    break
            if params is None:
                raise RuntimeError("motor bus not found in the registry")
            uart_id, tx, rx, baud = params
            # Position-mode servos on this UART cannot come across
            # (native slots configure wheel mode) — refuse BEFORE the
            # handover, while the MicroPython bus is still intact.
            # After the deinit there is no clean way back: the C
            # driver keeps the pins until power-cycle.
            stranded = list(getattr(bus, "_servos", ()))
            if stranded:
                raise RuntimeError(
                    "cannot adopt UART%s: position-mode servo id(s) "
                    "%s share it, and they have no native-slot path. "
                    "Put position-mode servos on a second UART, or "
                    "use the Motor class (run/run_angle) for them."
                    % (uart_id,
                       ", ".join(str(s._id) for s in stranded)))
            # Hand the UART over: MicroPython driver out, IDF in.
            bus._uart.deinit()
            del ST3215._buses[params]
        # A wheel constructed on an already-native bus holds a slot
        # of its own; adopt that rather than demanding a fixed index.
        held_l = getattr(left, "_native_slot", None)
        held_r = getattr(right, "_native_slot", None)
        try:
            eng = cls(left_id=left._id, right_id=right._id,
                      wheel_diameter_mm=wheel_diameter_mm,
                      axle_track_mm=axle_track_mm, imu=imu,
                      invert_left=left._invert,
                      invert_right=right._invert,
                      uart_id=uart_id, tx=tx, rx=rx, baud=baud,
                      accel_dps2=accel_dps2,
                      slot_l=held_l, slot_r=held_r, drive=drive)
        except BaseException:
            # Engine construction RAISES by design (dead wheel, slot
            # exhaustion, attach failure) — but the MicroPython bus
            # was already deinited and deregistered above. Without
            # putting it back, a caller that catches the error and
            # tries plain motor commands writes into a dead UART for
            # the rest of the program (self-healing only at the next
            # program boundary). Restore what we took.
            if bus is not None:
                from machine import UART
                ST3215._buses[params] = bus
                bus._uart = UART(uart_id, baudrate=baud, tx=tx, rx=rx,
                                 timeout=50)
            raise
        if held_l is None:
            left._adopt_native(eng._sb, eng._slot_l)
        if held_r is None:
            right._adopt_native(eng._sb, eng._slot_r)
        # Anything else that was sharing that UART comes across too.
        # The IDF driver owns the pins now, so a motor left on the
        # MicroPython bus would be talking into a closed UART — and
        # before 1.57.0, one that re-opened its own would put two
        # drivers on one wire and eat the hard tick's replies.
        if bus is not None:
            left.migrate_bus_to_native(bus, skip=(left, right))
        return eng

    def __init__(self, left_id, right_id, wheel_diameter_mm,
                 axle_track_mm, imu=None,
                 invert_left=False, invert_right=False,
                 uart_id=1, tx=14, rx=41, baud=1_000_000,
                 accel_dps2=1500.0, sb=None,
                 slot_l=None, slot_r=None, drive=DriveMode.DUTY,
                 turn_accel_dps2=1500.0):
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
        self._deadline_budget_ms = self._SETTLE_TIMEOUT_MS
        # Pybricks-parity defaults (1.90.0): straight = 40% of the
        # ST-3032's 888 dps rated speed, turn = 33% (their
        # drivebase_adopt_settings percentages applied to our motor).
        self._straight_speed_dps = 350
        self._turn_rate_dps = 300
        self._accel_dps2 = float(accel_dps2)
        self._turn_accel_dps2 = float(turn_accel_dps2)
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
        if not self._sb.attach_uart(uart_id, baud, tx, rx):
            raise RuntimeError("attach_uart(%d) failed" % uart_id)
        acc = int(accel_dps2 * _STEPS_PER_DEG / 100.0)
        acc = 0 if acc < 0 else (254 if acc > 254 else acc)
        # A wheel may ALREADY hold a slot: on a natively-owned bus a
        # motor claims one the moment it is constructed, and a script
        # is free to build its wheels in any order relative to its
        # task motors. Adopt the slot it has rather than insisting on
        # a fixed index — insisting is what made construction order
        # matter, and ran a 4-motor robot out of slots before the
        # DriveBase was even reached.
        self._slot_l = (slot_l if slot_l is not None
                        else self._claim_slot(left_id, invert_left, acc,
                                              "left"))
        self._slot_r = (slot_r if slot_r is not None
                        else self._claim_slot(right_id, invert_right, acc,
                                              "right"))
        parameters.check(DriveMode, drive, "drive")
        if drive == DriveMode.DUTY:
            # The default since 1.89.0 (dumb-mode directive): the
            # servo runs open-loop and the engine's FF+PI is the
            # speed controller — the whole drive loop is ours. The
            # flip re-runs each slot's config sequence (op_mode=2);
            # _require_live_wheels below waits that out like any
            # other config. drive=DriveMode.WHEEL restores the servo's
            # internal speed loop.
            self._sb.servo_drive_duty(self._slot_l, True)
            self._sb.servo_drive_duty(self._slot_r, True)
        self._sb.db_config(self._slot_l, self._slot_r,
                           float(wheel_diameter_mm), float(axle_track_mm),
                           float(accel_dps2))
        if turn_accel_dps2 != accel_dps2:
            self._sb.db_set_turn_accel(float(turn_accel_dps2))
        # Wiring/ID/power problems are found HERE, at construction,
        # not as mysterious non-motion later: attaching a slot only
        # claims it in C, it never asks the servo whether it exists.
        self._require_live_wheels()

    def _claim_slot(self, servo_id, invert, acc, side):
        """Take the first free slot for a wheel that does not have
        one yet (it was on the MicroPython bus until adoption)."""
        # Already driving this servo? Reuse its slot. One physical
        # servo, one slot — otherwise re-running a script in the same
        # boot claims a second slot for a motor that has one, and a
        # 4-motor robot on 4 slots runs out.
        slot_of = getattr(self._sb, "servo_slot_of", None)
        if slot_of is not None:
            held = slot_of(servo_id)
            if held >= 0:
                return held
        for slot in _ALL_SLOTS:
            if self._sb.servo_attach(slot, servo_id, bool(invert), acc):
                return slot
        raise RuntimeError(
            "no free native slot for the %s wheel (servo id %s): all "
            "%d slots are claimed. More motors than slots needs a "
            "second UART." % (side, servo_id, len(_ALL_SLOTS)))

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
        side = "left" if slot == self._slot_l else "right"
        ids = {self._slot_l: self._left_id,
               self._slot_r: self._right_id}
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
        wstats = getattr(self._sb, "servo_write_stats", None)
        deadline = time.ticks_add(time.ticks_ms(),
                                  self._LIVE_WHEEL_TIMEOUT_MS)
        pending = [self._slot_l, self._slot_r]
        while pending:
            pending = [s for s in pending if stats(s)[0] == 0]
            if not pending:
                return
            if wstats is not None:
                for slot in pending:
                    wfailed, latched = wstats(slot)
                    if latched:
                        # Configuration gave up. Two very different
                        # faults land here, and blaming the wiring
                        # for both sent one bench session pulling
                        # connectors on a healthy harness: a servo
                        # that never ACKED is absent (wiring/id/
                        # power), but a servo whose op_mode read-back
                        # NEVER MATCHED acked every write while still
                        # inside its own power-on init and silently
                        # dropped the EEPROM-backed mode commit
                        # (bench 2026-08-30: cold start + immediate
                        # button press left one wheel in mode 1,
                        # holding zero speed at full torque while
                        # duty commands hammered the register mode 1
                        # ignores — the robot pivoted around it).
                        cstate = getattr(self._sb, "servo_config_state",
                                         None)
                        if cstate is not None:
                            _f, _l, mismatch, got = cstate(slot)
                            if mismatch:
                                raise OSError(
                                    "%s ACKed its configuration but "
                                    "never applied it: op_mode reads "
                                    "%d after every write was "
                                    "acknowledged. The servo was "
                                    "still in its own power-on init "
                                    "— give it a second after power-"
                                    "on before starting the program, "
                                    "then retry."
                                    % (self._wheel_desc(slot), got))
                        raise self._dead_wheel_error(
                            slot,
                            "motor never ACKed its configuration "
                            "(%d writes unacknowledged)" % wfailed)
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                slot = pending[0]
                # Distinguish "asked and got silence" from "never
                # asked" — a wedged pump is a firmware fault, not a
                # wiring one, and saying "check your wiring" sends
                # the user hunting the wrong thing (1.57.2).
                if stats(slot)[1] == 0:
                    raise OSError(
                        "%s: the bus pump never polled it — 0 reads "
                        "ATTEMPTED, not 0 answered. Not a wiring "
                        "fault: the hard tick is not running, or the "
                        "bus is stuck mid-transaction. Power-cycle "
                        "the hub; if it persists, report it."
                        % self._wheel_desc(slot))
                raise self._dead_wheel_error(
                    slot, "motor is not answering on the bus")
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
        slot = self._slot_l if bits & 0x01 else self._slot_r
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

    def reset(self):
        """Re-zero the heading frame — yaw integrator, engine
        reference, and held target together in one locked C section
        (Pybricks ``DriveBase.reset()``). This is the sanctioned way
        to declare "current pose is heading zero" mid-mission;
        ``imu.reset_heading()`` refuses while the gyro steers a
        drive base precisely because it can't do this atomically.
        Raises ``RuntimeError`` while a move is active — stop first.
        """
        self._sb.db_reset()
        if self._use_gyro and not self._hard_gyro:
            # Soft-pump IMUs: restart the continuous frame at zero.
            self._gyro_cont = 0.0
            self._gyro_prev = self._imu.heading()

    # -- moves -----------------------------------------------------------

    def set_accel(self, accel_dps2):
        self._accel_dps2 = float(accel_dps2)
        self._sb.db_set_accel(float(accel_dps2))

    def set_turn_accel(self, accel_dps2):
        self._turn_accel_dps2 = float(accel_dps2)
        self._sb.db_set_turn_accel(float(accel_dps2))

    def arm_straight(self, distance_mm, carry=False):
        estop.check()
        mm_s = self._straight_speed_dps * self._wheel_circumference / 360.0
        self._sb.db_straight(float(distance_mm), float(mm_s),
                             1 if carry else 0)
        accel_mm = self._accel_dps2 * self._wheel_circumference / 360.0
        self._arm_deadline(self._profile_ms(distance_mm, mm_s, accel_mm))

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
        accel_body = (self._turn_accel_dps2 * self._wheel_circumference
                      / (math.pi * self._axle_track))
        self._arm_deadline(self._profile_ms(angle_deg, body_dps, accel_body))

    def turn(self, angle_deg):
        self.arm_turn(angle_deg)
        self._wait()

    def arm_curve(self, radius_mm, angle_deg, carry=False):
        """Pybricks ``curve()``: arc of ``|radius_mm|`` changing
        heading by ``angle_deg`` (CW-positive). Centre speed is the
        straight_speed setting scaled by |R|/(|R| + track/2) so the
        OUTER wheel never exceeds it; radius 0 degrades to a turn in
        place at the rim speed."""
        estop.check()
        mm_s = self._straight_speed_dps * self._wheel_circumference / 360.0
        outer_mm_s = mm_s
        r = abs(float(radius_mm))
        if r > 0:
            mm_s = mm_s * r / (r + self._axle_track / 2.0)
        self._sb.db_curve(float(radius_mm), float(angle_deg), float(mm_s),
                          1 if carry else 0)
        # Estimate on the OUTER wheel: it travels the longest arc at
        # (up to) the straight cruise speed.
        outer_arc_mm = (abs(float(angle_deg)) * math.pi / 180.0
                        * (r + self._axle_track / 2.0))
        accel_mm = self._accel_dps2 * self._wheel_circumference / 360.0
        self._arm_deadline(
            self._profile_ms(outer_arc_mm, outer_mm_s, accel_mm))

    def curve(self, radius_mm, angle_deg):
        self.arm_curve(radius_mm, angle_deg)
        self._wait()

    def move_wheels(self, left_wheel_speed, right_wheel_speed):
        """Independent per-wheel speeds (wheel-deg/s), both staged in
        one C critical section — the drivebase-owned equivalent of a
        SyncServoGroup over the two wheels (which adoption makes
        unreachable: the motors' MicroPython UART is gone). Targets
        ramp at the configured straight acceleration (proportional
        slew: the larger delta runs at full accel, both wheels arrive
        together), then the engine yields with the registers holding
        the final speeds."""
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

    # Stop.COAST/BRAKE/HOLD carry the native stop codes 0/1/2 as
    # their .value (the Pybricks numbering).

    def stop(self, then=None):
        """Stop the drivebase and yield the wheels.

        Without ``then`` the engine only yields (abort paths — the
        caller dispatches the wheels' end-state itself). With
        ``then`` the complete stop is staged atomically in C, so
        both wheels reach the end-state at the same bus-packet
        boundary: coast is one sync-torque write covering both
        servos, brake one sync-speed write, and hold captures both
        wheel poses at the same instant — instead of one motor at a
        time, a bus transaction apart.

        ``brake``/``hold`` decelerate as a coupled-controller stop
        trajectory, so the heading loop — the IMU, in gyro mode —
        stays closed through the ramp. A soft-pumped IMU gets one
        pump first: the ramp anchors to the heading of NOW, not of
        the last ``done()`` poll (a line-follow between the two has
        rotated the chassis without a single pump)."""
        if then is None:
            self._sb.db_stop()
            return
        parameters.check(Stop, then, "then",
                         allowed=(Stop.COAST, Stop.BRAKE, Stop.HOLD))
        if then != Stop.COAST and self._use_gyro:
            self._gyro_pump()
        ok = self._sb.db_stop(then.value)
        if then == Stop.HOLD and not ok:
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

    # Settle budget = commanded-profile estimate x margin + this
    # floor. The floor alone covered every move until someone drove
    # a 10 m straight: 65 s of healthy driving, killed at 8 s with
    # "wheel stalled" (bench 2026-08-10). The margin absorbs load,
    # settle wiggle and gyro-correction detours; the floor keeps
    # short moves on the old contract.
    _SETTLE_TIMEOUT_MS = 8000
    _SETTLE_MARGIN = 1.5

    @staticmethod
    def _profile_ms(travel, cruise, accel):
        """Upper-bound duration of a trapezoid/triangle profile, in
        ms. ``travel``/``cruise``/``accel`` in any one consistent
        unit family; non-positive inputs estimate 0 (the floor still
        applies)."""
        travel = abs(float(travel))
        cruise = abs(float(cruise))
        if travel <= 0.0 or cruise <= 0.0 or accel <= 0.0:
            return 0
        ramp_s = cruise / accel
        if cruise * ramp_s >= travel:       # never reaches cruise
            secs = 2.0 * math.sqrt(travel / accel)
        else:
            secs = travel / cruise + ramp_s
        return int(secs * 1000.0)

    def _arm_deadline(self, est_ms=0):
        self._deadline_budget_ms = (self._SETTLE_TIMEOUT_MS
                                    + int(est_ms * self._SETTLE_MARGIN))
        self._deadline = time.ticks_add(time.ticks_ms(),
                                        self._deadline_budget_ms)

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
               % self._deadline_budget_ms)
        stats = getattr(self._sb, "servo_stats", None)
        if stats is None:
            return msg
        return msg + (" [left%s; right%s]"
                      % (self._wheel_evidence(self._slot_l),
                         self._wheel_evidence(self._slot_r)))

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
