# SPDX-License-Identifier: MIT
"""
Two-wheel differential drivebase.

Thin Python wrapper over ``_openbricks_native.DriveBase`` — the C
implementation at ``native/user_c_modules/openbricks/drivebase.c`` that
runs 2-DOF coupled control at 1 kHz. Both motors are driven by a
single forward-progress trajectory and a heading-hold trajectory; a
heading-error feedback term keeps them in sync even when one wheel has
more friction than the other.

Public API matches the M1 Python version so existing code and tests
don't need to change:

    db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
    db.settings(straight_speed=200, turn_rate=180)   # deg/s at wheels
    db.straight(500)     # mm, blocking
    db.turn(90)          # deg body heading, blocking
    db.drive(100, 0)     # non-blocking kinematic mapping

Serial-bus motors (ST-3215 / ST-3032) are adopted transparently onto
the hard-tick engine (firmware) or the emulated bus (sim) — same class,
same code, one controller. There is no Python control loop: motor
pairs with neither a native servo nor a serial-bus adoption path get
open-loop ``drive()``/``stop()`` only, and ``straight()``/``turn()``
raise.

Open-loop ``drive()`` bypasses the coupled controller; it just maps
(speed_mm_s, turn_rate_dps) → (left_dps, right_dps) and hands them to
each servo's ``run_speed``. Useful for interactive control where
profile-based moves would feel sluggish.
"""

import math
import time

from openbricks import estop
from openbricks._native import DriveBase as _NativeDriveBase
from openbricks import parameters
from openbricks.parameters import Stop, DriveMode


class DriveBase:
    """A two-wheel differential drive robot: two motors, one chassis.

    Pybricks-compatible surface: ``straight(distance_mm)``,
    ``turn(angle_deg)``, ``drive(speed_mm_s, turn_rate_dps)``,
    ``stop(then=...)``, ``settings(...)``, ``use_gyro(True)`` and
    non-blocking moves via ``wait=False`` + ``done()``. Positive
    ``turn`` is right/clockwise viewed from above.

    Give it any two closed-loop motors and it picks the right
    controller automatically:

    * **Encoder servos** (``JGB37Motor``, ``MG370Motor``) — the native
      C 2-DOF coupled controller at 1 kHz.
    * **Serial-bus servos** (``ST3032Motor``, ``ST3215Motor``) — the
      motors are *adopted* onto the hard-tick native bus engine
      (~220 Hz odometry per wheel, immune to Python stalls). Their
      wheel-mode motor API keeps working after adoption.
    * **Open-loop motors** (``L298NMotor``) — kinematic ``drive()`` /
      ``stop()`` only; moves by distance need feedback and raise.

    Example::

        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics import DriveBase

        left  = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
        right = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41,
                            invert=True)
        db = DriveBase(left, right, wheel_diameter_mm=88,
                       axle_track_mm=138)
        db.straight(300)     # forward 300 mm
        db.turn(90)          # turn right 90 degrees

    Accurate ``wheel_diameter_mm`` / ``axle_track_mm`` values matter
    more than any controller gain — see :doc:`/measuring` for how to
    calibrate both in two short test drives.
    """

    def __init__(self, left, right, wheel_diameter_mm, axle_track_mm,
                 imu=None, drive=DriveMode.DUTY):
        """
        Args:
            left, right: Motor instances. The wrapper reaches through to
                ``.servo`` (for JGB37Motor) when constructing the native
                drivebase, since the C layer operates on the servo struct
                directly. Motors without a native servo (e.g. plain
                ``L298NMotor`` with no encoder) get open-loop
                ``drive()`` only.
            drive: serial-bus wheels only — ``DriveMode.DUTY`` (default) runs
                the servos open-loop with the engine's own FF+PI speed
                controller closing the loop over raw duty ("dumb
                mode": the entire drive loop is openbricks code);
                ``DriveMode.WHEEL`` uses the servo's internal speed loop
                instead. One caveat in duty mode: at move end,
                ``then=Stop.BRAKE``/``Stop.HOLD`` behave like coast at
                the wheel level (open loop has no hold torque) —
                while a move is ACTIVE the controller corrects as
                usual. Ignored on encoder/DC motor pairs.
            wheel_diameter_mm: wheel diameter in millimeters.
            axle_track_mm: distance between the two wheel contact points.
            imu: optional ``IMU``-conformant object (any driver with a
                ``.heading()`` method returning body heading in degrees —
                the bundled ``BNO055`` qualifies). When provided, call
                ``drivebase.use_gyro(True)`` to have the heading loop
                read from the IMU instead of computing from the encoder
                differential. Slip-immune. Works on both closed-loop
                paths: the native controller reads it on the 1 kHz
                tick, and the serial-bus engine pumps it into the
                hard-tick heading hold once per ``done()`` poll.
        """
        self._left = left
        self._right = right
        self._wheel_circumference = math.pi * wheel_diameter_mm
        self._axle_track = axle_track_mm
        self._imu = imu
        self._gyro_enabled = False

        # Serial-bus motors: adopt them onto the hard-tick engine
        # transparently (1.45.0 — ONE drivebase class, user decision).
        # On firmware that's the real st_bus + UART handover; in the
        # sim it's the emulated bus over MuJoCo wheels. Same user code
        # everywhere. Raises if the runtime has no bus — there is no
        # Python fallback loop.
        parameters.check(DriveMode, drive, "drive")
        self._serial_engine = self._try_adopt_serial(left, right, imu,
                                                     drive)

        # The native drivebase is only usable if both motors are
        # closed-loop servos. Motor pairs with neither engine get
        # open-loop ``drive()`` only.
        left_servo  = getattr(left,  "_servo", None)
        right_servo = getattr(right, "_servo", None)
        if left_servo is not None and right_servo is not None:
            self._native = _NativeDriveBase(
                left=left_servo,
                right=right_servo,
                wheel_diameter_mm=wheel_diameter_mm,
                axle_track_mm=axle_track_mm,
                imu=imu,
            )
        else:
            self._native = None

        # Default cruise parameters (wheel-degrees per second),
        # Pybricks-parity since 1.90.0: 40% / 33% of the ST-3032's
        # 888 dps rated speed. Tweak via ``settings()``.
        self._straight_speed_dps = 350
        self._turn_rate_dps      = 300

        # State for in-flight ``straight(wait=False)`` / ``turn(wait=False)``
        # moves. ``None`` means nothing pending; ``done()`` returns
        # True. ``stop()`` clears this. See ``done`` for the layout.
        self._pending = None

    def _try_adopt_serial(self, left, right, imu, drive=DriveMode.DUTY):
        # Polymorphic: each serial-motor family implements its own
        # adoption (firmware ST3215Motor -> real st_bus + UART
        # handover; the sim's shim motors -> the emulated bus over
        # MuJoCo wheels). Motors without the hook (encoder/open-loop
        # families) simply don't adopt.
        if not (hasattr(left, "_adopt_into_drivebase")
                and hasattr(right, "_adopt_into_drivebase")):
            return None
        engine = left._adopt_into_drivebase(
            right,
            wheel_diameter_mm=self._wheel_circumference / math.pi,
            axle_track_mm=self._axle_track, imu=imu, drive=drive,
            accel_dps2=1500.0)  # Pybricks-parity default (their
        # hardcoded 2000 dps^2 motor accel x 3/4 drivebase factor);
        # settings(acceleration=...) retunes it via db_set_accel.
        if engine is None:
            # Serial-bus motors with no bus behind them: this runtime
            # can't drive them closed-loop, and the Python fallback
            # loop was removed in 1.45.0. No silent degradation.
            raise RuntimeError(
                "serial-bus drivebase requires the native st_bus "
                "(firmware >= 1.45.0) or the sim's emulated bus; "
                "this runtime has neither")
        return engine

    def settings(self, straight_speed=None, turn_rate=None,
                 acceleration=None, turn_acceleration=None):
        """Tune cruise + ramp parameters for subsequent moves.

        Args:
            straight_speed: cruise speed for ``straight()``, wheel-deg/s.
            turn_rate: cruise rate for ``turn()``, wheel-deg/s.
            acceleration: STRAIGHT (and curve) trajectory
                acceleration, wheel-deg/s² — Pybricks'
                ``straight_acceleration``. Serial-engine default 1500
                (Pybricks parity: their 2000 dps² motor accel × 3/4)
                — lower it if the robot pitches or lifts its rear on
                launch. In mm/s² that's
                ``acceleration * wheel_circumference / 360``. Applies
                on both paths: the native (encoder-servo) controller
                arms its C trajectory with it, and the serial-bus
                engine forwards it to the hard-tick controller.
            turn_acceleration: ``turn()`` ramps' own acceleration,
                wheel-deg/s², independent of ``acceleration`` —
                Pybricks parity (serial default 1500). Serial-bus
                drivebase only; passing it on an encoder/DC pair
                raises.
        """
        if straight_speed is not None:
            self._straight_speed_dps = straight_speed
        if turn_rate is not None:
            self._turn_rate_dps = turn_rate
        if self._serial_engine is not None:
            self._serial_engine.settings(straight_speed=straight_speed,
                                         turn_rate=turn_rate)
        if acceleration is not None:
            if not acceleration > 0:
                raise ValueError(
                    "acceleration must be > 0 deg/s^2 (got %r)"
                    % (acceleration,))
            if self._native is not None:
                self._native.set_accel(float(acceleration))
            if self._serial_engine is not None:
                self._serial_engine.set_accel(float(acceleration))
        if turn_acceleration is not None:
            if not turn_acceleration > 0:
                raise ValueError(
                    "turn_acceleration must be > 0 deg/s^2 (got %r)"
                    % (turn_acceleration,))
            if self._serial_engine is None:
                raise ValueError(
                    "turn_acceleration is supported on the serial-bus "
                    "drivebase only (encoder/DC pairs share one "
                    "acceleration)")
            self._serial_engine.set_turn_accel(float(turn_acceleration))

    def use_gyro(self, enable):
        """Switch the heading feedback source between encoder-diff (default)
        and the attached IMU (when True). Pybricks-style.

        Requires an ``imu=`` argument to the constructor. With the gyro,
        heading is slip-immune — wheel slip or wildly asymmetric friction
        won't throw the robot off course, because the IMU sees actual body
        rotation regardless of what the wheels did. Works on both the
        native (encoder-servo) path and the serial-bus engine.
        """
        enable = bool(enable)
        if enable and self._imu is None:
            raise ValueError(
                "no imu attached; construct DriveBase(imu=...) first")
        if self._serial_engine is not None:
            self._serial_engine.use_gyro(enable)
        elif self._native is not None:
            self._native.use_gyro(enable)
        else:
            raise RuntimeError(
                "use_gyro needs a closed-loop drivebase (encoder "
                "servos or serial-bus motors); open-loop pairs have "
                "no heading-hold loop")
        self._gyro_enabled = enable

    def reset(self):
        """Re-zero the heading frame: after ``reset()``, the robot's
        CURRENT pose is heading zero — for the drive base's
        controller and ``imu.heading()`` together (Pybricks
        ``DriveBase.reset()``). Call it between moves; it raises
        while a move is active.

        This is the supported way to re-zero mid-mission.
        ``imu.reset_heading()`` refuses while a drive base steers by
        the gyro, because zeroing the integrator under an armed
        controller shifts the measurement out from under the held
        target — the next ``straight()`` then veers chasing the old
        frame.
        """
        if self._serial_engine is not None:
            self._serial_engine.reset()
        elif self._native is not None:
            if self._gyro_enabled:
                # Fresh frame via the enable transition — the same
                # "here, now is zero" the first enable performs.
                self._native.use_gyro(False)
                self._native.use_gyro(True)
        # Open-loop / encoder mode: the frame is re-derived at every
        # arm; nothing to re-base.

    # ---- non-blocking open-loop ----
    def drive(self, speed_mm_s, turn_rate_dps):
        """Start driving at a given forward speed + body turn rate.

        Kinematic one-shot — no coupled feedback. Call again (or
        ``stop()``) to change. Positive turn rate = right turn
        (clockwise viewed from above), Pybricks convention.

        Speed changes ramp at ``settings(acceleration=...)`` (the
        uniform-accel rule, 1.94.0) — proportionally across the two
        wheels, so an arc keeps its radius through the ramp.
        """
        # New command wins — same rule as move_wheels/stop. drive()
        # was the ONE motion verb that skipped it: a still-running
        # straight() overwrote its speeds every tick (~1 kHz) and a
        # later done() poll dispatched the stale move's then= on top.
        self._pending = None
        if self._native is not None:
            # Clear any in-flight straight/turn trajectory first.
            self._native.stop()

        fwd_wheel_dps  = speed_mm_s / self._wheel_circumference * 360
        turn_rad_s     = math.radians(turn_rate_dps)
        diff_mm_s      = turn_rad_s * (self._axle_track / 2)
        diff_wheel_dps = diff_mm_s / self._wheel_circumference * 360

        if self._serial_engine is not None:
            # Route through move_wheels: it aborts the engine's
            # in-flight move and ships both setpoints in one
            # sync-write. Sending per-motor run_speed here left the
            # drivebase writing its own targets over them.
            self._serial_engine.move_wheels(
                fwd_wheel_dps + diff_wheel_dps,
                fwd_wheel_dps - diff_wheel_dps)
            self._left._native_pending = None
            self._right._native_pending = None
            return
        self._run_at_dps(self._left,  fwd_wheel_dps + diff_wheel_dps)
        self._run_at_dps(self._right, fwd_wheel_dps - diff_wheel_dps)

    def check_motors(self):
        """Raise if a wheel has stopped answering the bus.

        Serial-bus wheels are checked at construction and on every
        move, so you rarely need this directly — reach for it in a
        long-running open-loop control loop (``move_wheels`` in a
        ``while`` loop already calls it for you), or to verify the
        chassis before a run. The error names the motor: side, bus
        id, slot, UART and pins.
        """
        if self._serial_engine is not None:
            self._serial_engine.check_motors()

    def move_wheels(self, left_wheel_speed, right_wheel_speed):
        """Drive the two wheels at independent speeds, in wheel-deg/s.

        Positive is forward on both sides (each motor's ``invert``
        is already applied), so ``move_wheels(200, 200)`` drives
        straight and ``move_wheels(200, -200)`` spins in place.

        Non-blocking and continuous, like ``drive()``: the wheels
        hold these speeds until you call it again, issue another
        move, or ``stop()``. It supersedes any move in flight.

        Use this instead of building a ``SyncServoGroup`` over the
        wheels. On serial-bus motors both setpoints leave in a
        single sync-write packet, so the wheels change speed at the
        same packet boundary — and a ``SyncServoGroup`` could not
        drive them anyway, because adopting them into a DriveBase
        hands their UART to the native driver. On encoder servos
        both targets are set and both servos subscribed inside one
        native call.

        Where ``drive(speed_mm_s, turn_rate_dps)`` speaks chassis
        kinematics, this speaks wheels directly — the right tool for
        line-following, tank-style teleop, or any controller that
        computes per-wheel outputs itself.

        Example::

            db.move_wheels(200, 120)     # gentle right-hand arc
            time.sleep_ms(500)
            db.stop()

        Open-loop motor pairs (no encoder, no serial bus) are
        supported but cannot batch: the two speeds are written one
        after the other.
        """
        estop.check()
        left  = float(left_wheel_speed)
        right = float(right_wheel_speed)
        # A direct wheel command supersedes a pending wait=False move
        # (pybricks "new command wins").
        self._pending = None
        if self._serial_engine is not None:
            self._serial_engine.move_wheels(left, right)
            self._left._native_pending = None
            self._right._native_pending = None
            return
        if self._native is not None:
            self._native.move_wheels(left, right)
            return
        self._run_at_dps(self._left, left)
        self._run_at_dps(self._right, right)

    def stop(self, then=Stop.COAST, wait=False):
        """Halt both wheels. Also clears any pending ``wait=False``
        move (new command supersedes, pybricks-style). ``then``
        selects the end-state:

        * ``Stop.COAST`` (default) — both motors free-wheel.
        * ``Stop.BRAKE`` — both motors actively resist motion at zero
          velocity.
        * ``Stop.HOLD`` — both motors actively hold their current angle.
          Requires motors that implement ``hold()`` (e.g. ``ST3215Motor``);
          open-loop drivers raise ``NotImplementedError``.

        Both wheels are always commanded together, never one motor at
        a time. On serial-bus (adopted) motors the whole stop is
        staged atomically in the C engine and reaches the wheels at
        the same bus-packet boundary. ``brake`` and ``hold``
        DECELERATE at ``settings(acceleration=...)`` first (the
        uniform-accel rule) as a move of the coupled controller, so
        the heading loop stays closed all the way down: with
        ``use_gyro(True)`` the IMU corrects any yaw the brake induces
        (one wheel gripping harder than the other) exactly as it does
        mid-straight, and the robot stops on the heading it had.
        Hold anchors where the robot actually stops. ``coast``
        releases torque immediately — a freewheel has no controlled
        deceleration, and nothing can steer wheels that carry no
        torque. After ``drive()`` / ``move_wheels`` (a line-follow),
        a brake/hold — like the next move — takes the heading the
        follow REACHED as its target rather than steering back to
        the pre-follow one. On encoder servos ``coast`` / ``brake``
        likewise apply to both bridges inside one native call, so
        the second wheel's 1 kHz control tick can't keep driving
        while the first is already released.

        Pybricks parity by default: the call returns immediately
        (their stop/brake do too — verified from their source).
        Since 2.4.0 short moves armed at speed raise their own
        deceleration to land at rest on target, so waiting is rarely
        needed; pass ``wait=True`` to BLOCK until both wheels'
        MEASURED speeds read ~0 — for ``brake``/``hold`` the decel
        ramp finishing plus settle, for ``coast`` the physical
        freewheel decay. ``wait=True`` raises ``ValueError`` on
        open-loop pairs (no measured speed to wait on) and
        ``RuntimeError`` if the wheels never settle within the
        timeout.
        """
        parameters.check(Stop, then, "then",
                         allowed=(Stop.COAST, Stop.BRAKE, Stop.HOLD))
        self._pending = None
        self._dispatch_stop(then)
        if wait:
            self._wait_until_stopped()

    # stop(wait=True): "almost zero" and how long we insist on it.
    _STOP_WAIT_TOL_DPS = 10.0
    _STOP_WAIT_POLL_MS = 10
    _STOP_WAIT_POLLS = 500          # 5 s budget
    _STOP_WAIT_QUIET = 3            # consecutive quiet reads required

    def _wait_until_stopped(self):
        readers = []
        for m in (self._left, self._right):
            fn = getattr(m, "speed", None)
            if fn is not None:
                try:
                    fn()
                except NotImplementedError:
                    fn = None   # interface stub: no real measurement
            if fn is None:
                raise ValueError(
                    "stop(wait=True) needs measured wheel speeds - "
                    "%s has no speed()" % type(m).__name__)
            readers.append(fn)
        quiet = 0
        speeds = []
        for _ in range(self._STOP_WAIT_POLLS):
            speeds = [r() for r in readers]
            if all(v is not None and abs(v) < self._STOP_WAIT_TOL_DPS
                   for v in speeds):
                quiet += 1
                if quiet >= self._STOP_WAIT_QUIET:
                    return
            else:
                quiet = 0
            time.sleep_ms(self._STOP_WAIT_POLL_MS)
        raise RuntimeError(
            "stop(wait=True): wheels still moving after %d ms - "
            "measured speeds %r dps (None = bus silent)"
            % (self._STOP_WAIT_POLLS * self._STOP_WAIT_POLL_MS, speeds))

    def _dispatch_stop(self, then):
        if self._serial_engine is not None:
            self._serial_engine.stop(then)
            # New command wins: the atomic stop supersedes any
            # motor-level wait=False move (the per-motor dispatch
            # used to clear these as a side effect).
            self._left._native_pending = None
            self._right._native_pending = None
            return
        if self._native is not None and then in (Stop.COAST, Stop.BRAKE):
            # Both bridges written inside the one native call.
            self._native.stop(then.value)
            return
        if self._native is not None:
            # then=Stop.HOLD: no native position hold on encoder servos,
            # so this falls through to the per-motor dispatch below
            # (which raises for motors without hold(), as documented).
            self._native.stop()
        if then == Stop.COAST:
            self._left.coast()
            self._right.coast()
        elif then == Stop.BRAKE:
            self._left.brake()
            self._right.brake()
        else:   # Stop.HOLD
            self._left.hold()
            self._right.hold()

    def done(self):
        """Pybricks-style status check for in-flight
        ``straight(wait=False)`` / ``turn(wait=False)``. Returns
        ``True`` if no move is pending or the active move has
        reached its target (and ``stop(then=…)`` has run). Returns
        ``False`` while the move is still progressing.

        The controller runs the trajectory independently on the hard
        tick (native path: 1 kHz C scheduler; serial path: the
        st_bus pump); ``done()`` checks a flag — plus, on the serial
        path with the gyro enabled, feeds the IMU heading into the
        hard-tick heading hold. The natural polling cadence is
        ``time.sleep_ms(10)``.
        """
        if self._pending is None:
            return True
        mode = self._pending["mode"]
        if mode in ("straight_native", "turn_native", "curve_native"):
            if self._native.is_done():
                self._finish_move()
                return True
            return False
        if mode in ("straight_serial", "turn_serial", "curve_serial"):
            if self._serial_engine.tick_done():
                self._finish_move()
                return True
            return False
        # Unknown mode — treat as done to avoid wedging the caller.
        self._pending = None
        return True

    def _finish_move(self):
        """Apply a completed move's end state. ``then=Stop.NONE``
        dispatches NOTHING: the engine keeps the wheels at the move's
        end speed until the next command supersedes it (Pybricks
        Stop.NONE)."""
        then = self._pending["then"]
        if then == Stop.NONE:
            self._pending = None
            return
        self.stop(then=then, wait=False)

    # ---- blocking moves via the C coupled controller ----
    def straight(self, distance_mm, then=Stop.COAST, wait=True):
        """Drive forward by ``distance_mm``. 2-DOF coupled.

        ``then`` is forwarded to ``stop()`` — see its docstring for
        coast/brake/hold semantics.

        ``wait=True`` (default) blocks until the move completes.
        ``wait=False`` returns immediately after arming the move;
        the caller polls ``done()`` to check completion, and the
        ``then=`` dispatch is deferred until ``done()`` reports
        the target was reached. Concurrent use with another
        wait=False move on a separate ``DriveBase`` (or with motor
        ``run_angle(wait=False)`` calls) is the intended pattern.

        Any subsequent move command supersedes the previous pending
        wait=False move (pybricks "new command wins").

        ``then=Stop.NONE`` (Pybricks ``Stop.NONE``) does not
        decelerate at the end: the move finishes AT cruise speed and
        the wheels keep it until the next command — chain
        ``straight``/``curve`` segments without stopping between
        them. ``"stop"`` is accepted as an alias of the default
        coast end state.

        Raises ``RuntimeError`` for open-loop motor pairs — moves by
        distance need feedback; use ``drive()``/``stop()``."""
        then = self._check_then(then, allow_continue=True)
        self._arm_straight(distance_mm, then)
        if wait:
            while not self.done():
                time.sleep_ms(10)

    def turn(self, angle_deg, then=Stop.COAST, wait=True):
        """Turn in place by ``angle_deg`` body heading (positive =
        right/clockwise viewed from above, Pybricks convention).

        Same ``then`` / ``wait`` semantics as ``straight()`` — see
        its docstring — except ``then=Stop.NONE``: a turn in place
        ends facing its target heading, so there is no speed worth
        carrying."""
        then = self._check_then(then, allow_continue=False)
        self._arm_turn(angle_deg, then)
        if wait:
            while not self.done():
                time.sleep_ms(10)

    def curve(self, radius, angle, then=Stop.COAST, wait=True):
        """Drive an arc along a circle of ``|radius|`` mm, changing
        heading by ``angle`` degrees — Pybricks ``DriveBase.curve()``,
        including the parameter names, so Pybricks-style keyword
        calls (``curve(radius=150, angle=90)``) work verbatim. The
        one deviation: our ``then`` defaults to ``Stop.COAST`` like
        every openbricks move (Pybricks defaults to hold) — pass
        ``then=Stop.HOLD`` for the Pybricks end state.

        Positive ``angle`` turns right (clockwise from above,
        the system-wide sign convention, same as ``turn()``); the
        SIGN of ``radius`` picks the travel direction along the
        arc (positive = forward, negative = backward).
        ``curve(150, 90)`` sweeps a forward quarter-circle to the
        right around a centre 150 mm to the robot's right;
        ``curve(150, -90)`` the mirror to the left.

        The forward and turn profiles run simultaneously with
        proportional speed AND acceleration, so heading stays
        proportional to distance at every instant — the path is a
        true circle through the accel/decel ramps, not just at the
        endpoints. The centre speed is the ``straight_speed``
        setting scaled by ``|R| / (|R| + track/2)`` so the OUTER
        wheel never exceeds ``straight_speed``. ``curve(0, angle)``
        degrades to a turn in place.

        Same ``then`` / ``wait`` semantics as ``straight()``,
        including ``then=Stop.NONE`` — the arc hands its full speed
        to the next command."""
        then = self._check_then(then, allow_continue=True)
        self._arm_curve(radius, angle, then)
        if wait:
            while not self.done():
                time.sleep_ms(10)

    @staticmethod
    def _check_then(then, allow_continue):
        """Validate a move's ``then=`` — a :class:`Stop` member;
        ``Stop.NONE`` only where the move can hand its speed on."""
        allowed = (Stop.COAST, Stop.BRAKE, Stop.HOLD, Stop.NONE) \
            if allow_continue else (Stop.COAST, Stop.BRAKE, Stop.HOLD)
        return parameters.check(Stop, then, "then", allowed=allowed)

    # ---- arm: stash pending state, kick off motion ----
    def _arm_straight(self, distance_mm, then):
        carry = then == Stop.NONE
        if self._serial_engine is not None:
            self._serial_engine.arm_straight(float(distance_mm), carry)
            self._pending = {"mode": "straight_serial", "then": then}
            return
        if self._native is not None:
            # The native drivebase subscribes BOTH servos itself, in
            # one C call (1.53.0). It used to be two Python
            # ``run_speed(0)`` calls here — non-atomic, and the
            # e-stop gate rode on them, hence the explicit check.
            estop.check()
            speed_mm_s = self._straight_speed_dps * self._wheel_circumference / 360
            self._native.straight(float(distance_mm), float(speed_mm_s),
                                  carry)
            self._pending = {"mode": "straight_native", "then": then}
            return
        raise RuntimeError(
            "straight() needs closed-loop motors (encoder servos or "
            "serial-bus motors); open-loop pairs use drive()/stop()")

    def _arm_turn(self, angle_deg, then):
        if self._serial_engine is not None:
            self._serial_engine.arm_turn(float(angle_deg))
            self._pending = {"mode": "turn_serial", "then": then}
            return
        if self._native is not None:
            estop.check()       # see _arm_straight
            self._native.turn(float(angle_deg), float(self._turn_rate_dps))
            self._pending = {"mode": "turn_native", "then": then}
            return
        raise RuntimeError(
            "turn() needs closed-loop motors (encoder servos or "
            "serial-bus motors); open-loop pairs use drive()/stop()")

    def _curve_speed_mm_s(self, radius_mm):
        """Centre speed for an arc: straight_speed scaled so the
        OUTER wheel (radius |R| + track/2) never exceeds it."""
        mm_s = self._straight_speed_dps * self._wheel_circumference / 360
        r = abs(float(radius_mm))
        if r > 0:
            mm_s = mm_s * r / (r + self._axle_track / 2.0)
        return mm_s

    def _arm_curve(self, radius_mm, angle_deg, then):
        carry = then == Stop.NONE
        if self._serial_engine is not None:
            self._serial_engine.arm_curve(float(radius_mm),
                                          float(angle_deg), carry)
            self._pending = {"mode": "curve_serial", "then": then}
            return
        if self._native is not None:
            estop.check()       # see _arm_straight
            self._native.curve(float(radius_mm), float(angle_deg),
                               float(self._curve_speed_mm_s(radius_mm)),
                               carry)
            self._pending = {"mode": "curve_native", "then": then}
            return
        raise RuntimeError(
            "curve() needs closed-loop motors (encoder servos or "
            "serial-bus motors); open-loop pairs use drive()/stop()")

    # ---- helpers ----
    @staticmethod
    def _run_at_dps(motor, dps):
        run_speed = getattr(motor, "run_speed", None)
        if callable(run_speed):
            try:
                run_speed(dps)
                return
            except NotImplementedError:
                pass
        # Open-loop mapping: assume ~300 dps rated.
        power = max(-100, min(100, dps / 300 * 100))
        motor.dc(power)
