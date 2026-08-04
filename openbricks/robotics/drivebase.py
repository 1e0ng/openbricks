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

from openbricks._native import DriveBase as _NativeDriveBase


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

        left  = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        right = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
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
                 imu=None):
        """
        Args:
            left, right: Motor instances. The wrapper reaches through to
                ``.servo`` (for JGB37Motor) when constructing the native
                drivebase, since the C layer operates on the servo struct
                directly. Motors without a native servo (e.g. plain
                ``L298NMotor`` with no encoder) get open-loop
                ``drive()`` only.
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

        # Serial-bus motors: adopt them onto the hard-tick engine
        # transparently (1.45.0 — ONE drivebase class, user decision).
        # On firmware that's the real st_bus + UART handover; in the
        # sim it's the emulated bus over MuJoCo wheels. Same user code
        # everywhere. Raises if the runtime has no bus — there is no
        # Python fallback loop.
        self._serial_engine = self._try_adopt_serial(left, right, imu)

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

        # Default cruise parameters (wheel-degrees per second). Tweak via
        # ``settings()``.
        self._straight_speed_dps = 200
        self._turn_rate_dps      = 180

        # State for in-flight ``straight(wait=False)`` / ``turn(wait=False)``
        # moves. ``None`` means nothing pending; ``done()`` returns
        # True. ``stop()`` clears this. See ``done`` for the layout.
        self._pending = None

    def _try_adopt_serial(self, left, right, imu):
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
            axle_track_mm=self._axle_track, imu=imu,
            accel_dps2=400.0)   # serial-tuned default (the bench
        # value every native square shipped with); settings(
        # acceleration=...) retunes it afterwards via db_set_accel.
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
                 acceleration=None):
        """Tune cruise + ramp parameters for subsequent moves.

        Args:
            straight_speed: cruise speed for ``straight()``, wheel-deg/s.
            turn_rate: cruise rate for ``turn()``, wheel-deg/s.
            acceleration: trajectory acceleration, wheel-deg/s², shared
                by ``straight()`` and ``turn()`` ramps. Default 1000
                (2 wheel-rev/s²) — lower it if the robot pitches or
                lifts its rear on launch. In mm/s² that's
                ``acceleration * wheel_circumference / 360``. Applies
                on both paths: the native (encoder-servo) controller
                arms its C trajectory with it, and the serial-bus
                engine forwards it to the hard-tick controller.
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

    # ---- non-blocking open-loop ----
    def drive(self, speed_mm_s, turn_rate_dps):
        """Start driving at a given forward speed + body turn rate.

        Kinematic one-shot — no coupled feedback. Call again (or
        ``stop()``) to change. Positive turn rate = right turn
        (clockwise viewed from above), Pybricks convention.
        """
        if self._native is not None:
            # Clear any in-flight straight/turn trajectory first.
            self._native.stop()

        fwd_wheel_dps  = speed_mm_s / self._wheel_circumference * 360
        turn_rad_s     = math.radians(turn_rate_dps)
        diff_mm_s      = turn_rad_s * (self._axle_track / 2)
        diff_wheel_dps = diff_mm_s / self._wheel_circumference * 360

        self._run_at_dps(self._left,  fwd_wheel_dps + diff_wheel_dps)
        self._run_at_dps(self._right, fwd_wheel_dps - diff_wheel_dps)

    def stop(self, then="coast"):
        """Halt both wheels. Also clears any pending ``wait=False``
        move (new command supersedes, pybricks-style). ``then``
        selects the end-state:

        * ``"coast"`` (default) — both motors free-wheel.
        * ``"brake"`` — both motors actively resist motion at zero velocity.
        * ``"hold"`` — both motors actively hold their current angle.
          Requires motors that implement ``hold()`` (e.g. ``ST3215Motor``);
          open-loop drivers raise ``NotImplementedError``.

        On serial-bus (adopted) motors the whole stop is staged
        atomically in the C engine: both wheels reach the end-state
        at the same bus-packet boundary — one sync-torque packet for
        coast, one sync-speed packet for brake, same-instant pose
        capture for hold — never one motor at a time.
        """
        if then not in ("coast", "brake", "hold"):
            raise ValueError(
                "then must be 'coast', 'brake', or 'hold' (got %r)" % then)
        self._pending = None
        if self._serial_engine is not None:
            self._serial_engine.stop(then)
            # New command wins: the atomic stop supersedes any
            # motor-level wait=False move (the per-motor dispatch
            # used to clear these as a side effect).
            self._left._native_pending = None
            self._right._native_pending = None
            return
        if self._native is not None:
            self._native.stop()
        if then == "coast":
            self._left.coast()
            self._right.coast()
        elif then == "brake":
            self._left.brake()
            self._right.brake()
        else:   # "hold"
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
        if mode == "straight_native" or mode == "turn_native":
            if self._native.is_done():
                self.stop(then=self._pending["then"])
                return True
            return False
        if mode == "straight_serial" or mode == "turn_serial":
            if self._serial_engine.tick_done():
                self.stop(then=self._pending["then"])
                return True
            return False
        # Unknown mode — treat as done to avoid wedging the caller.
        self._pending = None
        return True

    # ---- blocking moves via the C coupled controller ----
    def straight(self, distance_mm, then="coast", wait=True):
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

        Raises ``RuntimeError`` for open-loop motor pairs — moves by
        distance need feedback; use ``drive()``/``stop()``."""
        if then not in ("coast", "brake", "hold"):
            raise ValueError(
                "then must be 'coast', 'brake', or 'hold' (got %r)" % then)
        self._arm_straight(distance_mm, then)
        if wait:
            while not self.done():
                time.sleep_ms(10)

    def turn(self, angle_deg, then="coast", wait=True):
        """Turn in place by ``angle_deg`` body heading (positive =
        right/clockwise viewed from above, Pybricks convention).

        Same ``then`` / ``wait`` semantics as ``straight()`` — see
        its docstring."""
        if then not in ("coast", "brake", "hold"):
            raise ValueError(
                "then must be 'coast', 'brake', or 'hold' (got %r)" % then)
        self._arm_turn(angle_deg, then)
        if wait:
            while not self.done():
                time.sleep_ms(10)

    # ---- arm: stash pending state, kick off motion ----
    def _arm_straight(self, distance_mm, then):
        if self._serial_engine is not None:
            self._serial_engine.arm_straight(float(distance_mm))
            self._pending = {"mode": "straight_serial", "then": then}
            return
        if self._native is not None:
            # Ensure both servos are attached to motor_process; the
            # native drivebase writes directly to their target_dps
            # but doesn't subscribe them itself.
            self._left.run_speed(0)
            self._right.run_speed(0)
            speed_mm_s = self._straight_speed_dps * self._wheel_circumference / 360
            self._native.straight(float(distance_mm), float(speed_mm_s))
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
            self._left.run_speed(0)
            self._right.run_speed(0)
            self._native.turn(float(angle_deg), float(self._turn_rate_dps))
            self._pending = {"mode": "turn_native", "then": then}
            return
        raise RuntimeError(
            "turn() needs closed-loop motors (encoder servos or "
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
