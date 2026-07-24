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

Open-loop ``drive()`` bypasses the coupled controller; it just maps
(speed_mm_s, turn_rate_dps) → (left_dps, right_dps) and hands them to
each servo's ``run_speed``. Useful for interactive control where
profile-based moves would feel sluggish.
"""

import math
import time

from openbricks._native import DriveBase as _NativeDriveBase


class DriveBase:
    def __init__(self, left, right, wheel_diameter_mm, axle_track_mm,
                 imu=None):
        """
        Args:
            left, right: Motor instances. The wrapper reaches through to
                ``.servo`` (for JGB37Motor) when constructing the native
                drivebase, since the C layer operates on the servo struct
                directly. Motors without a native servo (e.g. plain
                ``L298NMotor`` with no encoder) fall back to an open-loop
                path.
            wheel_diameter_mm: wheel diameter in millimeters.
            axle_track_mm: distance between the two wheel contact points.
            imu: optional ``IMU``-conformant object (any driver with a
                ``.heading()`` method returning body heading in degrees —
                the bundled ``BNO055`` qualifies). When provided, call
                ``drivebase.use_gyro(True)`` to have the heading loop
                read from the IMU instead of computing from the encoder
                differential. Slip-immune. Works on both paths: the
                native controller reads it on the 1 kHz tick, and the
                serial-bus fallback reads it once per ``done()`` poll.
        """
        self._left = left
        self._right = right
        self._wheel_circumference = math.pi * wheel_diameter_mm
        self._axle_track = axle_track_mm
        self._imu = imu
        # Fallback-path gyro toggle. Meaningless when ``_native`` is
        # set — the native controller keeps its own flag internally.
        self._use_gyro = False

        # The native drivebase is only usable if both motors are
        # closed-loop servos. Otherwise the wrapper falls through to a
        # pure-Python open-loop implementation.
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
        # Trajectory acceleration (wheel-deg/s²). Mirrors the native
        # core's default so both paths launch identically. The native
        # path stores its own copy (set via set_accel); this one drives
        # the fallback's trapezoid.
        self._accel_dps2 = 1500.0

        # State for in-flight ``straight(wait=False)`` / ``turn(wait=False)``
        # moves. ``None`` means nothing pending; ``done()`` returns
        # True. ``stop()`` clears this. See ``done`` for the layout.
        self._pending = None

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
                fallback shapes its per-tick speed command with the
                same trapezoid.
        """
        if straight_speed is not None:
            self._straight_speed_dps = straight_speed
        if turn_rate is not None:
            self._turn_rate_dps = turn_rate
        if acceleration is not None:
            if not acceleration > 0:
                raise ValueError(
                    "acceleration must be > 0 deg/s^2 (got %r)"
                    % (acceleration,))
            self._accel_dps2 = float(acceleration)
            if self._native is not None:
                self._native.set_accel(float(acceleration))

    def use_gyro(self, enable):
        """Switch the heading feedback source between encoder-diff (default)
        and the attached IMU (when True). Pybricks-style.

        Requires an ``imu=`` argument to the constructor. With the gyro,
        heading is slip-immune — wheel slip or wildly asymmetric friction
        won't throw the robot off course, because the IMU sees actual body
        rotation regardless of what the wheels did. Works on both the
        native (encoder-servo) path and the serial-bus fallback.
        """
        enable = bool(enable)
        if enable and self._imu is None:
            raise ValueError(
                "no imu attached; construct DriveBase(imu=...) first")
        if self._native is not None:
            self._native.use_gyro(enable)
        else:
            self._use_gyro = enable

    # ---- non-blocking open-loop ----
    def drive(self, speed_mm_s, turn_rate_dps):
        """Start driving at a given forward speed + body turn rate.

        Kinematic one-shot — no coupled feedback. Call again (or
        ``stop()``) to change. Positive turn rate = left turn.
        """
        if self._native is not None:
            # Clear any in-flight straight/turn trajectory first.
            self._native.stop()

        fwd_wheel_dps  = speed_mm_s / self._wheel_circumference * 360
        turn_rad_s     = math.radians(turn_rate_dps)
        diff_mm_s      = turn_rad_s * (self._axle_track / 2)
        diff_wheel_dps = diff_mm_s / self._wheel_circumference * 360

        self._run_at_dps(self._left,  fwd_wheel_dps - diff_wheel_dps)
        self._run_at_dps(self._right, fwd_wheel_dps + diff_wheel_dps)

    def stop(self, then="coast"):
        """Halt both wheels. Also clears any pending ``wait=False``
        move (new command supersedes, pybricks-style). ``then``
        selects the end-state:

        * ``"coast"`` (default) — both motors free-wheel.
        * ``"brake"`` — both motors actively resist motion at zero velocity.
        * ``"hold"`` — both motors actively hold their current angle.
          Requires motors that implement ``hold()`` (e.g. ``ST3215Motor``);
          open-loop drivers raise ``NotImplementedError``.
        """
        if then not in ("coast", "brake", "hold"):
            raise ValueError(
                "then must be 'coast', 'brake', or 'hold' (got %r)" % then)
        self._pending = None
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

        On the fallback path (serial-bus servo drivebases), motors
        start moving on the ``straight()`` / ``turn()`` call itself;
        each ``done()`` invocation then runs ONE tick of the
        heading-hold loop — read both angles, apply differential
        correction, check the target. **You must keep polling until
        ``done()`` returns True** or the motors will continue running
        at the open-loop profile speed past the target. The natural
        cadence is ``time.sleep_ms(10)`` between polls.

        On the native path, the C scheduler runs the trajectory
        independently at 1 kHz; ``done()`` just checks a flag and
        motors stop automatically when the trajectory completes.
        """
        if self._pending is None:
            return True
        mode = self._pending["mode"]
        if mode == "straight_native" or mode == "turn_native":
            if self._native.is_done():
                self.stop(then=self._pending["then"])
                return True
            return False
        if mode == "straight_fallback":
            return self._straight_fallback_tick()
        if mode == "turn_fallback":
            return self._turn_fallback_tick()
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
        the caller polls ``done()`` to advance the state machine
        (fallback path) or just check completion (native path), and
        the ``then=`` dispatch is deferred until ``done()`` reports
        the target was reached. Concurrent use with another
        wait=False move on a separate ``DriveBase`` (or with motor
        ``run_angle(wait=False)`` calls) is the intended pattern.

        Any subsequent move command supersedes the previous pending
        wait=False move (pybricks "new command wins").

        Falls back to a pure-Python heading-hold loop for motors
        without a native servo (i.e. serial-bus ST-3215/ST-3032
        drivebases)."""
        if then not in ("coast", "brake", "hold"):
            raise ValueError(
                "then must be 'coast', 'brake', or 'hold' (got %r)" % then)
        self._arm_straight(distance_mm, then)
        if wait:
            while not self.done():
                time.sleep_ms(10)

    def turn(self, angle_deg, then="coast", wait=True):
        """Turn in place by ``angle_deg`` body heading (positive = left).

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
        # Fallback: snapshot anchors, stash state, motors start on
        # first done() tick.
        start_left  = self._read_angle_or_bail(self._left)
        start_right = self._read_angle_or_bail(self._right)
        if start_left is None or start_right is None:
            self.stop(then=then)
            return
        target_wheel_deg = distance_mm / self._wheel_circumference * 360
        direction = 1 if distance_mm >= 0 else -1
        speed = self._straight_speed_dps * direction
        self._pending = {
            "mode": "straight_fallback",
            "target_wheel_deg": target_wheel_deg,
            "direction": direction,
            "start_left":  start_left,
            "start_right": start_right,
            "start_heading": self._imu.heading() if self._use_gyro else None,
            "speed": speed,
            "consecutive_none": 0,
            "then": then,
            "start_ms":  time.ticks_ms(),
            "budget_ms": self._move_budget_ms(target_wheel_deg,
                                              self._straight_speed_dps),
        }
        # Kick off motion immediately so ``wait=False`` looks like
        # pybricks — motors start on the call, not on first ``done()``.
        # The launch is profile-shaped: at t=0 the trapezoid commands
        # the crawl floor, not cruise — a serial-bus drivebase used to
        # step straight to cruise speed here, an effectively infinite
        # acceleration that pitched real chassis on launch. Subsequent
        # ``done()`` ticks ramp up and apply heading-hold corrections.
        v0 = self._profile_speed(speed, 0.0, abs(target_wheel_deg))
        self._run_at_dps(self._left,  v0 * direction)
        self._run_at_dps(self._right, v0 * direction)

    def _arm_turn(self, angle_deg, then):
        if self._native is not None:
            self._left.run_speed(0)
            self._right.run_speed(0)
            self._native.turn(float(angle_deg), float(self._turn_rate_dps))
            self._pending = {"mode": "turn_native", "then": then}
            return
        start_left  = self._read_angle_or_bail(self._left)
        start_right = self._read_angle_or_bail(self._right)
        if start_left is None or start_right is None:
            self.stop(then=then)
            return
        arc_mm = math.radians(abs(angle_deg)) * (self._axle_track / 2)
        wheel_deg_each = arc_mm / self._wheel_circumference * 360
        direction = 1 if angle_deg >= 0 else -1
        speed = self._turn_rate_dps
        self._pending = {
            "mode": "turn_fallback",
            "wheel_deg_each": wheel_deg_each,
            "angle_deg": angle_deg,
            "direction": direction,
            "start_left":  start_left,
            "start_right": start_right,
            "start_heading": self._imu.heading() if self._use_gyro else None,
            "speed": speed,
            "consecutive_none": 0,
            "then": then,
            "start_ms":  time.ticks_ms(),
            "budget_ms": self._move_budget_ms(wheel_deg_each,
                                              self._turn_rate_dps),
        }
        # Kick off motion immediately — same reasoning as
        # ``_arm_straight``. In-place turn means opposite wheel signs.
        v0 = self._profile_speed(speed, 0.0, wheel_deg_each)
        self._run_at_dps(self._left,  -v0 * direction)
        self._run_at_dps(self._right,  v0 * direction)

    # ---- helpers ----
    # Minimum fallback speed command (dps). The decel curve approaches
    # zero speed exactly at the target; real serial-bus servos stall on
    # friction below a crawl speed and would time out just short of the
    # goal. The floor keeps the final approach moving — the target
    # check, not the profile, ends the move.
    _FALLBACK_MIN_DPS = 15.0

    def _profile_speed(self, cruise_dps, elapsed_s, remaining_deg):
        """Trapezoidal speed command for the fallback loop: ramp up at
        ``self._accel_dps2`` from move start, cruise, and ramp down so
        v² = 2·a·remaining reaches ~0 at the target. Returns a positive
        magnitude, floored at ``_FALLBACK_MIN_DPS``."""
        v = abs(cruise_dps)
        v_ramp = self._accel_dps2 * elapsed_s
        if v_ramp < v:
            v = v_ramp
        if remaining_deg > 0:
            v_decel = math.sqrt(2.0 * self._accel_dps2 * remaining_deg)
            if v_decel < v:
                v = v_decel
        if v < self._FALLBACK_MIN_DPS:
            v = self._FALLBACK_MIN_DPS
        return v

    def _heading_delta_deg(self, start_heading):
        """Body-heading rotation since ``start_heading``, wrapped into
        [-180, 180) so a move that crosses the BNO055's ±180 wrap
        point doesn't see a spurious ±360 jump. Mirrors the native
        core's ``drivebase_read_body_delta`` (drivebase.c)."""
        delta = self._imu.heading() - start_heading
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
        return delta

    def _gyro_diff_err_wheel_deg(self, start_heading):
        """Convert accumulated body-heading rotation into the same
        wheel-degree differential ``left_deg - right_deg`` would read
        for the same physical rotation — so swapping the heading
        source doesn't change the fallback's correction gain. Inverse
        of the native core's ``ob_drivebase_body_to_wheel_diff``
        (drivebase_core.c), scaled by 2 since that function returns
        the halved ``diff_pos`` and this fallback's ``err`` is the raw
        L-R differential."""
        delta = self._heading_delta_deg(start_heading)
        return -delta * (2.0 * self._axle_track * math.pi /
                          self._wheel_circumference)

    @staticmethod
    def _run_at_dps(motor, dps):
        run_speed = getattr(motor, "run_speed", None)
        if callable(run_speed):
            try:
                run_speed(dps)
                return
            except NotImplementedError:
                pass
        # Open-loop fallback: assume ~300 dps rated.
        power = max(-100, min(100, dps / 300 * 100))
        motor.dc(power)

    # ---- fallbacks for open-loop motor pairs ----
    #
    # Serial-bus servos (ST-3215, ST-3032) hit this path too, since
    # they don't subscribe to the native motor_process scheduler.
    # Their ``angle()`` returns ``None`` on a present-position read
    # timeout — one EMI burst or one half-duplex collision is enough.
    # We mirror ``ST3215Motor.run_angle``'s inner-loop pattern:
    # tolerate a None tick (re-use the last good readings, hold the
    # last commanded drive, sleep, retry), and only bail out after
    # ``_MAX_CONSECUTIVE_NONE`` ticks in a row — ≈500 ms at the
    # 10 ms loop period — when the bus is genuinely dead rather than
    # just glitchy.

    # Capacity for transient bus drops before _straight_fallback /
    # _turn_fallback give up and stop. 50 × 10 ms = 500 ms.
    _MAX_CONSECUTIVE_NONE = 50

    def _move_budget_ms(self, travel_deg, rate_dps):
        """Wall-clock budget for a fallback move to reach its target.

        The fallback watches ``angle()`` climb toward the target. If a
        wheel stalls (mechanically blocked, in the servo's overload
        protection, or slipping) while the bus still answers, that
        climb stops and the target is never met — without this budget
        the move's ``done()`` would return ``False`` forever.

        The ideal time is the trapezoid duration at the configured
        acceleration (triangular profile when the move is too short to
        reach cruise); ×4 leaves headroom for friction and heading-hold
        slowdown, and a 1 s floor covers tiny moves. Only a genuine
        stall overruns it."""
        travel = abs(travel_deg)
        rate = abs(rate_dps)
        if rate < 1:
            rate = 1
        accel = self._accel_dps2
        if travel * accel >= rate * rate:
            # Trapezoid: cruise phase exists.
            ideal_s = travel / rate + rate / accel
        else:
            # Triangular: never reaches cruise.
            ideal_s = 2.0 * math.sqrt(travel / accel)
        return int(ideal_s * 1000.0 * 4 + 1000)

    def _straight_fallback_tick(self):
        """One iteration of the fallback heading-hold loop.

        Returns ``True`` when the target wheel angle has been reached
        (and ``stop(then=…)`` has been run, clearing ``_pending``);
        ``False`` while the move is still progressing or while we're
        waiting for the bus to come back from a glitch. Snapshot
        anchors live in ``self._pending`` from ``_arm_straight``."""
        state = self._pending
        if time.ticks_diff(time.ticks_ms(), state["start_ms"]) > state["budget_ms"]:
            self.stop(then=state["then"])
            raise RuntimeError(
                "DriveBase.straight did not reach target within %d ms — "
                "wheel stalled, in overload protection, or blocked"
                % state["budget_ms"])
        left_now  = self._left.angle()
        right_now = self._right.angle()
        if left_now is None or right_now is None:
            state["consecutive_none"] += 1
            if state["consecutive_none"] >= self._MAX_CONSECUTIVE_NONE:
                self.stop(then=state["then"])
                return True
            return False
        state["consecutive_none"] = 0
        left_deg  = left_now  - state["start_left"]
        right_deg = right_now - state["start_right"]
        avg = (left_deg + right_deg) / 2
        direction = state["direction"]
        target = state["target_wheel_deg"]
        if (direction > 0 and avg >= target) or \
           (direction < 0 and avg <= target):
            self.stop(then=state["then"])
            return True
        # Forward progress always comes from the encoders — the gyro
        # only replaces the heading-hold error term below.
        if self._use_gyro:
            err = self._gyro_diff_err_wheel_deg(state["start_heading"])
        else:
            err = left_deg - right_deg
        # Trapezoid-shaped speed magnitude for this tick: ramp from
        # move start, cruise, decelerate into the remaining distance.
        elapsed_s = time.ticks_diff(time.ticks_ms(), state["start_ms"]) / 1000.0
        remaining = (target - avg) * direction
        speed = self._profile_speed(state["speed"], elapsed_s, remaining) * direction
        self._run_at_dps(self._left,  speed - err)
        self._run_at_dps(self._right, speed + err)
        return False

    def _turn_fallback_tick(self):
        """One iteration of the fallback in-place-turn loop. Same
        return semantics as ``_straight_fallback_tick``."""
        state = self._pending
        if time.ticks_diff(time.ticks_ms(), state["start_ms"]) > state["budget_ms"]:
            self.stop(then=state["then"])
            raise RuntimeError(
                "DriveBase.turn did not reach target within %d ms — "
                "wheel stalled, in overload protection, or blocked"
                % state["budget_ms"])
        left_now  = self._left.angle()
        right_now = self._right.angle()
        if left_now is None or right_now is None:
            state["consecutive_none"] += 1
            if state["consecutive_none"] >= self._MAX_CONSECUTIVE_NONE:
                self.stop(then=state["then"])
                return True
            return False
        state["consecutive_none"] = 0
        direction = state["direction"]

        if self._use_gyro:
            # Actual body rotation decides both when the turn is done
            # and how much is left — slip-immune, unlike the encoder
            # average below, which just reports what the wheels think
            # they did.
            progressed = abs(self._heading_delta_deg(state["start_heading"]))
            target = abs(state["angle_deg"])
            if progressed >= target:
                self.stop(then=state["then"])
                return True
            remaining = target - progressed
        else:
            left  = (left_now  - state["start_left"])  * (-direction)
            right = (right_now - state["start_right"]) * direction
            if left >= state["wheel_deg_each"] and \
               right >= state["wheel_deg_each"]:
                self.stop(then=state["then"])
                return True
            # Same trapezoid shaping as the straight tick, on the
            # average per-wheel progress toward this turn's arc length.
            remaining = state["wheel_deg_each"] - (left + right) / 2

        elapsed_s = time.ticks_diff(time.ticks_ms(), state["start_ms"]) / 1000.0
        speed = self._profile_speed(state["speed"], elapsed_s, remaining)
        self._run_at_dps(self._left,  -speed * direction)
        self._run_at_dps(self._right,  speed * direction)
        return False

    @staticmethod
    def _read_angle_or_bail(motor, retries=5):
        """Read ``motor.angle()``, retrying up to ``retries`` times if
        the bus drops the reply. Returns the float on success or
        ``None`` if every retry timed out. Used to seed the loop's
        starting anchor — if we can't get a baseline, the whole
        fallback bails cleanly rather than entering the loop with a
        bogus reference."""
        for _ in range(retries):
            a = motor.angle()
            if a is not None:
                return a
            time.sleep_ms(10)
        return None
