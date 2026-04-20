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
    def __init__(self, left, right, wheel_diameter_mm, axle_track_mm):
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
        """
        self._left = left
        self._right = right
        self._wheel_circumference = math.pi * wheel_diameter_mm
        self._axle_track = axle_track_mm

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
            )
        else:
            self._native = None

        # Default cruise parameters (wheel-degrees per second). Tweak via
        # ``settings()``.
        self._straight_speed_dps = 200
        self._turn_rate_dps      = 180

    def settings(self, straight_speed=None, turn_rate=None):
        if straight_speed is not None:
            self._straight_speed_dps = straight_speed
        if turn_rate is not None:
            self._turn_rate_dps = turn_rate

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

    def stop(self):
        if self._native is not None:
            self._native.stop()
        self._left.brake()
        self._right.brake()

    # ---- blocking moves via the C coupled controller ----
    def straight(self, distance_mm):
        """Drive forward by ``distance_mm``. Blocking, 2-DOF coupled.

        Falls back to a pure-Python open-loop sweep for motors without
        a native servo."""
        if self._native is None:
            self._straight_fallback(distance_mm)
            return

        # Ensure both servos are attached to motor_process; the native
        # drivebase writes directly to their target_dps but doesn't
        # subscribe them itself.
        self._left.run_speed(0)
        self._right.run_speed(0)

        speed_mm_s = self._straight_speed_dps * self._wheel_circumference / 360
        self._native.straight(float(distance_mm), float(speed_mm_s))

        while not self._native.is_done():
            time.sleep_ms(10)
        self.stop()

    def turn(self, angle_deg):
        """Turn in place by ``angle_deg`` body heading (positive = left)."""
        if self._native is None:
            self._turn_fallback(angle_deg)
            return

        self._left.run_speed(0)
        self._right.run_speed(0)

        self._native.turn(float(angle_deg), float(self._turn_rate_dps))

        while not self._native.is_done():
            time.sleep_ms(10)
        self.stop()

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
        # Open-loop fallback: assume ~300 dps rated.
        power = max(-100, min(100, dps / 300 * 100))
        motor.run(power)

    # ---- fallbacks for open-loop motor pairs ----
    def _straight_fallback(self, distance_mm):
        target_wheel_deg = distance_mm / self._wheel_circumference * 360
        self._left.reset_angle(0)
        self._right.reset_angle(0)

        direction = 1 if distance_mm >= 0 else -1
        speed = self._straight_speed_dps * direction

        while True:
            left_deg  = self._left.angle()
            right_deg = self._right.angle()
            avg = (left_deg + right_deg) / 2
            if direction > 0 and avg >= target_wheel_deg:
                break
            if direction < 0 and avg <= target_wheel_deg:
                break
            err = left_deg - right_deg
            self._run_at_dps(self._left,  speed - err)
            self._run_at_dps(self._right, speed + err)
            time.sleep_ms(10)
        self.stop()

    def _turn_fallback(self, angle_deg):
        arc_mm = math.radians(abs(angle_deg)) * (self._axle_track / 2)
        wheel_deg_each = arc_mm / self._wheel_circumference * 360

        self._left.reset_angle(0)
        self._right.reset_angle(0)

        direction = 1 if angle_deg >= 0 else -1
        speed = self._turn_rate_dps

        while True:
            left  = self._left.angle() * (-direction)
            right = self._right.angle() * direction
            if left >= wheel_deg_each and right >= wheel_deg_each:
                break
            self._run_at_dps(self._left,  -speed * direction)
            self._run_at_dps(self._right,  speed * direction)
            time.sleep_ms(10)
        self.stop()
