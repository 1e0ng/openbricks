# SPDX-License-Identifier: MIT
"""
Two-wheel differential drivebase.

Takes two ``Motor`` implementations (anything from ``openbricks.interfaces``)
plus physical parameters, and exposes straight / turn / drive. This is the
open-hardware analogue of Pybricks' ``DriveBase``, minus the sophisticated
control loop. For a robot with good encoders (JGB37-520) the simple P
controller plus encoder feedback is good enough to drive in straight lines
on a flat floor.

Closed-loop motors (e.g. ``JGB37Motor``) run their control step on the
always-on ``MotorProcess`` scheduler. DriveBase just updates setpoints via
``run_speed()`` in its polling loop — the motor attaches itself to the
scheduler on ``run_speed``, detaches on ``brake`` / ``coast``. Open-loop
motors (e.g. ``L298NMotor`` without an encoder) raise ``NotImplementedError``
from ``run_speed``, which ``_run_at_dps`` catches and falls back to a
rated-speed ``run(power)``.

Future work: a trapezoidal trajectory planner and a true 2-DOF coupled
drivebase controller land in M2 and M4 of the roadmap respectively.
"""

import math
import time


class DriveBase:
    def __init__(self, left, right, wheel_diameter_mm, axle_track_mm):
        """
        Args:
            left, right: Motor instances (need ``run``, ``brake``, and — for
                any distance/angle methods — ``angle`` + ``reset_angle``).
            wheel_diameter_mm: wheel diameter in millimeters.
            axle_track_mm: distance between the two wheel contact points.
        """
        self._left = left
        self._right = right
        self._wheel_circumference = math.pi * wheel_diameter_mm
        self._axle_track = axle_track_mm

        # Default cruise parameters. Tweak via ``settings()``.
        self._straight_speed_dps = 200  # degrees per second at the wheel
        self._turn_rate_dps = 180       # degrees per second of heading change

    def settings(self, straight_speed=None, turn_rate=None):
        if straight_speed is not None:
            self._straight_speed_dps = straight_speed
        if turn_rate is not None:
            self._turn_rate_dps = turn_rate

    # ---- low level open-loop ----
    def drive(self, speed_mm_s, turn_rate_dps):
        """Start driving at a given forward speed + turn rate. Non-blocking.

        Positive turn rate = turn left. Pass 0 to stop.
        """
        # Convert mm/s forward to wheel deg/s.
        fwd_wheel_dps = speed_mm_s / self._wheel_circumference * 360
        # Convert body turn rate to wheel differential.
        # v_diff = (turn_rate_rad/s) * (axle_track/2), wheel_dps = v_diff / circ * 360.
        turn_rad_s = math.radians(turn_rate_dps)
        diff_mm_s = turn_rad_s * (self._axle_track / 2)
        diff_wheel_dps = diff_mm_s / self._wheel_circumference * 360

        left_dps = fwd_wheel_dps - diff_wheel_dps
        right_dps = fwd_wheel_dps + diff_wheel_dps

        # If the motor supports closed-loop speed, use it. Otherwise fall back
        # to an open-loop power estimate (assume rated speed = 300 dps).
        self._run_at_dps(self._left, left_dps)
        self._run_at_dps(self._right, right_dps)

    def stop(self):
        self._left.brake()
        self._right.brake()

    # ---- blocking helpers ----
    def straight(self, distance_mm):
        """Drive forward by ``distance_mm``. Blocking.

        Requires motors with ``angle()`` + ``reset_angle()``.
        """
        target_wheel_deg = distance_mm / self._wheel_circumference * 360
        self._left.reset_angle(0)
        self._right.reset_angle(0)

        direction = 1 if distance_mm >= 0 else -1
        speed = self._straight_speed_dps * direction

        while True:
            left_deg = self._left.angle()
            right_deg = self._right.angle()
            avg = (left_deg + right_deg) / 2
            if direction > 0 and avg >= target_wheel_deg:
                break
            if direction < 0 and avg <= target_wheel_deg:
                break

            # Simple straight-line correction: nudge speeds to keep angles equal.
            err = left_deg - right_deg
            kp = 1.0
            self._run_at_dps(self._left, speed - kp * err)
            self._run_at_dps(self._right, speed + kp * err)
            time.sleep_ms(10)

        self.stop()

    def turn(self, angle_deg):
        """Turn in place by ``angle_deg`` degrees (positive = left)."""
        # Wheel travel needed (each wheel turns opposite directions).
        arc_mm = math.radians(abs(angle_deg)) * (self._axle_track / 2)
        wheel_deg_each = arc_mm / self._wheel_circumference * 360

        self._left.reset_angle(0)
        self._right.reset_angle(0)

        direction = 1 if angle_deg >= 0 else -1
        speed = self._turn_rate_dps  # magnitude

        while True:
            left = self._left.angle() * (-direction)   # left reverses on left turn
            right = self._right.angle() * direction    # right advances on left turn
            if left >= wheel_deg_each and right >= wheel_deg_each:
                break
            self._run_at_dps(self._left, -speed * direction)
            self._run_at_dps(self._right, speed * direction)
            time.sleep_ms(10)

        self.stop()

    # ---- helpers ----
    @staticmethod
    def _run_at_dps(motor, dps):
        # Prefer closed-loop if the motor supports it.
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
