# SPDX-License-Identifier: MIT
"""
JGB37-520 geared DC motor with magnetic quadrature encoder.

This is a 6-wire motor: two power leads (into an H-bridge, typically L298N),
Vcc and GND for the hall sensors, and two encoder channels A and B.

Typical spec sheet values (varies by gear ratio variant):
    * Encoder CPR at motor shaft: 11
    * Gear ratio (example): 1:30  -> output shaft CPR = 11 * 30 * 4 = 1320 edges
      (multiply by 4 because we count both edges on both channels)

The constructor takes ``counts_per_output_rev`` so you can tune it to your
specific gearbox variant. 1320 is a common default for JGB37-520 with 1:30
gearing.

Closed-loop control runs on the always-on ``MotorProcess`` scheduler. The
user-facing methods manage whether this motor is actively driven by that
scheduler:

    * ``run_speed(dps)`` enters closed-loop control; the control step
      registers with the scheduler if not already.
    * ``brake()`` / ``coast()`` set the H-bridge state *and* detach the
      control step so the next tick doesn't overwrite that state.
    * ``run(power)`` is an open-loop override; it detaches too.

Users never see the scheduler directly.

For Pybricks-quality control you'd want a full state observer + trajectory
planner — those land in M2/M3 of the roadmap. This driver's tick body is
currently a plain P-controller on raw encoder deltas.
"""

import time

from openbricks.drivers.encoder import QuadratureEncoder
from openbricks.drivers.l298n import L298NMotor
from openbricks.interfaces import Motor
from openbricks.tools.scheduler import MotorProcess


class JGB37Motor(Motor):
    def __init__(
        self,
        in1, in2, pwm,
        encoder_a, encoder_b,
        counts_per_output_rev=1320,
        invert=False,
        kp=0.3,
    ):
        self._driver = L298NMotor(in1=in1, in2=in2, pwm=pwm, invert=invert)
        self._enc = QuadratureEncoder(encoder_a, encoder_b)
        self._counts_per_rev = counts_per_output_rev
        self._kp = kp
        self._last_count = 0
        self._last_time_ms = time.ticks_ms()
        self._target_dps = 0.0
        self._active = False   # True iff ``_control_step`` is registered

    # --- Open-loop passthroughs (detach from closed-loop control) ---
    def run(self, power):
        self._detach()
        self._driver.run(power)

    def brake(self):
        self._detach()
        self._driver.brake()

    def coast(self):
        self._detach()
        self._driver.coast()

    # --- Closed-loop state ---
    def angle(self):
        """Output-shaft angle in degrees."""
        return self._enc.count() * 360 / self._counts_per_rev

    def reset_angle(self, angle=0):
        self._enc.reset(int(angle * self._counts_per_rev / 360))

    def run_speed(self, deg_per_s):
        """Enter closed-loop speed control with the given target (deg/s).

        Non-blocking; the scheduler's control step drives the H-bridge
        toward the target from the next tick onward.
        """
        self._target_dps = float(deg_per_s)
        self._attach()

    def run_angle(self, deg_per_s, target_angle, wait=True):
        """Rotate by ``target_angle`` degrees at approximately ``deg_per_s``.

        Uses the scheduler — the H-bridge is driven by periodic control
        ticks, and this method simply sleeps until the encoder reaches the
        target. With ``wait=False`` the scheduler keeps driving; the caller
        is responsible for eventually calling ``brake()`` or ``coast()``.
        """
        direction = 1 if target_angle >= 0 else -1
        end = self.angle() + target_angle
        self.run_speed(abs(deg_per_s) * direction)

        if not wait:
            return

        while True:
            a = self.angle()
            if direction > 0 and a >= end:
                break
            if direction < 0 and a <= end:
                break
            time.sleep_ms(10)

        self.brake()

    # --- Internals ---
    def _measure_speed_dps(self):
        """Instantaneous speed in degrees per second, from encoder deltas."""
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time_ms)
        if dt <= 0:
            return 0.0
        count = self._enc.count()
        d_count = count - self._last_count
        self._last_count = count
        self._last_time_ms = now
        return (d_count * 360_000) / (self._counts_per_rev * dt)

    def _control_step(self):
        """Scheduler tick body: P-control on measured speed."""
        measured = self._measure_speed_dps()
        error = self._target_dps - measured
        # Feed-forward: roughly deg_per_s / rated_speed * 100. Rated speed
        # varies by motor; we use 300 deg/s as a sane default. Retune kp +
        # this constant for your specific gearbox.
        feedforward = self._target_dps / 300.0 * 100.0
        power = feedforward + self._kp * error
        if power > 100:
            power = 100
        elif power < -100:
            power = -100
        self._driver.run(power)

    def _attach(self):
        if self._active:
            return
        # Reset the speed-measurement baselines so the first tick sees a
        # real dt rather than the interval since the motor was constructed.
        self._last_count = self._enc.count()
        self._last_time_ms = time.ticks_ms()
        MotorProcess.instance().register(self._control_step)
        self._active = True

    def _detach(self):
        if not self._active:
            return
        MotorProcess.instance().unregister(self._control_step)
        self._active = False
        self._target_dps = 0.0
