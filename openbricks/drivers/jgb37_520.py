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

The closed-loop methods use a very simple P controller on speed. For a
Pybricks-quality experience you'd want a full state observer + trapezoidal
trajectory planner — see ``docs/architecture.md``.
"""

import time

from openbricks.drivers.encoder import QuadratureEncoder
from openbricks.drivers.l298n import L298NMotor
from openbricks.interfaces import Motor


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

    # --- Open-loop passthroughs ---
    def run(self, power):
        self._driver.run(power)

    def brake(self):
        self._driver.brake()

    def coast(self):
        self._driver.coast()

    # --- Closed-loop ---
    def angle(self):
        """Output-shaft angle in degrees."""
        return self._enc.count() * 360 / self._counts_per_rev

    def reset_angle(self, angle=0):
        self._enc.reset(int(angle * self._counts_per_rev / 360))

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

    def run_speed(self, deg_per_s):
        """
        Hold a target shaft speed using a single P-control step.
        Call repeatedly in a loop. Non-blocking.
        """
        measured = self._measure_speed_dps()
        error = deg_per_s - measured
        # Feed-forward: roughly deg_per_s / rated_speed * 100. Rated speed
        # varies by motor; we use 300 deg/s as a sane default. Retune kp +
        # this constant for your specific gearbox.
        feedforward = deg_per_s / 300.0 * 100.0
        power = feedforward + self._kp * error
        if power > 100:
            power = 100
        elif power < -100:
            power = -100
        self.run(power)

    def run_angle(self, deg_per_s, target_angle, wait=True):
        """Rotate by ``target_angle`` degrees at approximately ``deg_per_s``."""
        start = self.angle()
        direction = 1 if target_angle >= 0 else -1
        end = start + target_angle
        deg_per_s = abs(deg_per_s) * direction

        if not wait:
            # Fire and forget — caller is responsible for stopping.
            self.run_speed(deg_per_s)
            return

        # Blocking loop with ~100 Hz update rate.
        while (direction > 0 and self.angle() < end) or \
              (direction < 0 and self.angle() > end):
            self.run_speed(deg_per_s)
            time.sleep_ms(10)
        self.brake()
