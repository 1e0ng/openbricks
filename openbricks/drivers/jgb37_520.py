# SPDX-License-Identifier: MIT
"""
JGB37-520 geared DC motor with magnetic quadrature encoder.

This is a 6-wire motor: two power leads (into an H-bridge, typically L298N),
Vcc and GND for the hall sensors, and two encoder channels A and B.

Typical spec sheet values (varies by gear ratio variant):
    * Encoder CPR at motor shaft: 11
    * Gear ratio (example): 1:30  -> output shaft CPR = 11 * 30 * 4 = 1320 edges
      (multiply by 4 because we count both edges on both channels)

This class is a thin Python wrapper over the C ``Servo`` type in
``_openbricks_native`` (see ``native/user_c_modules/openbricks/servo.c``).
The wrapper's job is to build the hardware handles (Pin / PWM / encoder)
from pin numbers and pass them to the native servo — the closed-loop
control tick runs in C at the ``motor_process`` tick rate (1 kHz default).
"""

import time

from machine import Pin, PWM

from openbricks._native import Servo
from openbricks.drivers.encoder import QuadratureEncoder
from openbricks.interfaces import Motor

_PWM_FREQ_HZ = 20_000
_DEFAULT_KP = 0.3


class JGB37Motor(Motor):
    def __init__(
        self,
        in1, in2, pwm,
        encoder_a, encoder_b,
        counts_per_output_rev=1320,
        invert=False,
        kp=_DEFAULT_KP,
    ):
        self._in1 = Pin(in1, Pin.OUT, value=0)
        self._in2 = Pin(in2, Pin.OUT, value=0)
        self._pwm = PWM(Pin(pwm), freq=_PWM_FREQ_HZ, duty=0)
        self._enc = QuadratureEncoder(encoder_a, encoder_b)
        self._servo = Servo(
            in1=self._in1,
            in2=self._in2,
            pwm=self._pwm,
            encoder=self._enc,
            counts_per_rev=counts_per_output_rev,
            invert=invert,
            kp=kp,
        )

    # --- Open-loop passthroughs (native Servo detaches from scheduler) ---
    def run(self, power):
        self._servo.run(float(power))

    def brake(self):
        self._servo.brake()

    def coast(self):
        self._servo.coast()

    # --- Closed-loop state ---
    def angle(self):
        return self._servo.angle()

    def reset_angle(self, angle=0):
        self._servo.reset_angle(float(angle))

    def run_speed(self, deg_per_s):
        """Enter closed-loop speed control with the given target (deg/s)."""
        self._servo.run_speed(float(deg_per_s))

    def run_angle(self, deg_per_s, target_angle, wait=True):
        """Rotate by ``target_angle`` degrees at approximately ``deg_per_s``.

        The native scheduler ticks the servo in C while this method sleeps.
        With ``wait=False`` the caller is responsible for eventually
        calling ``brake()`` or ``coast()``.
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
