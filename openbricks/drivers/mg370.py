# SPDX-License-Identifier: MIT
"""
MG370 geared DC motor with GMR (giant magnetoresistance) quadrature encoder.

The GMR variant has a 500-PPR encoder on the motor shaft, giving massively
more resolution than a standard Hall-effect encoder — at a 1:34 gearbox
the output shaft sees ``500 * 4 * 34.014 ≈ 68028 CPR``. That edge rate is
too fast for the software ``QuadratureEncoder`` (Pin.irq() tops out around
5-10 kHz), so this driver uses the ESP32 PCNT-based
``PCNTQuadratureEncoder`` instead.

Every MG370Motor instance needs its own PCNT unit — ESP32 has 8, ESP32-S3
has 4. Pick unit=0 for the first motor, unit=1 for the second, etc.

Apart from the encoder layer, MG370Motor is identical to JGB37Motor:
same native ``Servo`` underneath, same control tick, same closed-loop API.
"""

import time

from machine import Pin, PWM

from openbricks._native import Servo
from openbricks.drivers.pcnt_encoder import PCNTQuadratureEncoder
from openbricks.interfaces import Motor

_PWM_FREQ_HZ = 20_000
_DEFAULT_KP = 0.3

# Default for MG370 GMR with 1:34 gearbox (the spec-sheet ratio).
#   500 PPR * 4 (quadrature) * 34.014 ≈ 68028
# Override in the constructor if your gearbox ratio differs.
_DEFAULT_CPR = 68028


class MG370Motor(Motor):
    def __init__(
        self,
        in1, in2, pwm,
        encoder_a, encoder_b,
        pcnt_unit=0,
        pcnt_filter=1023,
        counts_per_output_rev=_DEFAULT_CPR,
        invert=False,
        kp=_DEFAULT_KP,
    ):
        self._in1 = Pin(in1, Pin.OUT, value=0)
        self._in2 = Pin(in2, Pin.OUT, value=0)
        self._pwm = PWM(Pin(pwm), freq=_PWM_FREQ_HZ, duty=0)
        self._enc = PCNTQuadratureEncoder(
            encoder_a, encoder_b,
            unit=pcnt_unit, filter=pcnt_filter,
        )
        self._servo = Servo(
            in1=self._in1,
            in2=self._in2,
            pwm=self._pwm,
            encoder=self._enc,
            counts_per_rev=counts_per_output_rev,
            invert=invert,
            kp=kp,
        )

    # --- Open-loop passthroughs ---
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
        self._servo.run_speed(float(deg_per_s))

    def run_angle(self, deg_per_s, target_angle, wait=True,
                  accel_dps2=720.0):
        self._servo.run_target(float(target_angle),
                               abs(float(deg_per_s)),
                               float(accel_dps2))
        if not wait:
            return
        while not self._servo.is_done():
            time.sleep_ms(10)
        self.brake()
