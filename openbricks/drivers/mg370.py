# SPDX-License-Identifier: MIT
"""
MG370 geared DC motor with GMR (giant magnetoresistance) quadrature encoder.

The GMR variant has a 500-PPR encoder on the motor shaft, giving massively
more resolution than a standard Hall-effect encoder — at a 1:34 gearbox
the output shaft sees ``500 * 4 * 34.014 ≈ 68028 CPR``. That edge rate is
too fast for the software ``QuadratureEncoder`` (Pin.irq() tops out around
5-10 kHz), so this driver uses the native ``PCNTEncoder`` — a C wrapper
over the ESP32 PCNT peripheral — baked into the firmware image.

Every MG370Motor instance needs its own PCNT unit — ESP32 has 8, ESP32-S3
has 4. Pick unit=0 for the first motor, unit=1 for the second, etc.

Apart from the encoder layer, MG370Motor is identical to JGB37Motor:
same native ``Servo`` underneath, same control tick, same closed-loop API.
"""

import time

from machine import Pin, PWM

from openbricks import pins
from openbricks import estop
from openbricks._native import PCNTEncoder, Servo
from openbricks.interfaces import Motor

_PWM_FREQ_HZ = 20_000
_DEFAULT_KP = 0.3

# Default for MG370 GMR with 1:34 gearbox (the spec-sheet ratio).
#   500 PPR * 4 (quadrature) * 34.014 ≈ 68028
# Override in the constructor if your gearbox ratio differs.
_DEFAULT_CPR = 68028


class _InvertedEncoder:
    """Wraps an encoder so its ``count()`` returns negated counts.

    Used by ``MG370Motor(encoder_invert=True)`` for the case where the
    encoder physically counts in the opposite direction of the motor's
    "forward" — common with mirror-mounted motor pairs (one side of a
    differential drivebase) where motor leads happen to be wired such
    that ``run(+N)`` already drives the wheel forward, but the encoder
    naturally counts down because the motor shaft spins the other way
    relative to the other side.

    Different from ``invert=True``: that flag flips both motor command
    AND encoder reading together (for motors wired backward end-to-end).
    ``encoder_invert`` is the right flag when only the encoder direction
    is "wrong" relative to motor direction.
    """

    def __init__(self, inner):
        self._inner = inner

    def count(self):
        return -self._inner.count()

    def reset(self, value=0):
        self._inner.reset(-int(value))


class MG370Motor(Motor):
    """MG370 DC gear motor with high-resolution GMR encoder, closed-loop.

    Same ``Motor`` contract and native 1 kHz servo core as
    :class:`~openbricks.drivers.jgb37_520.JGB37Motor`; the difference
    is the encoder path — the 500-PPR GMR encoder is counted by the
    ESP32 PCNT hardware peripheral (``pcnt_unit``), not GPIO
    interrupts, because its edge rate exceeds what ``Pin.irq`` can
    service.

    Args:
        in1, in2, pwm: H-bridge pins.
        encoder_a, encoder_b: encoder channel pins (into PCNT).
        pcnt_unit: PCNT peripheral unit, unique per motor (ESP32 has
            8, ESP32-S3 has 4 — first motor 0, second 1, ...).
        pcnt_filter: PCNT glitch filter in APB cycles (default 1023
            ≈ 12.8 µs; lower it only for encoders faster than ~35 kHz
            edge rate).
        counts_per_output_rev: encoder edges per output-shaft
            revolution (default matches the 1:34 GMR variant).
        invert / encoder_invert / kp: as in ``JGB37Motor``.
    """

    def __init__(
        self,
        in1, in2, pwm,
        encoder_a, encoder_b,
        pcnt_unit=0,
        pcnt_filter=1023,
        counts_per_output_rev=_DEFAULT_CPR,
        invert=False,
        encoder_invert=False,
        kp=_DEFAULT_KP,
    ):
        pins.check(in1, "H-bridge IN1")
        pins.check(in2, "H-bridge IN2")
        pins.check(pwm, "H-bridge PWM/EN")
        pins.check(encoder_a, "encoder channel A", output=False)
        pins.check(encoder_b, "encoder channel B", output=False)
        self._in1 = Pin(in1, Pin.OUT, value=0)
        self._in2 = Pin(in2, Pin.OUT, value=0)
        self._pwm = PWM(Pin(pwm), freq=_PWM_FREQ_HZ, duty=0)
        self._enc = PCNTEncoder(
            pin_a=encoder_a, pin_b=encoder_b,
            unit=pcnt_unit, filter=pcnt_filter,
        )
        encoder_for_servo = _InvertedEncoder(self._enc) if encoder_invert else self._enc
        self._servo = Servo(
            in1=self._in1,
            in2=self._in2,
            pwm=self._pwm,
            encoder=encoder_for_servo,
            counts_per_rev=counts_per_output_rev,
            invert=invert,
            kp=kp,
        )

    # --- Open-loop passthroughs ---
    def dc(self, duty):
        estop.check()
        self._servo.run(float(duty))

    def speed(self):
        # Measured deg/s from the native servo core's α-β observer
        # (already in output-shaft degrees; invert handled core-side).
        return self._servo.measured_dps()

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
        estop.check()
        self._servo.run_speed(float(deg_per_s))

    def run_angle(self, deg_per_s, target_angle, wait=True,
                  accel_dps2=1500.0):
        estop.check()
        self._servo.run_target(float(target_angle),
                               abs(float(deg_per_s)),
                               float(accel_dps2))
        if not wait:
            return
        while not self._servo.is_done():
            time.sleep_ms(10)
        self.brake()
