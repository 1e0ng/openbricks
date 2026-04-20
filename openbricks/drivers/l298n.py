# SPDX-License-Identifier: MIT
"""
L298N H-bridge motor driver.

The L298N drives a brushed DC motor via two direction pins (IN1/IN2) and one
PWM pin (EN-A or EN-B). It's open-loop: the class knows nothing about the
physical motor speed or position. For closed-loop use, wrap one of these in
``jgb37_520.JGB37Motor`` (which adds an encoder).

Pinout recap for one channel:

    IN1  = direction bit A
    IN2  = direction bit B
    PWM  = enable / speed (tie to VCC for 100%, PWM for speed control)

    IN1  IN2  result
    ---  ---  ----------
     0    0   coast
     0    1   reverse
     1    0   forward
     1    1   brake
"""

from machine import Pin, PWM

from openbricks.interfaces import Motor

_PWM_FREQ_HZ = 20_000  # Above audible range; fine for most hobby H-bridges.
_DUTY_MAX = 1023       # ESP32 default PWM resolution.


class L298NMotor(Motor):
    def __init__(self, in1, in2, pwm, invert=False, pwm_freq=_PWM_FREQ_HZ):
        """
        Args:
            in1, in2: GPIO numbers for the two direction pins.
            pwm: GPIO number for the EN pin (speed/duty).
            invert: swap forward/reverse. Handy when wiring goes the wrong way.
            pwm_freq: PWM frequency in Hz.
        """
        self._in1 = Pin(in1, Pin.OUT, value=0)
        self._in2 = Pin(in2, Pin.OUT, value=0)
        self._pwm = PWM(Pin(pwm), freq=pwm_freq, duty=0)
        self._invert = invert

    def run(self, power):
        """Power is -100..100."""
        if power > 100:
            power = 100
        elif power < -100:
            power = -100

        if self._invert:
            power = -power

        if power > 0:
            self._in1.value(1)
            self._in2.value(0)
        elif power < 0:
            self._in1.value(0)
            self._in2.value(1)
        else:
            self._in1.value(0)
            self._in2.value(0)

        duty = int(abs(power) * _DUTY_MAX / 100)
        self._pwm.duty(duty)

    def brake(self):
        """Short both terminals — active brake."""
        self._in1.value(1)
        self._in2.value(1)
        self._pwm.duty(_DUTY_MAX)

    def coast(self):
        """Cut drive entirely — motor spins freely."""
        self._in1.value(0)
        self._in2.value(0)
        self._pwm.duty(0)
