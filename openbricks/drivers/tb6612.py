# SPDX-License-Identifier: MIT
"""
TB6612FNG dual MOSFET H-bridge — re-exposes ``L298NMotor`` under the
TB6612 name.

Both chips drive each motor channel with the same three signals:

    IN1  — direction bit 1
    IN2  — direction bit 2
    PWM  — speed (PWM duty cycle)

…so the openbricks driver is identical. TB6612 is generally the better
commodity choice — MOSFETs instead of Darlingtons, ~0.3 V drop instead
of ~1.8 V, 3.3 V-logic compatible directly from an ESP32 GPIO, and 3.2
A peak per channel vs L298N's 2 A.

Wiring difference to be aware of: TB6612 has a chip-level **STBY** pin
that must be held high for either channel to operate. Tie it to 3.3 V
on your breakout, or drive a spare GPIO high at boot — openbricks
doesn't model STBY because most breakout modules already pull it up.
"""

from openbricks.drivers.l298n import L298NMotor as TB6612Motor  # noqa: F401
