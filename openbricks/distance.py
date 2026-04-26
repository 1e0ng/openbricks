# SPDX-License-Identifier: MIT
"""Distance-sensor interface, kept separate from ``interfaces.py``.

Why split: ``openbricks/__init__.py`` eagerly imports the four core
interfaces (Motor / Servo / IMU / ColorSensor) so that every
``import openbricks`` pays them. The observer's variance test runs
close to the MicroPython heap ceiling, so adding even a small class
to ``interfaces.py`` tips it over. Distance-sensor users explicitly
import this module."""


class DistanceSensor:
    """A forward-facing range sensor (HC-SR04 / VL53L0X)."""

    def distance_mm(self):
        """Distance ahead in millimetres; -1 if no echo / out of range."""
        raise NotImplementedError
