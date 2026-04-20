# SPDX-License-Identifier: MIT
"""
Abstract interfaces for openbricks components.

MicroPython doesn't ship full ``typing.Protocol`` support, so these are plain
base classes. Drivers should subclass the appropriate interface and fill in
every method. The higher-level modules (``robotics``, ``config``) only depend
on these interfaces, never on concrete drivers — that's what makes the system
plug-and-play.

If you add a new category of component (e.g. a distance sensor), add its
interface here.
"""


class Motor:
    """A bidirectional motor.

    Implementations range from an open-loop H-bridge driver (L298N) to a
    closed-loop geared motor with quadrature encoder (JGB37-520).

    Units
    -----
    * ``power`` is -100..100 (percent duty cycle, sign = direction).
    * ``speed`` is degrees per second at the output shaft (closed-loop only).
    * ``angle`` is degrees at the output shaft (closed-loop only).
    """

    def run(self, power):
        """Run at the given power (-100..100). Non-blocking."""
        raise NotImplementedError

    def brake(self):
        """Stop with active braking (both terminals shorted)."""
        raise NotImplementedError

    def coast(self):
        """Stop by cutting drive power (motor free-wheels)."""
        raise NotImplementedError

    # --- Optional closed-loop methods ---
    # Open-loop drivers may raise NotImplementedError or simply not override.

    def angle(self):
        """Return the current shaft angle in degrees."""
        raise NotImplementedError

    def reset_angle(self, angle=0):
        """Set the current angle to ``angle`` degrees."""
        raise NotImplementedError

    def run_speed(self, deg_per_s):
        """Hold a target speed (closed loop)."""
        raise NotImplementedError

    def run_angle(self, deg_per_s, target_angle, wait=True):
        """Rotate by ``target_angle`` degrees at ``deg_per_s``, blocking if ``wait``."""
        raise NotImplementedError


class Servo:
    """A position-controlled servo (angle-addressable)."""

    def move_to(self, angle_deg, speed=None, wait=True):
        """Move to absolute angle in degrees."""
        raise NotImplementedError

    def angle(self):
        """Read back the current angle."""
        raise NotImplementedError


class IMU:
    """A 3-axis inertial measurement unit.

    The expected unit convention is:
        * heading/yaw/pitch/roll in degrees
        * angular_velocity in degrees / second
        * acceleration in m / s^2
    """

    def heading(self):
        """Return heading (yaw) in degrees, wrapped to [-180, 180)."""
        raise NotImplementedError

    def angular_velocity(self):
        """Return (wx, wy, wz) in deg/s."""
        raise NotImplementedError

    def acceleration(self):
        """Return (ax, ay, az) in m/s^2."""
        raise NotImplementedError


class ColorSensor:
    """An RGB-ish color sensor."""

    def rgb(self):
        """Return ``(r, g, b)`` each in 0..255."""
        raise NotImplementedError

    def ambient(self):
        """Return ambient / clear-channel intensity in 0..100."""
        raise NotImplementedError

# Hub-layer interfaces (StatusLED, Button, Display, Hub) live in
# ``openbricks.hub`` alongside their concrete implementations so that
# tests which don't touch the hub don't pay the class-loading cost on
# MicroPython's tight unix heap.
