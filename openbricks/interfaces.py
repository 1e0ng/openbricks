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


class StatusLED:
    """A user-visible status LED on the hub.

    Plain single-colour LEDs implement ``on`` / ``off``. Addressable RGB
    LEDs (WS2812 and friends) additionally implement ``rgb``.
    """

    def on(self):
        raise NotImplementedError

    def off(self):
        raise NotImplementedError

    def rgb(self, r, g, b):
        """Set colour (each channel 0..255). Raises on non-addressable LEDs."""
        raise NotImplementedError


class Button:
    """A momentary pushbutton on the hub."""

    def pressed(self):
        """Return ``True`` while the button is held down."""
        raise NotImplementedError


class Display:
    """A pixel-addressable framebuffer display (e.g. an SSD1306 OLED).

    Implementations provide a FrameBuffer-style surface — at minimum
    ``text``, ``pixel``, ``fill``, and ``show`` — plus the ``clear``
    convenience defined here.
    """

    width = 0
    height = 0

    def text(self, s, x, y, c=1):
        raise NotImplementedError

    def pixel(self, x, y, c):
        raise NotImplementedError

    def fill(self, c):
        raise NotImplementedError

    def show(self):
        """Flush the RAM buffer to the physical display."""
        raise NotImplementedError

    def clear(self):
        """Erase the framebuffer and push a blank frame to the display."""
        self.fill(0)
        self.show()


class Hub:
    """Board-level peripherals baked into a specific MCU devkit.

    Every hub exposes a status LED and a user button. Concrete boards
    attach them (and any optional peripherals like a display) as
    attributes in ``__init__``.
    """

    led = None      # StatusLED
    button = None   # Button
    display = None  # optional — Display or None (reserved for M4 SSD1306)
