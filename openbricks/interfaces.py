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

from openbricks import parameters
from openbricks.parameters import Stop


class Motor:
    """A bidirectional motor.

    Implementations range from an open-loop H-bridge driver (L298N) to a
    closed-loop geared motor with quadrature encoder (JGB37-520).

    The method names and semantics follow the Pybricks Prime
    ``Motor`` API (1.21.0): ``run(speed)`` is degrees per second,
    closed loop; the raw-duty command is ``dc(duty)``. Additional
    openbricks methods (``coast``, ``run_speed``) remain as aliases.

    Units
    -----
    * ``duty`` is -100..100 (percent duty cycle, sign = direction).
    * ``speed`` is degrees per second at the output shaft (closed-loop only).
    * ``angle`` is degrees at the output shaft (closed-loop only).
    """

    def run(self, speed):
        """Run at ``speed`` degrees per second, closed loop —
        Pybricks ``Motor.run()``. Non-blocking. Concrete: delegates
        to ``run_speed()`` (the openbricks alias), so open-loop
        drivers surface its ``NotImplementedError``.

        BREAKING (1.21.0): before Pybricks parity this method took
        percent power. That command is now ``dc(duty)`` — a script
        still calling ``run(30)`` for power gets 30 deg/s instead
        (slow, not dangerous) or ``NotImplementedError`` on
        open-loop drivers.
        """
        self.run_speed(speed)

    def dc(self, duty):
        """Run at a fixed raw duty cycle (-100..100), open loop —
        Pybricks ``Motor.dc()``. Non-blocking. This is the pre-1.21.0
        ``run()``."""
        raise NotImplementedError

    def stop(self):
        """Stop and let the motor spin freely; it gradually stops
        from friction. Pybricks ``Motor.stop()`` semantics — the
        default of the three stop flavours (stop/brake/hold), and a
        concrete method: it delegates to ``coast()``, so every driver
        gets it for free.
        """
        self.coast()

    def brake(self):
        """Stop with active braking (both terminals shorted)."""
        raise NotImplementedError

    def coast(self):
        """Stop by cutting drive power (motor free-wheels)."""
        raise NotImplementedError

    def hold(self):
        """Stop and actively hold the current shaft angle via closed-loop
        control. Only motors with position-mode hardware (e.g. ST-3215) or
        a software position loop implement this; open-loop drivers raise
        ``NotImplementedError`` — pick ``brake`` or ``coast`` instead."""
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
        """Rotate by ``target_angle`` degrees at ``deg_per_s``. Blocks
        if ``wait``; otherwise returns immediately and the caller polls
        ``done()`` to advance the move and detect completion."""
        raise NotImplementedError

    def done(self):
        """Return ``True`` if no non-blocking move is in flight or
        the active ``run_angle(wait=False)`` move has reached its
        target. Drivers that don't support non-blocking moves always
        return ``True`` (a wait=True call is finished before
        returning to the caller, by definition)."""
        return True

    def speed(self):
        """Measured shaft speed in degrees per second — Pybricks
        ``Motor.speed()``. Closed-loop drivers implement it (encoder
        observer / servo present-speed register)."""
        raise NotImplementedError

    def load(self):
        """Measured torque at the shaft in mNm — Pybricks
        ``Motor.load()``. Drivers with load feedback (serial servos)
        implement it; the value is derived from the servo's load
        register and its datasheet stall torque, so treat it as an
        estimate."""
        raise NotImplementedError

    def stalled(self):
        """``True`` when the motor is pushing as hard as it can but
        cannot reach its commanded speed — Pybricks
        ``Motor.stalled()``. Drivers with load feedback implement
        it."""
        raise NotImplementedError

    # --- Pybricks composite maneuvers ---
    # Concrete: built from the primitives above, so every closed-loop
    # driver gets them. ``then`` is a :class:`openbricks.parameters.Stop`
    # member (Pybricks Stop.HOLD is the default).

    def _apply_then(self, then):
        parameters.check(Stop, then, "then")
        if then == Stop.HOLD:
            self.hold()
        elif then == Stop.BRAKE:
            self.brake()
        elif then == Stop.COAST:
            self.coast()
        # Stop.NONE: leave the motor running.

    def run_time(self, speed, time_ms, then=Stop.HOLD, wait=True):
        """Run at ``speed`` deg/s for ``time_ms`` ms, then stop with
        the ``then`` flavour — Pybricks ``Motor.run_time()``.
        ``wait=False`` is not supported (no background timer is
        allocated for it); pass ``wait=True`` or sequence it
        yourself."""
        if not wait:
            raise NotImplementedError(
                "run_time(wait=False) is not supported")
        import time
        self.run_speed(speed)
        time.sleep_ms(int(time_ms))
        self._apply_then(then)

    def run_target(self, speed, target_angle, then=Stop.HOLD, wait=True):
        """Run to the ABSOLUTE ``target_angle`` (degrees, in the
        ``reset_angle`` frame) at up to ``speed`` deg/s — Pybricks
        ``Motor.run_target()``. Built on the relative ``run_angle``:
        the delta is measured from ``angle()`` at call time."""
        here = self.angle()
        if here is None:
            raise OSError("cannot read angle for run_target")
        self.run_angle(speed, target_angle - here, wait=wait)
        if wait:
            self._apply_then(then)

    def run_until_stalled(self, speed, then=Stop.COAST, duty_limit=None):
        """Run at ``speed`` deg/s until ``stalled()``, apply the
        ``then`` flavour, and return the angle where it stalled —
        Pybricks ``Motor.run_until_stalled()`` (its default ``then``
        == Stop.COAST).

        ``duty_limit`` (percent, 0 < limit <= 100) caps the motor's
        torque for the duration of the run — the Pybricks gripper-
        homing pattern: drive gently into the end stop without
        crushing it. The cap is applied before the motion starts and
        restored afterwards, stall or not. Drivers opt in via
        ``_duty_limit_push`` / ``_duty_limit_pop`` (the ST3215/
        ST3032 serial servos implement it as a temporary torque-
        limit register write; their stall detection scales to the
        cap)."""
        import time
        restore = None
        if duty_limit is not None:
            restore = self._duty_limit_push(duty_limit)
        try:
            self.run_speed(speed)
            while not self.stalled():
                time.sleep_ms(20)
            self._apply_then(then)
            return self.angle()
        finally:
            if duty_limit is not None:
                self._duty_limit_pop(restore)

    def _duty_limit_push(self, duty_limit):
        """Apply a temporary torque cap of ``duty_limit`` percent;
        return the token ``_duty_limit_pop`` needs to undo it.
        Drivers with a torque-limiting mechanism override both."""
        raise NotImplementedError(
            "duty_limit is not supported on this motor type (the "
            "ST3215/ST3032 serial servos support it)")

    def _duty_limit_pop(self, restore):
        raise NotImplementedError(
            "duty_limit is not supported on this motor type (the "
            "ST3215/ST3032 serial servos support it)")


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


# NOTE: ``DistanceSensor`` lives in ``openbricks.distance`` rather
# than here, even though it's an interface. ``openbricks/__init__.py``
# eagerly loads this module to re-export Motor/Servo/IMU/ColorSensor,
# so every byte of bytecode here is paid by *every* import of
# anything in the openbricks package — including the observer test
# which runs against a tight MicroPython heap budget. Distance-sensor
# users explicitly ``from openbricks.distance import DistanceSensor``
# (or just import their concrete driver, which does).

# Hub-layer interfaces (StatusLED, Button, Display, Hub) live in
# ``openbricks.hub`` alongside their concrete implementations so that
# tests which don't touch the hub don't pay the class-loading cost on
# MicroPython's tight unix heap.
