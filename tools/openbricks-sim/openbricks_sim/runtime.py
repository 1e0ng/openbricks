# SPDX-License-Identifier: MIT
"""
Sim runtime — binds the shared native cores to a MuJoCo model.

Phase B left us with native ``Servo`` and ``DriveBase`` types whose
math is byte-identical to the firmware. This module is the *I/O*
layer that turns that into a working sim:

  * Reading the wheel angle from a MuJoCo ``jointpos`` sensor and
    converting to encoder counts the native ``Servo`` consumes.
  * Mapping the native servo's [-100, 100] power output onto the
    MuJoCo actuator's torque ``ctrl`` range.
  * Driving everything from a single ``SimRuntime.step()`` call so the
    user's main loop is just ``while not done: rt.step()``.

Design notes
------------

The firmware has a ``MotorProcess`` C-callback registry that the
``machine.Timer`` ISR fires at 1 kHz. The sim doesn't need that
infrastructure: there's no ISR, the "tick" is just the next
``mj_step``. ``SimRuntime`` keeps its own list of registered ticks
(every adapter registers itself at construction) and fires them in
order each step. The runtime's ``now_ms`` clock advances by
``timestep`` every step, mirroring the firmware's
``virtual_now_ms``.

User-facing classes here implement enough of the firmware driver
shape (``run_speed``, ``brake``, ``coast``, ``angle``,
``run_target``, plus DriveBase ``straight``/``turn``/``stop``/
``is_done``) for the existing tests + the planned ``openbricks-sim
run`` command (Phase C2) to drive them. Other interfaces (IMU,
ColorSensor) land in later phases when their MuJoCo sensor binding is
wired up.
"""

from __future__ import annotations

import math
from typing import Callable, List, Optional

import mujoco

from openbricks_sim import _native


# Power output / torque mapping — see SimMotor.tick() for the math.
# Held here as a module constant so tests can introspect it.
_POWER_FULL_SCALE = 100.0


class SimRuntime:
    """Owns the model + data + a clock + the per-tick callback list.

    Construct one per simulation. Adapters (``SimMotor``,
    ``SimDriveBase``, future IMU/colour sensors) take a
    ``SimRuntime`` reference and register themselves by calling
    ``add_tick(fn)`` — the runtime fires every registered fn in
    registration order on each ``step()`` *before* advancing the
    physics. That ordering matters: drivebase.tick() writes
    ``target_dps`` on each servo, then the servo ticks read it and
    write the actuator ``ctrl``, then ``mj_step`` integrates one
    physics step.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model      = model
        self.data       = data
        self.now_ms     = 0
        # Tick period in ms. Pulled from MuJoCo's timestep so the
        # virtual clock matches the physics clock exactly.
        self.timestep_ms = max(1, int(round(model.opt.timestep * 1000.0)))
        self._ticks: List[Callable[[int], None]] = []

    def add_tick(self, fn: Callable[[int], None]) -> None:
        """Register a per-step callback. Called with the *current*
        ``now_ms`` (not the next one) — same convention as the
        firmware's ``motor_process``."""
        if fn not in self._ticks:
            self._ticks.append(fn)

    def remove_tick(self, fn: Callable[[int], None]) -> None:
        """Unregister a previously-registered callback. No-op if the
        callback was never registered."""
        try:
            self._ticks.remove(fn)
        except ValueError:
            pass

    def step(self) -> None:
        """Advance one physics step + clock + fire ticks.

        Order:
          1. Advance ``now_ms`` first so subscribers see a consistent
             "now" that already accounts for this step's duration.
          2. Fire each registered tick. Tick callbacks read sensors,
             update controllers, and write actuator ``ctrl`` values.
          3. Step physics.
        """
        self.now_ms += self.timestep_ms
        for fn in self._ticks:
            fn(self.now_ms)
        mujoco.mj_step(self.model, self.data)


class SimMotor:
    """A single drive motor, MuJoCo-side.

    Wraps a native ``Servo`` state machine + a (sensor, actuator)
    pair on the MuJoCo model. Per tick:

      1. Read the joint-position sensor (radians).
      2. Convert to encoder counts via ``counts_per_rev``.
      3. ``servo.tick(count, now_ms)`` returns a desired power in
         [-100, 100].
      4. Map that onto the actuator's symmetric ``ctrlrange`` and
         write the ``ctrl`` value.

    User-facing methods (``run_speed`` / ``run_target`` / ``brake`` /
    ``coast`` / ``angle`` / ``reset_angle``) match the shape of the
    firmware's ``Servo`` so user code that targets the firmware
    interface works unchanged.
    """

    def __init__(self,
                 runtime: SimRuntime,
                 sensor_name: str,
                 actuator_name: str,
                 counts_per_rev: int = 1320,
                 kp: float = 0.3,
                 invert: bool = False) -> None:
        self.runtime        = runtime
        self.counts_per_rev = counts_per_rev
        self.invert         = invert

        self._sensor_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_SENSOR,   sensor_name)
        if self._sensor_id < 0:
            raise ValueError("no sensor named " + repr(sensor_name) +
                             " in model")
        self._sensor_addr = int(runtime.model.sensor_adr[self._sensor_id])

        self._actuator_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if self._actuator_id < 0:
            raise ValueError("no actuator named " + repr(actuator_name) +
                             " in model")
        # Use the larger half of the ctrlrange as the [-100, 100]
        # power magnitude. Most chassis actuators are symmetric so
        # this is just abs(min) == abs(max).
        lo, hi = runtime.model.actuator_ctrlrange[self._actuator_id]
        self._ctrl_scale = max(abs(float(lo)), abs(float(hi))) / _POWER_FULL_SCALE

        self.servo = _native.Servo(counts_per_rev=counts_per_rev,
                                    kp=kp, invert=invert)
        # Re-baseline the observer to whatever the joint reads right
        # now, so the first tick doesn't see a phantom delta.
        self.servo.baseline(self._read_count(), runtime.now_ms)

        # Don't auto-attach — same convention as the firmware
        # ``servo.c``. The first ``run_speed`` / ``run_target`` /
        # SimDriveBase attach call wakes the controller. Otherwise
        # zero-target controllers fight passive wheel settling and
        # destabilise the chassis.
        self._attached = False

    # -------- attach / detach lifecycle --------

    def _attach(self) -> None:
        if not self._attached:
            self.runtime.add_tick(self._tick)
            self._attached = True

    def _detach(self) -> None:
        if self._attached:
            self.runtime.remove_tick(self._tick)
            self._attached = False

    # -------- per-tick I/O --------

    def _read_count(self) -> int:
        rad = float(self.runtime.data.sensordata[self._sensor_addr])
        deg = rad * (180.0 / math.pi)
        return int(deg * self.counts_per_rev / 360.0)

    def _tick(self, now_ms: int) -> None:
        count = self._read_count()
        power = self.servo.tick(count, now_ms)
        # invert flag is a wiring concern; the core returns the raw
        # control-law power, so the binding shell is responsible for
        # the sign flip. (firmware does the same in ``servo.c``.)
        if self.invert:
            power = -power
        self.runtime.data.ctrl[self._actuator_id] = power * self._ctrl_scale

    # -------- user-facing API (Motor interface) --------

    def run_speed(self, dps: float) -> None:
        """Hold a target speed (deg/s) closed-loop. Cancels any
        active trajectory."""
        self.servo.set_speed(float(dps))
        self._attach()

    def run_target(self,
                   delta_deg: float,
                   cruise_dps: float,
                   accel: float = 720.0) -> None:
        """Trapezoidal move ``delta_deg`` from the current angle at
        ``cruise_dps`` cruise speed and ``accel`` deg/s² shaping."""
        self.servo.run_target(self._read_count(), self.runtime.now_ms,
                               float(delta_deg), float(cruise_dps),
                               float(accel))
        self._attach()

    def brake(self) -> None:
        """Cut closed-loop control and apply maximum opposing torque
        — the sim's analogue to the L298N short-the-terminals brake.
        We approximate by detaching from the tick loop and zeroing
        ctrl; MuJoCo's ``damping`` + ``frictionloss`` on the joint
        does the rest."""
        self._detach()
        self.runtime.data.ctrl[self._actuator_id] = 0.0

    def coast(self) -> None:
        """Same as ``brake`` for the sim — both end up writing 0
        torque. Real hardware brake actively shorts the motor; here
        the difference is invisible because there's no back-EMF
        model. Kept as a separate method for API parity."""
        self.brake()

    def angle(self) -> float:
        """Current shaft angle in degrees, derived from the joint
        sensor (independent of the observer)."""
        rad = float(self.runtime.data.sensordata[self._sensor_addr])
        return rad * (180.0 / math.pi)

    def reset_angle(self, angle: float = 0.0) -> None:
        """Re-baseline the observer so its position estimate matches
        ``angle``. Note: the underlying MuJoCo joint position isn't
        teleported (you'd corrupt physics); the observer just learns
        a new offset, which is the firmware's behaviour too."""
        # Convert the requested angle back into a synthetic count.
        synthetic_count = int(angle * self.counts_per_rev / 360.0)
        self.servo.baseline(synthetic_count, self.runtime.now_ms)

    # -------- introspection helpers (mostly for tests) --------

    def target_dps(self) -> float:
        return float(self.servo.target_dps())

    def observed_dps(self) -> float:
        return float(self.servo.observed_dps())

    def is_done(self) -> bool:
        return bool(self.servo.is_done())


class SimIMU:
    """6-DOF IMU adapter, MuJoCo-side.

    Wraps the chassis's ``accelerometer`` + ``gyro`` MuJoCo sensors
    plus the body's ``xmat`` rotation matrix to produce the same
    interface a real BNO055 driver exposes:

      * ``heading()`` — yaw in degrees, wrapped to [-180, 180).
        Pulled from the body's rotation matrix so it has zero
        integration drift (the sim's "ground truth" yaw, not a
        gyro-integrated estimate).
      * ``angular_velocity()`` — (wx, wy, wz) in deg/s, body frame.
      * ``acceleration()`` — (ax, ay, az) in m/s², body frame.

    The drivebase ``use_gyro`` path on the firmware is slip-immune —
    it doesn't matter how much the wheels spin, the heading comes
    from the IMU. The sim's SimIMU gives the same property under
    asymmetric friction / wheel-floor slip in MuJoCo.
    """

    def __init__(self,
                 runtime: SimRuntime,
                 body_name: str = "chassis",
                 accel_sensor_name: str = "chassis_accel",
                 gyro_sensor_name:  str = "chassis_gyro") -> None:
        self.runtime = runtime
        self._body_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if self._body_id < 0:
            raise ValueError("no body named " + repr(body_name) +
                             " in model")

        self._accel_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_SENSOR, accel_sensor_name)
        if self._accel_id < 0:
            raise ValueError("no sensor named " + repr(accel_sensor_name) +
                             " in model")
        self._accel_addr = int(runtime.model.sensor_adr[self._accel_id])

        self._gyro_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_SENSOR, gyro_sensor_name)
        if self._gyro_id < 0:
            raise ValueError("no sensor named " + repr(gyro_sensor_name) +
                             " in model")
        self._gyro_addr = int(runtime.model.sensor_adr[self._gyro_id])

    def heading(self) -> float:
        """Yaw angle (degrees) in [-180, 180), CCW positive."""
        R = self.runtime.data.xmat[self._body_id].reshape(3, 3)
        yaw_deg = math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0])))
        # Wrap to [-180, 180) to match the BNO055 driver shape.
        return ((yaw_deg + 180.0) % 360.0) - 180.0

    def angular_velocity(self):
        """(wx, wy, wz) in deg/s, body frame."""
        sd = self.runtime.data.sensordata
        a = float(sd[self._gyro_addr])
        b = float(sd[self._gyro_addr + 1])
        c = float(sd[self._gyro_addr + 2])
        return (math.degrees(a), math.degrees(b), math.degrees(c))

    def acceleration(self):
        """(ax, ay, az) in m/s², body frame."""
        sd = self.runtime.data.sensordata
        return (float(sd[self._accel_addr]),
                float(sd[self._accel_addr + 1]),
                float(sd[self._accel_addr + 2]))


class SimColorSensor:
    """Down-facing colour sensor, MuJoCo-side.

    Casts a ray from the camera position straight down (world -Z) and
    returns the colour of whatever geom it hits. The look direction
    is hard-coded to world -Z rather than the camera's actual local
    -Z so a slightly-pitched chassis (e.g. ~6.7° wheel-settle) still
    samples the floor directly under the sensor — the cosine error
    against camera-frame Z is < 1% at that tilt, which is well below
    the colour-sensor's quantisation in any case. Real TCS34725
    sensors integrate over a small FOV; this is a single-point
    approximation.

    Returned colour comes from the geom's material ``rgba`` (or the
    geom's own ``rgba`` if it has no material). Textured materials
    fall back to the texture-modulating rgba — the WRO mat texture
    image is NOT sampled (that needs offscreen rendering, not yet
    available on macOS in this codebase).

    Methods match :class:`openbricks.interfaces.ColorSensor`:

      * ``rgb()`` returns ``(r, g, b)`` ints in 0..255.
      * ``ambient()`` returns a 0..100 luminance score (BT.601).
    """

    def __init__(self,
                 runtime: SimRuntime,
                 camera_name: str = "chassis_cam_down",
                 chassis_body_name: str = "chassis") -> None:
        self.runtime = runtime
        self._cam_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if self._cam_id < 0:
            raise ValueError("no camera named " + repr(camera_name) +
                             " in model")
        # Exclude the chassis itself from raycasts so we don't
        # self-hit the chassis body geom.
        self._chassis_body_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_BODY, chassis_body_name)
        if self._chassis_body_id < 0:
            raise ValueError("no body named " + repr(chassis_body_name) +
                             " in model")

    # ------------- raycast -------------

    def _hit_rgba(self):
        """Return the rgba (4 floats, 0..1) of the geom under the
        camera, or None if the ray missed."""
        # Lazy-import numpy at call-site; mujoco depends on it so it's
        # always available, but keeping the runtime module's top-level
        # imports minimal helps cold-start time.
        import numpy as np

        # Force a forward pass so cam_xpos reflects the latest qpos.
        # (mj_step does this; explicit forward is for cases where the
        # caller queries before stepping.)
        mujoco.mj_forward(self.runtime.model, self.runtime.data)

        pnt = np.array(self.runtime.data.cam_xpos[self._cam_id],
                        dtype=np.float64)
        # World -Z. See class docstring for why we use world axis
        # rather than the camera's local -Z.
        vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        geom_id = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(self.runtime.model, self.runtime.data,
                              pnt, vec,
                              None,                    # geomgroup
                              1,                       # flg_static (include static)
                              self._chassis_body_id,   # bodyexclude
                              geom_id)
        if geom_id[0] < 0 or dist < 0:
            return None
        # Look up the material rgba; fall back to the geom's own
        # rgba when the geom has no material.
        m = self.runtime.model
        mat_id = int(m.geom_matid[geom_id[0]])
        if mat_id >= 0:
            return tuple(float(c) for c in m.mat_rgba[mat_id])
        return tuple(float(c) for c in m.geom_rgba[geom_id[0]])

    def rgb(self):
        rgba = self._hit_rgba()
        if rgba is None:
            return (0, 0, 0)
        return (
            min(255, max(0, int(rgba[0] * 255.0))),
            min(255, max(0, int(rgba[1] * 255.0))),
            min(255, max(0, int(rgba[2] * 255.0))),
        )

    def ambient(self):
        r, g, b = self.rgb()
        # BT.601 luma, scaled 0..100.
        lum = 0.299 * r + 0.587 * g + 0.114 * b
        return int(min(100, max(0, lum * 100.0 / 255.0)))


class SimDistanceSensor:
    """Forward-facing range sensor, MuJoCo-side.

    Casts a ray from the chassis ``chassis_dist`` site along body +X
    and returns the hit distance in millimetres. Excludes the
    chassis body so a self-hit on the front geom doesn't dominate
    every reading.

    Mirrors :class:`openbricks.distance.DistanceSensor`:
    ``distance_mm()`` returns mm; ``-1`` if no hit within
    ``max_range_mm``."""

    def __init__(self,
                 runtime: SimRuntime,
                 site_name: str = "chassis_dist",
                 chassis_body_name: str = "chassis",
                 max_range_mm: float = 4000.0) -> None:
        self.runtime = runtime
        self._site_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self._site_id < 0:
            raise ValueError("no site named " + repr(site_name) +
                             " in model")
        self._chassis_body_id = mujoco.mj_name2id(
            runtime.model, mujoco.mjtObj.mjOBJ_BODY, chassis_body_name)
        if self._chassis_body_id < 0:
            raise ValueError("no body named " + repr(chassis_body_name) +
                             " in model")
        self._max_range_m = float(max_range_mm) / 1000.0

    def distance_mm(self):
        import numpy as np
        mujoco.mj_forward(self.runtime.model, self.runtime.data)
        # Site origin in world frame.
        pnt = np.array(self.runtime.data.site_xpos[self._site_id],
                        dtype=np.float64)
        # Site rotation matrix's column 0 = body +X in world frame.
        R = np.array(self.runtime.data.site_xmat[self._site_id],
                      dtype=np.float64).reshape(3, 3)
        vec = R[:, 0]
        geom_id = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(self.runtime.model, self.runtime.data,
                              pnt, vec,
                              None,                    # geomgroup
                              1,                       # flg_static
                              self._chassis_body_id,   # bodyexclude
                              geom_id)
        if geom_id[0] < 0 or dist < 0 or dist > self._max_range_m:
            return -1
        return int(dist * 1000.0)


class SimDriveBase:
    """2-DOF coupled drivebase, MuJoCo-side.

    Wraps a native ``DriveBase`` over two ``SimMotor`` adapters. The
    drivebase tick runs *before* the per-motor ticks (writing
    ``target_dps`` on each native ``Servo``), so by the time each
    SimMotor reads the latest target_dps it's already the coupled
    setpoint.

    The IMU heading override (``set_heading_override``) is exposed so
    a future ``SimIMU`` can push body-yaw deltas in. Until that
    lands, encoder-differential heading is used (default behaviour).
    """

    def __init__(self,
                 runtime: SimRuntime,
                 left: SimMotor,
                 right: SimMotor,
                 wheel_diameter_mm: float,
                 axle_track_mm: float,
                 kp_sum: Optional[float] = None,
                 kp_diff: Optional[float] = None) -> None:
        self.runtime = runtime
        self.left    = left
        self.right   = right

        kwargs = {}
        if kp_sum  is not None: kwargs["kp_sum"]  = float(kp_sum)
        if kp_diff is not None: kwargs["kp_diff"] = float(kp_diff)
        self.db = _native.DriveBase(left.servo, right.servo,
                                     wheel_diameter_mm=float(wheel_diameter_mm),
                                     axle_track_mm=float(axle_track_mm),
                                     **kwargs)
        self._attached = False

    # -------- lifecycle --------

    def _attach(self) -> None:
        if self._attached:
            return
        # Order matters: drivebase tick must run BEFORE the per-motor
        # ticks so each motor sees the freshly-written target_dps.
        # The runtime fires ticks in registration order; we
        # re-register the motors *after* us to enforce the order.
        # (The motors registered themselves in their own __init__,
        # so we have to remove them here and append them.)
        self.runtime.remove_tick(self.left._tick)
        self.runtime.remove_tick(self.right._tick)
        self.runtime.add_tick(self._tick)
        self.runtime.add_tick(self.left._tick)
        self.runtime.add_tick(self.right._tick)
        self.left._attached  = True
        self.right._attached = True
        self._attached = True

    def _detach(self) -> None:
        if not self._attached:
            return
        self.runtime.remove_tick(self._tick)
        self._attached = False

    def _tick(self, now_ms: int) -> None:
        self.db.tick(now_ms)

    # -------- user-facing API --------

    def straight(self, distance_mm: float, speed_mm_s: float) -> None:
        self.db.straight(self.runtime.now_ms,
                          float(distance_mm), float(speed_mm_s))
        self._attach()

    def turn(self, angle_deg: float, rate_dps: float) -> None:
        self.db.turn(self.runtime.now_ms,
                      float(angle_deg), float(rate_dps))
        self._attach()

    def stop(self) -> None:
        self.db.stop()
        self._detach()

    def is_done(self) -> bool:
        return bool(self.db.is_done())

    def set_use_gyro(self, enable: bool) -> None:
        self.db.set_use_gyro(bool(enable))

    def set_heading_override(self, body_delta_deg: float) -> None:
        self.db.set_heading_override(float(body_delta_deg))
