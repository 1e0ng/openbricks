# SPDX-License-Identifier: MIT
"""
High-level user-facing entry point: ``SimRobot``.

Phase C1 shipped the low-level adapter classes (``SimRuntime``,
``SimMotor``, ``SimDriveBase``); they're powerful but verbose for a
user who just wants to drive a chassis around a WRO mat. This module
bundles the standard chassis + a 2-motor + drivebase setup behind one
class:

    from openbricks_sim.robot import SimRobot

    robot = SimRobot(world="wro-2026-elementary")
    robot.drivebase.straight(200.0, 100.0)
    robot.run_until(robot.drivebase.is_done, timeout_s=5.0)

A ``SimRobot`` always uses the default chassis from
``openbricks_sim.chassis``; users who want a non-default geometry
override via ``chassis_spec=ChassisSpec(...)``. The robot wires up:

  * Two ``SimMotor`` adapters bound to ``chassis_motor_l`` / ``_r``.
  * One ``SimDriveBase`` over those two motors with the geometry
    matching the current ``ChassisSpec`` (wheel diameter / axle).
  * (Future: SimIMU on the chassis IMU site, SimColorSensor over
    the down-camera. Land in C3 / C4.)

The runtime + viewer plumbing follows the same loop ``cmd_preview``
already uses: pick interactive vs headless, step in real-time when
interactive, step at full sim speed when headless.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Callable, Optional

import mujoco

from openbricks_sim.chassis import ChassisSpec, standalone_mjcf
from openbricks_sim.runtime import (SimRuntime, SimMotor, SimDriveBase,
                                     SimIMU, SimColorSensor,
                                     SimDistanceSensor)
from openbricks_sim.world import load_world


# Repo-relative aliases for the WRO worlds — same set ``cli.py``
# resolves for ``preview``. Resolved against the package root, so the
# call site doesn't need to know where it lives on disk.
_BUILTIN_WORLDS = {
    "empty":               None,
    "wro-2026-elementary": "worlds/wro_2026_elementary_robot_rockstars/world.xml",
    "wro-2026-junior":     "worlds/wro_2026_junior_heritage_heroes/world.xml",
    "wro-2026-senior":     "worlds/wro_2026_senior_mosaic_masters/world.xml",
}


def _resolve_world(world):
    """Aliases → on-disk path; ``None`` keeps the standalone preview."""
    if world is None or world == "empty":
        return None
    if world in _BUILTIN_WORLDS:
        rel = _BUILTIN_WORLDS[world]
        if rel is None:
            return None
        # Aliases are repo-relative; the openbricks-sim package lives
        # at ``tools/openbricks-sim/openbricks_sim/`` and the worlds
        # are siblings of the package directory.
        pkg_root = Path(__file__).resolve().parent.parent
        candidate = pkg_root / rel
        if candidate.is_file():
            return str(candidate)
        return world
    return world


class SimRobotError(RuntimeError):
    pass


class SimRobot:
    """Bundle of (model, data, runtime, motors, drivebase) for the
    canonical 2-motor differential-drive chassis.

    Constructor args:
      * ``world`` — alias or path. ``None`` / ``"empty"`` uses the
        standalone chassis preview (checker floor only).
      * ``chassis_spec`` — geometry override. Defaults to the
        :class:`ChassisSpec` defaults (60 mm wheels, 150 mm axle).
      * ``counts_per_rev`` / ``kp`` / ``kp_sum`` / ``kp_diff`` —
        forwarded to the underlying native controllers.

    After construction:

      * ``robot.runtime`` is the SimRuntime — call ``step()`` /
        ``run_until()`` / ``run_for()`` to advance the sim.
      * ``robot.left`` / ``robot.right`` are the two SimMotors.
      * ``robot.drivebase`` is the SimDriveBase composed over them.
    """

    def __init__(self,
                 world: Optional[str] = None,
                 chassis_spec: Optional[ChassisSpec] = None,
                 counts_per_rev: int = 1320,
                 kp: float = 0.3,
                 kp_sum: Optional[float] = None,
                 kp_diff: Optional[float] = None):
        spec = chassis_spec if chassis_spec is not None else ChassisSpec()
        path = _resolve_world(world)
        if path is None:
            xml = standalone_mjcf(spec)
            model = mujoco.MjModel.from_xml_string(xml)
            data  = mujoco.MjData(model)
        else:
            model, data, _ = load_world(path, chassis_spec=spec)

        self.model        = model
        self.data         = data
        self.chassis_spec = spec
        self.runtime      = SimRuntime(model, data)

        # Bind the two drive motors. Names match the chassis MJCF
        # generator — see openbricks_sim/chassis.py.
        self.left = SimMotor(
            self.runtime, "chassis_enc_l", "chassis_motor_l",
            counts_per_rev=counts_per_rev, kp=kp)
        self.right = SimMotor(
            self.runtime, "chassis_enc_r", "chassis_motor_r",
            counts_per_rev=counts_per_rev, kp=kp)

        # Drivebase geometry from the chassis spec — wheel diameter
        # and axle length are stored as metres there; the native
        # drivebase wants mm.
        wheel_d_mm = spec.wheel_radius * 2.0 * 1000.0
        axle_mm    = spec.axle_length * 1000.0
        self.drivebase = SimDriveBase(
            self.runtime, self.left, self.right,
            wheel_diameter_mm=wheel_d_mm,
            axle_track_mm=axle_mm,
            kp_sum=kp_sum, kp_diff=kp_diff)

        # IMU bound to the chassis IMU site + accel / gyro sensors.
        # Available even when the user's script doesn't ask for it —
        # the SimIMU just exposes data; nothing happens unless someone
        # calls heading() / angular_velocity() / acceleration().
        self.imu = SimIMU(self.runtime)

        # Down-facing colour sensor — raycast against the floor.
        self.color_sensor = SimColorSensor(self.runtime)

        # Forward-facing distance sensor (HC-SR04 / VL53L0X equivalent).
        # Raycasts from the chassis_dist site along body +X.
        self.distance_sensor = SimDistanceSensor(self.runtime)

    # ------------------------------------------------------------------
    # Time advancement helpers — thin wrappers over runtime.step() with
    # the kinds of conditions user scripts most often want.

    def step(self) -> None:
        """One physics step (1 ms by default)."""
        self.runtime.step()

    def run_for(self, seconds: float) -> None:
        """Step the sim for ``seconds`` of *sim time* (not wall clock)."""
        n = int(seconds * 1000.0 / self.runtime.timestep_ms)
        for _ in range(n):
            self.runtime.step()

    def run_until(self,
                  predicate: Callable[[], bool],
                  timeout_s: float = 30.0,
                  poll_every_ms: int = 1) -> bool:
        """Step until ``predicate()`` returns truthy, or ``timeout_s``
        of *sim time* elapses. Returns True if the predicate fired,
        False on timeout.

        ``poll_every_ms`` defaults to 1 ms (every step). Increase it if
        the predicate is expensive."""
        max_steps = int(timeout_s * 1000.0 / self.runtime.timestep_ms)
        steps_per_poll = max(1, poll_every_ms // self.runtime.timestep_ms)
        for i in range(max_steps):
            self.runtime.step()
            if i % steps_per_poll == 0 and predicate():
                return True
        return False

    # ------------------------------------------------------------------
    # Scenario reset — useful for iterating on a mission script
    # without restarting the sim process. A typical pattern:
    #
    #     robot = SimRobot(world="wro-2026-elementary")
    #     for attempt in range(10):
    #         robot.reset()
    #         run_my_mission(robot)
    #         score = evaluate(robot.chassis_pose())
    #         print(f"attempt {attempt}: score={score}")

    def reset(self) -> None:
        """Reset the sim to a fresh state.

        Stops the drivebase + motors, restores all qpos / qvel to
        the model's keyframe-zero state (back to spawn), zeroes the
        runtime clock, and re-baselines the encoder observers so the
        next tick doesn't see a phantom delta. Sensor adapters
        (IMU / colour / distance) are stateless wrappers around the
        current MuJoCo data, so they need no explicit reset.
        """
        # 1. Cancel any active drivebase move and detach motors so
        #    they stop writing actuator ctrl. ``stop`` puts both
        #    motors into brake mode; we additionally clear the
        #    runtime tick list so a stale closure can't fire after
        #    a reset.
        self.drivebase.stop()
        self.left.brake()
        self.right.brake()

        # 2. Restore physics to the spawn pose. ``mj_resetData`` zeros
        #    qpos/qvel/qacc/sensordata/etc. and reapplies the model's
        #    initial qpos (which is what ``ChassisSpec.pos_x/pos_y``
        #    set at construction). One ``mj_forward`` so the kinematic
        #    derived state (xpos/xmat/cam_xpos) reflects the new qpos
        #    before any sensor calls land.
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        # 3. Reset the virtual clock and re-baseline observers.
        self.runtime.now_ms = 0
        self.left.servo.baseline(self.left._read_count(), 0)
        self.right.servo.baseline(self.right._read_count(), 0)

    def set_pose(self, x_mm: float, y_mm: float,
                 yaw_deg: float = 0.0) -> None:
        """Teleport the chassis to ``(x_mm, y_mm)`` with heading
        ``yaw_deg``. Calls :meth:`reset` first, then writes the
        chassis free-joint qpos.

        Useful for parking the robot at a specific test location
        without rebuilding the model.
        """
        self.reset()
        cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                 "chassis_free")
        if cid < 0:
            raise SimRobotError("model has no joint named 'chassis_free'")
        # qpos for a free-joint is [x, y, z, qw, qx, qy, qz] (7 entries).
        qadr = self.model.jnt_qposadr[cid]
        # Preserve current Z so the chassis sits on its wheels.
        z = float(self.data.qpos[qadr + 2])
        yaw_rad = math.radians(float(yaw_deg))
        qw = math.cos(yaw_rad / 2.0)
        qz = math.sin(yaw_rad / 2.0)
        self.data.qpos[qadr + 0] = float(x_mm) / 1000.0
        self.data.qpos[qadr + 1] = float(y_mm) / 1000.0
        self.data.qpos[qadr + 2] = z
        self.data.qpos[qadr + 3] = qw
        self.data.qpos[qadr + 4] = 0.0
        self.data.qpos[qadr + 5] = 0.0
        self.data.qpos[qadr + 6] = qz
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------
    # Mission-scoring helpers — predicates over the chassis pose for
    # quickly answering "did the robot end up in the right place?"
    # without the user re-deriving Euclidean / box-intersection math.

    def chassis_in_box(self,
                       x_min_mm: float, y_min_mm: float,
                       x_max_mm: float, y_max_mm: float) -> bool:
        """True iff the chassis centre is inside the axis-aligned box
        ``[x_min_mm, x_max_mm] × [y_min_mm, y_max_mm]`` (world frame).
        """
        x, y, _ = self.chassis_pose()
        return x_min_mm <= x <= x_max_mm and y_min_mm <= y <= y_max_mm

    def chassis_in_circle(self,
                          cx_mm: float, cy_mm: float,
                          radius_mm: float) -> bool:
        """True iff the chassis centre is within ``radius_mm`` of
        ``(cx_mm, cy_mm)`` (world frame)."""
        x, y, _ = self.chassis_pose()
        dx = x - cx_mm
        dy = y - cy_mm
        return (dx * dx + dy * dy) <= (radius_mm * radius_mm)

    def distance_to(self, cx_mm: float, cy_mm: float) -> float:
        """Euclidean distance from chassis centre to ``(cx_mm, cy_mm)``,
        in millimetres."""
        x, y, _ = self.chassis_pose()
        dx = x - cx_mm
        dy = y - cy_mm
        return math.sqrt(dx * dx + dy * dy)

    def heading_aligned_with(self,
                             target_yaw_deg: float,
                             tolerance_deg: float = 10.0) -> bool:
        """True iff the chassis heading is within ``tolerance_deg`` of
        ``target_yaw_deg``. Handles wrap-around (e.g. -179° vs +179°
        is a 2° difference)."""
        _, _, yaw = self.chassis_pose()
        delta = (yaw - target_yaw_deg + 540.0) % 360.0 - 180.0
        return abs(delta) <= tolerance_deg

    # ------------------------------------------------------------------
    # Position introspection (mostly for tests + interactive scripts).

    def chassis_pose(self):
        """Return (x_mm, y_mm, yaw_deg) of the chassis body in the
        world frame. Uses MuJoCo's ``xpos`` / ``xmat`` directly."""
        cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        if cid < 0:
            raise SimRobotError("model has no body named 'chassis'")
        x_mm = float(self.data.xpos[cid, 0]) * 1000.0
        y_mm = float(self.data.xpos[cid, 1]) * 1000.0
        # xmat is the 3x3 rotation flattened. yaw = atan2(R[1,0], R[0,0]).
        R = self.data.xmat[cid].reshape(3, 3)
        yaw_rad = math.atan2(float(R[1, 0]), float(R[0, 0]))
        yaw_deg = math.degrees(yaw_rad)
        return (x_mm, y_mm, yaw_deg)

    # ------------------------------------------------------------------
    # Interactive viewer — mirror of cmd_preview, exposed as a method
    # so user scripts can opt into it without importing mujoco.viewer.

    def run_viewer(self, until: Optional[Callable[[], bool]] = None) -> None:
        """Open MuJoCo's bundled viewer and step real-time until the
        user closes the window or ``until()`` (if given) fires.

        Convenience for example scripts; tests use ``run_for`` /
        ``run_until`` instead."""
        try:
            import mujoco.viewer
        except ImportError as e:
            raise SimRobotError(
                "mujoco.viewer not available — install a newer mujoco "
                "wheel or use run_for / run_until instead") from e
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            t0 = time.time()
            while viewer.is_running():
                if until is not None and until():
                    break
                self.runtime.step()
                lag = self.data.time - (time.time() - t0)
                viewer.sync()
                if lag > 0.0:
                    time.sleep(lag)
