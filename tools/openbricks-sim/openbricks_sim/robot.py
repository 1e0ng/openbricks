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
                                     SimIMU, SimColorSensor)
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
