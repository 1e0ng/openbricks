# SPDX-License-Identifier: MIT
"""Tests for world loading + chassis injection."""

import os
import unittest
from pathlib import Path

import mujoco

from openbricks_sim.chassis import ChassisSpec
from openbricks_sim.world import WorldLoadError, load_world


# Shipped worlds live one level up from the openbricks-sim package.
_PKG_DIR  = Path(__file__).resolve().parent.parent
_WORLDS   = _PKG_DIR / "worlds"

_BUILTIN_WORLDS = [
    "wro_2026_elementary_robot_rockstars",
    "wro_2026_junior_heritage_heroes",
    "wro_2026_senior_mosaic_masters",
    "practice_zones",
    "practice_walls",
]


class LoadWorldTests(unittest.TestCase):

    def test_missing_world_raises(self):
        with self.assertRaises(WorldLoadError):
            load_world("/tmp/does-not-exist-world.xml")

    def test_each_shipped_world_loads_with_chassis(self):
        for name in _BUILTIN_WORLDS:
            path = str(_WORLDS / name / "world.xml")
            with self.subTest(world=name):
                m, d, merged = load_world(path,
                                          chassis_spec=ChassisSpec(pos_x=1.0,
                                                                   pos_y=-0.42))
                # Chassis + 2 wheels + caster = 4 extra bodies on top
                # of whatever the world had.
                cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
                self.assertGreaterEqual(cid, 1)
                # It should start at the requested spawn position.
                for _ in range(100):  # small settle so qpos is sane
                    mujoco.mj_step(m, d)
                self.assertAlmostEqual(d.xpos[cid, 0], 1.0, delta=0.02)
                self.assertAlmostEqual(d.xpos[cid, 1], -0.42, delta=0.02)
                # Chassis actuators are exposed.
                self.assertEqual(m.nu, 2)

    def test_chassis_can_drive_in_world(self):
        """Stepping with ctrl ≠ 0 advances the chassis."""
        path = str(_WORLDS / "wro_2026_elementary_robot_rockstars" / "world.xml")
        m, d, _ = load_world(path,
                             chassis_spec=ChassisSpec(pos_x=1.0, pos_y=-0.42))
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        act_l = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "chassis_motor_l")
        act_r = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "chassis_motor_r")
        # Settle briefly.
        for _ in range(200):
            mujoco.mj_step(m, d)
        x0 = d.xpos[cid, 0]
        # Drive forward for 2 s.
        d.ctrl[act_l] = 0.3
        d.ctrl[act_r] = 0.3
        for _ in range(2000):
            mujoco.mj_step(m, d)
        x1 = d.xpos[cid, 0]
        # +ctrl on both wheels spins them in the "forward" direction
        # for this hinge-axis orientation → chassis translates along
        # +X. The exact distance depends on wheel radius and gear; we
        # just assert "moved meaningfully" since numerics will wiggle.
        self.assertGreater(abs(x1 - x0), 0.02,
                           "motor ctrl should move the chassis (x went "
                           "from %.3f to %.3f)" % (x0, x1))


if __name__ == "__main__":
    unittest.main()
