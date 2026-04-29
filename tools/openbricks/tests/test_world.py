# SPDX-License-Identifier: MIT
"""Tests for world loading + chassis injection."""

import os
import tempfile
import unittest
from pathlib import Path

import mujoco

from openbricks_sim.chassis import ChassisSpec
from openbricks_sim.world import WorldLoadError, load_world, _expand_lego_props


# Shipped worlds live INSIDE the ``openbricks_sim`` package as of
# 0.10.6 (so they land in the wheel via package-data — see
# pyproject.toml). Locate them via the package, not the test file:
# importlib.resources keeps the test correct under both editable
# installs and built wheels.
import openbricks_sim
_WORLDS = Path(openbricks_sim.__file__).resolve().parent / "worlds"

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


class LegoPropExpansionTests(unittest.TestCase):
    """The ``<lego_prop name="..." ldr="..." pos="..." mass="..."/>``
    placeholder is the syntax world.xml uses to embed an LDraw-built
    prop without inlining hundreds of generated MJCF lines.
    ``_expand_lego_props`` reads the .ldr at load time and substitutes
    a full ``<body>`` block. These tests pin that pipeline."""

    def test_placeholder_replaced_with_body_geoms(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "props").mkdir()
            (tmp / "props" / "tiny.ldr").write_text(
                "0 Tiny test prop\n"
                "1 1 0 0 0 1 0 0 0 1 0 0 0 1 3003.dat\n")
            world = ('<lego_prop name="tinyp" ldr="props/tiny.ldr" '
                     'pos="0.0 0.0 0.0" mass="0.005"/>')
            expanded = _expand_lego_props(world, tmp)
            self.assertIn('<body name="tinyp"', expanded)
            self.assertIn('<freejoint/>', expanded)
            self.assertIn('material="lego_blue"', expanded)
            # 1 brick body + 4 stud cylinders (2x2 brick)
            self.assertEqual(expanded.count('<geom type="box"'), 1)
            self.assertEqual(expanded.count('<geom type="cylinder"'), 4)

    def test_missing_ldr_raises_loadly(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            world = ('<lego_prop name="x" ldr="props/no_such.ldr" '
                     'pos="0 0 0" mass="0.01"/>')
            with self.assertRaises(WorldLoadError) as cm:
                _expand_lego_props(world, tmp)
            self.assertIn("missing .ldr", str(cm.exception))

    def test_world_with_no_lego_prop_passes_through(self):
        # Non-prop XML must round-trip unchanged so we don't break
        # the existing world.xml structure.
        with tempfile.TemporaryDirectory() as tmp:
            world = '<worldbody><geom type="plane" size="1 1 0.1"/></worldbody>'
            self.assertEqual(_expand_lego_props(world, Path(tmp)), world)

    def test_elementary_world_clef_has_many_geoms(self):
        # Integration test: the shipped Elementary world's clef is a
        # ``<lego_prop>`` placeholder that, when expanded, becomes a
        # body with 31 brick boxes + 230+ stud cylinders. Pre-F2.1
        # the clef was a single-box approximation — pin the new
        # multi-geom shape so a regression to single-box is caught.
        path = (_WORLDS / "wro_2026_elementary_robot_rockstars" / "world.xml")
        m, _, _ = load_world(str(path), chassis_spec=ChassisSpec())
        clef_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "clef")
        self.assertGreaterEqual(clef_id, 0, "clef body missing from model")
        clef_geom_count = sum(
            1 for i in range(m.ngeom) if int(m.geom_bodyid[i]) == clef_id)
        self.assertGreater(
            clef_geom_count, 100,
            "clef should have >100 geoms (~31 bricks + ~230 studs); "
            "got %d — has the lego_prop expansion regressed to a "
            "single-box approximation?" % clef_geom_count)


if __name__ == "__main__":
    unittest.main()
