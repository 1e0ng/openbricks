# SPDX-License-Identifier: MIT
"""Tests for the LDraw → MJCF converter that builds WRO mission
props from per-brick LDraw layouts.

Pins:

  * LDraw → MJCF coordinate conversion (Y-down → Z-up).
  * Brick body box dimensions match the LDraw spec for each part
    type in the registry.
  * Stud cylinders are emitted at the right positions and tagged
    visual-only.
  * The full ``parse_ldr`` → ``emit_prop_body`` pipeline produces
    valid MJCF that MuJoCo can compile.
"""

import re
import unittest
from xml.etree import ElementTree as ET

import mujoco

from openbricks_sim import lego_mjcf


class CoordinateConversionTests(unittest.TestCase):

    def test_origin_maps_to_origin(self):
        self.assertEqual(
            lego_mjcf._ldraw_to_mjcf_xyz(0, 0, 0), (0.0, 0.0, 0.0))

    def test_x_axis_unchanged(self):
        # 20 LDU = 1 stud = 8 mm; should land at (+0.008, 0, 0).
        x, y, z = lego_mjcf._ldraw_to_mjcf_xyz(20, 0, 0)
        self.assertAlmostEqual(x, 0.008, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)

    def test_ldraw_y_down_becomes_mjcf_z_up(self):
        # LDraw +Y is the gravity direction; in MJCF that's -Z.
        # 24 LDU = 1 brick height = 9.6 mm → mjcf_z = -0.0096.
        _, _, mjcf_z = lego_mjcf._ldraw_to_mjcf_xyz(0, 24, 0)
        self.assertAlmostEqual(mjcf_z, -0.0096, places=6)

    def test_ldraw_z_becomes_mjcf_y(self):
        # LDraw +Z is "out of page"; MJCF +Y is forward.
        _, mjcf_y, _ = lego_mjcf._ldraw_to_mjcf_xyz(0, 0, 40)
        self.assertAlmostEqual(mjcf_y, 0.016, places=6)


class PartRegistryTests(unittest.TestCase):

    def test_2x4_brick_dimensions(self):
        # 3001.dat is a 2x4 brick: 40 LDU × 80 LDU × 24 LDU =
        # 16 mm × 32 mm × 9.6 mm.
        spec = lego_mjcf._PART_REGISTRY["3001.dat"]
        self.assertEqual(spec.w_ldu, 40)
        self.assertEqual(spec.d_ldu, 80)
        self.assertEqual(spec.h_ldu, 24)

    def test_2x2_plate_is_one_third_brick_height(self):
        # 3022.dat is a 2x2 plate: 8 LDU = 3.2 mm tall (vs
        # 24 LDU for a full brick).
        spec = lego_mjcf._PART_REGISTRY["3022.dat"]
        self.assertEqual(spec.h_ldu, 8)

    def test_1x6_brick_has_six_studs(self):
        spec = lego_mjcf._PART_REGISTRY["3009.dat"]
        self.assertEqual(len(spec.stud_grid), 6)

    def test_2x4_brick_has_eight_studs(self):
        spec = lego_mjcf._PART_REGISTRY["3001.dat"]
        self.assertEqual(len(spec.stud_grid), 8)

    def test_tile_has_no_studs(self):
        # 3069b.dat is a 1x2 tile — smooth top, no visible studs.
        spec = lego_mjcf._PART_REGISTRY["3069b.dat"]
        self.assertEqual(spec.stud_grid, ())


class ParseLdrTests(unittest.TestCase):

    def test_skips_comments_and_step_markers(self):
        text = (
            "0 Untitled Model\n"
            "0 Name: test\n"
            "0 STEP\n"
            "1 1 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat\n"
        )
        out = lego_mjcf.parse_ldr(text)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].part, "3001.dat")
        self.assertEqual(out[0].ldraw_color, 1)

    def test_extracts_translation_and_rotation(self):
        # Identity rotation, translation (10, 20, 30) LDU.
        text = "1 4 10 20 30 1 0 0 0 1 0 0 0 1 3003.dat\n"
        out = lego_mjcf.parse_ldr(text)
        self.assertEqual(out[0].translation, (10.0, 20.0, 30.0))
        self.assertEqual(out[0].rotation,
                         (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
        self.assertEqual(out[0].ldraw_color, 4)


class EmitPlacementTests(unittest.TestCase):

    def test_brick_body_geom_dimensions(self):
        # Identity rotation 2x4 brick at LDraw origin.
        placement = lego_mjcf.Placement(
            rotation=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            translation=(0.0, 0.0, 0.0),
            part="3001.dat",
            ldraw_color=1)
        out = lego_mjcf.emit_placement(placement)
        # Should contain a box geom with size 0.008 0.016 0.0048
        # (half-extents of 16×32×9.6 mm).
        m = re.search(r'<geom type="box" size="([^"]+)"', out)
        self.assertIsNotNone(m, "no box geom emitted: " + out)
        sizes = [float(s) for s in m.group(1).split()]
        self.assertAlmostEqual(sizes[0], 0.008, places=4)
        self.assertAlmostEqual(sizes[1], 0.016, places=4)
        self.assertAlmostEqual(sizes[2], 0.0048, places=4)

    def test_studs_are_visual_only(self):
        # 2x4 brick → 8 studs, each ``contype=0 conaffinity=0``.
        placement = lego_mjcf.Placement(
            rotation=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            translation=(0.0, 0.0, 0.0),
            part="3001.dat",
            ldraw_color=1)
        out = lego_mjcf.emit_placement(placement)
        cylinders = re.findall(r'<geom type="cylinder"[^/]*/>', out)
        self.assertEqual(len(cylinders), 8,
                         "expected 8 stud cylinders for a 2x4 brick, "
                         "got %d" % len(cylinders))
        for cyl in cylinders:
            self.assertIn('contype="0"', cyl)
            self.assertIn('conaffinity="0"', cyl)


class FullPipelineTests(unittest.TestCase):
    """Build a tiny prop end-to-end and verify MuJoCo compiles it."""

    def test_two_brick_prop_compiles_in_mujoco(self):
        ldr = (
            "0 Test prop — 2 stacked 2x2 bricks\n"
            "1 1 0 0 0 1 0 0 0 1 0 0 0 1 3003.dat\n"
            "1 1 0 -24 0 1 0 0 0 1 0 0 0 1 3003.dat\n"
        )
        body_mjcf = lego_mjcf.emit_prop_body(
            "test_prop", (0.0, 0.0, 0.05), ldr,
            total_mass_kg=0.01)
        # Wrap in a minimal MJCF that defines the materials this
        # body references and a flat ground geom for the prop to
        # rest on.
        full_mjcf = (
            '<mujoco><asset>'
            '<material name="lego_blue" rgba="0.1 0.3 0.8 1"/>'
            '<material name="lego_white" rgba="1 1 1 1"/>'
            '<material name="lego_black" rgba="0 0 0 1"/>'
            '</asset>'
            '<worldbody>'
            '<geom type="plane" size="1 1 0.1"/>'
            + body_mjcf +
            '</worldbody></mujoco>')
        # Compile — this is the most useful integration test;
        # MuJoCo's parser catches malformed XML, missing materials,
        # invalid sizes, etc.
        model = mujoco.MjModel.from_xml_string(full_mjcf)
        self.assertEqual(model.nbody, 2)  # world + prop body
        # Two bricks → 2 box geoms + 2*4 = 8 stud cylinders =
        # 10 total brick-prop geoms; plus the ground plane.
        # ngeom counts every geom in the world.
        self.assertGreaterEqual(model.ngeom, 1 + 2 + 8)


if __name__ == "__main__":
    unittest.main()
