# SPDX-License-Identifier: MIT
"""Tests for ``SimReflectanceArray`` — the sensor model behind the QTR
driver shim — and the ``FloorSampler`` it shares with the colour
sensor.

One downward ray per spot sample from a chassis site, luminance
averaged over the spot, dark = high: the reading a QTR element's
discharge timer would produce over that patch of mat."""

import unittest

import mujoco

from openbricks_sim.runtime import (SimRuntime, SimReflectanceArray,
                                     FloorSampler)


# A white plane with a 20 mm black line along +X at y=0 (a raised
# box, like practice-line) and a 4x4 red/blue checker plane region is
# NOT needed: the line world exercises the box path, the textured
# test below the plane-texture path.
_LINE_TEMPLATE = """\
<mujoco model="reflectance_line_test">
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="chassis" pos="{cx} {cy} 0.05" euler="0 0 {yaw}">
      <freejoint name="chassis_free"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
      <geom name="chassis_body" type="box" size="0.02 0.02 0.01"
            rgba="0.10 0.50 0.90 1.0"/>
      <site name="chassis_line" pos="0.06 0 0" size="0.003"/>
    </body>
    <geom name="floor" type="plane" size="0.5 0.5 0.1"
          rgba="1 1 1 1"/>
    <geom name="line" type="box" pos="0 0 0.001" size="0.4 0.010 0.001"
          rgba="0 0 0 1"/>
  </worldbody>
</mujoco>
"""

# Textured plane: MuJoCo's builtin 4x4 checker (2-pixel blocks) gives
# red at world (-x, +y) and (+x, -y), blue in the other two quadrants
# (same mapping tests/test_color_sensor.py documents).
_TEXTURED_TEMPLATE = """\
<mujoco model="reflectance_textured_test">
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <asset>
    <texture name="t" type="2d" builtin="checker"
             rgb1="1 0 0" rgb2="0 0 1" width="4" height="4"/>
    <material name="m" texture="t" texrepeat="1 1" texuniform="false"/>
  </asset>
  <worldbody>
    <body name="chassis" pos="{cx} {cy} 0.05">
      <freejoint name="chassis_free"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
      <geom name="chassis_body" type="box" size="0.02 0.02 0.01"/>
      <site name="chassis_line" pos="0 0 0" size="0.003"/>
    </body>
    <geom name="floor" type="plane" size="0.5 0.5 0.1" material="m"/>
  </worldbody>
</mujoco>
"""

# The bench window: ten elements, 56 mm, skip pattern.
_POSITIONS = (-28.0, -20.0, -16.0, -12.0, -4.0, 4.0, 12.0, 16.0, 20.0, 28.0)


def _line_runtime(cx=0.0, cy=0.0, yaw=0.0):
    model = mujoco.MjModel.from_xml_string(
        _LINE_TEMPLATE.format(cx=cx, cy=cy, yaw=yaw))
    return SimRuntime(model, mujoco.MjData(model))


def _textured_runtime(cx=0.0, cy=0.0):
    model = mujoco.MjModel.from_xml_string(
        _TEXTURED_TEMPLATE.format(cx=cx, cy=cy))
    return SimRuntime(model, mujoco.MjData(model))


class ConstructionTests(unittest.TestCase):
    def test_rejects_unknown_site(self):
        rt = _line_runtime()
        with self.assertRaises(ValueError):
            SimReflectanceArray(rt, _POSITIONS, site_name="nope")

    def test_rejects_empty_positions(self):
        rt = _line_runtime()
        with self.assertRaises(ValueError):
            SimReflectanceArray(rt, ())

    def test_rejects_zero_samples(self):
        rt = _line_runtime()
        with self.assertRaises(ValueError):
            SimReflectanceArray(rt, _POSITIONS, samples=0)


class LineReadingTests(unittest.TestCase):
    """Chassis on the line's centre, facing +X: the 20 mm line spans
    y = -10..+10, so the elements at +/-4 mm are black and the rest
    see white mat."""

    def test_elements_over_the_line_read_dark_high(self):
        arr = SimReflectanceArray(_line_runtime(), _POSITIONS)
        raw = arr.read_u16()
        self.assertEqual(len(raw), 10)
        self.assertEqual(raw[4], arr.FULL_SCALE)      # -4 mm
        self.assertEqual(raw[5], arr.FULL_SCALE)      # +4 mm
        for i in (0, 1, 2, 3, 6, 7, 8, 9):
            self.assertEqual(raw[i], 0, "element %d saw %d" % (i, raw[i]))

    def test_right_is_body_minus_y(self):
        # Shift the chassis 12 mm to +Y (left): the line now sits
        # under the elements at +12 and +16 mm — the RIGHT side of
        # the array, which is body -Y. A sign slip here would make
        # every edge follower steer the wrong way.
        arr = SimReflectanceArray(_line_runtime(cy=0.012), _POSITIONS)
        raw = arr.read_u16()
        self.assertEqual(raw[6], arr.FULL_SCALE)      # +12 mm
        self.assertEqual(raw[7], arr.FULL_SCALE)      # +16 mm
        self.assertEqual(raw[4], 0)                   # -4 mm now off it

    def test_spot_averaging_grades_the_edge(self):
        # Element at +12 mm with the chassis 2 mm left of the line
        # centre: the element's 3 mm spot straddles the line's edge
        # at y = -10 (element centre at y = -10) — half in, half out.
        arr = SimReflectanceArray(_line_runtime(cy=0.002), _POSITIONS,
                                  spot_mm=3.0, samples=3)
        raw = arr.read_u16()
        self.assertGreater(raw[6], arr.FULL_SCALE * 0.4)
        self.assertLess(raw[6], arr.FULL_SCALE * 0.7)
        # ...and a point sampler sees a hard step there.
        point = SimReflectanceArray(_line_runtime(cy=0.002), _POSITIONS,
                                    samples=1)
        self.assertIn(point.read_u16()[6], (0, point.FULL_SCALE))

    def test_heading_rotates_the_array_with_the_chassis(self):
        # Facing +Y, the array lies along X; the line along X then
        # runs under EVERY element (all of them at the line's y).
        arr = SimReflectanceArray(_line_runtime(yaw=90.0), _POSITIONS)
        raw = arr.read_u16()
        # The site sits 60 mm ahead of the chassis = at y=0.06, off the
        # line's centre... the line is 20 mm wide about y=0, so no
        # element is over it: all white.
        self.assertEqual(raw, [0] * 10)
        # Chassis backed to y=-0.06 puts the site on y=0: all dark.
        arr = SimReflectanceArray(_line_runtime(cy=-0.06, yaw=90.0),
                                  _POSITIONS)
        self.assertEqual(arr.read_u16(), [arr.FULL_SCALE] * 10)

    def test_ray_miss_reads_dark(self):
        # Past the plane's edge nothing reflects — full scale, the
        # way a real element over a void discharges to its timeout.
        arr = SimReflectanceArray(_line_runtime(cx=0.60), _POSITIONS)
        self.assertEqual(arr.read_u16(), [arr.FULL_SCALE] * 10)


class TexturedPlaneTests(unittest.TestCase):
    def test_reads_texels_through_the_shared_floor_sampler(self):
        # Red (luma 0.299) vs blue (luma 0.114): a plane texture
        # resolves per texel, the same code path the colour sensor
        # uses, so a mat PNG reads the same for both sensors.
        arr = SimReflectanceArray(_textured_runtime(cx=-0.25, cy=0.25),
                                  (0.0,), samples=1)
        red = arr.luminance()[0]
        arr = SimReflectanceArray(_textured_runtime(cx=0.25, cy=0.25),
                                  (0.0,), samples=1)
        blue = arr.luminance()[0]
        self.assertAlmostEqual(red, 0.299, places=3)
        self.assertAlmostEqual(blue, 0.114, places=3)

    def test_floor_sampler_is_the_colour_sensor_s_answer(self):
        rt = _textured_runtime(cx=-0.25, cy=0.25)
        mujoco.mj_forward(rt.model, rt.data)
        rgba = FloorSampler(rt).rgba_below((-0.25, 0.25, 0.05))
        self.assertEqual(tuple(round(c, 3) for c in rgba[:3]), (1.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
