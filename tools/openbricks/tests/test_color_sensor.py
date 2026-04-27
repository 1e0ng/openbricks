# SPDX-License-Identifier: MIT
"""Tests for ``SimColorSensor`` + the shim ``TCS34725``.

The chassis on the standalone preview floor sees a checker-grid
material (off-white). For colour assertions we build a tiny custom
model with a coloured floor geom and verify the raycast returns
the expected RGB triple.
"""

import unittest

import mujoco

from openbricks_sim.runtime import SimRuntime, SimColorSensor


# A bare model with a single coloured floor geom + the same chassis
# camera + IMU site setup so SimColorSensor can attach. Generated
# inline so each test can pick its own floor colour.
_MODEL_TEMPLATE = """\
<mujoco model="color_sensor_test">
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="chassis" pos="0 0 0.05">
      <freejoint name="chassis_free"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
      <geom name="chassis_body" type="box" size="0.02 0.02 0.01"
            rgba="0.10 0.50 0.90 1.0"/>
      <site name="chassis_imu" pos="0 0 0" size="0.005"/>
      <camera name="chassis_cam_down" pos="0 0 0"
              xyaxes="0 -1 0 1 0 0" fovy="20"/>
    </body>
    <geom name="floor" type="plane" size="1 1 0.1"
          rgba="{r} {g} {b} 1"/>
  </worldbody>
</mujoco>
"""


def _make_runtime(r=0.5, g=0.5, b=0.5):
    xml = _MODEL_TEMPLATE.format(r=r, g=g, b=b)
    model = mujoco.MjModel.from_xml_string(xml)
    data  = mujoco.MjData(model)
    return SimRuntime(model, data)


class SimColorSensorTests(unittest.TestCase):

    def test_construct_rejects_unknown_camera(self):
        rt = _make_runtime()
        with self.assertRaises(ValueError):
            SimColorSensor(rt, camera_name="no_such_cam")

    def test_construct_rejects_unknown_chassis_body(self):
        rt = _make_runtime()
        with self.assertRaises(ValueError):
            SimColorSensor(rt, chassis_body_name="no_such_body")

    def test_white_floor_reads_white(self):
        rt = _make_runtime(r=1.0, g=1.0, b=1.0)
        cs = SimColorSensor(rt)
        r, g, b = cs.rgb()
        self.assertEqual(r, 255)
        self.assertEqual(g, 255)
        self.assertEqual(b, 255)

    def test_black_floor_reads_black(self):
        rt = _make_runtime(r=0.0, g=0.0, b=0.0)
        cs = SimColorSensor(rt)
        r, g, b = cs.rgb()
        self.assertEqual(r, 0)
        self.assertEqual(g, 0)
        self.assertEqual(b, 0)

    def test_red_floor_reads_red_dominant(self):
        rt = _make_runtime(r=1.0, g=0.0, b=0.0)
        cs = SimColorSensor(rt)
        r, g, b = cs.rgb()
        self.assertEqual(r, 255)
        self.assertEqual(g, 0)
        self.assertEqual(b, 0)

    def test_ambient_white_is_max(self):
        rt = _make_runtime(r=1.0, g=1.0, b=1.0)
        cs = SimColorSensor(rt)
        self.assertEqual(cs.ambient(), 100)

    def test_ambient_black_is_zero(self):
        rt = _make_runtime(r=0.0, g=0.0, b=0.0)
        cs = SimColorSensor(rt)
        self.assertEqual(cs.ambient(), 0)


# ---------------------------------------------------------------------
# End-to-end: construct via the shim using the real openbricks
# ColorSensor interface.

class ShimTCS34725Tests(unittest.TestCase):

    def setUp(self):
        from openbricks_sim import shim
        from openbricks_sim.robot import SimRobot
        if shim.is_installed():
            shim.uninstall()
        self.robot = SimRobot()   # standalone chassis on checker floor
        shim.install(self.robot.runtime)

    def tearDown(self):
        from openbricks_sim import shim
        shim.uninstall()

    def test_user_code_path_imports_real_tcs(self):
        # Patched by the shim — import must resolve to ShimTCS34725.
        from openbricks.drivers.tcs34725 import TCS34725
        from openbricks_sim.shim import ShimTCS34725
        self.assertIs(TCS34725, ShimTCS34725)

    def test_construct_with_firmware_args(self):
        from openbricks.drivers.tcs34725 import TCS34725
        from machine import I2C
        i2c = I2C(0, scl=22, sda=21, freq=400_000)
        sensor = TCS34725(i2c=i2c, address=0x29,
                          integration_ms=24, gain=4)
        # rgb / ambient / raw — the full ColorSensor surface.
        r, g, b = sensor.rgb()
        self.assertGreaterEqual(r, 0); self.assertLessEqual(r, 255)
        self.assertGreaterEqual(g, 0); self.assertLessEqual(g, 255)
        self.assertGreaterEqual(b, 0); self.assertLessEqual(b, 255)
        amb = sensor.ambient()
        self.assertGreaterEqual(amb, 0); self.assertLessEqual(amb, 100)
        c, r16, g16, b16 = sensor.raw()
        self.assertGreaterEqual(c, 0); self.assertLessEqual(c, 65535)


if __name__ == "__main__":
    unittest.main()
