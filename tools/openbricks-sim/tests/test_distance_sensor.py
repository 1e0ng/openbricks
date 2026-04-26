# SPDX-License-Identifier: MIT
"""Tests for ``SimDistanceSensor`` + the shim ``HCSR04``."""

import unittest

import mujoco

from openbricks_sim.runtime import SimRuntime, SimDistanceSensor


# Bare model with a single forward-of-chassis wall geom. Distance
# from the chassis_dist site to the wall is parameterised so each
# test can set its own.
_MODEL_TEMPLATE = """\
<mujoco model="distance_sensor_test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="chassis" pos="0 0 0.05">
      <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
      <geom name="chassis_body" type="box" size="0.02 0.02 0.01"
            rgba="0.10 0.50 0.90 1.0"/>
      <site name="chassis_dist" pos="0.021 0 0" size="0.005"/>
    </body>
    <geom name="wall" type="box"
          pos="{wall_x} 0 0.05" size="0.01 0.5 0.05"
          rgba="0.5 0.5 0.5 1"/>
  </worldbody>
</mujoco>
"""


def _make_runtime(wall_x_m=0.5):
    xml = _MODEL_TEMPLATE.format(wall_x=wall_x_m)
    model = mujoco.MjModel.from_xml_string(xml)
    data  = mujoco.MjData(model)
    return SimRuntime(model, data)


class SimDistanceSensorTests(unittest.TestCase):

    def test_construct_rejects_unknown_site(self):
        rt = _make_runtime()
        with self.assertRaises(ValueError):
            SimDistanceSensor(rt, site_name="no_such_site")

    def test_construct_rejects_unknown_chassis_body(self):
        rt = _make_runtime()
        with self.assertRaises(ValueError):
            SimDistanceSensor(rt, chassis_body_name="no_such_body")

    def test_wall_at_500mm_reads_around_500(self):
        # Site is at (0.021, 0, 0) inside chassis at world (0,0,0.05).
        # Wall at world x=0.5, near face at x=0.49. Distance ≈ 469 mm.
        rt = _make_runtime(wall_x_m=0.5)
        ds = SimDistanceSensor(rt)
        d = ds.distance_mm()
        # Generous bound — exact site-to-wall distance is set by the
        # geom's near face. We just want "in the right ballpark".
        self.assertGreater(d, 400)
        self.assertLess(d, 500)

    def test_wall_at_100mm_reads_close(self):
        rt = _make_runtime(wall_x_m=0.1)
        ds = SimDistanceSensor(rt)
        d = ds.distance_mm()
        self.assertGreater(d, 50)
        self.assertLess(d, 100)

    def test_no_wall_returns_minus_one(self):
        # Empty world — only the chassis. The body-exclude flag means
        # the chassis self-geom doesn't count, and no other geom is in
        # the way → -1.
        xml = """\
<mujoco model="empty_test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="chassis" pos="0 0 0.05">
      <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
      <geom name="chassis_body" type="box" size="0.02 0.02 0.01"/>
      <site name="chassis_dist" pos="0.021 0 0" size="0.005"/>
    </body>
  </worldbody>
</mujoco>
"""
        model = mujoco.MjModel.from_xml_string(xml)
        data  = mujoco.MjData(model)
        ds = SimDistanceSensor(SimRuntime(model, data))
        self.assertEqual(ds.distance_mm(), -1)

    def test_max_range_truncates(self):
        # Wall is at 1 m but max_range_mm is 500 — should report -1.
        rt = _make_runtime(wall_x_m=1.0)
        ds = SimDistanceSensor(rt, max_range_mm=500.0)
        self.assertEqual(ds.distance_mm(), -1)


# ---------------------------------------------------------------------
# End-to-end: import openbricks.drivers.hcsr04.HCSR04 through the shim.

class ShimHCSR04Tests(unittest.TestCase):

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

    def test_user_code_imports_real_hcsr04(self):
        from openbricks.drivers.hcsr04 import HCSR04
        from openbricks_sim.shim import ShimHCSR04
        self.assertIs(HCSR04, ShimHCSR04)

    def test_construct_with_firmware_args(self):
        from openbricks.drivers.hcsr04 import HCSR04
        sensor = HCSR04(trig=12, echo=13, timeout_us=20_000)
        d = sensor.distance_mm()
        # On the empty world the forward ray sees nothing → -1.
        self.assertEqual(d, -1)


if __name__ == "__main__":
    unittest.main()
