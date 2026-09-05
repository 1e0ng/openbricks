# SPDX-License-Identifier: MIT
"""Tests for the default chassis MJCF generator."""

import unittest

import mujoco

from openbricks_sim.chassis import ChassisSpec, chassis_mjcf, standalone_mjcf


class ChassisFragmentTests(unittest.TestCase):
    """The chassis generator produces an MJCF fragment whose shape
    is stable so later phases can splice it into worlds."""

    def setUp(self):
        self.fragment = chassis_mjcf()

    def test_has_one_chassis_body_with_freejoint(self):
        # Exactly one ``name="chassis"`` body so world-injection
        # doesn't produce collisions.
        self.assertEqual(self.fragment.count('name="chassis"'), 1)
        self.assertIn('<freejoint name="chassis_free"/>', self.fragment)

    def test_has_two_wheels_with_hinge_joints(self):
        self.assertIn('name="chassis_wheel_l"', self.fragment)
        self.assertIn('name="chassis_wheel_r"', self.fragment)
        self.assertIn('name="chassis_hinge_l"', self.fragment)
        self.assertIn('name="chassis_hinge_r"', self.fragment)

    def test_has_motor_actuators(self):
        self.assertIn('<motor name="chassis_motor_l"', self.fragment)
        self.assertIn('<motor name="chassis_motor_r"', self.fragment)

    def test_has_encoder_and_imu_sensors(self):
        self.assertIn('<jointpos name="chassis_enc_l"', self.fragment)
        self.assertIn('<jointpos name="chassis_enc_r"', self.fragment)
        self.assertIn('<jointvel name="chassis_encvel_l"', self.fragment)
        self.assertIn('<accelerometer', self.fragment)
        self.assertIn('<gyro', self.fragment)

    def test_has_downward_camera(self):
        self.assertIn('camera name="chassis_cam_down"', self.fragment)

    def test_custom_name_propagates(self):
        frag = chassis_mjcf(name="robotA")
        self.assertIn('name="robotA"', frag)
        self.assertIn('name="robotA_motor_l"', frag)
        self.assertNotIn('name="chassis"', frag)

    def test_spawn_position_propagates(self):
        spec = ChassisSpec(pos_x=0.7, pos_y=-0.3)
        frag = chassis_mjcf(spec)
        # The chassis body pos should start with the requested x/y.
        self.assertIn('pos="0.7000 -0.3000', frag)

    def test_spawn_yaw_propagates(self):
        # yaw_deg is the root body's euler Z (degrees, CCW from +X);
        # the default faces +X.
        self.assertIn('euler="0 0 0.0000"', self.fragment)
        frag = chassis_mjcf(ChassisSpec(yaw_deg=90.0))
        self.assertIn('euler="0 0 90.0000"', frag)

    def test_has_line_sensor_site(self):
        # The QTR shim spreads its elements from this site.
        self.assertIn('<site name="chassis_line" pos="0.0600 0', self.fragment)
        frag = chassis_mjcf(ChassisSpec(line_sensor_x=0.095))
        self.assertIn('<site name="chassis_line" pos="0.0950 0', frag)

    def test_colour_sensor_placement_moves_all_three_cameras(self):
        # Defaults reproduce the historical layout (front centre, the
        # pair 18 mm either side)...
        self.assertIn('camera name="chassis_cam_down" pos="0.0600 0.0000',
                      self.fragment)
        self.assertIn('pos="0.0600 0.0180', self.fragment)
        self.assertIn('pos="0.0600 -0.0180', self.fragment)
        # ...and a robot whose colour sensor hangs off to the left
        # moves the centre camera AND the pair with it.
        frag = chassis_mjcf(ChassisSpec(color_sensor_x=0.02,
                                        color_sensor_y=0.18))
        self.assertIn('camera name="chassis_cam_down" pos="0.0200 0.1800',
                      frag)
        self.assertIn('pos="0.0200 0.1980', frag)
        self.assertIn('pos="0.0200 0.1620', frag)


class ResizeTests(unittest.TestCase):
    """``apply_drivebase_dims_to_model`` puts a resized wheel ON the
    floor, keeps the caster there too, and refreshes the collision
    bounds — each of which it used to get wrong (buried wheel, a
    chassis pitched back onto a floating caster, a grown wheel whose
    stale bounding sphere never reached the floor)."""

    def _resized(self, wheel_mm, axle_mm):
        from openbricks_sim.chassis import apply_drivebase_dims_to_model
        m = mujoco.MjModel.from_xml_string(standalone_mjcf())
        d = mujoco.MjData(m)
        apply_drivebase_dims_to_model(m, wheel_diameter_mm=wheel_mm,
                                      axle_track_mm=axle_mm, data=d)
        return m, d

    def _contact_zs(self, m, d, body_name):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
        for _ in range(500):
            mujoco.mj_step(m, d)
        return [float(d.contact[i].pos[2]) for i in range(d.ncon)
                if int(m.geom_bodyid[d.contact[i].geom1]) == bid
                or int(m.geom_bodyid[d.contact[i].geom2]) == bid]

    def test_grown_wheels_stand_on_the_floor_level(self):
        m, d = self._resized(88.0, 136.0)
        wl = self._contact_zs(m, d, "chassis_wheel_l")
        caster = self._contact_zs(m, d, "chassis_caster")
        self.assertTrue(wl, "the grown wheel makes no floor contact")
        self.assertTrue(caster, "the caster lost the floor")
        self.assertTrue(all(abs(z) < 0.002 for z in wl), wl)
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        R = d.xmat[cid].reshape(3, 3)
        pitch_deg = abs(60.0 * float(R[2, 0]))   # small-angle, deg-ish
        self.assertLess(pitch_deg, 2.0, "chassis pitched %.1f" % pitch_deg)
        # Ride height follows the radius: 44 mm wheel + 5 mm clearance.
        self.assertAlmostEqual(float(d.xpos[cid, 2]), 0.049, delta=0.004)

    def test_bounds_follow_the_radius(self):
        m, _ = self._resized(88.0, 136.0)
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis_wheel_l")
        gid = next(g for g in range(m.ngeom) if int(m.geom_bodyid[g]) == bid)
        self.assertGreater(float(m.geom_rbound[gid]), 0.044)
        self.assertAlmostEqual(float(m.geom_aabb[gid, 3]), 0.044, delta=1e-6)

    def test_custom_spec_body_height_is_honoured(self):
        from openbricks_sim.chassis import apply_drivebase_dims_to_model
        spec = ChassisSpec(body_height=0.090, caster_radius=0.015)
        m = mujoco.MjModel.from_xml_string(standalone_mjcf(spec))
        d = mujoco.MjData(m)
        apply_drivebase_dims_to_model(m, wheel_diameter_mm=80.0,
                                      axle_track_mm=150.0,
                                      chassis_spec=spec, data=d)
        cbid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis_caster")
        # caster centre = its radius above the floor, whatever the body.
        self.assertAlmostEqual(float(m.body_pos[cbid, 2]), 0.015 - 0.045,
                               delta=1e-9)


class StandaloneChassisTests(unittest.TestCase):
    """`standalone_mjcf` should parse under MuJoCo and let the
    chassis settle on its wheels."""

    def test_parses_and_settles_upright(self):
        m = mujoco.MjModel.from_xml_string(standalone_mjcf())
        d = mujoco.MjData(m)
        for _ in range(2000):  # 2 s
            mujoco.mj_step(m, d)
        # Chassis body id → check it hasn't fallen over. xpos[2] is
        # the world-frame z; should be ~0.05 m above the floor.
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.assertGreater(d.xpos[cid, 2], 0.02)
        self.assertLess(d.xpos[cid, 2], 0.10)

    def test_motor_actuators_drive_the_wheels(self):
        m = mujoco.MjModel.from_xml_string(standalone_mjcf())
        d = mujoco.MjData(m)
        # Identify the encoder sensors by name and read initial angle.
        enc_l_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_enc_l")
        enc_r_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_enc_r")
        # Drive both motors forward for 1 s.
        act_l = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "chassis_motor_l")
        act_r = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "chassis_motor_r")
        d.ctrl[act_l] = 0.3
        d.ctrl[act_r] = 0.3
        for _ in range(1000):
            mujoco.mj_step(m, d)
        # Wheels should have rotated (encoder values > 0).
        self.assertGreater(d.sensordata[enc_l_id], 0.5)
        self.assertGreater(d.sensordata[enc_r_id], 0.5)


if __name__ == "__main__":
    unittest.main()
