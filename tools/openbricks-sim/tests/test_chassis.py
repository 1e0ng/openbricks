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
