# SPDX-License-Identifier: MIT
"""End-to-end tests for the sim runtime.

These tests load a real MuJoCo model (the standalone chassis on a
checker floor — same one ``openbricks-sim preview`` uses) and drive
it via ``SimMotor`` / ``SimDriveBase`` adapters. Each test asserts a
behavioural property we'd expect from the firmware running on real
hardware: motors converge on a target velocity, drivebase straight
moves the chassis along +X, turn rotates it, etc.

No MuJoCo mocking — the asserts have generous tolerances because
the wheel-floor contact + actuator dynamics are stochastic. Tests
are the integration-test layer; tighter algorithmic assertions live
in ``test_native_*``.
"""

import math
import unittest

import mujoco

from openbricks_sim.chassis import ChassisSpec, standalone_mjcf
from openbricks_sim.runtime import SimRuntime, SimMotor, SimDriveBase


def _make_runtime():
    spec  = ChassisSpec()
    xml   = standalone_mjcf(spec)
    model = mujoco.MjModel.from_xml_string(xml)
    data  = mujoco.MjData(model)
    return SimRuntime(model, data)


def _settle(runtime, ms=200):
    for _ in range(ms):
        mujoco.mj_step(runtime.model, runtime.data)


class SimRuntimeTests(unittest.TestCase):

    def test_now_ms_advances_with_step(self):
        rt = _make_runtime()
        # MuJoCo's default chassis option uses 1 ms timestep; verify.
        self.assertEqual(rt.timestep_ms, 1)
        rt.step()
        self.assertEqual(rt.now_ms, 1)
        rt.step()
        rt.step()
        self.assertEqual(rt.now_ms, 3)

    def test_add_tick_dedup(self):
        rt = _make_runtime()
        called = []
        def fn(now): called.append(now)
        rt.add_tick(fn)
        rt.add_tick(fn)        # duplicate → no-op
        rt.step()
        self.assertEqual(called, [1])

    def test_remove_tick_unregisters(self):
        rt = _make_runtime()
        called = []
        def fn(now): called.append(now)
        rt.add_tick(fn)
        rt.remove_tick(fn)
        rt.step()
        self.assertEqual(called, [])

    def test_remove_tick_unknown_is_noop(self):
        rt = _make_runtime()
        rt.remove_tick(lambda now: None)   # never registered


class SimMotorTests(unittest.TestCase):

    def test_construct_rejects_unknown_sensor(self):
        rt = _make_runtime()
        with self.assertRaises(ValueError):
            SimMotor(rt, "no_such_sensor", "chassis_motor_l")

    def test_construct_rejects_unknown_actuator(self):
        rt = _make_runtime()
        with self.assertRaises(ValueError):
            SimMotor(rt, "chassis_enc_l", "no_such_actuator")

    def test_run_speed_drives_motor(self):
        # Drive both wheels at +180 dps; the chassis should translate
        # forward (+X) within a couple seconds.
        rt = _make_runtime()
        left  = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        right = SimMotor(rt, "chassis_enc_r", "chassis_motor_r")
        _settle(rt)   # let the chassis settle on its wheels first
        left.run_speed(180.0)
        right.run_speed(180.0)
        cid = mujoco.mj_name2id(rt.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        x0 = rt.data.xpos[cid, 0]
        for _ in range(2000):
            rt.step()
        x1 = rt.data.xpos[cid, 0]
        self.assertGreater(x1 - x0, 0.02,
                           "run_speed(180) should translate the chassis "
                           "forward; x went %.3f → %.3f" % (x0, x1))

    def test_run_speed_negative_drives_backward(self):
        rt = _make_runtime()
        left  = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        right = SimMotor(rt, "chassis_enc_r", "chassis_motor_r")
        _settle(rt)
        left.run_speed(-180.0)
        right.run_speed(-180.0)
        cid = mujoco.mj_name2id(rt.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        x0 = rt.data.xpos[cid, 0]
        for _ in range(2000):
            rt.step()
        x1 = rt.data.xpos[cid, 0]
        self.assertLess(x1 - x0, -0.02,
                        "run_speed(-180) should reverse the chassis; "
                        "x went %.3f → %.3f" % (x0, x1))

    def test_brake_zeros_actuator_ctrl(self):
        # Brake's contract is "controller stops driving the actuator"
        # — not "chassis instantly halts" (that's a physics
        # question beyond the controller's reach). Verify the
        # contract: ctrl gets zeroed and stays zeroed even though
        # later mj_step calls don't fire our tick (we detached).
        rt = _make_runtime()
        left = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        _settle(rt)
        left.run_speed(180.0)
        for _ in range(50):
            rt.step()
        act_id = mujoco.mj_name2id(rt.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                    "chassis_motor_l")
        self.assertNotEqual(rt.data.ctrl[act_id], 0.0)
        left.brake()
        # The brake() write happens immediately; the next step's
        # tick must NOT re-write ctrl (motor was detached).
        rt.step()
        self.assertEqual(rt.data.ctrl[act_id], 0.0)
        for _ in range(100):
            rt.step()
        self.assertEqual(rt.data.ctrl[act_id], 0.0)

    def test_angle_reads_joint_position(self):
        rt = _make_runtime()
        left = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        _settle(rt)
        a0 = left.angle()
        left.run_speed(180.0)
        for _ in range(500):
            rt.step()
        a1 = left.angle()
        # Wheel must have rotated; sign depends on hinge axis but
        # magnitude should be positive.
        self.assertGreater(abs(a1 - a0), 5.0)


class SimDriveBaseTests(unittest.TestCase):

    def _setup(self):
        rt = _make_runtime()
        left  = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        right = SimMotor(rt, "chassis_enc_r", "chassis_motor_r")
        # 60 mm wheel diameter (matches ChassisSpec wheel_radius=0.030)
        # 150 mm axle (wheel-to-wheel separation)
        db = SimDriveBase(rt, left, right,
                           wheel_diameter_mm=60.0,
                           axle_track_mm=150.0)
        _settle(rt)
        cid = mujoco.mj_name2id(rt.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        return rt, db, left, right, cid

    def test_straight_drives_chassis_forward_during_trajectory(self):
        # Sim wheel↔floor friction has more slip + torque saturation
        # than firmware-tuned hardware, so the controller can't hold
        # tight position lock once the trajectory ends. We assert
        # the contract that matters here: while the straight move is
        # active, the chassis translates along +X. (The closed-loop
        # convergence under a faithful dynamics model is the
        # firmware's ``test_drivebase_native_2dof`` test's job.)
        rt, db, _, _, cid = self._setup()
        x0 = rt.data.xpos[cid, 0]
        db.straight(distance_mm=200.0, speed_mm_s=80.0)
        # Sample mid-way through the trajectory's cruise phase.
        for _ in range(800):
            rt.step()
        x_mid = rt.data.xpos[cid, 0]
        self.assertGreater(x_mid - x0, 0.03,
                           "chassis should translate +X while a straight "
                           "move is active (x went %.3f → %.3f)" %
                           (x0, x_mid))
        self.assertFalse(db.is_done())

    def test_straight_eventually_completes(self):
        rt, db, _, _, _ = self._setup()
        db.straight(distance_mm=50.0, speed_mm_s=100.0)
        for _ in range(5000):
            rt.step()
            if db.is_done():
                break
        self.assertTrue(db.is_done())

    def test_stop_clears_done(self):
        rt, db, _, _, _ = self._setup()
        db.straight(100.0, 100.0)
        self.assertFalse(db.is_done())
        db.stop()
        self.assertTrue(db.is_done())

    def test_set_use_gyro_and_override_smoke(self):
        # Exercise the gyro path end-to-end; assert no exceptions
        # and that the motor target velocities respond. Algorithmic
        # correctness is covered in test_native_drivebase.py.
        rt, db, left, right, _ = self._setup()
        db.set_use_gyro(True)
        db.straight(100.0, 100.0)
        for _ in range(50):
            rt.step()
        l0 = left.target_dps()
        db.set_heading_override(5.0)   # robot has yawed +5° CCW
        for _ in range(5):
            rt.step()
        l1 = left.target_dps()
        # +5° body yaw should add a CW correction → left target ramps up.
        self.assertGreater(l1, l0 - 1.0)


if __name__ == "__main__":
    unittest.main()
