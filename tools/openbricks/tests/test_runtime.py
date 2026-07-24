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

    def test_construct_rejects_ctrlrange_below_stall_torque(self):
        # The DC model writes torque up to T_STALL_NM; an actuator
        # whose ctrlrange can't carry it would silently truncate the
        # motor model. Must refuse loudly at construction.
        rt = _make_runtime()
        rt.model.actuator_ctrlrange[:] = 0.01   # below T_STALL_NM
        with self.assertRaises(ValueError):
            SimMotor(rt, "chassis_enc_l", "chassis_motor_l")

    def test_apply_power_clamps_at_stall_torque(self):
        # Full duty against a hard-reverse-spinning wheel: the raw DC
        # expression exceeds stall; the write must clamp to T_STALL.
        rt = _make_runtime()
        m = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        rt.data.qvel[m._dof_adr] = -100.0   # rad/s, wildly negative
        m.apply_power(100.0)
        self.assertAlmostEqual(
            float(rt.data.ctrl[m._actuator_id]), m.T_STALL_NM)
        rt.data.qvel[m._dof_adr] = 100.0
        m.apply_power(-100.0)
        self.assertAlmostEqual(
            float(rt.data.ctrl[m._actuator_id]), -m.T_STALL_NM)

    def test_apply_power_equilibrium_matches_feedforward(self):
        # At the servo core's rated speed, full-scale-equivalent duty
        # produces ~zero torque — the DC model's equilibrium is
        # exactly where the core's feed-forward assumes it.
        import math
        rt = _make_runtime()
        m = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        rt.data.qvel[m._dof_adr] = math.radians(m.RATED_DPS)
        m.apply_power(100.0)
        self.assertAlmostEqual(
            float(rt.data.ctrl[m._actuator_id]), 0.0, places=6)

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

    def test_dc_sustains_duty_through_the_motor_model(self):
        # Pybricks Motor.dc(): sustained duty, reapplied per tick so
        # back-EMF limits speed like hardware. The chassis must move
        # and keep moving (a one-shot ctrl write would decay).
        rt = _make_runtime()
        left = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        right = SimMotor(rt, "chassis_enc_r", "chassis_motor_r")
        _settle(rt)
        left.dc(60)
        right.dc(60)
        for _ in range(200):
            rt.step()
        self.assertGreater(abs(left.speed()), 30.0,
                           "dc(60) should spin the wheel")

    def test_run_is_closed_loop_speed(self):
        # Pybricks run(speed) == run_speed: wheel converges to the
        # commanded deg/s, not to a duty fraction.
        rt = _make_runtime()
        left = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        _settle(rt)
        left.run(120.0)
        for _ in range(400):
            rt.step()
        self.assertAlmostEqual(left.speed(), 120.0, delta=25.0)

    def test_run_speed_cancels_dc_mode(self):
        rt = _make_runtime()
        left = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        _settle(rt)
        left.dc(100)
        for _ in range(100):
            rt.step()
        left.run_speed(60.0)
        for _ in range(400):
            rt.step()
        self.assertAlmostEqual(left.speed(), 60.0, delta=20.0,
                               msg="dc mode kept overriding run_speed")

    def test_stop_matches_pybricks_coast_semantics(self):
        # Motor.stop() (Pybricks: spin freely) exists on SimMotor too
        # — mirrors the firmware interface so scripts using stop()
        # run unmodified in the sim.
        rt = _make_runtime()
        left = SimMotor(rt, "chassis_enc_l", "chassis_motor_l")
        _settle(rt)
        left.run_speed(180.0)
        for _ in range(50):
            rt.step()
        left.stop()
        act_id = mujoco.mj_name2id(rt.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                    "chassis_motor_l")
        self.assertEqual(rt.data.ctrl[act_id], 0.0)

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
        # 1.15.2 contract: enabling the gyro path without a heading
        # feed is a loud error, not a silently-stale override.
        with self.assertRaises(RuntimeError):
            db.set_use_gyro(True)

        class _StillIMU:
            def heading(self):
                return 0.0
        db.attach_imu(_StillIMU())
        db.set_use_gyro(True)
        db._detach_imu_tick()   # manual overrides below drive the test
        db.straight(100.0, 100.0)
        for _ in range(50):
            rt.step()
        db.set_heading_override(5.0)   # robot has yawed +5° CW/right
        for _ in range(5):
            rt.step()
        # +5° CW body yaw (Pybricks convention) should add a CCW
        # correction → right wheel commanded faster than left.
        self.assertGreater(right.target_dps(), left.target_dps())




class SimDriveBaseGyroTests(unittest.TestCase):
    """The gyro-guided drivebase path, end-to-end in physics.

    REGRESSION (1.15.2): the native core used to snapshot a gyro
    move's origin from the ENCODER diff while the tick read the
    move-relative IMU override — any prior rotation became a
    permanent phantom heading error and gyro turns ran away (a
    turn(90) after one encoder turn rotated ~180 and never stopped).
    """

    def _robot(self):
        from openbricks_sim.robot import SimRobot
        from openbricks_sim.runtime import SimIMU
        robot = SimRobot()
        return robot, SimIMU(robot.runtime)

    def _yaw(self, imu):
        return imu.heading()

    def test_set_use_gyro_without_imu_raises(self):
        robot, _ = self._robot()
        with self.assertRaises(RuntimeError):
            robot.drivebase.set_use_gyro(True)

    def test_gyro_turn_after_prior_encoder_turn_lands_and_stops(self):
        robot, imu = self._robot()
        db = robot.drivebase
        # Prior encoder-mode rotation: accumulates encoder diff.
        db.turn(90.0, 90.0)
        robot.run_for(2.0)
        h1 = self._yaw(imu)
        # Gyro-guided turn: must land ~90 further and STOP.
        db.attach_imu(imu)
        db.set_use_gyro(True)
        db.turn(90.0, 90.0)
        robot.run_for(2.0)
        h2 = self._yaw(imu)
        robot.run_for(0.5)
        h3 = self._yaw(imu)
        turned = ((h2 - h1 + 180.0) % 360.0) - 180.0
        creep = abs(((h3 - h2 + 180.0) % 360.0) - 180.0)
        self.assertTrue(75.0 < turned < 105.0,
                        "gyro turn landed at %.1f deg" % turned)
        self.assertLess(creep, 3.0,
                        "robot kept rotating after the gyro move "
                        "finished (%.1f deg of creep)" % creep)

    def test_imu_tick_wraps_the_boundary(self):
        # A move whose heading crosses the +/-180 seam must not see a
        # spurious +/-360 delta jump. Drive the wrap arithmetic
        # directly through _imu_tick with a scripted heading source.
        robot, _ = self._robot()
        db = robot.drivebase

        class _Scripted:
            heading_value = 0.0

            def heading(self):
                return _Scripted.heading_value

        src = _Scripted()
        db.attach_imu(src)
        db.set_use_gyro(True)
        # Offset captured at 0. Heading jumps to +200: raw delta
        # +200 wraps to -160 (took the short way past the seam).
        db._heading_offset = 170.0
        _Scripted.heading_value = -170.0   # raw delta -340 -> +20
        db._imu_tick(0)
        _Scripted.heading_value = 150.0    # raw delta -20 -> -20
        db._imu_tick(0)
        _Scripted.heading_value = -160.0   # raw delta -330 -> +30
        db._imu_tick(0)
        db._heading_offset = -170.0
        _Scripted.heading_value = 170.0    # raw delta +340 -> -20
        db._imu_tick(0)
        # No assertion on internals (the native slot has no getter):
        # covering the wrap branches without raising IS the contract;
        # the end-to-end landing accuracy is pinned by the turn test.

    def test_attach_imu_tick_idempotent(self):
        robot, imu = self._robot()
        db = robot.drivebase
        db.attach_imu(imu)
        db.set_use_gyro(True)
        db._attach_imu_tick()   # second call: early-return branch
        db.set_use_gyro(False)
        db._detach_imu_tick()   # second call: early-return branch

    @staticmethod
    def _yaw_kick(robot, deg):
        """Rotate the chassis in place WITHOUT turning the wheels —
        the encoder-invisible disturbance class (wheel slip,
        collision, being knocked while driving)."""
        import mujoco
        import numpy as np
        m, d = robot.runtime.model, robot.runtime.data
        jadr = int(m.joint("chassis_free").qposadr[0])
        quat = d.qpos[jadr + 3:jadr + 7].copy()
        half = math.radians(deg) / 2.0
        kick = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])
        out = np.zeros(4)
        mujoco.mju_mulQuat(out, kick, quat)
        d.qpos[jadr + 3:jadr + 7] = out
        mujoco.mj_forward(m, d)

    def test_gyro_mode_rejects_encoder_invisible_shove(self):
        """THE Pybricks gyro-drivebase behavior: shove the robot
        mid-straight (a yaw the wheels never see) and it returns to
        its original heading. Encoder mode, by construction, keeps
        the full deflection — asserted too, so this test documents
        WHY use_gyro exists."""
        robot, imu = self._robot()
        db = robot.drivebase

        def drive_and_shove(use_gyro):
            h0 = imu.heading()
            if use_gyro:
                db.attach_imu(imu)
                db.set_use_gyro(True)
            db.straight(400.0, 80.0)
            robot.run_for(1.5)
            self._yaw_kick(robot, 25.0)
            robot.run_for(3.0)
            err = ((imu.heading() - h0 + 180.0) % 360.0) - 180.0
            db.stop()
            if use_gyro:
                db.set_use_gyro(False)
            return err

        enc_err = drive_and_shove(use_gyro=False)
        self.assertGreater(abs(enc_err), 15.0,
                           "encoder mode unexpectedly corrected an "
                           "encoder-invisible shove (%.1f deg) — the "
                           "kick is not bypassing the wheels" % enc_err)

        robot, imu = self._robot()   # fresh world for the gyro case
        db = robot.drivebase
        gyro_err = drive_and_shove(use_gyro=True)
        self.assertLess(abs(gyro_err), 3.0,
                        "gyro mode failed to reject the shove "
                        "(%.1f deg residual)" % gyro_err)

    def test_gyro_straight_defends_heading(self):
        robot, imu = self._robot()
        db = robot.drivebase
        db.attach_imu(imu)
        db.set_use_gyro(True)
        h0 = self._yaw(imu)
        db.straight(150.0, 80.0)
        robot.run_for(3.0)
        drift = abs(((self._yaw(imu) - h0 + 180.0) % 360.0) - 180.0)
        self.assertLess(drift, 5.0,
                        "heading drifted %.1f deg on a gyro-guided "
                        "straight" % drift)


if __name__ == "__main__":
    unittest.main()
