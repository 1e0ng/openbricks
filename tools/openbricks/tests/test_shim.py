# SPDX-License-Identifier: MIT
"""Tests for the ``openbricks_sim.shim`` driver-shim layer.

Two surfaces:

  * ``install`` / ``uninstall`` lifecycle — ``sys.modules`` swaps and
    ``time.*`` patches are reversible.
  * The shim ``Servo`` / ``DriveBase`` classes accept the firmware's
    constructor signatures and route them onto a SimRuntime + chassis.

The integration test at the bottom drives the chassis via *real
firmware code* — by importing ``openbricks.drivers.jgb37_520.JGB37Motor``
and ``openbricks.robotics.drivebase.DriveBase`` after installing the
shim. That's the full "run firmware code unchanged in the sim"
scenario.
"""

import sys
import time
import unittest

from openbricks_sim.robot import SimRobot
from openbricks_sim import shim


class _ShimTestBase(unittest.TestCase):
    """Make sure each test starts from a clean state and the shim
    is uninstalled afterwards even when an assertion fails."""

    def setUp(self):
        if shim.is_installed():
            shim.uninstall()
        self.robot = SimRobot()
        shim.install(self.robot.runtime)

    def tearDown(self):
        shim.uninstall()


class InstallLifecycleTests(unittest.TestCase):

    def test_install_and_uninstall_round_trip_sys_modules(self):
        # Save baseline state.
        prev_machine = sys.modules.get("machine")
        prev_native  = sys.modules.get("_openbricks_native")

        robot = SimRobot()
        shim.install(robot.runtime)
        try:
            # Both fakes are now installed.
            self.assertIn("machine", sys.modules)
            self.assertIn("_openbricks_native", sys.modules)
            # And different from whatever was there before.
            import machine
            self.assertIs(machine, sys.modules["machine"])
            self.assertNotEqual(machine, prev_machine)
        finally:
            shim.uninstall()

        # Uninstall restores prior entries (or removes if absent).
        self.assertEqual(sys.modules.get("machine"), prev_machine)
        self.assertEqual(sys.modules.get("_openbricks_native"), prev_native)

    def test_install_twice_raises(self):
        robot = SimRobot()
        shim.install(robot.runtime)
        try:
            with self.assertRaises(RuntimeError):
                shim.install(robot.runtime)
        finally:
            shim.uninstall()

    def test_uninstall_when_not_installed_is_noop(self):
        # Should not raise even when nothing's installed.
        shim.uninstall()
        self.assertFalse(shim.is_installed())

    def test_install_patches_time_sleep_ms_to_advance_sim(self):
        robot = SimRobot()
        shim.install(robot.runtime)
        try:
            self.assertEqual(robot.runtime.now_ms, 0)
            time.sleep_ms(50)   # patched: 50 ms of sim time
            self.assertEqual(robot.runtime.now_ms, 50)
        finally:
            shim.uninstall()
        # After uninstall, time.sleep_ms is restored to its previous
        # state — typically nonexistent on CPython.
        self.assertFalse(hasattr(time, "sleep_ms"),
                          "uninstall should remove the sleep_ms patch")


class MotorSlotAllocationTests(_ShimTestBase):

    def test_first_servo_binds_left_second_binds_right(self):
        from _openbricks_native import Servo
        s_left  = Servo(in1=12, in2=14, pwm=27, encoder=None,
                        counts_per_rev=1320, kp=0.3)
        s_right = Servo(in1=13, in2=15, pwm=26, encoder=None,
                        counts_per_rev=1320, kp=0.3)
        # The shim allocates chassis_motor_l → first, _r → second.
        self.assertNotEqual(s_left._adapter._actuator_id,
                             s_right._adapter._actuator_id)

    def test_third_servo_construction_raises(self):
        from _openbricks_native import Servo
        Servo(in1=12, in2=14, pwm=27, encoder=None)
        Servo(in1=13, in2=15, pwm=26, encoder=None)
        with self.assertRaises(RuntimeError):
            Servo(in1=99, in2=98, pwm=97, encoder=None)


class ShimServoBehaviourTests(_ShimTestBase):

    def test_run_speed_drives_actuator(self):
        from _openbricks_native import Servo
        s = Servo(in1=12, in2=14, pwm=27, encoder=None)
        s.run_speed(180.0)
        # Step a few times so the controller writes a non-zero ctrl.
        for _ in range(10):
            self.robot.runtime.step()
        adapter = s._adapter
        ctrl = float(self.robot.data.ctrl[adapter._actuator_id])
        self.assertGreater(abs(ctrl), 0.0,
                            "run_speed should drive the actuator")

    def test_run_target_completes_via_sleep_busy_wait(self):
        # Mirrors the firmware's JGB37Motor.run_angle wait pattern.
        from _openbricks_native import Servo
        s = Servo(in1=12, in2=14, pwm=27, encoder=None,
                  counts_per_rev=360, kp=0.0)   # open-loop trajectory
        s.run_target(delta_deg=90.0, cruise_dps=180.0, accel=720.0)
        # Drive the wait loop the same way openbricks's wrappers do.
        deadline = 0
        while not s.is_done() and deadline < 5000:
            time.sleep_ms(10)   # patched: advances sim
            deadline += 10
        self.assertTrue(s.is_done())

    def test_brake_detaches(self):
        from _openbricks_native import Servo
        s = Servo(in1=12, in2=14, pwm=27, encoder=None)
        s.run_speed(100.0)
        self.assertTrue(s._adapter._attached)
        s.brake()
        self.assertFalse(s._adapter._attached)


class ShimDriveBaseTests(_ShimTestBase):

    def test_construct_with_shim_servos(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        self.assertTrue(db.is_done())   # idle at construction

    def test_construct_with_non_shim_servo_raises(self):
        from _openbricks_native import DriveBase
        with self.assertRaises(TypeError):
            DriveBase(left="not a servo", right=None,
                       wheel_diameter_mm=60, axle_track_mm=150)

    def test_straight_via_busy_wait(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        db.straight(50.0, 80.0)
        deadline = 0
        while not db.is_done() and deadline < 5000:
            time.sleep_ms(10)
            deadline += 10
        self.assertTrue(db.is_done())

    def test_construction_resizes_chassis_wheels_to_match_dims(self):
        # The user's robot.py is the same script the firmware runs;
        # ``DriveBase(wheel_diameter_mm=80, axle_track_mm=200)`` is
        # the single source of truth for chassis dims. Pin: at
        # ShimDriveBase construction time, the sim model's wheel geom
        # gets resized to the user's wheel_diameter_mm and the wheel
        # bodies are repositioned for axle_track_mm. Without this,
        # encoders rotate a default-size wheel while the user's
        # odometry math thinks they're rotating a different-size wheel.
        import mujoco
        from _openbricks_native import Servo, DriveBase

        m = self.robot.model
        wl_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis_wheel_l")
        wr_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis_wheel_r")
        # Find each wheel body's cylinder geom (one per body).
        def _wheel_geom(bid):
            for gid in range(m.ngeom):
                if int(m.geom_bodyid[gid]) == bid:
                    return gid
            raise AssertionError("no geom found for body %d" % bid)
        wl_gid = _wheel_geom(wl_id)
        wr_gid = _wheel_geom(wr_id)

        # Construct DriveBase with non-default dims.
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        DriveBase(left=l, right=r,
                  wheel_diameter_mm=80.0,    # default is 60 mm
                  axle_track_mm=200.0)       # default is 150 mm

        # Wheel cylinder radius in metres = 80 / 2000 = 0.040.
        self.assertAlmostEqual(float(m.geom_size[wl_gid, 0]), 0.040, delta=1e-6)
        self.assertAlmostEqual(float(m.geom_size[wr_gid, 0]), 0.040, delta=1e-6)
        # Axle Y on each side = ±200 / 2000 = ±0.100 m.
        self.assertAlmostEqual(float(m.body_pos[wl_id, 1]), +0.100, delta=1e-6)
        self.assertAlmostEqual(float(m.body_pos[wr_id, 1]), -0.100, delta=1e-6)

    def test_use_gyro_without_imu_raises(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        with self.assertRaises(RuntimeError):
            db.use_gyro(True)

    def test_use_gyro_with_imu_installs_imu_tick(self):
        from _openbricks_native import Servo, DriveBase, BNO055
        imu = BNO055(i2c=None, address=0x28)
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0,
                        imu=imu)
        # Toggling on captures the heading offset and installs the
        # imu tick. Subsequent steps must not crash.
        db.use_gyro(True)
        for _ in range(10):
            self.robot.runtime.step()
        # And toggling off restores the encoder-differential path
        # without leaving the imu tick behind.
        db.use_gyro(False)
        for _ in range(10):
            self.robot.runtime.step()

    def test_use_gyro_false_is_always_allowed(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        # Should NOT raise — turning gyro feedback off is the default
        # state and never depends on an IMU.
        db.use_gyro(False)


class MachineFakeTests(_ShimTestBase):
    """Verify the ``machine`` fake covers the interfaces openbricks
    drivers actually instantiate."""

    def test_pin_pwm_construct_no_args_no_kwargs(self):
        import machine
        machine.Pin(0)
        machine.PWM(machine.Pin(1), freq=20_000, duty=0)
        machine.I2C(0, sda=21, scl=22, freq=400_000)

    def test_pin_value_returns_int(self):
        import machine
        p = machine.Pin(0, machine.Pin.OUT, value=0)
        self.assertEqual(p.value(), 0)
        p.value(1)   # write — accepted, return ignored
        # Reads return 0 by convention (no-op fake).


class FullFirmwareCodeIntegrationTest(_ShimTestBase):
    """End-to-end: import the *real* openbricks driver classes through
    the shim, construct them with hardware-style pin numbers, and drive
    a straight move. If this passes, firmware-targeting user code can
    run unchanged inside the sim."""

    def test_jgb37_drivebase_straight(self):
        # Add the openbricks package to sys.path the same way
        # shim.install does (for our module-level ``openbricks`` import
        # below). install() already did this — verify the import works.
        from openbricks.drivers.jgb37_520 import JGB37Motor
        from openbricks.robotics.drivebase import DriveBase

        m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                              encoder_a=18, encoder_b=19)
        m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                              encoder_a=20, encoder_b=21)
        db = DriveBase(m_left, m_right,
                        wheel_diameter_mm=60, axle_track_mm=150)
        # The drivebase wrapper at the openbricks side sets cruise via
        # settings(); use the default 200 dps.
        db.settings(straight_speed=180, turn_rate=120)
        # Blocking call: straight() busy-waits on time.sleep_ms which
        # the shim patched to step the sim. So this returns when the
        # native trajectory is done.
        db.straight(50)
        # After return, the chassis should have translated some +X.
        x_mm, _, _ = self.robot.chassis_pose()
        self.assertGreater(x_mm, 5.0,
                            "drivebase.straight should have moved the "
                            "chassis +X (got x=%.1f mm)" % x_mm)


if __name__ == "__main__":
    unittest.main()
