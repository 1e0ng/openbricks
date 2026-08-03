# SPDX-License-Identifier: MIT
"""Driver tests for ``openbricks.drivers.icm45686`` — stubbed native
module + NVS, both runtimes. The C layers have their own suites
(icm45686_core c-unit, imu_yaw_core c-unit, st_bus MP tests); these
own the Python boundary: config plumbing, unit conversions, the
hard-source marker, and calibration persistence."""

import tests._fakes  # noqa: F401

import sys
import unittest

# CPython has no _openbricks_native (the C module is MP-only);
# install the same minimal stand-in test_native_drivebase uses so
# the re-export shim loads. On unix MP the real module exists.
try:
    import _openbricks_native  # noqa: F401
except ImportError:
    class _Stub:
        pass

    class _StubMotorProcess:
        @staticmethod
        def reset():
            pass

        @staticmethod
        def stop():
            pass

        @staticmethod
        def hard_tick_selftest():
            return True

    _stub_mod = type(sys)("_openbricks_native")
    _stub_mod.motor_process = _StubMotorProcess()
    for _name in ("Servo", "TrapezoidalProfile", "Observer",
                  "DriveBase", "PCNTEncoder", "QuadratureEncoder",
                  "BNO055"):
        setattr(_stub_mod, _name, _Stub())
    sys.modules["_openbricks_native"] = _stub_mod


class _StubIcm:
    def __init__(self):
        self.cfg = None

    def config(self, **kw):
        self.cfg = kw

    def read(self):
        # (ax, ay, az, gx, gy, gz)
        return (0.0, 0.0, 1.0, 1.5, -2.5, 30.0)

    def stats(self):
        return (100, 2, True)


class _StubMP:
    def __init__(self):
        self.yaw = 42.5
        self.seeded = []
        self.resets = 0
        self.state = (0.8, True, 900.0)

    def hard_yaw_deg(self):
        return self.yaw

    def hard_yaw_reset(self):
        self.resets += 1

    def hard_yaw_state(self):
        return self.state

    def hard_yaw_seed_bias(self, b):
        self.seeded.append(b)


class _StubNVS:
    store = {}

    def __init__(self, ns):
        self.ns = ns

    def set_i32(self, key, val):
        _StubNVS.store[(self.ns, key)] = val

    def get_i32(self, key):
        try:
            return _StubNVS.store[(self.ns, key)]
        except KeyError:
            raise OSError(-4354)

    def commit(self):
        pass


class ICM45686DriverTests(unittest.TestCase):
    def setUp(self):
        from openbricks import _native
        _StubNVS.store = {}
        self.icm = _StubIcm()
        self.mp = _StubMP()
        self._had_icm = hasattr(_native, "icm45686")
        self._old_icm = getattr(_native, "icm45686", None)
        self._old_mp = _native.motor_process
        _native.icm45686 = self.icm
        _native.motor_process = self.mp

        class _Esp32:
            NVS = _StubNVS
        self._old_esp32 = sys.modules.get("esp32")
        sys.modules["esp32"] = _Esp32()

    def tearDown(self):
        from openbricks import _native
        _native.motor_process = self._old_mp
        if self._had_icm and self._old_icm is not None:
            _native.icm45686 = self._old_icm
        else:
            try:
                del _native.icm45686
            except AttributeError:
                pass
        if self._old_esp32 is None:
            try:
                del sys.modules["esp32"]
            except KeyError:
                pass
        else:
            sys.modules["esp32"] = self._old_esp32

    def _imu(self, **kw):
        from openbricks.drivers.icm45686 import ICM45686
        args = dict(sck=8, mosi=9, miso=17, cs=18)
        args.update(kw)
        return ICM45686(**args)

    def test_construction_configures_the_native_module(self):
        imu = self._imu()
        self.assertEqual(self.icm.cfg["sck"], 8)
        self.assertEqual(self.icm.cfg["cs"], 18)
        self.assertEqual(self.icm.cfg["hz"], 8_000_000)
        self.assertEqual(self.icm.cfg["mode"], 3)
        self.assertEqual(self.icm.cfg["scale"], -1.0)
        self.assertTrue(imu._hard_heading_source)

    def test_heading_and_sensors_route(self):
        imu = self._imu()
        self.assertEqual(imu.heading(), 42.5)
        self.assertEqual(imu.gyro(), (1.5, -2.5, 30.0))
        self.assertEqual(imu.acceleration(), (0.0, 0.0, 1.0))
        self.assertTrue(imu.calibrated())
        self.assertEqual(imu.stats(), (100, 2, True))
        imu.reset_heading()
        self.assertEqual(self.mp.resets, 1)

    def test_no_saved_calibration_means_no_seed(self):
        self._imu()
        self.assertEqual(self.mp.seeded, [])

    def test_calibration_round_trips_through_nvs(self):
        imu = self._imu()
        imu.save_calibration()             # bias 0.8 from the stub
        self.assertEqual(
            _StubNVS.store[("openbricks", "imu_bias_udps")], 800000)
        self.mp.seeded = []
        self._imu()                        # fresh boot: seeds
        self.assertEqual(len(self.mp.seeded), 1)
        self.assertTrue(abs(self.mp.seeded[0] - 0.8) < 1e-9)

    def test_save_without_lock_is_loud(self):
        imu = self._imu()
        self.mp.state = (0.0, False, 0.0)
        try:
            imu.save_calibration()
            self.fail("expected OSError")
        except OSError:
            pass

    def test_claimed_pin_is_rejected(self):
        # A pin already claimed by another role (off-hardware the
        # chip rules don't apply, but the claim registry does) must
        # refuse the wiring loudly.
        from openbricks import pins
        pins.claim(8, "program button", "test claim")
        try:
            self._imu(sck=8)
            self.fail("expected ValueError")
        except ValueError:
            pass
        finally:
            pins.release(8)


if __name__ == "__main__":
    unittest.main()
