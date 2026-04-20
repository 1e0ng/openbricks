# SPDX-License-Identifier: MIT
"""Tests for the declarative config loader."""

import tests._fakes  # noqa: F401

import json
import tempfile
import unittest

import openbricks.config as config
import openbricks.platforms.esp32 as esp32
import openbricks.robotics as robotics


SAMPLE_CONFIG = {
    "platform": "esp32",
    "i2c": {"i2c0": {"sda": 21, "scl": 22, "freq": 400000}},
    "uart": {"uart1": {"tx": 17, "rx": 16, "baud": 1000000}},
    "motors": {
        "left":  {"driver": "l298n", "in1": 25, "in2": 26, "pwm": 27},
        "right": {"driver": "l298n", "in1": 14, "in2": 12, "pwm": 13, "invert": True},
    },
    "sensors": {
        "imu":   {"driver": "bno055", "bus": "i2c0"},
        "color": {"driver": "tcs34725", "bus": "i2c0", "gain": 16},
    },
    "drivebase": {
        "left": "left", "right": "right",
        "wheel_diameter_mm": 65, "axle_track_mm": 120,
    },
}


class TestLoader(unittest.TestCase):
    def setUp(self):
        # Snapshot and replace hardware-touching bits with recording stubs.
        self._saved_registry = dict(config._DRIVER_REGISTRY)
        self._saved_drivebase = config.DriveBase
        self._saved_make_i2c = esp32.make_i2c
        self._saved_make_uart = esp32.make_uart

        self.calls = []

        def factory(name):
            def f(**kwargs):
                self.calls.append((name, kwargs))
                return ("instance_of", name, kwargs)
            return f

        for name in list(config._DRIVER_REGISTRY):
            config._DRIVER_REGISTRY[name] = factory(name)

        config.DriveBase = lambda left, right, **kw: ("drivebase", left, right, kw)
        robotics.DriveBase = config.DriveBase
        esp32.make_i2c = lambda **kw: ("i2c_handle", kw)
        esp32.make_uart = lambda **kw: ("uart_handle", kw)

        tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(SAMPLE_CONFIG, tmp)
        tmp.close()
        self.path = tmp.name

    def tearDown(self):
        config._DRIVER_REGISTRY.clear()
        config._DRIVER_REGISTRY.update(self._saved_registry)
        config.DriveBase = self._saved_drivebase
        robotics.DriveBase = self._saved_drivebase
        esp32.make_i2c = self._saved_make_i2c
        esp32.make_uart = self._saved_make_uart

    def test_loads_all_sections(self):
        robot = config.load_robot(self.path)
        self.assertIn("i2c0", robot.i2c)
        self.assertIn("uart1", robot.uart)
        self.assertEqual(set(robot.motors), {"left", "right"})
        self.assertEqual(set(robot.sensors), {"imu", "color"})
        self.assertEqual(robot.drivebase[0], "drivebase")

    def test_sensor_receives_bus_handle(self):
        config.load_robot(self.path)
        imu_call = next(c for c in self.calls if c[0] == "bno055")
        self.assertIn("bus", imu_call[1])
        self.assertEqual(imu_call[1]["bus"][0], "i2c_handle")

    def test_invert_passed_through(self):
        config.load_robot(self.path)
        right_call = [c for c in self.calls if c[1].get("in1") == 14][0]
        self.assertTrue(right_call[1]["invert"])

    def test_unknown_driver_raises(self):
        bad = dict(SAMPLE_CONFIG)
        bad["motors"] = {"nope": {"driver": "does_not_exist"}}
        tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(bad, tmp)
        tmp.close()
        with self.assertRaises(ValueError):
            config.load_robot(tmp.name)

    def test_unknown_bus_raises(self):
        bad = json.loads(json.dumps(SAMPLE_CONFIG))  # deep copy
        bad["sensors"]["imu"]["bus"] = "does_not_exist"
        tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(bad, tmp)
        tmp.close()
        with self.assertRaises(ValueError):
            config.load_robot(tmp.name)


if __name__ == "__main__":
    unittest.main()
