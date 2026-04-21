# SPDX-License-Identifier: MIT
"""
Declarative robot configuration.

Given a JSON file, instantiate all the components and wire them up into a
``Robot`` object. This is the "plug and play" layer: users describe *what*
hardware they have, not *how* to initialize it.

The registry is just a dict of driver name -> factory callable. To add a new
driver, either edit ``_DRIVER_REGISTRY`` below, or call ``register_driver()``
from your user code before calling ``load_robot()``.

Supported config schema (all sections optional):

    {
        "platform": "esp32",
        "i2c": { "i2c0": {"sda": 21, "scl": 22, "freq": 400000} },
        "uart": { "uart1": {"tx": 17, "rx": 16, "baud": 1000000} },
        "motors":  { "<name>": {"driver": "<name>", ...driver kwargs} },
        "servos":  { "<name>": {"driver": "<name>", ...driver kwargs} },
        "sensors": { "<name>": {"driver": "<name>", ...driver kwargs} },
        "drivebase": {
            "left": "<motor name>",
            "right": "<motor name>",
            "wheel_diameter_mm": 56,
            "axle_track_mm": 114
        }
    }
"""

import json

from openbricks.robotics import DriveBase


# ---------- driver registry ----------

def _make_l298n(**kwargs):
    from openbricks.drivers.l298n import L298NMotor
    return L298NMotor(**kwargs)


def _make_jgb37(**kwargs):
    from openbricks.drivers.jgb37_520 import JGB37Motor
    return JGB37Motor(**kwargs)


def _make_mg370(**kwargs):
    from openbricks.drivers.mg370 import MG370Motor
    return MG370Motor(**kwargs)


def _make_bno055(bus, address=0x28, **_ignored):
    from openbricks.drivers.bno055 import BNO055
    return BNO055(bus, address=address)


def _make_tcs34725(bus, address=0x29, integration_ms=24, gain=4, **_ignored):
    from openbricks.drivers.tcs34725 import TCS34725
    return TCS34725(bus, address=address, integration_ms=integration_ms, gain=gain)


def _make_st3215(**kwargs):
    from openbricks.drivers.st3215 import ST3215
    return ST3215(**kwargs)


_DRIVER_REGISTRY = {
    "l298n":    _make_l298n,
    "jgb37_520": _make_jgb37,
    "mg370":    _make_mg370,
    "bno055":   _make_bno055,
    "tcs34725": _make_tcs34725,
    "st3215":   _make_st3215,
}


def register_driver(name, factory):
    """Register a new driver by name. ``factory`` is called with the kwargs
    from the config section, plus any resolved bus handle."""
    _DRIVER_REGISTRY[name] = factory


# ---------- loader ----------

class Robot:
    """Holds everything the config describes, accessible by name."""

    def __init__(self):
        self.motors = {}
        self.servos = {}
        self.sensors = {}
        self.i2c = {}
        self.uart = {}
        self.drivebase = None


def load_robot(path):
    with open(path, "r") as f:
        cfg = json.load(f)

    robot = Robot()

    # --- buses ---
    for name, params in cfg.get("i2c", {}).items():
        robot.i2c[name] = _make_i2c(**params)
    for name, params in cfg.get("uart", {}).items():
        robot.uart[name] = _make_uart(**params)

    # --- motors ---
    for name, params in cfg.get("motors", {}).items():
        robot.motors[name] = _instantiate(params)

    # --- servos ---
    for name, params in cfg.get("servos", {}).items():
        robot.servos[name] = _instantiate(params)

    # --- sensors (may need a bus reference) ---
    for name, params in cfg.get("sensors", {}).items():
        params = dict(params)
        bus_name = params.pop("bus", None)
        kwargs = {k: v for k, v in params.items() if k != "driver"}
        factory = _DRIVER_REGISTRY[params["driver"]]
        if bus_name is not None:
            bus = robot.i2c.get(bus_name) or robot.uart.get(bus_name)
            if bus is None:
                raise ValueError("sensor %r refers to unknown bus %r" % (name, bus_name))
            robot.sensors[name] = factory(bus=bus, **kwargs)
        else:
            robot.sensors[name] = factory(**kwargs)

    # --- drivebase ---
    db_cfg = cfg.get("drivebase")
    if db_cfg:
        left = robot.motors[db_cfg["left"]]
        right = robot.motors[db_cfg["right"]]
        robot.drivebase = DriveBase(
            left, right,
            wheel_diameter_mm=db_cfg["wheel_diameter_mm"],
            axle_track_mm=db_cfg["axle_track_mm"],
        )

    return robot


def _instantiate(params):
    params = dict(params)
    driver = params.pop("driver")
    factory = _DRIVER_REGISTRY.get(driver)
    if factory is None:
        raise ValueError("unknown driver: %r" % driver)
    return factory(**params)


def _make_i2c(sda, scl, freq=400_000, bus_id=0):
    from openbricks.platforms.esp32 import make_i2c
    return make_i2c(bus_id=bus_id, sda=sda, scl=scl, freq=freq)


def _make_uart(tx, rx, baud=1_000_000, bus_id=1):
    from openbricks.platforms.esp32 import make_uart
    return make_uart(bus_id=bus_id, tx=tx, rx=rx, baud=baud)
