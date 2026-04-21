# openbricks

![coverage](https://img.shields.io/badge/coverage-86%25-green)

> A Pybricks-style MicroPython firmware for **open hardware** — commodity MCUs, commodity motors, commodity sensors.

Pybricks gives LEGO users a delightful Python API, but it only runs on a handful of LEGO hubs with LEGO-branded motors and sensors. `openbricks` takes the same shape — a custom MicroPython firmware that bakes the robotics library into the runtime — and targets commodity components you can buy off the shelf.

Like Pybricks, openbricks is a **firmware you flash to an MCU**, not a library you `pip install` on top of stock MicroPython. That means we own the runtime: background control loops, hardware timers, and native C extensions all live inside the firmware image. The motor scheduler and per-motor state machine are already in C (see `native/user_c_modules/openbricks/`); the state observer, trapezoidal trajectory planner, and 2-DOF drivebase controller get the same treatment in upcoming milestones. The three-layer API you write code against (drivers → abstract interfaces → robotics) is what `import openbricks` gives you out of the box.

## What's in the box

**Target platforms for the firmware build**

- ESP32 (Xtensa LX6) — primary target
- ESP32-S3 (Xtensa LX7, native USB) — first-class, built alongside ESP32 in CI

Each platform ships as a separate firmware image.

**Bundled component drivers**

| Component | Type | Driver module |
|-----------|------|---------------|
| L298N H-bridge | DC motor driver (open loop) | `drivers.l298n` |
| TB6612FNG H-bridge | MOSFET H-bridge, 3.3 V-logic, drop-in IN1/IN2/PWM (alias of `L298NMotor`) | `drivers.tb6612` |
| JGB37-520 | DC gear motor with quadrature encoder (closed loop, via L298N) | `drivers.jgb37_520` |
| BNO055 | 9-DOF IMU with onboard sensor fusion | `drivers.bno055` |
| TCS34725 | RGB + clear color sensor | `drivers.tcs34725` |
| ST-3215-C018 | Serial bus servo (FeeTech/SCServo protocol) | `drivers.st3215` |

New drivers just need to implement one of the abstract interfaces in `openbricks/interfaces.py`. Drop a file into `openbricks/drivers/`, register it in `openbricks/config.py`, and rebuild the firmware.

## Quick start

Once the firmware is flashed, `openbricks` is already importable at the REPL:

**Option 1 — Wire it up in Python:**

```python
from machine import I2C, Pin
from openbricks.drivers.l298n import L298NMotor
from openbricks.drivers.bno055 import BNO055
from openbricks.drivers.tcs34725 import TCS34725
from openbricks.robotics import DriveBase

i2c   = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)
left  = L298NMotor(in1=25, in2=26, pwm=27)
right = L298NMotor(in1=14, in2=12, pwm=13)
imu   = BNO055(i2c)
color = TCS34725(i2c)

db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
db.straight(500)           # mm
db.turn(90)                # degrees
print(color.rgb())         # (r, g, b) 0-255
print(imu.heading())       # degrees
```

**Option 2 — Declarative config:**

Drop a `robot.json` next to `main.py`:

```json
{
  "platform": "esp32",
  "i2c": { "i2c0": { "sda": 21, "scl": 22, "freq": 400000 } },
  "motors": {
    "left":  { "driver": "l298n", "in1": 25, "in2": 26, "pwm": 27 },
    "right": { "driver": "l298n", "in1": 14, "in2": 12, "pwm": 13 }
  },
  "sensors": {
    "imu":   { "driver": "bno055",   "bus": "i2c0" },
    "color": { "driver": "tcs34725", "bus": "i2c0" }
  },
  "drivebase": {
    "left": "left", "right": "right",
    "wheel_diameter_mm": 56, "axle_track_mm": 114
  }
}
```

Then:

```python
from openbricks.config import load_robot
robot = load_robot("robot.json")

robot.drivebase.straight(500)
print(robot.sensors["color"].rgb())
```

## Status

Pbio-parity control is landed in C: always-on 1 kHz scheduler, trapezoidal trajectory planner, α-β state observer, and 2-DOF coupled drivebase all live as `user_c_modules` inside the firmware. Every bundled driver works end to end; the 85-test suite runs against the real C implementation under the unix MicroPython binary (no Python mirrors).

**Flashable firmware is published automatically**: every push to `main` updates a rolling [`latest` pre-release](../../releases/tag/latest), and every `v*` tag gets a versioned release. Grab `openbricks-esp32-firmware.bin` and flash with `esptool.py` — see `docs/build.md`.

Next on the roadmap: 1.0 polish + release (M5). The M4 hub abstraction (status LED, user button) and the SSD1306 OLED driver are both on `main`.

## License

MIT. See `LICENSE`.
