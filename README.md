# openbricks

> A Pybricks-style MicroPython robotics library for **open hardware** — bring your own MCU, your own motors, your own sensors.

Pybricks gives LEGO users a delightful Python API, but it only runs on a handful of LEGO hubs with LEGO-branded motors and sensors. `openbricks` borrows the three-layer architecture (drivers → abstract interfaces → robotics) and applies it to commodity components you can buy off the shelf.

## What's in the box

**Supported MCUs out of the box**

- ESP32 (primary target)
- Any MicroPython port that exposes `machine.Pin`, `machine.PWM`, `machine.I2C`, `machine.UART` should work with minor adjustments.

**Bundled component drivers**

| Component | Type | Driver module |
|-----------|------|---------------|
| L298N H-bridge | DC motor driver (open loop) | `drivers.l298n` |
| JGB37-520 | DC gear motor with quadrature encoder (closed loop, via L298N) | `drivers.jgb37_520` |
| BNO055 | 9-DOF IMU with onboard sensor fusion | `drivers.bno055` |
| TCS34725 | RGB + clear color sensor | `drivers.tcs34725` |
| ST-3215-C018 | Serial bus servo (FeeTech/SCServo protocol) | `drivers.st3215` |

New drivers just need to implement one of the abstract interfaces in `openbricks/interfaces.py`. Drop a file into `openbricks/drivers/`, register it in `openbricks/config.py`, and the config loader will pick it up.

## Quick start

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

This is an early scaffold. Core architecture is in place and simple drivers (L298N, TCS34725, BNO055 read-only, ST-3215 basic protocol) work end to end. Closed-loop motor control is intentionally minimal — a trapezoidal profile with encoder feedback. Production-quality control (state observer, trajectory planning à la Pybricks' `pbio`) is future work; see `docs/architecture.md` for where it would slot in.

## License

MIT. See `LICENSE`.
