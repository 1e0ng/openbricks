# openbricks

[![firmware C](https://codecov.io/gh/1e0ng/openbricks/branch/main/graph/badge.svg?flag=c-core)](https://app.codecov.io/gh/1e0ng/openbricks?flags[0]=c-core)
[![firmware Python](https://codecov.io/gh/1e0ng/openbricks/branch/main/graph/badge.svg?flag=openbricks-py)](https://app.codecov.io/gh/1e0ng/openbricks?flags[0]=openbricks-py)
[![openbricks-dev](https://codecov.io/gh/1e0ng/openbricks/branch/main/graph/badge.svg?flag=openbricks-dev)](https://app.codecov.io/gh/1e0ng/openbricks?flags[0]=openbricks-dev)
[![openbricks-sim](https://codecov.io/gh/1e0ng/openbricks/branch/main/graph/badge.svg?flag=openbricks-sim)](https://app.codecov.io/gh/1e0ng/openbricks?flags[0]=openbricks-sim)

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
| JGB37-520 | DC gear motor with quadrature Hall encoder (closed loop via native `QuadratureEncoder` — GPIO IRQ) | `drivers.jgb37_520` |
| MG370 GMR | DC gear motor with 500-PPR GMR quadrature encoder (closed loop via native `PCNTEncoder` — ESP32 PCNT hardware counter) | `drivers.mg370` |
| BNO055 | 9-DOF IMU with onboard sensor fusion | `drivers.bno055` |
| TCS34725 | RGB + clear color sensor | `drivers.tcs34725` |
| HC-SR04 | Ultrasonic distance sensor (echo-pulse timing, 20–4000 mm) | `drivers.hcsr04` |
| VL53L0X | Laser ToF distance sensor (I2C, 30–2000 mm) | `drivers.vl53l0x` |
| VL53L1X | Laser ToF distance sensor (I2C, longer range, up to 4000 mm; VL53L4CD pin-compatible) | `drivers.vl53l1x` |
| ST-3215-C018 | Serial bus servo (FeeTech/SCServo protocol) | `drivers.st3215` |
| SSD1306 | 128×64 / 128×32 monochrome OLED display over I2C | `drivers.ssd1306` |

New drivers just need to implement one of the abstract interfaces in `openbricks/interfaces.py`. Drop a file into `openbricks/drivers/` and rebuild the firmware — users import it directly.

## Quick start

Once the firmware is flashed, `openbricks` is already importable at the REPL. Wire it up in Python, Pybricks-style:

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

robot = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
robot.straight(500)        # mm
robot.turn(90)             # degrees
print(color.rgb())         # (r, g, b) 0-255
print(imu.heading())       # degrees
```

See `examples/full_robot.py` for a longer end-to-end demo.

## Why openbricks (vs Pybricks)

Pybricks is the gold-standard MicroPython firmware for educational robotics — we modelled openbricks's API and three-layer architecture on it directly. Where openbricks differs:

- **Open hardware.** Pybricks runs on LEGO hubs with LEGO motors and LEGO sensors. openbricks runs on commodity ESP32 / ESP32-S3 boards driving any motor (L298N / TB6612 / JGB37-520 / MG370 / ST-3215), any IMU (BNO055), any I2C colour sensor (TCS34725), any I2C OLED (SSD1306). New driver = one Python file under `openbricks/drivers/`.
- **Hardware-accurate simulator (`openbricks-sim`).** A MuJoCo-backed sim with the *same C control cores* as the firmware — `*_core.c` files compile into both targets, so the sim's hot-path math is byte-identical to the hub's. Write your `main.py` once, test it in MuJoCo against a WRO mat, then flash. Pybricks has no comparable sim — closest equivalent is the LEGO virtual brick, which simulates the API but not the physics.
- **Driver shim — same script, both targets.** `openbricks-sim run main.py` installs a shim that replaces `machine`, `openbricks._native`, and the I2C driver classes (`TCS34725` etc.) with sim-aware versions. Code that imports `from openbricks.drivers.jgb37_520 import JGB37Motor` runs unchanged in MuJoCo. No "if simulator else hardware" branches in user code.
- **Slip-immune drivebase.** `DriveBase.use_gyro(True)` routes heading feedback through the IMU instead of the encoder differential — a robot that wheel-slips on a slippery patch keeps its course. The firmware C drivebase + sim adapter both honour it.
- **`openbricks log` for untethered runs.** Every program execution has its `print` output tee'd to `/openbricks_logs/run_N.log` on the hub (3 rotating slots, 64 KB each). When you run a program battery-only with no laptop attached, you can plug in afterwards and read what happened: `openbricks log -n RobotA`. Pybricks-dev has no equivalent — once the program ends, untethered output is gone.
- **C cores you can read.** The firmware's 1 kHz hot path lives in `native/user_c_modules/openbricks/*_core.c` — pbio-style trapezoidal trajectory, α-β observer, 2-DOF coupled drivebase, 1 kHz scheduler. About 1500 lines of straight C math; auditable. Pybricks's pbio is also open source — same shape.
- **Flashable firmware on every push to main.** [Rolling `latest` pre-release](../../releases/tag/latest) is rebuilt by CI per push; versioned `v*` tags get their own releases. `openbricks flash` writes the image and the hub's BLE name in one step.

## Status

Pbio-parity control is landed in C: always-on 1 kHz scheduler, trapezoidal trajectory planner, α-β state observer, 2-DOF coupled drivebase, and both quadrature encoders (software-IRQ and hardware-PCNT) all live as `user_c_modules` inside the firmware — the entire tick body is C, nothing on the hot path goes through a Python frame. Every bundled driver works end to end; the 136-test suite runs against the real C implementation under the unix MicroPython binary (no Python mirrors).

**Flashable firmware is published automatically**: every push to `main` updates a rolling [`latest` pre-release](../../releases/tag/latest), and every `v*` tag gets a versioned release. Download the `openbricks-<target>-firmware-<version>.bin` for your board and flash with `openbricks flash` (see `tools/openbricks-dev/README.md`) or `esptool.py` directly — details in `docs/build.md`.

Next on the roadmap: 1.0 polish + release (M5). The M4 hub abstraction (status LED, user button) and the SSD1306 OLED driver are both on `main`.

## License

MIT. See `LICENSE`.
