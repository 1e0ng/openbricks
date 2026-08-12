# openbricks

| Surface | Coverage |
|---|---|
| firmware C (`native/user_c_modules`) | [![c-core](https://codecov.io/gh/1e0ng/openbricks/branch/main/graph/badge.svg?flag=c-core)](https://app.codecov.io/gh/1e0ng/openbricks?flags[0]=c-core) |
| firmware Python (`openbricks/`) | [![openbricks-py](https://codecov.io/gh/1e0ng/openbricks/branch/main/graph/badge.svg?flag=openbricks-py)](https://app.codecov.io/gh/1e0ng/openbricks?flags[0]=openbricks-py) |
| host CLI (`openbricks_dev`) | [![openbricks-dev](https://codecov.io/gh/1e0ng/openbricks/branch/main/graph/badge.svg?flag=openbricks-dev)](https://app.codecov.io/gh/1e0ng/openbricks?flags[0]=openbricks-dev) |
| host sim (`openbricks_sim`) | [![openbricks-sim](https://codecov.io/gh/1e0ng/openbricks/branch/main/graph/badge.svg?flag=openbricks-sim)](https://app.codecov.io/gh/1e0ng/openbricks?flags[0]=openbricks-sim) |

> A Pybricks-style MicroPython firmware for **open hardware** — commodity MCUs, commodity motors, commodity sensors.

**Website**: <https://openbricks.dev> · **Documentation**: <https://docs.openbricks.dev>

Pybricks gives LEGO users a delightful Python API, but it only runs on a handful of LEGO hubs with LEGO-branded motors and sensors. `openbricks` takes the same shape — a custom MicroPython firmware that bakes the robotics library into the runtime — and targets commodity components you can buy off the shelf.

Like Pybricks, openbricks is a **firmware you flash to an MCU**, not a library you `pip install` on top of stock MicroPython. That means we own the runtime: background control loops, hardware timers, and native C extensions all live inside the firmware image. The entire 1 kHz hot path is C (see `native/user_c_modules/openbricks/`): motor scheduler, per-motor state machine, state observer, trapezoidal trajectory planner, and 2-DOF drivebase controller. The three-layer API you write code against (drivers → abstract interfaces → robotics) is what `import openbricks` gives you out of the box.

## What's in the box

**Target platforms for the firmware build**

- ESP32-S3 (Xtensa LX7, native USB) — primary target
- ESP32 (Xtensa LX6) — first-class, built alongside ESP32-S3 in CI

Each platform ships as a separate firmware image.

**Bundled component drivers**

| Component | Type | Driver module |
|-----------|------|---------------|
| ST-3032 (Feetech STS3032, 12 V) | Serial bus servo — the recommended drive motor (wheel mode + `DriveBase`) | `drivers.st3032` |
| ST-3215-C018 | Serial bus servo (FeeTech/SCServo protocol) — slower, more torque; arms / grippers | `drivers.st3215` |
| TCS34725 | RGB + clear color sensor | `drivers.tcs34725` |
| TCA9548A | 8-channel I2C multiplexer — run several fixed-address sensors (e.g. a TCS34725 array) on one bus | `drivers.tca9548a` |
| ICM-45686 | 6-axis IMU over SPI — read inside the 1 kHz control tick; the heading source for `use_gyro` | `drivers.icm45686` |
| BNO055 | 9-DOF IMU (legacy I2C option — new builds should use the ICM-45686) | `drivers.bno055` |
| QTRX-HD-15A | 10-channel reflectance bar — `QTRLineSensor` line following / edge alignment | `drivers.qtr` |
| HC-SR04 | Ultrasonic distance sensor (echo-pulse timing, 20–4000 mm) | `drivers.hcsr04` |
| VL53L0X | Laser ToF distance sensor (I2C, 30–2000 mm) | `drivers.vl53l0x` |
| VL53L1X | Laser ToF distance sensor (I2C, longer range, up to 4000 mm; VL53L4CD pin-compatible) | `drivers.vl53l1x` |
| SSD1306 | 128×64 / 128×32 monochrome OLED display over I2C | `drivers.ssd1306` |
| WS2812 / WS2812B | Addressable RGB LED strip / stick (e.g. the ×8 module), single data GPIO | `drivers.ws2812` |
| JGB37-520 | DC gear motor with quadrature Hall encoder (closed loop via native `QuadratureEncoder` — GPIO IRQ) | `drivers.jgb37_520` |
| MG370 GMR | DC gear motor with 500-PPR GMR quadrature encoder (closed loop via native `PCNTEncoder` — ESP32 PCNT hardware counter) | `drivers.mg370` |
| L298N H-bridge | DC motor driver (open loop) | `drivers.l298n` |
| TB6612FNG H-bridge | MOSFET H-bridge, 3.3 V-logic, drop-in IN1/IN2/PWM (alias of `L298NMotor`) | `drivers.tb6612` |

New drivers just need to implement one of the abstract interfaces in `openbricks/interfaces.py`. Drop a file into `openbricks/drivers/` and rebuild the firmware — users import it directly.

## Quick start

### Install the host tooling

```
pipx install 'openbricks[sim]'   # CLI + MuJoCo simulator
# or, lighter:
pipx install openbricks          # CLI only (flash / run / log)
```

(`pip install` works too; `pipx` is recommended on modern macOS / Linux to avoid the "externally managed environment" error.) The package is on PyPI: <https://pypi.org/project/openbricks/>.

### Flash the firmware

Connect the board over USB, then flash and name the hub in one step:

```
openbricks flash --name RobotA
```

That's the whole command: the serial port is auto-detected (when exactly one ESP device is connected), the chip type is probed, and the newest matching firmware image is downloaded from [Releases](../../releases) automatically (cached under `~/.cache/openbricks/firmware`). `--name` is the BLE advertising identifier you'll use later with `openbricks run -n …`; pick a unique one per hub. With several serial devices connected pass `--port` explicitly (`/dev/ttyUSB0` on Linux, `/dev/cu.usbserial-*` on macOS, `COM5` on Windows), and pass `--firmware path/to/firmware.bin` to flash a specific image instead of the newest release. Skip this step entirely if you only want to run code in the simulator.

### Drive a robot

Once the firmware is flashed, write a `main.py` and push it to the hub:

```python
# main.py — ESP32-S3 pins; see docs/hardware.md for the wiring
from machine import I2C, Pin
from openbricks.drivers.icm45686 import ICM45686
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.drivers.tca9548a import TCA9548A
from openbricks.drivers.tcs34725 import TCS34725
from openbricks.robotics import DriveBase

# Two ST-3032 serial bus servos, daisy-chained on one URT-2 adapter.
left  = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
right = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41, invert=True)

# Colour sensor array through the mux (TCS34725 is fixed at 0x29, so
# more than one needs the TCA9548A); the IMU is on SPI, read inside
# the firmware's 1 kHz control tick.
i2c     = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)
mux     = TCA9548A(i2c)
sensors = [TCS34725(mux[ch]) for ch in range(2)]
imu     = ICM45686(sck=12, mosi=13, miso=11, cs=17)

robot = DriveBase(left, right, wheel_diameter_mm=88, axle_track_mm=136,
                  imu=imu)
robot.use_gyro(True)               # heading corrected at 1 kHz in C
robot.settings(acceleration=180)   # soften the launch (wheel-deg/s²; default 1500)
robot.straight(500)                # mm
robot.turn(90)                     # degrees
print([s.rgb() for s in sensors])  # [(r, g, b), ...] 0-255
print(imu.heading())               # degrees
```

Then run it:

```
openbricks run -n RobotA main.py             # on a real hub over BLE
```

To iterate without hardware, the MuJoCo simulator runs the same firmware APIs — `openbricks sim run main.py --world empty`, including the `main.py` above. The sim's driver shim replaces `machine`, `openbricks._native`, and the driver classes (serial-bus `ST3032Motor` / `ST3215Motor`, encoder-motor `JGB37Motor`, and the I2C sensors) with sim-aware versions, so the script runs unchanged; bus/pin arguments are ignored as wiring concerns. See `tools/openbricks/examples/` for sim samples and `tools/openbricks/CHANGELOG.md` for the migration story if you're coming from the legacy `openbricks-dev` / `openbricks-sim` packages.

## Why openbricks (vs Pybricks)

Pybricks is the gold-standard MicroPython firmware for educational robotics — we modelled openbricks's API and three-layer architecture on it directly. Where openbricks differs:

- **Open hardware.** Pybricks runs on LEGO hubs with LEGO motors and LEGO sensors. openbricks runs on commodity ESP32-S3 / ESP32 boards driving any motor (ST-3032 / ST-3215 serial bus servos, or JGB37-520 / MG370 / L298N / TB6612 DC stacks), an ICM-45686 IMU read at 1 kHz inside the control tick, any I2C colour sensor (TCS34725 — arrays of them via the TCA9548A mux), a 10-channel QTR reflectance bar, any I2C OLED (SSD1306). New driver = one Python file under `openbricks/drivers/`.
- **Hardware-accurate simulator (`openbricks sim`).** A MuJoCo-backed sim with the *same C control cores* as the firmware — `*_core.c` files compile into both targets, so the sim's hot-path math is byte-identical to the hub's. Write your `main.py` once, test it in MuJoCo against a WRO mat, then flash. Pybricks has no comparable sim — closest equivalent is the LEGO virtual brick, which simulates the API but not the physics.
- **Driver shim — same script, both targets.** `openbricks sim run main.py` installs a shim that replaces `machine`, `openbricks._native`, and the I2C driver classes (`TCS34725` etc.) with sim-aware versions. Code that imports `from openbricks.drivers.jgb37_520 import JGB37Motor` runs unchanged in MuJoCo. No "if simulator else hardware" branches in user code.
- **Slip-immune drivebase.** `DriveBase.use_gyro(True)` routes heading feedback through the IMU instead of the encoder differential — a robot that wheel-slips on a slippery patch keeps its course. The firmware C drivebase + sim adapter both honour it.
- **`openbricks log` for untethered runs.** Every program execution has its `print` output tee'd to `/openbricks_logs/run_N.log` on the hub (3 rotating slots, 64 KB each). When you run a program battery-only with no laptop attached, you can plug in afterwards and read what happened: `openbricks log -n RobotA`. Pybricks-dev has no equivalent — once the program ends, untethered output is gone.
- **C cores you can read.** The firmware's 1 kHz hot path lives in `native/user_c_modules/openbricks/*_core.c` — pbio-style trapezoidal trajectory, α-β observer, 2-DOF coupled drivebase, 1 kHz scheduler. About 1500 lines of straight C math; auditable. Pybricks's pbio is also open source — same shape.
- **Flashable firmware on every push to main.** [Rolling `latest` pre-release](../../releases/tag/latest) is rebuilt by CI per push; versioned `v*` tags get their own releases. `openbricks flash` writes the image and the hub's BLE name in one step.

## Status

**All foundational milestones (M0–M5) shipped.** Pbio-parity control is landed in C: always-on 1 kHz scheduler, trapezoidal trajectory planner, α-β state observer, 2-DOF coupled drivebase, and both quadrature encoders (software-IRQ and hardware-PCNT) all live as `user_c_modules` inside the firmware — the entire tick body is C, nothing on the hot path goes through a Python frame. The hub abstraction (status LED, user button) and SSD1306 OLED driver round out the on-hub user surface; ESP32 + ESP32-S3 firmware images both build from the same codebase. Every bundled driver works end to end; the test suite runs against the real C implementation under the unix MicroPython binary (no Python mirrors).

**Flashable firmware is published automatically**: every push to `main` updates a rolling [`latest` pre-release](../../releases/tag/latest), and every `v*` tag gets a versioned release. Download the `openbricks-<target>-firmware-<version>.bin` for your board and flash with `openbricks flash` (see `tools/openbricks/README.md`) or `esptool.py` directly — details in `docs/build.md`.

The host-side simulator (`openbricks sim`) shares the same C control cores with the firmware via `*_core.c` files compiled into both targets, so sim runs are byte-identical to hardware on the hot path. Pixel-accurate colour-sensor texture sampling on the WRO mat is on `main` (Phase E1) — driving across a printed map reads the actual printed colours, not the material's flat tint. Future work: model-based observer (precision lift, picked up once we have hardware to measure against) and full MuJoCo offscreen rendering (only relevant when scenes grow shadows / lighting / overlays beyond a flat printed mat).

## License

MIT. See `LICENSE`.
