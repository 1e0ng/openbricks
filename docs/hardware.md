# Hardware guide

A starter parts list and wiring notes. Everything here is commodity stuff you
can buy on AliExpress / Amazon / Adafruit.

## Recommended starter robot

| Part | Qty | Notes |
|------|----:|-------|
| ESP32-S3 DevKitC-1 | 1 | Any ESP32-S3 board with ≥10 free GPIOs works; classic ESP32 also supported |
| Feetech STS3032 serial bus servo (12V) | 2 | Drive wheels, continuous-rotation wheel mode. 10 kg·cm, 148 RPM no-load — datasheet in `docs/datasheets/` |
| URT-2 serial bus adapter | 1 | One half-duplex bus daisy-chains every servo |
| 3S LiPo battery (11.1V nominal) + balance charger | 1 | Feeds the servo rail directly. ST-3032 brown-out floor is ~9V, so a sagging 2S (7.4V) is not enough |
| Buck converter (12V → 5V, ≥1A) | 1 | Powers the ESP32-S3 and sensors |
| TCS34725 breakout | 2–4 | Colour sensor array for line following / zone detection |
| TCA9548A I2C multiplexer breakout | 1 | Required for more than one TCS34725 — its address is fixed at `0x29` |
| BNO055 breakout | 1 | Adafruit, Adafruit-compatible, or BNO085 with driver changes |
| ST-3215 serial bus servo | 0–4 | Optional; good for arms / grippers. Same bus and protocol, but do **not** share a 6V rail setup — see servo notes |
| Jumper wires, M3 standoffs, chassis plate | — | Your robot, your build |

A DC-gear-motor build (JGB37-520 / MG370 + H-bridge) is still fully
supported — see [Alternative: DC gear motors with encoders](#alternative-dc-gear-motors-with-encoders)
at the bottom.

## Power budget

Two rails, one battery:

```
    [ 3S LiPo 11.1V ]
        │
        ├─────────────►  URT-2 servo rail   (all ST-3032 / ST-3215 power)
        │
        └──► Buck  ──►  5V rail
                         │
                         ├──►  ESP32-S3 VIN (5V pin)
                         └──►  Sensors via 3.3V regulator on the board
```

- Never power the servos from the ESP32's 5V pin or USB — a single
  ST-3032 stall pulls more than a dev board can source.
- The ST-3032's brown-out floor is ~9V. A 3S pack sags to ~9.9V near
  empty, which still clears it; a 2S pack does not.
- Tie all grounds together: battery, buck, URT-2, ESP32, sensor
  breakouts. This sounds obvious but it's the #1 reason new builds
  misbehave.

## GPIO map (ESP32-S3)

The serial-bus build needs very few pins — that's most of its charm:

| Function          | GPIO(s) | Devices on this line |
|-------------------|---------|----------------------|
| I2C0 (SDA, SCL)   | 15, 16  | TCA9548A mux (0x70) + colour sensors behind it, IMU (BNO055, 0x28), shared bus |
| UART1 (TX, RX)    | 14, 6   | URT-2 serial bus — every ST-3032 / ST-3215 daisy-chained |

Pin gotchas on the ESP32-S3:

- **GPIO 22–25 don't exist** — the pin list is 0–21 then 26–48.
- **GPIO 26–32 (and 33–37 on octal-PSRAM modules) are flash/PSRAM.**
  Do not use them.
- **GPIO 19/20 are the native-USB D-/D+** and **0/3/45/46 are
  strapping pins.**
- I2C and UART route through the GPIO matrix, so none of these
  assignments are fixed — they're the convention the bundled examples
  use. On a **classic ESP32**, the usual equivalents are I2C on
  21/22 and any two free pins for the UART.

## Sensor wiring (I2C)

The baseline build has **several colour sensors** (a line-follower /
zone-detection array) plus one IMU. The TCS34725's address is fixed at
`0x29` — no address-select pins — so two of them collide on a bare
bus. The TCA9548A multiplexer solves this: it sits at `0x70` and fans
the bus out to eight isolated channels, one colour sensor per channel.
The IMU has its own address (`0x28`) and connects straight to the bus.

Wire the mux to the ESP32-S3, then each sensor to a mux channel:

| TCA9548A pin | Connect to | Notes |
|--------------|------------|-------|
| VIN          | 3.3V       | |
| GND          | GND        | Common ground with everything else |
| SDA / SCL    | GPIO 15 / 16 | The main bus — shared with the BNO055 |
| SD0/SC0 … SD7/SC7 | one TCS34725 each | Isolated channels; every sensor sits at its own `0x29` |

| TCS34725 / BNO055 pin | Connect to | Notes |
|--------------|------------|-------|
| VIN (or VCC) | 3.3V       | The breakouts have onboard regulators, but the board's 3.3V is cleanest |
| GND          | GND        | |
| SDA / SCL    | mux channel `SDn`/`SCn` (colour sensors); GPIO 15/16 directly (IMU) | |

The Adafruit breakouts include ~10 kΩ SDA/SCL pull-ups, so for a
handful of devices you don't need to add your own. In code,
`mux[n]` behaves like an I2C bus, so the sensor driver is constructed
exactly as it would be on a bare bus:

```python
from machine import I2C, Pin
from openbricks.drivers.tca9548a import TCA9548A
from openbricks.drivers.tcs34725 import TCS34725
from openbricks.drivers.bno055 import BNO055

i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)   # ESP32-S3
mux = TCA9548A(i2c)                                    # 0x70 by default
sensors = [TCS34725(mux[ch]) for ch in range(3)]       # left, mid, right
imu = BNO055(i2c)                                      # 0x28, straight on the bus
```

For a complete program — a 2-sensor array that combines each sensor's
``ambient()`` and ``rgb()`` readings to name the colour under it
(red / blue / green / yellow / white / black) — see
`examples/color_array.py`.

Simplifications when you need fewer parts:

- **One colour sensor**: skip the mux entirely; wire the TCS34725 and
  BNO055 both to GPIO 15/16 (`0x29` and `0x28` coexist fine).
- **Exactly two colour sensors**: the ESP32's second hardware I2C
  controller also works (`I2C(1, sda=..., scl=...)` on any two free
  pins), one sensor per bus — no mux.

### TCS34725 LED pin

The colour sensor breakout has two extra pins beyond power and I2C:

- **LED** — drives the onboard white illumination LED. On Adafruit
  boards it defaults **on** (tied to VIN through the ADC-enable trace).
  To control it, wire it to a spare GPIO and drive high/low; to force it
  **off**, tie LED to GND. Leave it on for consistent colour readings —
  ambient light alone is unreliable across environments.
- **INT** — the interrupt output. The driver polls, so leave INT
  **unconnected**.

## Serial bus servo notes (ST-3032 / ST-3215)

- **Every servo needs a unique bus ID.** Factory default is 1; re-ID
  one servo at a time with `examples/st3215_reid.py` (same protocol —
  it works for the ST-3032 too). The bundled drivebase examples assume
  left = 1, right = 2.
- **Speed limits.** Per the STS3032 datasheet (`docs/datasheets/`),
  no-load top speed at 12V is 148 RPM = 888 °/s; under the rated
  3.3 kg·cm load it drops to roughly ⅔ of that. The driver's default
  `max_dps=600` clamps requests at the loaded operating point —
  construct with an explicit `max_dps=900` to chase the no-load number.
- **Mixed fleets:** the ST-3215 tolerates lower voltages, but the
  ST-3032 browns out below ~9V. If you daisy-chain both on one URT-2,
  the rail must satisfy the strictest member: 12V.
- **Drivebase:** `ST3032Motor` drops straight into `DriveBase`:

```python
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

left  = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
right = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)
db = DriveBase(left, right, wheel_diameter_mm=65, axle_track_mm=120)
```

Bench-test a fresh build with `examples/st3032_drivebase_test.py`.

## Calibrating the drivebase

`wheel_diameter_mm` and `axle_track_mm` are the two physical parameters that
matter for straight-line distance and turn accuracy. Measure them with
calipers or a ruler (wheel contact patch to wheel contact patch for axle
track, not hub to hub). If `straight(1000)` undershoots, your wheel
diameter value is too large; if `turn(360)` overshoots, your axle track is
too small.

High-torque 12V servos also deliver the default launch profile much more
stiffly than small DC motors — if the chassis pitches or lifts its rear
when a move starts, soften the ramp:

```python
db.settings(acceleration=180)   # wheel-deg/s²; default 720
```

## Alternative: DC gear motors with encoders

The original starter build — still fully supported. (Both this and
the serial-servo build run in the MuJoCo simulator via the driver
shim.)

| Part | Qty | Notes |
|------|----:|-------|
| JGB37-520 DC motor with encoder (1:30 gearing) | 2 | Pick the 12V version; runs fine off 7.4V 2S LiPo |
| L298N dual H-bridge module | 1 | Cheap and robust. TB6612FNG is a better choice if you can find it |
| 2S LiPo (7.4V) + buck converter (→5V, ≥2A) | 1 each | Don't power the ESP32 from the L298N's onboard 78M05 regulator — it's good for ~300 mA and browns out the moment the motors draw current |

Wiring topology:

```
    [ 2S LiPo 7.4V ]
        │
        ├─────────────►  L298N  Vmotor   (motor power)
        │
        └──► Buck  ──►  5V rail
                         │
                         ├──►  ESP32 VIN
                         ├──►  L298N  +5V (logic only)
                         └──►  Sensors via 3.3V regulator on ESP32
```

GPIO map (see `examples/esp32_drivebase.py` for the ESP32-S3 pin
assignments; the classic-ESP32 equivalents are in git history):

| Function          | ESP32-S3 GPIO(s) | Devices on this line |
|-------------------|------------------|----------------------|
| Left motor dir    | 1, 2             | L298N / TB6612 IN1, IN2 |
| Left motor PWM    | 17               | L298N / TB6612 ENA |
| Left encoder A, B | 7, 8             | JGB37-520 encoder channels |
| Right motor dir   | 9, 10            | L298N / TB6612 IN3, IN4 |
| Right motor PWM   | 11               | L298N / TB6612 ENB |
| Right encoder A, B| 12, 13           | JGB37-520 encoder channels |

The map deliberately leaves GPIO **4 and 5** free — those are the
firmware's default **program button** and **BLE-toggle button** (see
{class}`openbricks.hub.ESP32S3DevkitHub`). The launcher polls GPIO 4
as an input, so a motor driver toggling it would read as button
presses and stop your program. GPIO 15/16 (I2C) and 14/6 (serial-bus
UART) are also kept free so sensors and a serial-servo arm can join
the same build unchanged.

### Calibrating encoder counts

The default in `jgb37_520.py` is `counts_per_output_rev=1320`, which is
`11 CPR × 30:1 × 4 (quadrature edges)`. If you have a different gearbox
variant, recompute:

    counts_per_output_rev = encoder_CPR × gear_ratio × 4

Or measure empirically: rotate the output shaft by hand exactly one full
turn and read `motor.angle()`. Whatever it reports is what
`counts_per_output_rev` should be, scaled so that one turn = 360°.
