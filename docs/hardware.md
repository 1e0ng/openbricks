# Hardware guide

A starter parts list and wiring notes. Everything here is commodity stuff you
can buy on AliExpress / Amazon / Adafruit.

## Recommended starter robot (~$30 USD in parts)

| Part | Qty | Notes |
|------|----:|-------|
| ESP32 DevKit v1 (38-pin) | 1 | Any ESP32 board with ≥18 free GPIOs works |
| JGB37-520 DC motor with encoder (1:30 gearing) | 2 | Pick the 12V version; runs fine off 7.4V 2S LiPo |
| L298N dual H-bridge module | 1 | Cheap and robust. TB6612FNG is a better choice if you can find it |
| 2S LiPo battery (7.4V, 1000+ mAh) + balance charger | 1 | Or 2x 18650 in series in a holder |
| Buck converter (7.4V → 5V, ≥2A) | 1 | Powers the ESP32 and sensors. The L298N's onboard regulator is too weak |
| TCS34725 breakout | 1 | Adafruit or clone, 3.3V-tolerant |
| BNO055 breakout | 1 | Adafruit, Adafruit-compatible, or BNO085 with driver changes |
| ST-3215 serial bus servo | 0–4 | Optional; good for arms / grippers |
| Jumper wires, M3 standoffs, chassis plate | — | Your robot, your build |

## Power budget

Don't power the ESP32 from the L298N's onboard 5V regulator. It's a 78M05
linear regulator good for ~300 mA at best, and it browns out the moment the
motors draw current. Use a dedicated buck converter from the main battery.

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

Tie all grounds together. This sounds obvious but it's the #1 reason
new builds misbehave.

## GPIO map

ESP32 has enough pins for everything here, but a few gotchas:

- **GPIOs 34–39 are input-only.** Fine for encoder channels, wrong for
  direction pins or PWM.
- **GPIOs 6–11 are wired to the on-package SPI flash** on most modules.
  Do not use them.
- **GPIOs 0, 2, 12, 15 are strapping pins.** Using 0 or 15 for a motor
  direction line can prevent boot. Safe to use if they're pulled low by
  your H-bridge when the ESP32 is booting, but verify.

A conservative default:

| Function          | GPIO(s)      | Devices on this line                     |
|-------------------|--------------|------------------------------------------|
| I2C0 (SDA, SCL)   | 21, 22       | Colour sensor (TCS34725, 0x29) + IMU (BNO055, 0x28), shared bus |
| UART1 (TX, RX)    | 17, 16       | ST-3215 serial bus servos                |
| Left motor dir    | 25, 26       | L298N / TB6612 IN1, IN2                   |
| Left motor PWM    | 27           | L298N / TB6612 ENA                        |
| Left encoder A, B | 32, 33       | JGB37-520 encoder channels               |
| Right motor dir   | 14, 12       | L298N / TB6612 IN3, IN4                   |
| Right motor PWM   | 13           | L298N / TB6612 ENB                        |
| Right encoder A, B| 34, 35 (in)  | JGB37-520 encoder channels               |

The colour sensor and IMU share I2C0 — see [Sensor wiring](#sensor-wiring-i2c)
below for the per-pin breakout connections.

### ESP32-S3: different pin numbers

The map above (and the `21`/`22` I2C pins used throughout the examples)
is for the **classic ESP32 DevKit v1** — the recommended starter board.
The **ESP32-S3** numbers its pins differently, and several assumptions
here don't carry over:

- **GPIO 22–25 don't exist on the S3** — the pin list is 0–21 then
  26–48. So `SCL = 22` is impossible; you must pick different pins.
- The input-only range (34–39) and the SPI-flash pins (6–11) are
  classic-ESP32 quirks. On the S3, GPIO 26–32 (and 33–37 on octal-PSRAM
  modules) are taken by flash/PSRAM, GPIO 19/20 are the native-USB D-/D+,
  and the strapping pins are 0/3/45/46.
- On both chips the I2C, UART, and PWM peripherals route through the
  **GPIO matrix**, so their pins aren't fixed — you assign them in code.
  The examples that ship with openbricks default to the **ESP32-S3
  DevKitC-1** convention used across the codebase, GPIO **15 (SDA) / 16
  (SCL)**:

  ```python
  i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)   # ESP32-S3
  i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)   # classic ESP32
  ```

## Sensor wiring (I2C)

The TCS34725 colour sensor and the BNO055 IMU are both I2C devices. Any
number of I2C devices can share one two-wire bus *as long as each has a
distinct address* — wire every breakout to the same SDA/SCL pins and
power rail. In the starter build that's one colour sensor (`0x29`) and
one IMU (`0x28`), which coexist happily:

| Breakout pin | Connect to        | Notes |
|--------------|-------------------|-------|
| VIN (or VCC) | 3.3V              | The breakouts have onboard regulators, but the ESP32's own 3.3V is cleanest. Don't feed 5V into an ESP32 GPIO. |
| GND          | GND               | Common ground with the ESP32 — same as everything else. |
| SDA          | GPIO 21           | Classic-ESP32 default; any free GPIO works. Shared by every device on the bus. |
| SCL          | GPIO 22           | Classic-ESP32 default. **No GPIO 22 on the ESP32-S3** — see [ESP32-S3 pin numbers](#esp32-s3-different-pin-numbers). Shared by every device on the bus. |

The Adafruit breakouts include ~10 kΩ SDA/SCL pull-ups, so for a handful
of devices you don't need to add your own. In code, one `I2C` object
drives the whole bus:

```python
from machine import I2C, Pin
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)

from openbricks.drivers.tcs34725 import TCS34725
from openbricks.drivers.bno055 import BNO055
color = TCS34725(i2c)   # 0x29
imu   = BNO055(i2c)     # 0x28
```

### Multiple colour sensors

The TCS34725's address is **fixed at `0x29`** — the chip has no
address-select pins — so you *cannot* put two of them on the same bus;
they'd both answer to `0x29` and collide. A line-following or sorting
robot that wants several colour sensors has two options:

- **A second I2C bus.** The ESP32 has two hardware I2C controllers, so a
  second sensor can live on I2C1 with any two free GPIOs:

  ```python
  bus0 = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)
  bus1 = I2C(1, sda=Pin(19), scl=Pin(18), freq=400_000)
  left  = TCS34725(bus0)   # 0x29 on bus 0
  right = TCS34725(bus1)   # 0x29 on bus 1
  ```

  That tops out at two colour sensors (plus the IMU, which can share
  either bus at `0x28`).

- **A TCA9548A I2C multiplexer** for three or more. It fans one bus out
  to eight isolated channels; you select a channel, then talk to the
  `0x29` sensor on it. Wire all colour sensors through the mux and the
  IMU straight to the bus.

  The `openbricks.drivers.tca9548a.TCA9548A` driver makes this
  transparent: `mux[n]` behaves like an I2C bus that auto-selects
  channel `n`, so the sensor driver is constructed exactly as it would
  be on a bare bus:

  ```python
  from openbricks.drivers.tca9548a import TCA9548A
  from openbricks.drivers.tcs34725 import TCS34725

  i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)
  mux = TCA9548A(i2c)                 # 0x70 by default
  sensors = [TCS34725(mux[ch]) for ch in range(4)]   # four 0x29 sensors
  ```

  For a complete program — a 3-sensor line-follower array that reduces
  the row of readings to a steering signal — see
  `examples/color_array.py`.

### TCS34725 LED pin

The colour sensor breakout has two extra pins beyond power and I2C:

- **LED** — drives the onboard white illumination LED. On Adafruit
  boards it defaults **on** (tied to VIN through the ADC-enable trace).
  To control it, wire it to a spare GPIO and drive high/low; to force it
  **off**, tie LED to GND. Leave it on for consistent colour readings —
  ambient light alone is unreliable across environments.
- **INT** — the interrupt output. The driver polls, so leave INT
  **unconnected**.

## Calibrating encoder counts

The default in `jgb37_520.py` is `counts_per_output_rev=1320`, which is
`11 CPR × 30:1 × 4 (quadrature edges)`. If you have a different gearbox
variant, recompute:

    counts_per_output_rev = encoder_CPR × gear_ratio × 4

Or measure empirically: rotate the output shaft by hand exactly one full
turn and read `motor.angle()`. Whatever it reports is what
`counts_per_output_rev` should be, scaled so that one turn = 360°.

## Calibrating the drivebase

`wheel_diameter_mm` and `axle_track_mm` are the two physical parameters that
matter for straight-line distance and turn accuracy. Measure them with
calipers or a ruler (wheel contact patch to wheel contact patch for axle
track, not hub to hub). If `straight(1000)` undershoots, your wheel
diameter value is too large; if `turn(360)` overshoots, your axle track is
too small.
