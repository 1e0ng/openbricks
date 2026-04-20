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

| Function          | GPIO(s)      |
|-------------------|--------------|
| I2C0 (SDA, SCL)   | 21, 22       |
| UART1 (TX, RX)    | 17, 16       |
| Left motor dir    | 25, 26       |
| Left motor PWM    | 27           |
| Left encoder A, B | 32, 33       |
| Right motor dir   | 14, 12       |
| Right motor PWM   | 13           |
| Right encoder A, B| 34, 35 (in)  |

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
