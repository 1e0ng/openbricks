# SPDX-License-Identifier: MIT
"""
Bench test for a WS2812B x8 RGB LED stick.

Walks the strip through a quick visual checklist: solid colors,
a running dot, and a brightness ramp — enough to confirm wiring,
data-line integrity, and that all 8 pixels answer.

Hardware:
    * WS2812B x8 stick: DIN -> DATA_PIN below, 5V -> 5 V supply
      (or 3.3 V — fine for the small x8 boards), GND -> common GND.
    * The ESP32's 3.3 V data level is out of spec for a 5 V-supplied
      strip but works with virtually every module; if colors glitch,
      power the stick from 3.3 V or add a level shifter.

Run with:
    openbricks run -n ls examples/ws2812_test.py
"""

import time

from openbricks.drivers.ws2812 import WS2812


DATA_PIN = 4      # EDIT to your wiring
NUM_LEDS = 8


def main():
    strip = WS2812(pin=DATA_PIN, n=NUM_LEDS)

    print("[1] solid colors (red, green, blue, white) ...")
    for color in ((255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)):
        strip.fill(color)
        time.sleep(0.6)

    print("[2] running dot, 3 laps ...")
    for lap in range(3):
        for i in range(len(strip)):
            strip.clear()
            strip[i] = (255, 90, 0)
            strip.show()
            time.sleep(0.08)

    print("[3] brightness ramp on white ...")
    for pct in (5, 10, 20, 40, 70, 100):
        strip.brightness = pct / 100
        strip.fill((255, 255, 255))
        time.sleep(0.3)

    strip.clear()
    print("--- done: all 8 pixels should have answered every step ---")


main()
