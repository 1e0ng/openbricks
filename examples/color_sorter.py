# SPDX-License-Identifier: MIT
"""
Demo: name the colour under a TCS34725 by nearest-match to a palette.

``read_color.py`` classifies with hand-tuned ``r > g > b`` rules —
enough to tell red from green, but it has no notion of orange,
yellow, or "closest of several known bricks". This demo takes the
approach real colour-sorting robots use: keep a small table of
reference RGBs (one per brick colour you care about) and report
whichever reference the live reading sits closest to.

Because it's just "nearest point in RGB space", adding a colour is a
one-line addition to ``PALETTE`` — no new branches to hand-tune.

Hardware:
    * ESP32
    * TCS34725 breakout on I2C bus 0 (SDA=21, SCL=22, 3.3V, GND)

Calibration matters: TCS34725 readings shift with ambient light and
integration time. The values below are only a starting point — for
best results, print ``sensor.rgb()`` while holding each of your own
bricks under the sensor and paste the numbers into ``PALETTE``.
"""

# name -> reference (r, g, b), each 0..255 as returned by
# TCS34725.rgb() (clear-normalised, so a white surface reads
# roughly (255, 255, 255) at any brightness in range).
PALETTE = (
    ("red",    (200,  45,  40)),
    ("green",  ( 55, 175,  70)),
    ("blue",   ( 45,  80, 190)),
    ("yellow", (215, 200,  55)),
    ("white",  (240, 240, 240)),
    ("black",  ( 30,  30,  30)),
)


def nearest_color(rgb, palette=PALETTE):
    """Return the ``palette`` name whose reference RGB is closest to
    ``rgb`` by squared Euclidean distance. Ties resolve to the earlier
    palette entry, so list ``palette`` most-specific first.
    """
    r, g, b = rgb
    best_name = None
    best_dist = None
    for name, (pr, pg, pb) in palette:
        dist = (r - pr) ** 2 + (g - pg) ** 2 + (b - pb) ** 2
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_name = name
    return best_name


def main():
    from machine import I2C, Pin

    from openbricks.drivers.tcs34725 import TCS34725
    from openbricks.tools import wait

    i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)
    sensor = TCS34725(i2c, integration_ms=50, gain=4)

    print("Hold a coloured brick under the sensor (Ctrl-C to stop)...")
    while True:
        r, g, b = sensor.rgb()
        print("rgb=({:3d},{:3d},{:3d})  ambient={:3d}  ->  {}".format(
            r, g, b, sensor.ambient(), nearest_color((r, g, b))))
        wait(200)


if __name__ == "__main__":
    main()
