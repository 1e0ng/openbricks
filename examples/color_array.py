# SPDX-License-Identifier: MIT
"""
Demo: name the colour under each of 2 TCS34725s through a TCA9548A mux.

The TCS34725's I2C address is fixed at 0x29, so two of them can't
share a bare bus — this is exactly the job the TCA9548A multiplexer
exists for. Each sensor hangs off its own mux channel, and ``mux[n]``
behaves like a normal I2C bus, so the driver is constructed unchanged.

Each loop reads **both signals** from every sensor and combines them:

  * ``ambient()`` (0..100) — brightness. Separates black (too dark to
    trust hue at all) and, together with the channel spread, white.
  * ``rgb()`` (0..255 each, clear-normalised) — hue. Splits the
    chromatic colours: red, yellow, green, blue.

``classify()`` reduces one sensor's (ambient, rgb) pair to one of
``red / blue / green / yellow / white / black``.

Hardware:
    * ESP32-S3 (or classic ESP32)
    * TCA9548A breakout on I2C bus 0 (3.3V, GND)
        SDA=15, SCL=16 on ESP32-S3; SDA=21, SCL=22 on classic ESP32
    * 2x TCS34725, one per mux channel 0..1, facing the surface

Calibration matters: TCS34725 readings shift with illumination LED,
distance, and surface gloss. The thresholds below are starting points
— print ``sensor.ambient()`` and ``sensor.rgb()`` over each of your
own surfaces and adjust.
"""

# Below this ambient the surface is too dark to trust the normalised
# hue (the clear channel is mostly noise): call it black outright.
BLACK_AMBIENT = 12

# Channel spread (max - min of r, g, b) below which the surface is
# achromatic — grey-ish. Bright grey is white; dim grey is black.
NEUTRAL_SPREAD = 45
WHITE_AMBIENT = 35

# Yellow is "red AND green both strong"; a green channel at least
# this fraction of red separates it from pure red.
YELLOW_G_OVER_R = 0.6


def classify(ambient, rgb):
    """Reduce one sensor's (ambient 0..100, (r, g, b) 0..255) reading
    to ``"red" / "blue" / "green" / "yellow" / "white" / "black"``.

    Decision order: black by darkness first (hue is noise down
    there), then white/black by neutral spread + brightness, then the
    chromatic colours by channel dominance.
    """
    r, g, b = rgb
    if ambient < BLACK_AMBIENT:
        return "black"
    spread = max(r, g, b) - min(r, g, b)
    if spread < NEUTRAL_SPREAD:
        return "white" if ambient >= WHITE_AMBIENT else "black"
    if b >= r and b >= g:
        return "blue"
    if g >= r:
        return "green"
    # Red-dominant: yellow when green rides high alongside red.
    if g >= r * YELLOW_G_OVER_R:
        return "yellow"
    return "red"


def main():
    from machine import I2C, Pin

    from openbricks.drivers.tca9548a import TCA9548A
    from openbricks.drivers.tcs34725 import TCS34725
    from openbricks.tools import wait

    i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)   # ESP32-S3; 21/22 on classic ESP32
    mux = TCA9548A(i2c)                                    # 0x70 default
    sensors = [TCS34725(mux[ch], gain=16) for ch in range(2)]

    print("Hold surfaces under both sensors (Ctrl-C to stop)...")
    while True:
        parts = []
        for i, s in enumerate(sensors):
            ambient = s.ambient()
            rgb = s.rgb()
            parts.append("s%d: ambient=%3d rgb=(%3d,%3d,%3d) -> %s" % (
                i, ambient, rgb[0], rgb[1], rgb[2],
                classify(ambient, rgb)))
        print("  |  ".join(parts))
        wait(200)


if __name__ == "__main__":
    main()
