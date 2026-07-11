# SPDX-License-Identifier: MIT
"""
Demo: a 3-sensor TCS34725 line-follower array through a TCA9548A mux.

One TCS34725 tells you "am I over the line?"; a row of them tells you
*where* the line is, which is what a line follower actually steers on.
But the TCS34725's I2C address is fixed at 0x29, so several of them
can't share a bus — this is exactly the job the TCA9548A multiplexer
exists for. Each sensor hangs off its own mux channel, and ``mux[n]``
behaves like a normal I2C bus, so the driver is constructed unchanged.

``line_position()`` reduces the row of ambient readings to a single
steering signal in [-1, +1]: the darkness-weighted centroid across the
array. -1 means the line is under the leftmost sensor, +1 the
rightmost, 0 centered. Feed it to a P(ID) controller as the error term.

Hardware:
    * ESP32-S3 (or classic ESP32)
    * TCA9548A breakout on I2C bus 0 (3.3V, GND)
        SDA=15, SCL=16 on ESP32-S3; SDA=21, SCL=22 on classic ESP32
    * 3x TCS34725, one per mux channel 0..2, left-to-right,
      facing the floor
"""

# Readings at or above this ambient (0..100) count as fully "floor";
# darkness below it is what the centroid weighs. On a white floor with
# a black tape line, floor reads ~60-90 and tape ~5-20; calibrate by
# printing sensor.ambient() over both surfaces.
FLOOR_AMBIENT = 60


def line_position(ambients, floor=FLOOR_AMBIENT):
    """Reduce per-sensor ambient readings to a position in [-1, +1].

    Each sensor contributes weight ``max(0, floor - ambient)`` — how
    much darker than the floor it reads. The weighted centroid of the
    sensor positions (spread evenly from -1 to +1, left to right) is
    the line position estimate.

    Returns ``None`` when no sensor reads darker than ``floor`` — the
    line is lost, which callers must handle explicitly (spin to
    search, stop, ...) rather than mistaking it for "centered".
    """
    n = len(ambients)
    if n < 2:
        raise ValueError("need at least 2 sensors, got %d" % n)
    total = 0
    weighted = 0.0
    for i, ambient in enumerate(ambients):
        w = floor - ambient
        if w <= 0:
            continue
        pos = -1.0 + 2.0 * i / (n - 1)   # sensor i's position in [-1, 1]
        total += w
        weighted += w * pos
    if total == 0:
        return None
    return weighted / total


def main():
    from machine import I2C, Pin

    from openbricks.drivers.tca9548a import TCA9548A
    from openbricks.drivers.tcs34725 import TCS34725
    from openbricks.tools import wait

    i2c = I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000)   # ESP32-S3; 21/22 on classic ESP32
    mux = TCA9548A(i2c)                                    # 0x70 default
    sensors = [TCS34725(mux[ch]) for ch in range(2)]       # left, mid, right

    print("Line array over a dark line on a light floor (Ctrl-C to stop)...")
    while True:
        ambients = [s.ambient() for s in sensors]
        pos = line_position(ambients)
        if pos is None:
            print("ambients={}  ->  line lost".format(ambients))
        else:
            print("ambients={}  ->  position {:+.2f}".format(ambients, pos))
        wait(200)


if __name__ == "__main__":
    main()
