# SPDX-License-Identifier: MIT
"""
Example: read RGB from a TCS34725 over I2C and classify the color.

Hardware:
    * ESP32
    * TCS34725 breakout on I2C bus 0 (SDA=21, SCL=22, 3.3V, GND)
"""

from machine import I2C, Pin

from openbricks.drivers.tcs34725 import TCS34725
from openbricks.tools import wait


i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)
color = TCS34725(i2c, integration_ms=50, gain=4)


def classify(r, g, b):
    """Very rough color classifier. Good enough to tell red from green."""
    if r > g and r > b:
        return "red"
    if g > r and g > b:
        return "green"
    if b > r and b > g:
        return "blue"
    if r > 200 and g > 200 and b > 200:
        return "white"
    if r < 40 and g < 40 and b < 40:
        return "black"
    return "unknown"


while True:
    r, g, b = color.rgb()
    print("rgb=({:3d},{:3d},{:3d})  ambient={:3d}  ->  {}".format(
        r, g, b, color.ambient(), classify(r, g, b)
    ))
    wait(200)
