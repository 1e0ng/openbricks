# SPDX-License-Identifier: MIT
"""
ESP32 platform helpers.

The ESP32 has two hardware I2C blocks (0 and 1) and up to three UARTs (0, 1,
2). Pin mapping is flexible — almost any GPIO can route to any peripheral via
the internal GPIO matrix.

This module centralizes peripheral creation so drivers don't each reimplement
"set up an I2C bus". The config loader uses these helpers.
"""

from machine import I2C, Pin, UART


def make_i2c(bus_id=0, sda=21, scl=22, freq=400_000):
    return I2C(bus_id, sda=Pin(sda), scl=Pin(scl), freq=freq)


def make_uart(bus_id=1, tx=17, rx=16, baud=1_000_000, timeout=50):
    return UART(bus_id, baudrate=baud, tx=tx, rx=rx, timeout=timeout)
