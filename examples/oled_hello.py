# SPDX-License-Identifier: MIT
"""
Example: greet the user on an SSD1306 OLED and tick an uptime counter.

Wiring (any I2C SSD1306, typically 128x64 at address 0x3C):

    ESP32 classic DevKitC-V4     ESP32-S3 DevKitC-1
    --------------------------   --------------------------
    OLED VCC -> 3.3V             OLED VCC -> 3.3V
    OLED GND -> GND              OLED GND -> GND
    OLED SDA -> GPIO 21          OLED SDA -> GPIO 15
    OLED SCL -> GPIO 22          OLED SCL -> GPIO 16

Set ``SDA_PIN`` / ``SCL_PIN`` below to match your board.

How to run:

    # Copy this file onto the flashed firmware as ``main.py`` so it
    # runs automatically on boot.
    mpremote connect /dev/tty.usbserial-XXXX cp examples/oled_hello.py :main.py

    # Or run it once without persisting:
    mpremote connect /dev/tty.usbserial-XXXX run examples/oled_hello.py

Wire a momentary button between GPIO 5 and GND to reset the counter.
(GPIO 5 is the hub's default ``bluetooth_button_pin``; override via
``ESP32DevkitHub(bluetooth_button_pin=<N>)`` if you wired it elsewhere.)
"""

import time

from machine import I2C, Pin

from openbricks.drivers.ssd1306 import SSD1306
from openbricks.hub import ESP32DevkitHub

# ----- wiring -----
SDA_PIN = 21     # ESP32 DevKitC-V4 default; use 15 on ESP32-S3 DevKitC-1
SCL_PIN = 22     # ESP32 DevKitC-V4 default; use 16 on ESP32-S3 DevKitC-1
OLED_ADDR = 0x3C


def init_display(sda_pin, scl_pin, addr=OLED_ADDR, width=128, height=64,
                 i2c_id=0, freq=400_000):
    """Build an I2C bus on the given pins and return a fresh SSD1306.

    Factored out so user code can wire up the display in one call — and
    so other examples can reuse the same init without copy-pasting the
    I2C/Pin boilerplate.
    """
    i2c = I2C(i2c_id, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=freq)
    return SSD1306(i2c, addr=addr, width=width, height=height)


# ----- init -----
# The display is an I2C component, separate from the hub's board-level
# peripherals (LED, button). Instantiate it directly and use it directly.
display = init_display(SDA_PIN, SCL_PIN)
hub = ESP32DevkitHub()

# ----- main loop -----
seconds = 0
while True:
    if hub.button.pressed():
        seconds = 0

    display.fill(0)
    display.text("hello, openbricks", 0, 0)
    display.text("uptime:", 0, 20)
    display.text("%d s" % seconds, 0, 32)
    display.text("(hold btn to zero)", 0, 52)
    display.show()

    time.sleep(1)
    seconds += 1
