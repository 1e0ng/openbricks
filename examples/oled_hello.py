# SPDX-License-Identifier: MIT
"""
Example: greet the user on an SSD1306 OLED and tick an uptime counter.

Wiring (any I2C SSD1306, typically 128x64 at address 0x3C):

    ESP32 classic DevKitC-V4     ESP32-S3 DevKitC-1
    --------------------------   --------------------------
    OLED VCC -> 3.3V             OLED VCC -> 3.3V
    OLED GND -> GND              OLED GND -> GND
    OLED SDA -> GPIO 21          OLED SDA -> GPIO 8
    OLED SCL -> GPIO 22          OLED SCL -> GPIO 9

Set ``SDA_PIN`` / ``SCL_PIN`` below to match your board.

How to run:

    # Copy this file onto the flashed firmware as ``main.py`` so it
    # runs automatically on boot.
    mpremote connect /dev/tty.usbserial-XXXX cp examples/oled_hello.py :main.py

    # Or run it once without persisting:
    mpremote connect /dev/tty.usbserial-XXXX run examples/oled_hello.py

Press the BOOT button (GPIO 0) to reset the counter back to zero.
"""

import time

from machine import I2C, Pin

from openbricks.drivers.ssd1306 import SSD1306
from openbricks.hub import ESP32DevkitHub

# ----- wiring -----
SDA_PIN = 21     # change to 8 on ESP32-S3 DevKitC-1
SCL_PIN = 22     # change to 9 on ESP32-S3 DevKitC-1
OLED_ADDR = 0x3C

# ----- init -----
i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400_000)
display = SSD1306(i2c, addr=OLED_ADDR)
hub = ESP32DevkitHub(display=display)

# ----- main loop -----
seconds = 0
while True:
    if hub.button.pressed():
        seconds = 0

    hub.display.fill(0)
    hub.display.text("hello, openbricks", 0, 0)
    hub.display.text("uptime:", 0, 20)
    hub.display.text("%d s" % seconds, 0, 32)
    hub.display.text("(BOOT to reset)", 0, 52)
    hub.display.show()

    time.sleep(1)
    seconds += 1
