import os
import time

print("=== 0) environment ===")
print(os.uname())

print("=== 1) full-white WS2812 probe, both bitstream timings ===")
from machine import Pin
import neopixel
for p in (48, 38, 47, 21):
    for timing in (1, 0):
        try:
            np = neopixel.NeoPixel(Pin(p), 1, timing=timing)
            np[0] = (255, 255, 255)
            np.write()
            print("GPIO %d timing=%d: wrote FULL WHITE — lit?" % (p, timing))
            time.sleep(1.5)
            np[0] = (0, 0, 0)
            np.write()
        except Exception as e:
            print("GPIO %d timing=%d: raised %r" % (p, timing, e))

print("=== 2) raw machine.bitstream on GPIO 48 ===")
try:
    from machine import bitstream
    pin = Pin(48, Pin.OUT)
    bitstream(pin, 0, (400, 850, 800, 450), b"\xff\xff\xff")
    print("bitstream wrote 0xFFFFFF on GPIO 48 — lit?")
    time.sleep(1.5)
    bitstream(pin, 0, (400, 850, 800, 450), b"\x00\x00\x00")
except Exception as e:
    print("bitstream raised %r" % (e,))

print("=== 3) BLE state (for the record) ===")
from openbricks import bluetooth
print("bluetooth.is_enabled():", bluetooth.is_enabled())
print("=== done — report which step, if any, lit the LED ===")
