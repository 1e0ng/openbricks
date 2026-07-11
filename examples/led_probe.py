# LED diagnostic — safe, no motors. Flashes the WS2812 candidates and
# reports what the boot sequence actually wired.
import time

print("=== 1) raw WS2812 probe on GPIO 48 and 38 ===")
# DevKitC-1 v1.0 has the RGB LED on GPIO 48; v1.1 boards (and several
# clones) moved it to GPIO 38. Watch the board: report which pin (if
# any) flashes green.
from machine import Pin
import neopixel
for p in (48, 38):
    try:
        np = neopixel.NeoPixel(Pin(p), 1)
        np[0] = (0, 60, 0)
        np.write()
        print("GPIO %d: wrote GREEN — is the LED lit now?" % p)
        time.sleep(2)
        np[0] = (0, 0, 0)
        np.write()
    except Exception as e:
        print("GPIO %d: raised %r" % (p, e))

print("=== 2) what boot wired (pin claims) ===")
# {48: LED, 5: button, 4: program} = S3 hub constructed at boot.
# {2: LED, ...} = boot silently fell back to the classic-ESP32 hub.
# {4 only} = both hub constructors raised.
try:
    from openbricks import pins
    print("claims:", pins._claims)
except ImportError:
    print("no openbricks.pins (firmware older than 1.10.0)")

print("=== 3) reconstruct the S3 hub and catch the real error ===")
try:
    from openbricks.hub import ESP32S3DevkitHub
    h = ESP32S3DevkitHub(bluetooth=False)
    print("hub constructed OK, led =", h.led)
    if h.led is not None:
        h.led.rgb(0, 0, 255)
        print("painted BLUE via hub.led — is it lit?")
        time.sleep(2)
        h.led.rgb(0, 0, 0)
except Exception as e:
    print("ESP32S3DevkitHub RAISED:", repr(e))

print("=== 4) BLE state ===")
from openbricks import bluetooth
print("bluetooth.is_enabled():", bluetooth.is_enabled())
print("=== done ===")
