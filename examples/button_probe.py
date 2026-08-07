# SPDX-License-Identifier: MIT
"""Find the stop button and confirm the firmware actually sees it.

The launcher watches GPIO4 (``DEFAULT_BUTTON_PIN``) for the program
start/stop button. If pressing the button doesn't stop a running
program, the most likely causes are (a) the button is wired to a
different GPIO than 4, or (b) the press isn't reaching the firmware at
all. This probe distinguishes them.

It configures a set of candidate GPIOs as pulled-up inputs and prints
EVERY level change. Run it, then press your stop button several times
and watch the output:

* A line like ``GPIO0  1 -> 0  PRESSED`` when you press tells you the
  button is on that GPIO. If it's not 4, that's the bug — the launcher
  is watching the wrong pin (see the note printed at the end).
* No line at all on press → the firmware never sees the button: wrong
  wiring, not actually connected, or the pin is one this probe skipped.

Cross-check on the launcher itself: this probe does NOT catch
KeyboardInterrupt, and it runs under ``openbricks run`` (so the
launcher's own GPIO4 watcher is active with ``_running`` True). So:

* If pressing the button STOPS this probe (you see
  ``openbricks: stopped by button press.``) → the launcher detection
  works and the problem is specific to your program (e.g. it catches
  KeyboardInterrupt, or blocks in a long call). Tell me and we'll look
  there.
* If the probe keeps printing AND shows a transition → hardware is
  fine, the launcher just isn't acting on it (wrong pin / dispatch).

    openbricks run -n ls examples/button_probe.py

The bench servo-bus pins (GPIO14 TX, GPIO6 RX) and the USB pins
(19/20) are intentionally skipped so the probe doesn't disturb them.
"""

import time

from machine import Pin

_CANDIDATES = [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18,
               21, 38, 39, 40, 41, 42, 45, 47, 48]

pins = {}
for g in _CANDIDATES:
    try:
        pins[g] = Pin(g, Pin.IN, Pin.PULL_UP)
    except Exception as e:
        print("  (skip GPIO%d: %r)" % (g, e))

watched = sorted(pins)
last = {}
for g in watched:
    last[g] = pins[g].value()

print("=== button probe ===")
print("watching GPIOs:", watched)
print("initial levels:", [last[g] for g in watched])
print("Now press your stop button a few times (Ctrl-C / button to exit).")

t0 = time.ticks_ms()
beat = 0
while True:
    for g in watched:
        v = pins[g].value()
        if v != last[g]:
            print("t=%6dms  GPIO%-2d %d -> %d  %s" % (
                time.ticks_diff(time.ticks_ms(), t0), g, last[g], v,
                "PRESSED" if v == 0 else "released"))
            last[g] = v
    time.sleep_ms(5)
    beat += 1
    if beat % 1000 == 0:
        print("t=%6dms  ...watching, no change since last beat" %
              time.ticks_diff(time.ticks_ms(), t0))
