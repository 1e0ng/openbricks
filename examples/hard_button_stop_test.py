# SPDX-License-Identifier: MIT
"""
Hard-button stop test (1.44.0): press STOP while I2C blocks the VM.

Back-to-back blocking ``i2c.scan()`` calls are the exact load class
that starved the scheduler-dispatched button watcher (bench: 981 ms
tick gaps). Since 1.44.0 the program button is sampled on the hard
tick (esp_timer task, core 0), so a stop press must cut this program
within ~2 ms no matter what Python is doing.

The loop prints a heartbeat line every 20 scans so the BLE stream
shows liveness — when you press the STOP button, the stream should
cut with a KeyboardInterrupt mid-heartbeat. That cut IS the pass
result. If instead the program keeps printing for a second or more
after the press, the hard path failed — report the delay.

Afterwards, read the press counters:

    openbricks run -n ls -c "from _openbricks_native import motor_process; print(motor_process.hard_button_stats())"

Run:
    openbricks run -n ls examples/hard_button_stop_test.py
"""

import time

from machine import I2C, Pin

I2C_SDA, I2C_SCL = 15, 16     # ESP32-S3 convention (21/22 on classic)

i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400_000)
print("hammering i2c.scan() — press the STOP button whenever you like")

n = 0
t0 = time.ticks_ms()
while True:
    devices = i2c.scan()
    n += 1
    if n % 20 == 0:
        elapsed = time.ticks_diff(time.ticks_ms(), t0) / 1000.0
        print("scan #%d (%.1fs) devices=%s" % (n, elapsed, devices))
