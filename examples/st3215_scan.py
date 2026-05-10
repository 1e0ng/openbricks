# SPDX-License-Identifier: MIT
"""Scan the SCS bus for live servos.

When ``st3215_bench_test.py`` can't reach the servo at the default
ID 1 / 1 Mbps, run this to find what's actually on the bus. Tries
the standard FeeTech SCS baud rates and pings IDs 1..30 at each.
Anything that replies prints its ID and baud.

Run:
    openbricks run -n ls examples/st3215_scan.py
"""

import time

from machine import UART, Pin


TX_PIN = 14
RX_PIN = 6
UART_ID = 1

# FeeTech SCS bauds, most-likely-default first.
BAUDS = [1_000_000, 500_000, 250_000, 115_200, 57_600, 38_400]

ID_RANGE = range(1, 31)   # factory default is 1; users rarely go beyond 30.


def _checksum(parts):
    s = 0
    for p in parts:
        s += p
    return (~s) & 0xFF


def ping_at(uart, servo_id):
    body = bytes([servo_id, 2, 0x01])             # PING
    pkt  = b"\xFF\xFF" + body + bytes([_checksum(body)])
    # Drain anything stale before TX.
    while uart.any():
        uart.read(uart.any())
    uart.write(pkt)

    # Reply: FF FF ID LEN ERR CHK  →  6 bytes.
    deadline = time.ticks_ms() + 50
    buf = b""
    while len(buf) < 6 and time.ticks_diff(deadline, time.ticks_ms()) > 0:
        chunk = uart.read(6 - len(buf))
        if chunk:
            buf += chunk
        else:
            time.sleep_ms(2)

    # Reject 6 bytes of noise: must start with the SCS header AND
    # echo the ID we pinged.
    if len(buf) != 6:
        return False
    if not buf.startswith(b"\xFF\xFF"):
        return False
    if buf[2] != servo_id:
        return False
    return True


def main():
    print("--- SCS bus scan ---")
    found_any = False
    for baud in BAUDS:
        uart = UART(UART_ID, baudrate=baud, tx=TX_PIN, rx=RX_PIN, timeout=50)
        # Settle.
        time.sleep_ms(20)
        hits = []
        for sid in ID_RANGE:
            if ping_at(uart, sid):
                hits.append(sid)
        if hits:
            print("  baud %d: IDs %s" % (baud, hits))
            found_any = True
        else:
            print("  baud %d: nothing" % baud)
    if not found_any:
        print("--- no servos responded at any baud / ID 1..30 ---")
        print("Likely causes (in order):")
        print("  1. URT-2 TX/RX pins not actually mapped to the SCS data line")
        print("     (try swapping; if that doesn't help, check URT-2 docs)")
        print("  2. Servo ID > 30 (rare unless explicitly re-configured)")
        print("  3. Bad servo cable (3-pin connector seated fully?)")
        print("  4. Servo internally damaged")
    else:
        print("--- found at least one servo ---")
        print("Reconstructor with: ST3215Motor(servo_id=<id>, "
              "uart_id=1, tx=14, rx=6, baud=<baud>)")


main()
