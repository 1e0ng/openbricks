#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Diagnose a hub whose USB REPL has gone quiet.

Motivating bug (2026-08-13): immediately after ``openbricks log``
(a BLE session), ``openbricks flash`` finds the serial port but gets
silence from BOTH the REPL probe and esptool's bootloader handshake;
a power-cycle clears it. This script captures what the port actually
does at each step so the failure state can be classified instead of
guessed at.

Run it on the machine the hub is plugged into::

    python3 scripts/diagnose-usb-serial.py            # auto-detect port
    python3 scripts/diagnose-usb-serial.py --port /dev/cu.usbmodemXXXX

It is read-mostly: the only bytes sent are the REPL control
characters any ``mpremote`` session sends (Ctrl-C/Ctrl-B/Ctrl-A and
a newline). It never resets the chip, never writes flash, and leaves
the REPL in friendly mode. Paste the full output back for analysis.
"""

import argparse
import sys
import time


# Each probe step: (label, bytes to send or None for passive listen,
# seconds to listen afterwards).
PROBE_STEPS = [
    ("passive listen",       None,        2.0),
    ("newline",              b"\r\n",     1.0),
    ("interrupt (Ctrl-C x2)", b"\x03\x03", 1.0),
    ("friendly mode (Ctrl-B)", b"\x02",   1.0),
    ("raw mode (Ctrl-A)",    b"\x01",     1.0),
    ("back to friendly (Ctrl-B)", b"\x02", 1.0),
]


def hexdump(data, limit=256):
    """Compact printable dump: ASCII where printable, \\xNN elsewhere,
    truncated with a marker beyond ``limit`` bytes."""
    shown = data[:limit]
    parts = []
    for b in shown:
        if 32 <= b < 127:
            parts.append(chr(b))
        elif b == 0x0A:
            parts.append("\\n")
        elif b == 0x0D:
            parts.append("\\r")
        else:
            parts.append("\\x%02x" % b)
    tail = " …(+%d bytes)" % (len(data) - limit) if len(data) > limit else ""
    return "".join(parts) + tail


def classify(step_outputs):
    """Verdict from the per-step outputs (list of (label, bytes)).

    Pure function so it is unit-testable without hardware.
    """
    total = b"".join(data for _, data in step_outputs)
    by_label = dict(step_outputs)
    if not total:
        return ("DEAD: the port carries no bytes in either direction "
                "— UART path to the chip is down (bridge wedged, chip "
                "hung with interrupts off, or wiring). esptool failing "
                "too is consistent: its handshake rides the same pair.")
    if b">>>" in total:
        return ("FRIENDLY REPL alive: the hub answers normally — the "
                "flash probe should work; if it does not, the failure "
                "is host-side (port contention), not the hub.")
    if b"raw REPL" in total or by_label.get("raw mode (Ctrl-A)"):
        return ("RAW-REPL state: the hub responds but sits in raw "
                "mode (a BLE session's parking state). mpremote should "
                "still recover this — capture is the evidence needed.")
    if by_label.get("passive listen"):
        return ("SPEWING: the hub streams output without being asked "
                "— a program is running and printing; the REPL probe "
                "times out behind the flood.")
    return ("RESPONSIVE-BUT-ODD: bytes flow but match no known REPL "
            "state — likely garbled framing (baud mismatch or a "
            "crashed console). The dump above is the evidence.")


def _autodetect_port():
    from serial.tools import list_ports
    candidates = [
        p.device for p in list_ports.comports()
        if "usbmodem" in p.device or "usbserial" in p.device
        or "wchusbserial" in p.device or "ttyUSB" in p.device
        or "ttyACM" in p.device
    ]
    if not candidates:
        print("no candidate serial port found — is the hub plugged in?",
              file=sys.stderr)
        sys.exit(1)
    if len(candidates) > 1:
        print("multiple ports: %s — pass --port" % ", ".join(candidates),
              file=sys.stderr)
        sys.exit(1)
    return candidates[0]


def main():
    ap = argparse.ArgumentParser(
        description="Capture what the hub's USB serial port does.")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    import serial
    port = args.port or _autodetect_port()
    print("port: %s @ %d" % (port, args.baud))

    ser = serial.Serial(port, args.baud, timeout=0.2)
    try:
        print("after open: DTR=%s RTS=%s" % (ser.dtr, ser.rts))
        outputs = []
        for label, to_send, listen_s in PROBE_STEPS:
            if to_send is not None:
                ser.write(to_send)
                ser.flush()
            buf = b""
            deadline = time.monotonic() + listen_s
            while time.monotonic() < deadline:
                chunk = ser.read(256)
                if chunk:
                    buf += chunk
            outputs.append((label, buf))
            print("[%-26s] %4d bytes: %s"
                  % (label, len(buf), hexdump(buf) if buf else "-"))
        print()
        print("verdict:", classify(outputs))
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
