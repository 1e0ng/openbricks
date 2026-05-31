# SPDX-License-Identifier: MIT
"""LOW-LEVEL diagnostic for ST3032 step mode (op_mode=3).

The v1.8.0 multi-turn probe failed in a way that looks like a
*measurement* bug, not a motion bug: a +720° command read back as
−719.91° (right magnitude, wrong sign) and several moves collapsed to
±360°. That's the signature of angle()'s within-turn accumulator
misreading the present-position register once multi-turn step mode is
enabled (limits=0) — the register very likely stops being a clean
0..4095 within-turn value and starts returning a SIGNED multi-turn
value (sign-magnitude, per the FeeTech SDK's scs_tohost(pos, 15)).

This script bypasses angle() entirely and dumps the RAW present-
position register in every plausible interpretation while issuing
known relative steps, so we can read the truth off the wire:

  * raw      — the unsigned 16-bit register value
  * mask12   — raw & 0x0FFF  (what _read_present_pos does today)
  * signmag  — sign-magnitude decode (bit 15 = sign)
  * 2comp    — two's-complement decode

What each case tells us:
  before/after deltas → did the shaft actually move, and which way?
  which column is monotonic & matches the commanded counts → the
    correct decode to use in _read_present_pos.
  B (a second +1024) → is the step cumulative (relative) as expected?
  D (+8192) → does one write do two full turns?

Run against the ALREADY-FLASHED v1.8.0 firmware (no re-flash needed —
this talks to the bus directly):

    openbricks run -n ls examples/st3032_stepmode_probe.py
"""

import time

from openbricks.drivers.st3032 import ST3032Motor

_REG_MIN_ANGLE = 0x09
_REG_MAX_ANGLE = 0x0B
_REG_MODE      = 0x21
_REG_TORQUE    = 0x28
_REG_GOAL_POS  = 0x2A
_REG_GOAL_SPEED = 0x2E
_REG_PRESENT   = 0x38

SID = 1

m = ST3032Motor(servo_id=SID, uart_id=1, tx=14, rx=6)
bus = m._bus


def read_raw():
    d = bus.read(SID, _REG_PRESENT, 2)
    if d is None:
        return None
    return d[0] | (d[1] << 8)


def fmt(raw):
    if raw is None:
        return "raw=None (bus silent)"
    mask12 = raw & 0x0FFF
    signmag = -(raw & 0x7FFF) if (raw & 0x8000) else raw
    twos = raw - 0x10000 if raw >= 0x8000 else raw
    return ("raw=0x%04X  u=%5d  mask12=%4d  signmag=%+6d  2comp=%+6d"
            % (raw, raw, mask12, signmag, twos))


def step_and_trace(label, counts, settle_ms=2000, poll_ms=100):
    print("\n=== %s : step %+d counts ===" % (label, counts))
    print("  before     :", fmt(read_raw()))
    mag = abs(counts)
    v = mag | (0x8000 if counts < 0 else 0)
    bus.write(SID, _REG_GOAL_POS, bytes([v & 0xFF, (v >> 8) & 0xFF]))
    n = settle_ms // poll_ms
    for i in range(n):
        time.sleep_ms(poll_ms)
        print("  t=%4dms   " % ((i + 1) * poll_ms), fmt(read_raw()))


print("\n========== ST3032 step-mode raw-register probe ==========")

# Read present in the servo's BOOT mode first (wheel mode, stock limits)
# to see the baseline format before we touch anything.
print("\n--- present-position in wheel mode (stock limits) ---")
print("  ", fmt(read_raw()))

# Enable multi-turn step mode exactly as run_angle does.
bus.write(SID, _REG_TORQUE, bytes([1]))
bus.write(SID, _REG_MIN_ANGLE, bytes([0, 0]))
bus.write(SID, _REG_MAX_ANGLE, bytes([0, 0]))
bus.write(SID, _REG_MODE, bytes([3]))
time.sleep_ms(100)
print("\n--- present-position just after entering step mode ---")
print("  ", fmt(read_raw()))

# Modest speed cap (≈120 dps × 11.378 steps/dps).
sp = 1365
bus.write(SID, _REG_GOAL_SPEED, bytes([sp & 0xFF, (sp >> 8) & 0xFF]))

step_and_trace("A  +1024  (quarter turn fwd)",  1024)
step_and_trace("B  +1024  (again — cumulative?)", 1024)
step_and_trace("C  -2048  (half turn back)",    -2048)
step_and_trace("D  +8192  (two full turns fwd)", 8192, settle_ms=4000)

# Park: cut torque so the shaft is free.
bus.write(SID, _REG_TORQUE, bytes([0]))
print("\n--- done (torque off) ---")
