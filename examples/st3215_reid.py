# SPDX-License-Identifier: MIT
"""Re-ID a single ST-3215 on the URT-2 bus.

Factory ST-3215s ship at ID=1. Two motors at the same ID will reply
simultaneously and produce garbage on the bus, so before you can
daisy-chain a second servo you have to give each one a unique ID.

Procedure:
    1. Unplug ALL servos from the URT-2 except the one you want to
       re-ID. This is critical — the broadcast would otherwise hit
       every servo on the bus.
    2. Edit ``NEW_ID`` below to the value you want (e.g. 2 for the
       right wheel) and run::

           openbricks run -n ls examples/st3215_reid.py

    3. Power-cycle the servo (unplug + replug, or toggle 12 V), then
       re-run ``examples/st3215_scan.py`` to confirm the new ID.
    4. Repeat for any additional servos.

The new ID persists across power cycles (the servo writes it into
its own EEPROM). You don't need to redo this each session.

⚠ DO NOT re-ID with multiple servos on the bus — every servo at
the source ID gets rewritten, so you'd end up with N servos all
at the new ID, back to the collision problem.
"""

import time

from openbricks.drivers.st3215 import _SCServoBus


CURRENT_ID = 1     # the ID the servo answers on right now (factory = 1)
NEW_ID     = 2     # the ID you want it to use (e.g. 2 for the right wheel)

UART_ID, TX_PIN, RX_PIN = 1, 14, 6

# SCS register addresses we use:
#   0x37  Lock — 0 = unlocked (allow EEPROM writes), 1 = locked
#   0x05  ID
_REG_LOCK = 0x37
_REG_ID   = 0x05


def main():
    print("--- ST-3215 re-ID: %d → %d ---" % (CURRENT_ID, NEW_ID))
    if NEW_ID < 1 or NEW_ID > 253:
        print("NEW_ID must be in 1..253 (0 and 254 are reserved).")
        return

    bus = _SCServoBus(UART_ID, TX_PIN, RX_PIN)

    # Verify the source servo is actually there.
    if not bus.ping(CURRENT_ID):
        print("No servo replied at ID %d. Aborting." % CURRENT_ID)
        print("Check: only one servo plugged in? 12 V present? cable seated?")
        return
    print("[1] found servo at ID %d" % CURRENT_ID)

    # Unlock EEPROM writes. Required by SCS firmware before any
    # EEPROM register (lock=0x37, id=0x05, baud=0x06, ...) accepts a write.
    bus.write(CURRENT_ID, _REG_LOCK, bytes([0]))
    print("[2] EEPROM unlocked")

    # Write the new ID. After this packet, the servo IMMEDIATELY
    # starts replying at NEW_ID — any further writes to CURRENT_ID
    # are ignored.
    bus.write(CURRENT_ID, _REG_ID, bytes([NEW_ID]))
    print("[3] new ID written (servo is now ID=%d)" % NEW_ID)
    time.sleep_ms(50)

    # Re-lock so the new ID survives accidental writes.
    bus.write(NEW_ID, _REG_LOCK, bytes([1]))
    print("[4] EEPROM re-locked")

    # Verify by pinging the new ID.
    if bus.ping(NEW_ID):
        print("[5] confirmed: servo now answers at ID %d." % NEW_ID)
    else:
        print("[5] no reply at ID %d — write may not have stuck." % NEW_ID)
        print("    Power-cycle the servo and run examples/st3215_scan.py.")


main()
