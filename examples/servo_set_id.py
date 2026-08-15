# SPDX-License-Identifier: MIT
"""Assign a Feetech servo's bus ID THROUGH THE HUB — no URT-2 USB
cable needed: the servo stays wired to the robot's bus and this runs
on the hub (``openbricks run -n NAME examples/servo_set_id.py``).

Set NEW_ID below (and OLD_ID when more than one servo is attached),
or set SCAN_ONLY = True to just list who answers. Safety matches the
``openbricks servo-id`` CLI tool: full-bus scan first, refusal to
guess between multiple servos, and the result is verified — the new
ID must answer a PING and the old ID must have gone silent. (The ID
write itself runs unverified: its status reply can already carry the
new ID, so the PING verification is the real check.)
"""

NEW_ID = 3
OLD_ID = None          # required when several servos are on the bus
SCAN_ONLY = False

UART_ID = 1
TX = 14
RX = 41

REG_ID = 0x05
REG_LOCK = 0x37

# --- control law (pure logic, unit-tested in tests/test_servo_set_id.py) ---


def resolve_target(alive, old_id, new_id):
    """The servo that gets re-ID'd, or ValueError with the reason."""
    if not 0 <= new_id <= 253:
        raise ValueError("NEW_ID must be 0..253 (254 is broadcast)")
    if old_id is None:
        if not alive:
            raise ValueError(
                "no servo answered the scan - check power and wiring")
        if len(alive) > 1:
            raise ValueError(
                "%d servos on the bus %s - set OLD_ID so the right "
                "one is re-ID'd" % (len(alive), alive))
        old_id = alive[0]
    elif old_id not in alive:
        raise ValueError(
            "no servo at OLD_ID %d (bus has %s)" % (old_id, alive))
    if new_id == old_id:
        raise ValueError("servo %d already has that ID" % old_id)
    if new_id in alive:
        raise ValueError("ID %d is already taken on this bus" % new_id)
    return old_id

# --- end control law ---


from openbricks.drivers.st3215 import _SCServoBus

bus = _SCServoBus(UART_ID, TX, RX)

print("scanning IDs 0..253 ...")
alive = []
for sid in range(254):
    if bus.ping(sid):
        alive.append(sid)
    if sid % 32 == 31:
        print("  ...%d/254, found %s" % (sid + 1, alive))
print("servos on the bus:", alive)

if SCAN_ONLY:
    print("SCAN_ONLY - nothing changed.")
else:
    target = resolve_target(alive, OLD_ID, NEW_ID)
    print("re-ID: %d -> %d" % (target, NEW_ID))
    bus.write(target, REG_LOCK, [0])
    bus.verify_writes = False
    bus.write(target, REG_ID, [NEW_ID])
    bus.verify_writes = True
    bus.write(NEW_ID, REG_LOCK, [1])
    if not bus.ping(NEW_ID):
        raise RuntimeError(
            "verify FAILED: ID %d does not answer - the servo may "
            "still be at %d" % (NEW_ID, target))
    if bus.ping(target):
        raise RuntimeError(
            "verify FAILED: old ID %d still answers - two servos on "
            "the bus may now share an ID" % target)
    print("done - servo now answers at ID %d (old ID %d silent)"
          % (NEW_ID, target))
