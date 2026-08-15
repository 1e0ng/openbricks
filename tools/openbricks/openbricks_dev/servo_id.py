# SPDX-License-Identifier: MIT
"""
``openbricks servo-id`` — assign a Feetech SCS/STS servo bus ID.

Two transports, one contract:

    openbricks servo-id 3           # via USB adapter (URT-2), auto-detected
    openbricks servo-id --scan      # who's there? (USB adapter)
    openbricks servo-id -n ls 3     # THROUGH THE HUB over BLE — the
    openbricks servo-id -n ls --scan  # servo stays wired to the robot

The adapter path talks the Feetech half-duplex protocol directly
from the host (e.g. the URT-2 board's USB port), so a fresh servo
can be given its bus ID before it ever meets the hub. The hub path
sends a one-shot program over BLE that does the same scan/re-ID on
the hub's own servo bus (firmware driver's echo-safe helpers; pins
--tx/--rx, default 14/41).

The adapter's port is auto-detected when exactly one USB serial
device is connected (same filter the flash command uses); with the
hub also plugged in there are two candidates, and the tool demands
``-p`` rather than guess which device's EEPROM to rewrite.

The ID lives in EEPROM register 0x05 behind the lock register 0x37
(0 = unlock, 1 = lock), so a write is unlock -> write -> re-lock at
the new ID. Safety differences from the usual one-shot scripts:

* The whole bus (IDs 0..253) is scanned first. With MORE than one
  servo attached the tool refuses to guess and demands ``--old-id``
  — re-ID'ing whichever servo happens to answer first is how two
  motors end up with the same ID.
* The result is verified: the new ID must answer a PING and the old
  ID must have gone silent, or the command fails loudly.

Wire format (Feetech SCS): ``FF FF <id> <len> <instr> <params...>
<checksum>`` where len = param count + 2 and checksum = bitwise-NOT
of the byte sum from id through the last param. A status reply is 6
bytes minimum: ``FF FF <id> <len> <error> <checksum>``.
"""

import sys

_BROADCAST_ID = 0xFE

_INSTR_PING  = 0x01
_INSTR_WRITE = 0x03

_REG_ID   = 0x05
_REG_LOCK = 0x37


class ServoIdError(Exception):
    pass


def _packet(sid, instr, params):
    """Build one Feetech SCS instruction packet."""
    body = bytes([sid, len(params) + 2, instr]) + bytes(params)
    return b"\xFF\xFF" + body + bytes([(~sum(body)) & 0xFF])


def _ping(ser, sid):
    """True if ``sid`` answers a PING with a well-formed status."""
    ser.reset_input_buffer()
    ser.write(_packet(sid, _INSTR_PING, []))
    resp = ser.read(6)
    return len(resp) == 6 and resp[:2] == b"\xFF\xFF" and resp[2] == sid


def _write_reg(ser, sid, reg, data):
    """WRITE ``data`` at ``reg``; the status reply is drained so it
    can't be misread as the next PING's answer."""
    ser.write(_packet(sid, _INSTR_WRITE, [reg] + list(data)))
    ser.read(6)


def scan_bus(ser):
    """All IDs (0..253) that answer a PING, ascending."""
    return [sid for sid in range(_BROADCAST_ID) if _ping(ser, sid)]


def set_servo_id(ser, new_id, old_id=None, out=None):
    """Scan, pick the source servo, rewrite its ID, verify.

    ``old_id=None`` means "the only servo on the bus" — with several
    attached that is ambiguous and raises instead of guessing.
    """
    out = out or sys.stdout
    found = scan_bus(ser)
    if not found:
        raise ServoIdError(
            "no servo answered on the bus — check power, wiring, and "
            "the adapter's slide switch")
    print("servos found: %s" % ", ".join(str(i) for i in found), file=out)

    if old_id is None:
        if len(found) > 1:
            raise ServoIdError(
                "%d servos on the bus — pass --old-id to say which one "
                "to re-ID (guessing could give two motors the same ID)"
                % len(found))
        old_id = found[0]
    elif old_id not in found:
        raise ServoIdError(
            "no servo at --old-id %d (bus answered: %s)"
            % (old_id, ", ".join(str(i) for i in found)))

    if old_id == new_id:
        print("servo already at ID %d, nothing to do" % new_id, file=out)
        return

    if new_id in found:
        raise ServoIdError(
            "ID %d is already taken by another servo on this bus"
            % new_id)

    _write_reg(ser, old_id, _REG_LOCK, [0])      # unlock EEPROM
    _write_reg(ser, old_id, _REG_ID, [new_id])   # rewrite the ID
    _write_reg(ser, new_id, _REG_LOCK, [1])      # re-lock at the new ID

    if not _ping(ser, new_id):
        raise ServoIdError(
            "servo does not answer at new ID %d after the write — "
            "power-cycle the servo and re-run --scan" % new_id)
    if _ping(ser, old_id):
        raise ServoIdError(
            "a servo still answers at old ID %d after the write — "
            "re-run --scan and check for duplicate-ID servos" % old_id)
    print("set servo ID %d -> %d (verified)" % (old_id, new_id), file=out)


def _open_serial(port, baudrate, timeout):
    """Open the adapter port. Separate function so tests can inject a
    scripted fake."""
    try:
        import serial
    except ImportError as e:
        raise ServoIdError(
            "pyserial is required for servo-id "
            "(pip install pyserial): %s" % e)
    try:
        return serial.Serial(port, baudrate, timeout=timeout)
    except Exception as e:
        raise ServoIdError("cannot open %s: %s" % (port, e))


_HUB_OK_SENTINEL = "SERVO-ID-OK"

# The hub-path program: same contract as the adapter path — full-bus
# scan, refusal to guess between servos, PING-verified result — run
# ON the hub over its own servo bus, reusing the firmware driver's
# echo-safe helpers. Mirrors examples/servo_set_id.py; the sentinel
# is how the host learns the hub side finished without an exception.
_HUB_PROGRAM = """\
import time
from openbricks.drivers.st3215 import _SCServoBus
NEW_ID = %(new_id)r
OLD_ID = %(old_id)r
SCAN_ONLY = %(scan)r
bus = _SCServoBus(1, %(tx)d, %(rx)d)
alive = []
for sid in range(254):
    if bus.ping(sid):
        alive.append(sid)
print("servos on the bus:", alive)
if SCAN_ONLY:
    print("%(sentinel)s")
else:
    old = OLD_ID
    if old is None:
        if not alive:
            raise ValueError("no servo answered the scan - check power and wiring")
        if len(alive) > 1:
            raise ValueError("%%d servos on the bus %%s - pass --old-id so the right one is re-ID'd" %% (len(alive), alive))
        old = alive[0]
    elif old not in alive:
        raise ValueError("no servo at --old-id %%d (bus has %%s)" %% (old, alive))
    if NEW_ID == old:
        raise ValueError("servo %%d already has that ID" %% old)
    if NEW_ID in alive:
        raise ValueError("ID %%d is already taken on this bus" %% NEW_ID)
    print("re-ID: %%d -> %%d" %% (old, NEW_ID))
    bus.write(old, 0x37, [0])
    bus.verify_writes = False
    bus.write(old, 0x05, [NEW_ID])
    bus.verify_writes = True
    time.sleep_ms(100)
    bus.write(NEW_ID, 0x37, [1])
    if not bus.ping(NEW_ID):
        raise RuntimeError("verify FAILED: ID %%d does not answer" %% NEW_ID)
    if bus.ping(old):
        raise RuntimeError("verify FAILED: old ID %%d still answers" %% old)
    print("set servo ID %%d -> %%d (verified)" %% (old, NEW_ID))
    print("%(sentinel)s")
"""


def _compose_hub_program(new_id, old_id, scan, tx, rx):
    return (_HUB_PROGRAM % {
        "new_id": new_id, "old_id": old_id, "scan": bool(scan),
        "tx": tx, "rx": rx, "sentinel": _HUB_OK_SENTINEL,
    }).encode()


class _TeeCapture:
    """stdout passthrough that remembers the streamed text, so the
    host can check for the hub program's success sentinel."""

    def __init__(self, out):
        self._out = out
        self.text = ""

    def write(self, s):
        self.text += s
        self._out.write(s)

    def flush(self):
        self._out.flush()


async def _hub_async(name, program, scan_timeout):
    import sys as _sys
    from openbricks_dev._nus import NUSLink, NUSError
    from openbricks_dev import run as run_mod
    print("connecting to %r ..." % name, file=_sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout)
    except NUSError as e:
        raise ServoIdError(str(e))
    tee = _TeeCapture(_sys.stdout)
    async with link:
        blink = run_mod._BufferedLink(link)
        await run_mod._enter_raw_repl(blink, link)
        try:
            await run_mod._raw_paste_upload(blink, link, program)
            await run_mod._stream_output(blink, link, tee)
        finally:
            try:
                await run_mod._restore_idle_loop(link)
            except Exception:
                pass
    return tee.text


def _run_hub(args):
    import asyncio
    program = _compose_hub_program(
        args.new_id, args.old_id, args.scan, args.tx, args.rx)
    try:
        text = asyncio.run(_hub_async(
            args.name, program, args.scan_timeout))
    except KeyboardInterrupt:
        import sys as _sys
        print("\naborted.", file=_sys.stderr)
        return 130
    if _HUB_OK_SENTINEL not in text:
        raise ServoIdError(
            "the hub-side program did not complete — see its output "
            "above")
    return 0


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    if not args.scan and args.new_id is None:
        raise ServoIdError("pass NEW_ID to assign, or --scan to look")
    if args.new_id is not None and not 0 <= args.new_id < _BROADCAST_ID:
        raise ServoIdError(
            "NEW_ID must be 0..%d (%d is the broadcast address)"
            % (_BROADCAST_ID - 1, _BROADCAST_ID))

    if getattr(args, "name", None) is not None:
        if args.port is not None:
            raise ServoIdError(
                "pass either -n (through the hub) or -p (through the "
                "USB adapter), not both")
        return _run_hub(args)

    port = args.port
    if port is None:
        from openbricks_dev._ports import autodetect_port
        port = autodetect_port(
            ServoIdError, "re-writing servo EEPROM through the wrong "
            "device would be destructive")
    ser = _open_serial(port, args.baudrate, args.timeout)
    try:
        if args.scan:
            found = scan_bus(ser)
            if found:
                print("servos found: %s"
                      % ", ".join(str(i) for i in found))
            else:
                print("no servo answered on the bus — check power, "
                      "wiring, and the adapter's slide switch")
            return 0
        set_servo_id(ser, args.new_id, old_id=args.old_id)
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass
