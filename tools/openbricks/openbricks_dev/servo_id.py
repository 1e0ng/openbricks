# SPDX-License-Identifier: MIT
"""
``openbricks servo-id`` — assign a Feetech SCS/STS servo bus ID.

Talks the Feetech half-duplex serial protocol directly from the host
through a USB serial adapter (e.g. the URT-2 board's USB port), so a
fresh servo can be given its bus ID before it ever meets the hub:

    openbricks servo-id 3        # set ID -> 3
    openbricks servo-id --scan   # who's there?

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


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    if not args.scan and args.new_id is None:
        raise ServoIdError("pass NEW_ID to assign, or --scan to look")
    if args.new_id is not None and not 0 <= args.new_id < _BROADCAST_ID:
        raise ServoIdError(
            "NEW_ID must be 0..%d (%d is the broadcast address)"
            % (_BROADCAST_ID - 1, _BROADCAST_ID))

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
