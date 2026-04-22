#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Flash an openbricks firmware image onto an ESP32 / ESP32-S3 hub and
write the hub's BLE name into NVS.

One firmware image is shared across every hub — per-hub identity (the
BLE advertising / addressing name, analogous to ``pybricks-dev run -n``)
is set here at flash time, not at build time. We require ``--name``:
two hubs that answer to the same name can't be individually addressed
over BLE, so refusing to flash a nameless hub is safer than picking a
shared default.

Usage:

    scripts/flash_firmware.py \\
        --name RobotA \\
        --port /dev/ttyUSB0 \\
        --firmware native/micropython/ports/esp32/build-openbricks_esp32s3/firmware.bin

Steps executed:

  1. ``esptool.py --port PORT erase_flash`` (clean slate — wipes NVS so
     the new name isn't shadowed by a stale one).
  2. ``esptool.py --port PORT write_flash 0x0 FIRMWARE`` (the combined
     openbricks image already includes bootloader + partition table).
  3. Wait for the device to boot, then ``mpremote connect PORT exec ...``
     to write the hub name into ``esp32.NVS("openbricks").hub_name``.
  4. Read the name back via mpremote and verify it matches.

Requirements (all pip-installable; cross-platform):

    pip install esptool mpremote

Windows/macOS/Linux all work — port syntax differs (``COM5`` on Windows,
``/dev/ttyUSB0`` / ``/dev/cu.usbserial-*`` elsewhere).
"""

import argparse
import shutil
import subprocess
import sys
import time


_NVS_NAMESPACE = "openbricks"
_NVS_KEY       = "hub_name"


def _die(msg, code=1):
    print("error: " + msg, file=sys.stderr)
    sys.exit(code)


def _require_tool(name):
    path = shutil.which(name)
    if path is None:
        _die("%s not found on PATH. Install with: pip install esptool mpremote" % name)
    return path


def _run(cmd, check=True):
    """Stream the subprocess output and optionally raise on non-zero exit."""
    print(">>> " + " ".join(cmd), flush=True)
    rc = subprocess.call(cmd)
    if check and rc != 0:
        _die("command failed (rc=%d): %s" % (rc, " ".join(cmd)), code=rc)
    return rc


def _mpremote_exec(mpremote, port, snippet):
    """Run a one-liner Python snippet on the device over mpremote.

    Returns (rc, stdout, stderr). We don't use ``_run`` because we need
    to capture stdout for the readback verification.
    """
    cmd = [mpremote, "connect", port, "exec", snippet]
    print(">>> " + " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout, proc.stderr


def _wait_for_repl(mpremote, port, timeout_s=20):
    """Poll mpremote until the device answers a trivial ``print``.

    After a flash the device reboots and the serial link takes a second
    or two to come back; ``mpremote`` has no built-in retry. Idle-poll
    every 500 ms.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        rc, out, err = _mpremote_exec(mpremote, port, "print('ok')")
        if rc == 0 and "ok" in out:
            return
        time.sleep(0.5)
    _die("timed out waiting for device REPL on %s" % port)


def _write_hub_name(mpremote, port, name):
    # Quote carefully: the name goes into a Python bytes literal. We
    # already banned quotes above, so a plain b"..." is fine.
    snippet = (
        "import esp32; "
        "nvs = esp32.NVS(%r); "
        "nvs.set_blob(%r, %r); "
        "nvs.commit(); "
        "print('wrote:', %r)"
    ) % (_NVS_NAMESPACE, _NVS_KEY, name.encode(), name)
    rc, out, err = _mpremote_exec(mpremote, port, snippet)
    if rc != 0:
        _die("failed to write hub name to NVS:\n" + (err or out))
    print(out.strip())


def _read_hub_name(mpremote, port):
    snippet = (
        "import esp32; "
        "nvs = esp32.NVS(%r); "
        "buf = bytearray(64); "
        "n = nvs.get_blob(%r, buf); "
        "print(bytes(buf[:n]).decode())"
    ) % (_NVS_NAMESPACE, _NVS_KEY)
    rc, out, err = _mpremote_exec(mpremote, port, snippet)
    if rc != 0:
        _die("failed to read hub name back from NVS:\n" + (err or out))
    return out.strip()


def main():
    ap = argparse.ArgumentParser(
        description="Flash openbricks firmware and write the hub's BLE name.",
    )
    ap.add_argument(
        "--name", required=True,
        help="Hub identifier for BLE. Required — two hubs with the "
             "same name can't be addressed individually. Keep it "
             "short (<=20 chars).",
    )
    ap.add_argument(
        "--port", required=True,
        help="Serial port (e.g. /dev/ttyUSB0, /dev/cu.usbserial-*, COM5).",
    )
    ap.add_argument(
        "--firmware", required=True,
        help="Path to the combined firmware.bin produced by "
             "scripts/build_firmware.sh.",
    )
    ap.add_argument(
        "--chip", default="auto",
        help="esptool --chip value (esp32, esp32s3, or 'auto').",
    )
    ap.add_argument(
        "--baud", default="460800",
        help="esptool flash baud rate (default: 460800).",
    )
    ap.add_argument(
        "--skip-erase", action="store_true",
        help="Skip erase_flash. Faster for iterative dev, but any stale "
             "NVS state survives — don't use for shipping a hub.",
    )
    args = ap.parse_args()

    # Validate name. The GAP advertising payload caps at ~29 bytes
    # including header, and set_blob takes a bytes value so non-ASCII
    # is OK — but keep it short and unambiguous.
    if not args.name:
        _die("--name cannot be empty")
    if len(args.name.encode()) > 29:
        _die("--name is %d bytes after UTF-8 encoding; BLE GAP caps at ~29"
             % len(args.name.encode()))
    if "\x00" in args.name:
        _die("--name cannot contain NUL bytes")

    esptool   = _require_tool("esptool.py")
    mpremote  = _require_tool("mpremote")

    print("=== openbricks flash: name=%r port=%s ===" % (args.name, args.port))

    if not args.skip_erase:
        _run([esptool, "--chip", args.chip, "--port", args.port, "erase_flash"])

    _run([
        esptool, "--chip", args.chip, "--port", args.port,
        "--baud", args.baud, "write_flash", "0x0", args.firmware,
    ])

    # esptool leaves the device in a reset state; give it time to boot
    # and the serial port time to re-enumerate on USB-CDC chips.
    time.sleep(2.0)
    _wait_for_repl(mpremote, args.port)

    _write_hub_name(mpremote, args.port, args.name)

    readback = _read_hub_name(mpremote, args.port)
    if readback != args.name:
        _die("verification failed: wrote %r, read back %r" % (args.name, readback))
    print("verified: hub name is %r" % readback)

    # Soft-reboot so user code sees the new name immediately.
    _run([mpremote, "connect", args.port, "reset"], check=False)

    print("done — hub %r flashed on %s." % (args.name, args.port))


if __name__ == "__main__":
    main()
