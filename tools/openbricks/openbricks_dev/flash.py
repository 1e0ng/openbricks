# SPDX-License-Identifier: MIT
"""
``openbricks-dev flash`` — flash firmware + bake in the hub's BLE name.

Two-step flow:

  1. ``esptool ... write-flash`` writes the combined firmware image.
  2. ``mpremote ... exec`` pokes the BLE advertising name into NVS under
     namespace ``openbricks``, key ``hub_name`` (where ``openbricks``
     reads it via ``openbricks._read_hub_name``).

Step 1 is commodity tooling; step 2 is how we achieve per-device identity
without rebuilding the firmware per hub. The name is verified via an NVS
readback after the write, so a quiet failure on the device can't be
mistaken for success.
"""

import shutil
import subprocess
import sys
import time


_NVS_NAMESPACE = "openbricks"
_NVS_KEY       = "hub_name"


class FlashError(Exception):
    """Raised by ``run`` when any step of the flash / name-write fails."""


def _die(msg):
    raise FlashError(msg)


def _require_tool(name):
    path = shutil.which(name)
    if path is None:
        _die("%s not found on PATH — install with: pip install esptool mpremote"
             % name)
    return path


def _esptool_paths_and_commands():
    """Return ``(binary_path, write_cmd, erase_cmd)`` for the esptool
    install on PATH.

    Prefer the v5 binary name (``esptool``) and v5 kebab-case
    commands (``write-flash`` / ``erase-flash``) when both are
    available — output is clean of "DEPRECATED" warnings. Fall
    back to the v4 names when v5 isn't installed (typical on
    Python 3.9, since esptool 5+ requires Python ≥ 3.10).
    """
    if shutil.which("esptool") is not None:
        return shutil.which("esptool"), "write-flash", "erase-flash"
    if shutil.which("esptool.py") is not None:
        return shutil.which("esptool.py"), "write_flash", "erase_flash"
    _die("esptool not found on PATH — install with: pip install esptool mpremote")


def _run(cmd, check=True):
    """Stream subprocess output; optionally raise on non-zero exit."""
    print(">>> " + " ".join(cmd), flush=True)
    rc = subprocess.call(cmd)
    if check and rc != 0:
        _die("command failed (rc=%d): %s" % (rc, " ".join(cmd)))
    return rc


def _mpremote_exec(mpremote, port, snippet):
    """Run a one-liner Python snippet on the device over mpremote.

    Returns ``(returncode, stdout, stderr)``. ``_run`` isn't used here
    because we need to capture stdout for readback verification.

    The ``resume`` argument is critical: without it, each
    ``mpremote exec`` invocation does a soft reset before entering
    raw REPL. Once the hub name has been written to NVS, the
    soft-reset re-runs frozen ``main.py`` which now activates BLE
    and blocks in ``launcher.run()`` — and the *next* mpremote
    invocation can't enter raw REPL, failing the readback step
    with "could not enter raw repl". ``resume`` flips
    ``_auto_soft_reset`` to ``False`` so the chip's REPL state
    persists across our flash-flow steps.
    """
    cmd = [mpremote, "connect", port, "resume", "exec", snippet]
    print(">>> " + " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout, proc.stderr


def _wait_for_repl(mpremote, port, timeout_s=20):
    """Poll mpremote until the device answers a trivial ``print``.

    After flashing, the device reboots and the serial link takes a second
    or two to come back; mpremote has no built-in retry. Idle-poll every
    500 ms.

    On timeout, surface mpremote's last rc / stdout / stderr so the
    user can see *why* it's failing — a bare "timed out" message is
    indistinguishable between "chip is stuck in user code that never
    yields to the REPL", "chip is in download mode after a flash
    failure", and "USB-Serial-JTAG never re-enumerated".
    """
    deadline = time.time() + timeout_s
    last_rc, last_out, last_err = -1, "", ""
    while time.time() < deadline:
        last_rc, last_out, last_err = _mpremote_exec(mpremote, port, "print('ok')")
        if last_rc == 0 and "ok" in last_out:
            return
        time.sleep(0.5)
    _die(
        "timed out waiting for device REPL on %s after %.0fs\n"
        "  last mpremote rc:     %d\n"
        "  last mpremote stdout: %r\n"
        "  last mpremote stderr: %r\n"
        "hint: if stderr mentions 'could not enter raw repl', the chip's\n"
        "      main.py is stuck in user code — power-cycle and try again,\n"
        "      or hold the BOOT button while pressing reset to enter the\n"
        "      ROM bootloader (then re-run with --skip-erase)."
        % (port, timeout_s, last_rc, last_out.strip(), last_err.strip())
    )


def _write_hub_name(mpremote, port, name):
    # The name already passed validation in ``_validate_name``; plain
    # bytes literal is safe.
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


def _validate_name(name):
    if not name:
        _die("--name cannot be empty")
    if len(name.encode()) > 29:
        _die("--name is %d bytes after UTF-8 encoding; BLE GAP caps at ~29"
             % len(name.encode()))
    if "\x00" in name:
        _die("--name cannot contain NUL bytes")


def run(args):
    """Subcommand entry. ``args`` is an argparse ``Namespace``."""
    _validate_name(args.name)

    # esptool v5 renamed the binary (``esptool.py`` → ``esptool``) and
    # switched commands to kebab-case (``write_flash`` → ``write-flash``,
    # ``erase_flash`` → ``erase-flash``). The legacy forms still work in
    # v5 but emit deprecation warnings on every flash. We can't pin v5
    # as a floor (it requires Python >= 3.10 and openbricks supports
    # >= 3.9), so detect which is on PATH at runtime and pick command
    # spelling accordingly.
    esptool, write_cmd, erase_cmd = _esptool_paths_and_commands()
    mpremote = _require_tool("mpremote")

    print("=== openbricks flash: name=%r port=%s ===" % (args.name, args.port))

    if not args.skip_erase:
        _run([esptool, "--chip", args.chip, "--port", args.port, erase_cmd])

    _run([
        esptool, "--chip", args.chip, "--port", args.port,
        "--baud", args.baud, write_cmd, "0x0", args.firmware,
    ])

    # esptool leaves the device reset; give USB-CDC ports time to
    # re-enumerate before asking mpremote to reconnect.
    time.sleep(2.0)
    _wait_for_repl(mpremote, args.port)

    _write_hub_name(mpremote, args.port, args.name)

    readback = _read_hub_name(mpremote, args.port)
    if readback != args.name:
        _die("verification failed: wrote %r, read back %r" % (args.name, readback))
    print("verified: hub name is %r" % readback)

    # Trigger a hardware reset so the freshly-flashed chip boots into
    # the BLE-active runtime state (now that hub_name is in NVS).
    #
    # Why ``resume exec --no-follow machine.reset()`` rather than the
    # ``mpremote ... reset`` alias: the alias expands to
    # ``exec --no-follow "machine.reset()"`` with the default
    # ``enter_raw_repl(soft_reset=True)``. That soft reset itself
    # boots the chip into the new state, blocking in
    # ``launcher.run()`` before mpremote can reach the raw REPL to
    # actually send ``machine.reset()`` — surfacing as
    # ``TransportError: could not enter raw repl``. ``resume`` flips
    # ``_auto_soft_reset`` off so we go straight from friendly REPL
    # into raw REPL once, send the snippet, and disconnect; the
    # snippet's own ``machine.reset()`` is what reboots the chip.
    _run([mpremote, "connect", args.port, "resume",
          "exec", "--no-follow",
          "import machine; machine.reset()"], check=False)

    print("done — hub %r flashed on %s." % (args.name, args.port))
    return 0


def main_standalone():
    """Called when someone invokes this module directly.

    Not the primary entry point (``cli.main`` is) — present so the module
    is runnable as ``python -m openbricks_dev.flash ...`` during
    development.
    """
    from openbricks_dev.cli import _build_parser
    parser = _build_parser()
    # Reparse argv with 'flash' prepended so the user can invoke without
    # typing the subcommand name.
    args = parser.parse_args(["flash"] + sys.argv[1:])
    try:
        return run(args)
    except FlashError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main_standalone())
