# SPDX-License-Identifier: MIT
"""
``openbricks-dev download -n NAME script.py`` — persist a Python script
to a hub so it runs automatically at every boot.

Same role as ``pybricks-dev download``: unlike ``run`` (which executes
once and goes away), ``download`` commits the script to ``/main.py``
on the hub's filesystem, which MicroPython runs on every power-up.

Boot-safety preamble
--------------------

We prepend a tiny bootstrap block to the user's script before writing:

    try:
        from openbricks import bluetooth
        bluetooth.apply_persisted_state()
    except Exception as _e:
        print("openbricks: BLE auto-start failed:", _e)

Reasons:

1. If the user's code doesn't start BLE themselves, a reboot would
   leave the hub silent and we'd lose the only recovery path
   (``openbricks-dev run`` / ``download`` both need BLE). The preamble
   brings BLE + REPL bridge up before any user code runs.
2. Wrapping in ``try/except`` means a missing/misnamed openbricks
   install doesn't brick boot — the user still gets a REPL on the
   next-tier fallback (USB serial).

Transport
---------

Same NUS + raw-paste bridge as ``openbricks-dev run``. We upload a
one-shot program that opens the target path and writes the combined
bytes, then optionally calls ``machine.reset()`` so the hub boots into
the new main.py immediately. Upload output streams back, so users see
the ``downloaded N bytes`` confirmation before the disconnect.
"""

import asyncio
import sys

from openbricks_dev._nus import NUSLink, NUSError
from openbricks_dev import run as run_mod


class DownloadError(Exception):
    pass


_BOOT_PREAMBLE = (
    "try:\n"
    "    from openbricks import bluetooth\n"
    "    bluetooth.apply_persisted_state()\n"
    "except Exception as _e:\n"
    "    print('openbricks: BLE auto-start failed:', _e)\n"
)


# Soft upper bound. Raw-paste can carry more, but a script above this
# is almost certainly accidentally-huge (maybe a stray binary blob)
# and we'd rather fail the client than spend a minute uploading.
_MAX_SCRIPT_BYTES = 64 * 1024


def _compose_payload(user_bytes):
    """Glue the boot-safety preamble to the user's script."""
    return _BOOT_PREAMBLE.encode() + b"\n" + user_bytes


def _compose_upload_program(target_path, payload, reset_after):
    """Return the Python source we'll upload to run on the hub.

    Uses a raw bytes literal via ``repr()`` so the payload round-trips
    byte-for-byte regardless of its contents (including NULs, high
    bits, and characters that confuse paste heuristics).
    """
    lines = [
        "with open(%r, 'wb') as f:" % target_path,
        "    f.write(%s)" % repr(payload),
        "print('downloaded', %d, 'bytes to', %r)" % (len(payload), target_path),
    ]
    if reset_after:
        # Reset after printing so the user sees confirmation before the
        # link drops. The hub will disconnect; we treat that as success.
        lines.append("import machine")
        lines.append("machine.reset()")
    return "\n".join(lines).encode() + b"\n"


async def _download_async(name, script_path, target_path, reset_after, scan_timeout):
    try:
        with open(script_path, "rb") as f:
            user_bytes = f.read()
    except OSError as e:
        raise DownloadError(
            "cannot read script %r: %s" % (script_path, e))

    if len(user_bytes) > _MAX_SCRIPT_BYTES:
        raise DownloadError(
            "script is %d bytes, exceeding the %d-byte soft limit; "
            "split the code or bump _MAX_SCRIPT_BYTES" % (
                len(user_bytes), _MAX_SCRIPT_BYTES))

    payload = _compose_payload(user_bytes)
    upload_program = _compose_upload_program(target_path, payload, reset_after)

    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout)
    except NUSError as e:
        raise DownloadError(str(e))

    async with link:
        blink = run_mod._BufferedLink(link)
        await run_mod._enter_raw_repl(blink, link)
        try:
            await run_mod._raw_paste_upload(blink, link, upload_program)
            try:
                await run_mod._stream_output(blink, sys.stdout)
            except run_mod.RunError:
                # When ``reset_after`` is True the hub hard-resets mid-
                # stream, so stream_output's 30 s read timeout will
                # fire. That's the success signal, not a failure.
                if not reset_after:
                    raise
        finally:
            if not reset_after:
                try:
                    await run_mod._leave_raw_repl(link)
                except Exception:
                    pass


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    try:
        asyncio.run(_download_async(
            args.name,
            args.script,
            args.path,
            not args.no_reset,
            args.scan_timeout,
        ))
    except DownloadError:
        raise
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    return 0
