# SPDX-License-Identifier: MIT
"""
``openbricks-dev download -n NAME script.py`` — stage a script on the
hub. The hub does **not** run it automatically; the user presses the
hub button to launch (and presses again to stop). Same workflow as
Pybricks Prime-hub ``pybricksdev download``.

Destination defaults to ``/program.py``. The firmware's frozen
``main.py`` watches the hub button and exec's that file on each short
press, so a fresh download takes effect after the user places the
robot and presses the button.

Transport: NUS + raw-paste mode via the same helpers ``run`` uses.
The upload runs a one-shot Python program on the hub that opens the
target path and writes the bytes, then prints a confirmation. No
``machine.reset()`` — the downloaded code does not execute until the
user triggers it.

For custom boot flows that replace the frozen ``main.py``, pass
``--path /main.py`` (or whichever path your own boot code reads from);
the default stays at ``/program.py`` so the out-of-the-box launcher
keeps working.
"""

import asyncio
import sys

from openbricks_dev._nus import NUSLink, NUSError
from openbricks_dev import run as run_mod


class DownloadError(Exception):
    pass


DEFAULT_PROGRAM_PATH = "/program.py"


# Soft upper bound. Raw-paste can carry more, but a script above this
# is almost certainly an accident (stray binary blob) and we'd rather
# fail the client than spend a minute uploading.
_MAX_SCRIPT_BYTES = 64 * 1024


def _compose_upload_program(target_path, payload):
    """Return the one-shot Python program we upload to the hub.

    Baking the payload into a ``repr()`` bytes literal lets any bytes
    (NULs, high bits, quotes, newlines) round-trip verbatim. The hub
    exec's this, writes the file, and prints a size confirmation that
    streams back to our terminal.
    """
    lines = [
        "with open(%r, 'wb') as f:" % target_path,
        "    f.write(%s)" % repr(payload),
        "print('downloaded', %d, 'bytes to', %r)" % (len(payload), target_path),
    ]
    return "\n".join(lines).encode() + b"\n"


async def _download_async(name, script_path, target_path, scan_timeout):
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

    upload_program = _compose_upload_program(target_path, user_bytes)

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
            await run_mod._stream_output(blink, sys.stdout)
        finally:
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
            args.scan_timeout,
        ))
    except DownloadError:
        raise
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    print("\nready — press the hub button to run %s." % args.path,
          file=sys.stderr)
    return 0
