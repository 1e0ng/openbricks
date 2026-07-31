# SPDX-License-Identifier: MIT
"""
``openbricks upload -n NAME script.py`` — stage a script on the
hub. The hub does **not** run it automatically; the user presses the
hub button to launch (and presses again to stop).

Destination defaults to ``/program.py``. The firmware's frozen
``main.py`` watches the hub button and exec's that file on each short
press, so a fresh upload takes effect after the user places the
robot and presses the button.

(Pybricks calls this same operation ``download`` from the host's
"download to the hub" perspective. We use ``upload`` because from
the user's terminal the bytes are flowing *up* to the hub — the
direction-of-data-travel naming is more intuitive for new users
seeing this command for the first time.)

Transport: NUS + raw-paste mode via the same helpers ``run`` uses.
The upload runs a one-shot Python program on the hub that opens the
target path and writes the bytes, then prints a confirmation. No
``machine.reset()`` — the uploaded code does not execute until the
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


class UploadError(Exception):
    pass


DEFAULT_PROGRAM_PATH = "/program.py"


# Soft upper bound. Raw-paste can carry more, but a script above this
# is almost certainly an accident (stray binary blob) and we'd rather
# fail the client than spend a minute uploading.
_MAX_SCRIPT_BYTES = 64 * 1024


def _compose_confirm_program(target_path, expected_len):
    """The small post-staging program: sync the RTC and print the
    size confirmation the user sees. The payload itself is staged in
    bounded chunks by ``run_mod._stage_file`` — a one-shot paste of
    the whole file needs one contiguous hub-side buffer and dies on a
    fragmented heap (bench: 177 KB free, 5.2 KB max hole, 9.4 KB
    paste aborted)."""
    lines = run_mod.rtc_sync_lines() + [
        "import os",
        "print('uploaded', os.stat(%r)[6], 'bytes to', %r)" % (
            target_path, target_path),
        "assert os.stat(%r)[6] == %d, 'size mismatch after staging'" % (
            target_path, expected_len),
    ]
    return "\n".join(lines).encode() + b"\n"


async def _upload_async(name, script_path, target_path, scan_timeout):
    try:
        with open(script_path, "rb") as f:
            user_bytes = f.read()
    except OSError as e:
        raise UploadError(
            "cannot read script %r: %s" % (script_path, e))

    if len(user_bytes) > _MAX_SCRIPT_BYTES:
        raise UploadError(
            "script is %d bytes, exceeding the %d-byte soft limit; "
            "split the code or bump _MAX_SCRIPT_BYTES" % (
                len(user_bytes), _MAX_SCRIPT_BYTES))

    confirm_program = _compose_confirm_program(target_path, len(user_bytes))

    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout)
    except NUSError as e:
        raise UploadError(str(e))

    async with link:
        blink = run_mod._BufferedLink(link)
        await run_mod._enter_raw_repl(blink, link)
        try:
            await run_mod._stage_file(blink, link, target_path, user_bytes,
                                      name)
            await run_mod._raw_paste_upload(blink, link, confirm_program)
            await run_mod._stream_output(blink, link, sys.stdout)
        finally:
            try:
                await run_mod._restore_idle_loop(link)
            except Exception:
                pass


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    try:
        asyncio.run(_upload_async(
            args.name,
            args.script,
            args.path,
            args.scan_timeout,
        ))
    except UploadError:
        raise
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    print("\nready — press the hub button to run %s." % args.path,
          file=sys.stderr)
    return 0
