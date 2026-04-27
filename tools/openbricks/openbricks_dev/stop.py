# SPDX-License-Identifier: MIT
"""
``openbricks-dev stop -n NAME`` — interrupt the script currently running
on a hub. Sends a single Ctrl-C over the NUS REPL bridge, which the
MicroPython REPL interprets as ``KeyboardInterrupt``.
"""

import asyncio
import sys

from openbricks_dev._nus import NUSLink, NUSError


class StopError(Exception):
    pass


_CTRL_C = b"\x03"


# Post-Ctrl-C drain window. Split out so tests can patch it to zero
# without monkeying with ``asyncio.sleep`` globally.
_DRAIN_DELAY_S = 0.3


async def _stop_async(name, scan_timeout):
    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout)
    except NUSError as e:
        raise StopError(str(e))

    async with link:
        # A single Ctrl-C is enough; some MP builds coalesce rapid
        # interrupts, so we don't pound on it.
        await link.write(_CTRL_C)
        # Drain briefly so the traceback (if any) lands in our transcript
        # — but don't block long; the user wants the command to return.
        await asyncio.sleep(_DRAIN_DELAY_S)
        tail = await link.read(timeout=0)
        if tail:
            sys.stdout.write(tail.decode("utf-8", "replace"))
            sys.stdout.flush()
    print("\nstopped.", file=sys.stderr)


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    try:
        asyncio.run(_stop_async(args.name, args.scan_timeout))
    except StopError:
        raise
    return 0
