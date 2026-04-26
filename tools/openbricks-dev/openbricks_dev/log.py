# SPDX-License-Identifier: MIT
"""
``openbricks-dev log -n NAME`` — pull a script-run log file off a hub.

Every program executed via :mod:`openbricks.launcher` (button-press OR
``openbricks-dev run``) gets its ``stdout`` + ``stderr`` tee'd to a
file under ``/openbricks_logs/`` on the hub. Three rotating slots —
``run_0.log`` through ``run_2.log``. ``log`` reads any of them back.

Without arguments, prints the most-recent run's content. ``--list``
shows the file index. ``--run N`` selects a specific slot.

Transport: NUS + raw-paste, the same shape ``run`` and ``download``
use. The upload is a tiny one-shot Python program that imports
``openbricks.log``, runs the requested operation, and prints the
output. We stream the response back to the host's stdout.
"""

import asyncio
import sys

from openbricks_dev._nus import NUSLink, NUSError
from openbricks_dev import run as run_mod


class LogError(Exception):
    pass


def _compose_list_program():
    """Print one ``run_<idx>\\t<bytes>`` per line, oldest first."""
    return (
        "import os\n"
        "from openbricks import log as _log\n"
        "for idx, path in _log.list_runs():\n"
        "    try:\n"
        "        sz = os.stat(path)[6]\n"
        "    except OSError:\n"
        "        sz = -1\n"
        "    print('run_%d\\t%d\\t%s' % (idx, sz, path))\n"
    ).encode()


def _compose_dump_program(index):
    """Print the contents of the requested run, or ``--no log--`` on
    KeyError. ``index`` of ``None`` means "the latest run"."""
    if index is None:
        return (
            "from openbricks import log as _log\n"
            "_runs = _log.list_runs()\n"
            "if not _runs:\n"
            "    print('--no log--')\n"
            "else:\n"
            "    _idx, _path = _runs[-1]\n"
            "    print('-- run_%d (%s) --' % (_idx, _path))\n"
            "    with open(_path) as _f:\n"
            "        for _line in _f:\n"
            "            print(_line, end='')\n"
        ).encode()
    return (
        "from openbricks import log as _log\n"
        "try:\n"
        "    print(_log.read_run(%d))\n"
        "except OSError:\n"
        "    print('--no log--')\n"
    ).encode() % index


async def _log_async(name, op_program, scan_timeout):
    """Run ``op_program`` on the hub via raw-paste and stream the
    response to stdout. Shared between list / dump."""
    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout)
    except NUSError as e:
        raise LogError(str(e))

    async with link:
        blink = run_mod._BufferedLink(link)
        await run_mod._enter_raw_repl(blink, link)
        try:
            await run_mod._raw_paste_upload(blink, link, op_program)
            await run_mod._stream_output(blink, sys.stdout)
        finally:
            try:
                await run_mod._leave_raw_repl(link)
            except Exception:
                pass


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    if args.list:
        prog = _compose_list_program()
    else:
        prog = _compose_dump_program(args.run)
    try:
        asyncio.run(_log_async(args.name, prog, args.scan_timeout))
    except LogError:
        raise
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    return 0
