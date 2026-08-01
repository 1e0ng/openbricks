# SPDX-License-Identifier: MIT
"""
``openbricks log -n NAME`` — pull a script-run log file off a hub.

Every program executed via :mod:`openbricks.launcher` (button-press OR
``openbricks run``) gets its ``stdout`` + ``stderr`` tee'd to a
file under ``/openbricks_logs/`` on the hub. Ten rotating slots (the
firmware's ``openbricks.log.MAX_RUNS``). ``log`` reads any back.

Without arguments, prints the most-recent run's content. ``--list``
shows the file index. ``--run N`` selects a specific slot.

Transport: NUS + raw-paste, the same shape ``run`` and ``download``
use. The upload is a tiny one-shot Python program that imports
``openbricks.log``, runs the requested operation, and prints the
output. We stream the response back to the host's stdout.
"""

import asyncio
import sys
from datetime import datetime

from openbricks_dev._nus import NUSLink, NUSError
from openbricks_dev import run as run_mod


class LogError(Exception):
    pass


class _StampRenderer:
    """Line-oriented display filter for hub log dumps.

    The hub stores each log line prefixed with a raw int64 UTC Unix
    epoch in milliseconds (firmware >= 1.12.0). This wrapper converts
    that number to ``[YYYY-MM-DD HH:MM:SS.mmm]`` in the host's local
    timezone — the only place the conversion happens. Lines without a
    leading stamp (older-firmware logs, the ``-- run_N --`` header,
    ``--list`` rows) pass through untouched.

    ``write()`` may receive arbitrary chunk boundaries from the BLE
    stream, so partial lines are buffered until their newline arrives;
    call ``drain()`` after the stream ends to emit an unterminated
    tail.
    """

    def __init__(self, out):
        self._out = out
        self._tail = ""

    @staticmethod
    def _render(line):
        head, sep, rest = line.partition(" ")
        # Epoch ms between 2000-01-01 (12 digits, an unsynced-RTC hub)
        # and beyond 2100 (14 digits) — anything else is user output.
        if sep and 12 <= len(head) <= 14 and head.isdigit():
            dt = datetime.fromtimestamp(int(head) / 1000.0)
            return "[%s.%03d] %s" % (
                dt.strftime("%Y-%m-%d %H:%M:%S"),
                dt.microsecond // 1000, rest)
        return line

    def write(self, s):
        self._tail += s
        lines = self._tail.split("\n")
        self._tail = lines.pop()
        for line in lines:
            self._out.write(self._render(line) + "\n")

    def flush(self):
        self._out.flush()

    def drain(self):
        if self._tail:
            self._out.write(self._render(self._tail))
            self._tail = ""
        self._out.flush()


def _compose_list_program():
    """Print one ``run_<idx>\\t<bytes>`` per line, oldest first."""
    rtc = "\n".join(run_mod.rtc_sync_lines()) + "\n"
    return (
        rtc +
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
    rtc = "\n".join(run_mod.rtc_sync_lines()) + "\n"
    if index is None:
        return (
            rtc +
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
        rtc.encode() +
        (
            "from openbricks import log as _log\n"
            "try:\n"
            "    print(_log.read_run(%d))\n"
            "except OSError:\n"
            "    print('--no log--')\n"
        ).encode() % index
    )


async def _log_async(name, op_program, scan_timeout):
    """Run ``op_program`` on the hub via raw-paste and stream the
    response to stdout (log lines' epoch-ms stamps rendered as local
    time by ``_StampRenderer``). Shared between list / dump."""
    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout)
    except NUSError as e:
        raise LogError(str(e))

    async with link:
        blink = run_mod._BufferedLink(link)
        await run_mod._enter_raw_repl(blink, link)
        out = _StampRenderer(sys.stdout)
        try:
            await run_mod._raw_paste_upload(blink, link, op_program)
            await run_mod._stream_output(blink, link, out)
        finally:
            out.drain()
            try:
                await run_mod._restore_idle_loop(link)
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
