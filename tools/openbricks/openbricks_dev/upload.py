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
The upload is ONE raw-paste program on the hub: it prints the
firmware version, writes the target path, syncs the RTC, prints a
size confirmation and drops back into the launcher's idle loop —
the host reads the confirmation and the idle banner off the stream
and hangs up. No ``machine.reset()`` — the uploaded code does not
execute until the user presses the button.

For custom boot flows that replace the frozen ``main.py``, pass
``--path /main.py`` (or whichever path your own boot code reads from);
the default stays at ``/program.py`` so the out-of-the-box launcher
keeps working.
"""

import asyncio
import os
import sys
import time

from openbricks_dev._nus import NUSLink, NUSError
from openbricks_dev._uplock import UploadLock, UploadInProgress
from openbricks_dev import mpycompile
from openbricks_dev import run as run_mod


class UploadError(Exception):
    pass


DEFAULT_PROGRAM_PATH = "/program.py"


# Soft upper bound. Raw-paste can carry more, but a script above this
# is almost certainly an accident (stray binary blob) and we'd rather
# fail the client than spend a minute uploading.
_MAX_SCRIPT_BYTES = 64 * 1024


def _compose_confirm_lines(target_path, expected_len, remove_stale=None):
    """What follows the file write in the staged program: sync the
    RTC, delete the stale staging sibling (``/program.py`` after a
    ``.mpy`` stage — the launcher's button path runs source when
    present), print the size confirmation the user sees, check it
    against what was sent, and re-enter the launcher's idle loop so
    the button works the moment the link drops."""
    lines = run_mod.rtc_sync_lines() + ["import os"]
    if remove_stale is not None:
        lines += [
            "try:",
            "    os.remove(%r)" % remove_stale,
            "except OSError:",
            "    pass",
        ]
    lines += [
        "print('uploaded', os.stat(%r)[6], 'bytes to', %r)" % (
            target_path, target_path),
        "assert os.stat(%r)[6] == %d, 'size mismatch after staging'" % (
            target_path, expected_len),
        "from openbricks import launcher",
        "launcher.run()",
    ]
    return "\n".join(lines).encode() + b"\n"


def _compose_upload_program(target_path, payload, hub_name, use_mpy,
                            remove_stale=None):
    """The whole upload as ONE raw-paste program: version line, the
    file written in place, RTC, confirmation, idle loop. The launcher
    never returns, so the host reads the confirmation and the idle
    banner from the stream and then hangs up."""
    return run_mod._compose_staged_program(
        target_path, payload, hub_name, use_mpy,
        _compose_confirm_lines(target_path, len(payload), remove_stale))


# Kept for callers that composed the old three-exec flow's tail.
def _compose_confirm_program(target_path, expected_len, remove_stale=None):
    return _compose_confirm_lines(target_path, expected_len, remove_stale)


_IDLE_BANNER = run_mod._RESTORE_BANNER
_CONFIRM_WAIT_S = 20.0


async def _await_confirmation(blink, link):
    """After the staged program's version line: its stdout up to the
    launcher's idle banner (the confirmation is in there), or — when
    the program ended first — its stderr as an error. Returns the
    stdout text and whether the idle loop was seen."""
    blink._step = "waiting for the upload confirmation"
    deadline = time.monotonic() + _CONFIRM_WAIT_S
    while True:
        if _IDLE_BANNER in blink._buf:
            idx = blink._buf.index(_IDLE_BANNER)
            text = bytes(blink._buf[:idx]).decode("utf-8", "replace")
            blink._buf = bytearray()
            return text, True
        if run_mod._CTRL_D in blink._buf:
            # the program ended before the idle loop: an error
            idx = blink._buf.index(run_mod._CTRL_D)
            out = bytes(blink._buf[:idx]).decode("utf-8", "replace")
            blink._buf = blink._buf[idx + 1:]
            err = await blink.read_until(run_mod._CTRL_D)
            raise UploadError("hub error during the upload:\n%s%s" % (
                out, err.decode("utf-8", "replace")))
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise UploadError(run_mod._format_timeout(
                link, blink._step, blink._buf))
        try:
            await blink._fill(timeout=min(remaining, 5.0))
        except run_mod.RunError as e:
            raise UploadError(str(e))


def _phases_line(total, phases):
    """``staged in 1.23 s (scan 0.31, connect 0.62, ...)`` — the one
    line that tells where an upload's time went."""
    parts = ", ".join("%s %.2f" % (k, v) for k, v in phases if v is not None)
    return "staged in %.2f s (%s)" % (total, parts)


async def _upload_async(name, script_path, target_path, scan_timeout,
                        debug=False):
    """Stage the script; returns the hub path it landed on.

    ``target_path`` of ``None`` selects the default program flow:
    compile with mpy-cross on the host and stage ``/program.mpy``
    (source at ``/program.py`` for pre-1.92.0 firmware, announced).
    An explicit ``target_path`` stages the file VERBATIM — custom
    boot flows read whatever bytes their path holds.
    """
    t_start = time.monotonic()
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

    default_flow = target_path is None
    mpy_bytes = None
    if default_flow:
        # Compile before connecting — syntax errors cost milliseconds,
        # not a scan + connect. Which format is staged is decided by
        # what the CLI remembers of the hub (or an in-session probe).
        mpy_bytes = mpycompile.compile_source(
            user_bytes, os.path.basename(script_path))

    # One transfer per hub per machine (3.10.0): the OS shares the BLE
    # link between processes, so a second run/upload from another
    # terminal used to interleave its paste with this one. Refuse at
    # once — before the scan — while another transfer is in flight.
    # Held for the whole session: an upload IS its transfer.
    try:
        upload_lock = UploadLock(name)
        upload_lock.acquire()
    except UploadInProgress as e:
        raise UploadError(str(e))

    phases = []
    idle_seen = False
    try:
        print("connecting to %r ..." % name, file=sys.stderr)
        try:
            link = await NUSLink.connect(name, scan_timeout=scan_timeout,
                                         debug=debug)
        except NUSError as e:
            raise UploadError(str(e))
        timings = getattr(link, "timings", {}) or {}
        phases += [("scan", timings.get("scan")),
                   ("connect", timings.get("connect")),
                   ("subscribe", timings.get("subscribe"))]

        async with link:
            blink = run_mod._BufferedLink(link)
            t0 = time.monotonic()
            await run_mod._enter_raw_repl(blink, link)
            phases.append(("raw repl", time.monotonic() - t0))
            try:
                t0 = time.monotonic()
                if default_flow:
                    target_path, use_mpy, remove_stale = (
                        await run_mod._pick_staging(blink, link, name))
                    phases.append(("probe", time.monotonic() - t0))
                else:
                    use_mpy, remove_stale = False, None
                for _attempt in (1, 2):
                    payload = mpy_bytes if use_mpy else user_bytes
                    program = _compose_upload_program(
                        target_path, payload, name, use_mpy, remove_stale)
                    t0 = time.monotonic()
                    await run_mod._raw_paste_upload(blink, link, program)
                    phases.append(("paste", time.monotonic() - t0))
                    t0 = time.monotonic()
                    if default_flow:
                        fw = await run_mod._read_version_line(blink, name)
                        if use_mpy and fw < run_mod._MIN_MPY_FIRMWARE:
                            # the cache was stale (a re-flashed hub): the
                            # guard refused the compiled program
                            await run_mod._consume_refused_exec(blink)
                            run_mod._announce_source(fw)
                            target_path, use_mpy, remove_stale = (
                                run_mod._plan_for(fw))
                            continue
                    text, idle_seen = await _await_confirmation(blink, link)
                    phases.append(("confirm", time.monotonic() - t0))
                    for line in text.splitlines():
                        if line.strip():
                            print(line.rstrip("\r"))
                    sys.stdout.flush()
                    break
            finally:
                if not idle_seen:
                    # something went wrong before the staged program
                    # reached the idle loop: re-arm the button the old way
                    try:
                        await run_mod._restore_idle_loop(link)
                    except Exception:
                        pass
            t0 = time.monotonic()
        phases.append(("close", time.monotonic() - t0))
    finally:
        upload_lock.release()
    total = time.monotonic() - t_start
    print(_phases_line(total, phases), file=sys.stderr)
    return target_path


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    try:
        staged_path = asyncio.run(_upload_async(
            args.name,
            args.script,
            args.path,
            args.scan_timeout,
            debug=getattr(args, "debug", False),
        ))
    except UploadError:
        raise
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    print("\nready — press the hub button to run %s." % staged_path,
          file=sys.stderr)
    return 0
