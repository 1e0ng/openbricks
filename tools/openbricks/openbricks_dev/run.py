# SPDX-License-Identifier: MIT
"""
``openbricks-dev run -n NAME script.py`` — stage a script to the hub,
launch it immediately, stream output back, exit when the program stops.

Semantics mirror ``pybricks-dev run``:

* The script is written to ``/program.py`` on the hub (same file
  ``openbricks-dev upload`` stages to).
* The hub's launcher execs it right away — no button press required to
  start.
* While running, pressing the hub button raises ``KeyboardInterrupt``
  in the program (same path the launcher uses for upload-then-press).
* When the program stops (finished, raised, or interrupted by button),
  the terminal exits.

Transport: NUS + raw-paste mode. The upload is a single Python script
that does two things on the hub: write the staged file and call
``openbricks.launcher.run_program`` to execute it. That keeps all the
start/stop bookkeeping on the hub side where the Timer-driven button
watcher lives.
"""

import asyncio
import sys

from openbricks_dev._nus import NUSLink, NUSError


class RunError(Exception):
    pass


# Control bytes we send.
_CTRL_A = b"\x01"
_CTRL_B = b"\x02"
_CTRL_C = b"\x03"
_CTRL_D = b"\x04"
_CTRL_E = b"\x05"

# Raw-paste handshake.
_RAW_PASTE_REQUEST   = b"\x05A\x01"
_RAW_PASTE_SUPPORTED = b"R\x01"
_RAW_REPL_BANNER     = b"raw REPL; CTRL-B to exit\r\n>"

_FLOW_ACK   = b"\x01"
_FLOW_ABORT = b"\x04"

# Where the script lands; same target as ``openbricks-dev upload`` so
# the post-run state matches what a follow-up button press would rerun.
_TARGET_PATH = "/program.py"


# Soft upper bound — same as upload, since the upload shape is the same.
_MAX_SCRIPT_BYTES = 64 * 1024


def _format_timeout(link, step, partial_buf):
    """Build the multi-line error a timeout surfaces. Includes the
    NUS link's diagnostic counters plus what we were doing when the
    timeout fired and any partial bytes we had received — the
    minimum needed to tell which side of the link is broken.

    notify_count == 0 → hub never sent anything. Either the chip
    didn't process our writes, or its notify path is broken.

    notify_count > 0 but waiting → hub IS sending, but not what we
    expected. Re-run with --debug to see the actual bytes.
    """
    s = link.stats()
    last_ago = ("never" if s["last_byte_ago"] is None
                else "%.2fs ago" % s["last_byte_ago"])
    uptime   = ("?" if s["uptime"] is None
                else "%.2fs" % s["uptime"])
    partial  = bytes(partial_buf)
    partial_repr = repr(partial[:80]) + (" ..." if len(partial) > 80 else "")
    return (
        "timed out reading from hub\n"
        "  step:               %s\n"
        "  ble connected:      %s\n"
        "  link uptime:        %s\n"
        "  notify packets rx:  %d\n"
        "  bytes received:     %d (last byte: %s)\n"
        "  partial buffer:     %s\n"
        "hint: notify_count=0 means the hub never sent anything — its "
        "BLE notify path is broken or no central is registered. "
        "Re-run with --debug to log every notify packet as it arrives."
    ) % (step, s["connected"], uptime, s["notify_count"],
         s["byte_count"], last_ago, partial_repr)


class _BufferedLink:
    """Pushback buffer over ``NUSLink``. Same helper the upload flow
    uses — duplicated here instead of a cross-module import so neither
    subcommand leaks internals of the other.
    """

    def __init__(self, link):
        self._link = link
        self._buf = bytearray()
        # ``_step`` is what we were trying to do when a timeout fires —
        # set by the high-level functions below before each blocking
        # read, surfaced in the timeout error.
        self._step = "(unknown)"

    async def _fill(self, timeout):
        chunk = await self._link.read(timeout=timeout)
        if not chunk:
            raise RunError(_format_timeout(self._link, self._step, self._buf))
        self._buf += chunk

    async def read_exact(self, n, timeout=5.0):
        while len(self._buf) < n:
            await self._fill(timeout)
        out = bytes(self._buf[:n])
        self._buf = self._buf[n:]
        return out

    async def read_until(self, delim, timeout=30.0):
        while delim not in self._buf:
            await self._fill(timeout)
        idx = self._buf.index(delim)
        out = bytes(self._buf[:idx])
        self._buf = self._buf[idx + len(delim):]
        return out

    async def drain(self, timeout=0.3):
        try:
            chunk = await asyncio.wait_for(
                self._link.read(timeout=timeout), timeout=timeout + 0.1)
            if chunk:
                self._buf += chunk
        except asyncio.TimeoutError:
            pass
        self._buf = bytearray()


async def _enter_raw_repl(blink, link):
    blink._step = "interrupting any running program (Ctrl-C)"
    await link.write(b"\r" + _CTRL_C + _CTRL_C)
    await blink.drain()
    blink._step = "waiting for raw REPL banner after Ctrl-A"
    await link.write(b"\r" + _CTRL_A)
    await blink.read_until(_RAW_REPL_BANNER)


async def _leave_raw_repl(link):
    await link.write(b"\r" + _CTRL_B)


async def _raw_paste_upload(blink, link, script_bytes):
    blink._step = "raw-paste handshake (waiting for 'R\\x01')"
    await link.write(_RAW_PASTE_REQUEST)
    resp = await blink.read_exact(2)
    if resp != _RAW_PASTE_SUPPORTED:
        raise RunError(
            "hub did not acknowledge raw-paste mode (got %r); "
            "firmware older than MicroPython 1.14?" % resp)
    blink._step = "raw-paste handshake (reading window size)"
    win_bytes = await blink.read_exact(2)
    window_size = win_bytes[0] | (win_bytes[1] << 8)
    window_remaining = window_size

    i = 0
    n = len(script_bytes)
    while i < n:
        while window_remaining == 0 or len(blink._buf) > 0:
            b = await blink.read_exact(1, timeout=30.0)
            if b == _FLOW_ACK:
                window_remaining += window_size
            elif b == _FLOW_ABORT:
                await link.write(_CTRL_D)
                raise RunError("hub aborted the upload")
            else:
                raise RunError("unexpected byte %r during raw-paste upload" % b)
            if window_remaining > 0:
                break
        step = min(window_remaining, n - i)
        await link.write(script_bytes[i:i + step])
        window_remaining -= step
        i += step

    await link.write(_CTRL_D)
    while True:
        b = await blink.read_exact(1, timeout=10.0)
        if b == _CTRL_D:
            return
        if b == _FLOW_ACK:
            continue
        raise RunError("unexpected byte %r after raw-paste end" % b)


async def _stream_output(blink, link, out):
    """Stream stdout live, then stderr (typically a traceback) if any.

    The hub frames stdout and stderr each with a trailing ``\\x04``.
    Live stdout flushing lets users see prints as they happen; stderr
    arrives after stdout ends and is surfaced with a leading blank
    line so a traceback is visually distinct from normal output.
    """
    # --- stdout ---
    blink._step = "streaming script stdout"
    while True:
        if blink._buf:
            chunk = bytes(blink._buf)
            blink._buf = bytearray()
        else:
            chunk = await blink._link.read(timeout=30.0)
            if not chunk:
                raise RunError(_format_timeout(link, blink._step, blink._buf))
        idx = chunk.find(_CTRL_D)
        if idx >= 0:
            out.write(chunk[:idx].decode("utf-8", "replace"))
            out.flush()
            blink._buf = bytearray(chunk[idx + 1:])
            break
        out.write(chunk.decode("utf-8", "replace"))
        out.flush()

    # --- stderr ---
    blink._step = "streaming script stderr"
    err = await blink.read_until(_CTRL_D)
    if err:
        text = err.decode("utf-8", "replace")
        if text.strip():
            if not text.startswith("\n"):
                out.write("\n")
            out.write(text)
            out.flush()


def _compose_bootstrap(user_bytes):
    """Build the raw-paste payload: write /program.py, then trigger the
    hub-side launcher. Wrapping the launcher call in a try/except turns
    a ``KeyboardInterrupt`` (from a button-press stop) into a clean
    print instead of a raw traceback — the client is exiting anyway,
    and we'd rather not scare the user with the message that comes
    with an uncaught interrupt.
    """
    lines = [
        "with open(%r, 'wb') as f:" % _TARGET_PATH,
        "    f.write(%s)" % repr(user_bytes),
        "from openbricks import launcher",
        "try:",
        "    launcher.run_program(%r)" % _TARGET_PATH,
        "except KeyboardInterrupt:",
        "    print('openbricks: stopped by button press.')",
    ]
    return ("\n".join(lines) + "\n").encode()


async def _run_async(name, script_path, scan_timeout, debug=False):
    try:
        with open(script_path, "rb") as f:
            user_bytes = f.read()
    except OSError as e:
        raise RunError("cannot read script %r: %s" % (script_path, e))
    if len(user_bytes) > _MAX_SCRIPT_BYTES:
        raise RunError(
            "script is %d bytes, exceeding the %d-byte soft limit" % (
                len(user_bytes), _MAX_SCRIPT_BYTES))

    bootstrap = _compose_bootstrap(user_bytes)

    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout, debug=debug)
    except NUSError as e:
        raise RunError(str(e))

    async with link:
        blink = _BufferedLink(link)
        blink._step = "scan + connect"
        await _enter_raw_repl(blink, link)
        try:
            await _raw_paste_upload(blink, link, bootstrap)
            out = sys.stdout
            try:
                await _stream_output(blink, link, out)
            except asyncio.CancelledError:
                # Host-side Ctrl-C — forward and drain the interrupt
                # traceback before disconnecting.
                await link.write(_CTRL_C)
                try:
                    await _stream_output(blink, link, out)
                except Exception:
                    pass
                raise
        finally:
            try:
                await _leave_raw_repl(link)
            except Exception:
                pass


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    debug = getattr(args, "debug", False)
    try:
        asyncio.run(_run_async(args.name, args.script, args.scan_timeout, debug=debug))
    except RunError:
        raise
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    return 0
