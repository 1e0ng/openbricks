# SPDX-License-Identifier: MIT
"""
``openbricks-dev run -n NAME script.py`` — push a Python script to a hub
over BLE and stream its output to the terminal while it executes.

Transport: MicroPython **raw-paste mode** (same protocol ``mpremote``
uses over serial) on top of the NUS REPL bridge.

Why raw-paste over plain paste mode:

* Paste mode echoes every uploaded line back with a ``=== `` prefix,
  polluting the user's transcript.
* Raw-paste has window-based flow control so long scripts don't
  overrun a slow BLE link.
* stdout and stderr are framed by ``\\x04`` delimiters, so we can
  stream stdout live and still surface a clean exception block.

Flow (see ``micropython/ports/unix/modrepl.c`` for the device side):

    host: Ctrl-C Ctrl-C     # interrupt whatever was running
    host: Ctrl-A            # enter raw REPL
    hub:  "raw REPL; CTRL-B to exit\\r\\n>"
    host: Ctrl-E 'A' Ctrl-A # request raw-paste
    hub:  "R\\x01" + <LE16 window-size>
    host: <script bytes>    # respecting flow-control bytes:
    hub:  Ctrl-A            #   (raw REPL flow control: \\x01 → window refilled)
    host: Ctrl-D            # commit
    hub:  Ctrl-D            # ack — script starts running
    hub:  <stdout bytes...>
    hub:  Ctrl-D            # end of stdout
    hub:  <stderr bytes...>
    hub:  Ctrl-D            # end of stderr
    hub:  ">"               # back at raw REPL prompt

Host-side Ctrl-C forwards a Ctrl-C byte to the hub and lets the
traceback drain.
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

# Flow-control bytes the hub emits during raw-paste upload.
_FLOW_ACK   = b"\x01"   # window refilled by ``window_size`` bytes
_FLOW_ABORT = b"\x04"   # hub bailed; abort the upload


class _BufferedLink:
    """Thin buffer over ``NUSLink`` exposing exact-byte reads.

    ``NUSLink.read(timeout)`` returns whatever has arrived since last
    call — one notification or several coalesced. The raw-paste
    protocol needs to read specific byte counts and scan for
    delimiters, so we interpose a small pushback buffer.
    """

    def __init__(self, link):
        self._link = link
        self._buf = bytearray()

    async def _fill(self, timeout):
        chunk = await self._link.read(timeout=timeout)
        if not chunk:
            raise RunError("timed out reading from hub")
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
        """Best-effort consume whatever is buffered or arrives briefly.
        Used after an interrupt to flush tracebacks / prompts."""
        try:
            chunk = await asyncio.wait_for(
                self._link.read(timeout=timeout), timeout=timeout + 0.1)
            if chunk:
                self._buf += chunk
        except asyncio.TimeoutError:
            pass
        self._buf = bytearray()


async def _enter_raw_repl(blink, link):
    # Interrupt anything running. Two Ctrl-Cs because MP collapses very
    # rapid ones; one can race with a still-initialising interpreter.
    await link.write(b"\r" + _CTRL_C + _CTRL_C)
    await blink.drain()
    # Ctrl-A → raw REPL. Banner ends in "\r\n>" at raw-REPL-prompt level.
    await link.write(b"\r" + _CTRL_A)
    await blink.read_until(_RAW_REPL_BANNER)


async def _leave_raw_repl(link):
    await link.write(b"\r" + _CTRL_B)


async def _raw_paste_upload(blink, link, script_bytes):
    """Stream ``script_bytes`` to the hub under raw-paste flow control."""
    await link.write(_RAW_PASTE_REQUEST)
    resp = await blink.read_exact(2)
    if resp != _RAW_PASTE_SUPPORTED:
        raise RunError(
            "hub did not acknowledge raw-paste mode (got %r); is the "
            "firmware older than MicroPython 1.14?" % resp)
    win_bytes = await blink.read_exact(2)
    window_size = win_bytes[0] | (win_bytes[1] << 8)
    window_remaining = window_size

    i = 0
    n = len(script_bytes)
    while i < n:
        # Consume any flow-control bytes that landed since the last write,
        # then block if the window is empty.
        while window_remaining == 0 or len(blink._buf) > 0:
            b = await blink.read_exact(1, timeout=30.0)
            if b == _FLOW_ACK:
                window_remaining += window_size
            elif b == _FLOW_ABORT:
                # Tell the hub we heard it, then bail.
                await link.write(_CTRL_D)
                raise RunError("hub aborted the upload")
            else:
                # Any other byte here is a protocol violation; surface it.
                raise RunError("unexpected byte %r during raw-paste upload" % b)
            if window_remaining > 0:
                break

        step = min(window_remaining, n - i)
        await link.write(script_bytes[i:i + step])
        window_remaining -= step
        i += step

    # End-of-data marker.
    await link.write(_CTRL_D)
    # Consume any final window-ACKs then the mandatory Ctrl-D acknowledgement.
    while True:
        b = await blink.read_exact(1, timeout=10.0)
        if b == _CTRL_D:
            return
        if b == _FLOW_ACK:
            continue  # late ACK; ignore
        raise RunError("unexpected byte %r after raw-paste end" % b)


async def _stream_output(blink, out):
    """Pipe hub stdout → ``out`` until we hit the stdout-end Ctrl-D.

    The raw-REPL protocol frames stdout and stderr both with trailing
    ``\\x04``. We stream stdout live (flush after each chunk) so long-
    running prints are visible immediately. stderr (typically an
    exception traceback) arrives after stdout ends; we echo it with a
    blank-line separator when non-empty so the user can see it.
    """
    # --- stdout ---
    while True:
        # Drain whatever's in the pushback buffer first.
        if blink._buf:
            chunk = bytes(blink._buf)
            blink._buf = bytearray()
        else:
            chunk = await blink._link.read(timeout=30.0)
            if not chunk:
                raise RunError("timed out waiting for script output")
        idx = chunk.find(_CTRL_D)
        if idx >= 0:
            out.write(chunk[:idx].decode("utf-8", "replace"))
            out.flush()
            # Push anything past \x04 back for the stderr pass.
            blink._buf = bytearray(chunk[idx + 1:])
            break
        out.write(chunk.decode("utf-8", "replace"))
        out.flush()

    # --- stderr ---
    err = await blink.read_until(_CTRL_D)
    if err:
        text = err.decode("utf-8", "replace")
        if text.strip():
            # Separator so the exception is visually distinct from any
            # script output that preceded it.
            if not text.startswith("\n"):
                out.write("\n")
            out.write(text)
            out.flush()


async def _run_async(name, script_path, scan_timeout):
    try:
        with open(script_path, "rb") as f:
            script_bytes = f.read()
    except OSError as e:
        raise RunError("cannot read script %r: %s" % (script_path, e))

    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout)
    except NUSError as e:
        raise RunError(str(e))

    async with link:
        blink = _BufferedLink(link)
        await _enter_raw_repl(blink, link)
        try:
            await _raw_paste_upload(blink, link, script_bytes)
            out = sys.stdout
            try:
                await _stream_output(blink, out)
            except asyncio.CancelledError:
                # Host Ctrl-C: forward to the hub, then let the
                # traceback drain through the normal stream path.
                await link.write(_CTRL_C)
                try:
                    await _stream_output(blink, out)
                except Exception:
                    pass
                raise
        finally:
            # Leave raw REPL so the hub ends up at a friendly ">>> " prompt
            # for the next connection.
            try:
                await _leave_raw_repl(link)
            except Exception:
                pass


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    try:
        asyncio.run(_run_async(args.name, args.script, args.scan_timeout))
    except RunError:
        raise
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    return 0
