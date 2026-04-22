# SPDX-License-Identifier: MIT
"""
``openbricks-dev run -n NAME script.py`` — push a Python script to a hub
over BLE and stream its output to the terminal while it executes.

Semantics mirror ``pybricks-dev run``: the script is transient — it runs
once in the REPL's context and nothing persists to flash. For "install
and auto-run at boot" use ``openbricks-dev download`` (PR 4).

Transport: MicroPython **paste mode** over the NUS REPL bridge.

    host: Ctrl-C                 # interrupt whatever was running
    host: Ctrl-E                 # enter paste mode
    hub:  "paste mode; ..."      # acks
    host: <script bytes>
    host: Ctrl-D                 # commit, execute
    hub:  <stdout/stderr stream>
    hub:  ">>> "                 # back at normal prompt → done

We picked paste mode over raw REPL (Ctrl-A / Ctrl-D framing) because
raw REPL delivers stdout+stderr in two batched chunks at the end,
which kills live streaming — users writing ``for i in range(10):
print(i); time.sleep(1)`` want to see digits appear every second, not
a 10-second stall followed by a wall of text. Paste mode echoes the
sent script line-by-line (the "=== " prefix), which is a small visible
artifact but gives us the streaming semantics with no state machine.

Host-side Ctrl-C interrupts the remote program; we forward a Ctrl-C
byte to the hub and keep the connection open so the interrupt traceback
streams back.
"""

import asyncio
import sys

from openbricks_dev._nus import NUSLink, NUSError


class RunError(Exception):
    pass


# Control characters in the cooked REPL protocol.
_CTRL_C = b"\x03"
_CTRL_D = b"\x04"
_CTRL_E = b"\x05"

# The paste-mode prompt begins with "=== " on its own line. Checking for
# the literal banner is brittle across MP versions, so we wait for the
# "=== " prefix which hasn't changed since the feature landed.
_PASTE_BANNER_MARKER = b"paste mode"


async def _stream_until(link, marker, echo_out=None):
    """Pull notifications until ``marker`` appears in the stream.

    ``echo_out`` (a file-like, typically ``sys.stdout``) receives
    whatever bytes we see so users see output live. We decode as
    UTF-8 with error replacement — the REPL is text, but occasional
    stray high bytes shouldn't abort the run.
    """
    buf = bytearray()
    while marker not in buf:
        chunk = await link.read(timeout=30.0)
        if not chunk:
            raise RunError("timed out waiting for %r" % marker)
        buf += chunk
        if echo_out is not None:
            echo_out.write(chunk.decode("utf-8", "replace"))
            echo_out.flush()
    return bytes(buf)


async def _enter_paste_mode(link):
    # Kick anything that was running; a couple of Ctrl-Cs are idiomatic
    # because interrupt handling on MP can coalesce very-rapid ones.
    await link.write(b"\r" + _CTRL_C + _CTRL_C)
    # Drain whatever the interrupt may have dumped (traceback, prompt).
    await link.read(timeout=0.3)
    # Enter paste mode.
    await link.write(_CTRL_E)
    # Paste-mode banner — don't echo it to the user, it's a protocol
    # artifact, not script output.
    await _stream_until(link, _PASTE_BANNER_MARKER, echo_out=None)
    # Eat the rest of the banner line + the initial "=== " prompt so
    # what follows is just our script echo + its output.
    await link.read(timeout=0.2)


async def _execute_and_stream(link, script_bytes, out=None):
    # Resolve stdout at call time so tests that patch ``sys.stdout``
    # capture the stream properly.
    if out is None:
        out = sys.stdout
    await link.write(script_bytes)
    # Ctrl-D commits the paste. Everything after is the live run.
    await link.write(_CTRL_D)
    # Stream until the REPL prompt reappears (script finished or raised).
    try:
        await _stream_until(link, b">>> ", echo_out=out)
    except asyncio.CancelledError:
        # Host-side Ctrl-C — forward to hub so the program actually stops.
        await link.write(_CTRL_C)
        # Let the interrupt traceback drain.
        try:
            await _stream_until(link, b">>> ", echo_out=out)
        except Exception:
            pass
        raise


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
        await _enter_paste_mode(link)
        await _execute_and_stream(link, script_bytes)


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    try:
        asyncio.run(_run_async(args.name, args.script, args.scan_timeout))
    except RunError:
        raise
    except KeyboardInterrupt:
        # _execute_and_stream swallows the first Ctrl-C to forward it to
        # the hub. A second Ctrl-C here bubbles up.
        print("\naborted.", file=sys.stderr)
        return 130
    return 0
