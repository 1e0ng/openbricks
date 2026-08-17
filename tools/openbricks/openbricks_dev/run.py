# SPDX-License-Identifier: MIT
"""
``openbricks run -n NAME script.py`` — stage a script to the hub,
launch it immediately, stream output back, exit when the program stops.

Semantics mirror ``pybricksdev run``:

* The script is written to ``/program.py`` on the hub (same file
  ``openbricks upload`` stages to).
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
import os
import signal
import sys
import time

from openbricks_dev import mpycompile
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

# Where the script lands; same target as ``openbricks upload`` so
# the post-run state matches what a follow-up button press would rerun.
_TARGET_PATH = "/program.py"

# Compiled-program staging (CLI >= 1.92.0). The launcher's button
# path prefers /program.py when present, so every stage of one format
# removes the other — see _compose_runner(remove_stale=...).
_MPY_TARGET_PATH = "/program.mpy"

# ``run`` stages to the BUTTON slot (/program.*) — an upload-then-run
# (user decision 2026-08-17, reversing the 1.97.0 run-slot split and
# deliberately different from Pybricks): a run that fails mid-way
# leaves the program on the hub, so the start button reruns it
# without another BLE round trip. The accepted price is the 1.97.0
# hazard returning: running a calibration or diagnostic script
# replaces the button program (bench 2026-08-14: upload aa.py, run
# qtr_calibrate, press start -> the robot invisibly re-ran the
# calibration) — re-upload your mission after one-shot tools.

# First firmware whose launcher runs /program.mpy (native exec_mpy).
_MIN_MPY_FIRMWARE = (1, 92, 0)

_PROBE_VERSION_PROGRAM = (
    b"import openbricks\n"
    b"print('fwv=' + openbricks.__version__)\n")


def _parse_fw_version(text):
    """``(major, minor, patch)`` from the probe's ``fwv=X.Y.Z`` line,
    or ``None`` when absent or unparseable (which reads as "too old
    to stage .mpy" — the visible source-fallback notice names it)."""
    for line in text.splitlines():
        if line.startswith("fwv="):
            parts = line[4:].strip().split(".")
            if len(parts) < 3:
                return None
            try:
                return tuple(int(p) for p in parts[:3])
            except ValueError:
                return None
    return None


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

    # Pick a hint based on what the counters actually say. Three
    # broad cases at timeout: the hub never sent anything (BLE path
    # broken / unsubscribed), the hub started talking but went
    # quiet (script blocking on hardware), or it's still actively
    # sending but not the bytes we wanted.
    if s["notify_count"] == 0:
        hint = (
            "hint: notify_count=0 — the hub never sent anything. "
            "Its BLE notify path is broken or no central is registered. "
            "Re-run with --debug to log every notify packet as it arrives."
        )
    elif s["last_byte_ago"] is not None and s["last_byte_ago"] >= 5.0:
        hint = (
            "hint: hub WAS sending (%d packets, %d bytes), then went "
            "quiet for %.1fs. Most often: your script is blocked on "
            "hardware (sensor never responds, motor never reaches "
            "target). Re-run with --debug to see the last bytes; try "
            "a hello-world script first to confirm the BLE path is fine."
            % (s["notify_count"], s["byte_count"], s["last_byte_ago"])
        )
    else:
        hint = (
            "hint: hub is sending bytes but not what we expected at "
            "this step. Re-run with --debug to see exactly what it's "
            "sending and where the protocol diverged."
        )

    return (
        "timed out reading from hub\n"
        "  step:               %s\n"
        "  ble connected:      %s\n"
        "  link uptime:        %s\n"
        "  notify packets rx:  %d\n"
        "  bytes received:     %d (last byte: %s)\n"
        "  partial buffer:     %s\n"
        "%s"
    ) % (step, s["connected"], uptime, s["notify_count"],
         s["byte_count"], last_ago, partial_repr, hint)


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


# Raw-REPL entry is retried: the Ctrl-C is delivered on the hub as an
# injected KeyboardInterrupt, which raises in whatever main-thread
# frame is executing — a scheduled callback (BLE TX flush, button
# poll) can eat it, the same disease the stop button had. One eaten
# interrupt must cost one retry period, not a 30 s hang and a failed
# connect. Ctrl-C and Ctrl-A are both idempotent at the REPL, so
# re-sending is always safe. Shared by run / upload / log.
_RAW_REPL_ATTEMPTS = 6
_RAW_REPL_WAIT_S   = 4.0


async def _enter_raw_repl(blink, link):
    last_err = None
    for attempt in range(1, _RAW_REPL_ATTEMPTS + 1):
        blink._step = (
            "interrupting any running program (Ctrl-C, attempt %d/%d)"
            % (attempt, _RAW_REPL_ATTEMPTS))
        await link.write(b"\r" + _CTRL_C + _CTRL_C)
        await blink.drain()
        blink._step = (
            "waiting for raw REPL banner after Ctrl-A (attempt %d/%d)"
            % (attempt, _RAW_REPL_ATTEMPTS))
        await link.write(b"\r" + _CTRL_A)
        try:
            await blink.read_until(_RAW_REPL_BANNER,
                                   timeout=_RAW_REPL_WAIT_S)
            return
        except RunError as e:
            last_err = e
    raise last_err


async def _leave_raw_repl(link):
    await link.write(b"\r" + _CTRL_B)


# Fire-and-forget raw-REPL exec that re-enters the launcher idle loop.
# ``launcher.run()`` blocks forever by design, so callers must NOT wait
# for completion — only for the idle BANNER it prints on entry.
_RESTORE_IDLE_SNIPPET = (
    b"from openbricks import launcher\r"
    b"launcher.run()\r"
)
_RESTORE_BANNER = b"Press button to run"
_RESTORE_ATTEMPTS = 3
_RESTORE_WAIT_S = 1.5


async def _restore_idle_loop(link):
    """Re-enter the hub's launcher idle loop before disconnecting —
    VERIFIED, not fire-and-forget.

    Every BLE tool gets its REPL by Ctrl-C'ing whatever the hub was
    doing — usually the frozen main.py's ``launcher.run()`` idle loop.
    Leaving the hub parked at the REPL afterwards breaks the button
    workflow: with no idle loop draining, a button press starts the
    program through the degraded schedule-exec path (stop button
    starved), or — when the dispatch is lost entirely — does nothing
    at all.

    The old implementation sent the restore and disconnected. A write
    lost in the session-teardown race left the hub parked with a dead
    idle loop, and button presses silently died until a power-cycle —
    the intermittent "pressed N times, nothing" bench reports. Now we
    wait for the idle loop's own entry banner and retry the send; if
    the banner never comes, say so instead of leaving a silent trap.
    """
    for _ in range(_RESTORE_ATTEMPTS):
        try:
            await link.write(_RESTORE_IDLE_SNIPPET + _CTRL_D)
        except Exception:
            continue
        buf = b""
        deadline = time.monotonic() + _RESTORE_WAIT_S
        while time.monotonic() < deadline:
            try:
                chunk = await link.read(timeout=0.3)
            except Exception:
                break
            if chunk:
                buf += chunk
                if _RESTORE_BANNER in buf:
                    return
    print("warning: could not confirm the hub's idle loop restarted "
          "— the start button may be dead until a power-cycle or the "
          "next successful BLE session.", file=sys.stderr)


async def _raw_paste_upload(blink, link, script_bytes):
    # A retried raw-REPL entry can queue a SECOND banner: read_until
    # consumed only through the first one, so the duplicate's bytes
    # sit ahead of the handshake reply and read_exact answers "ra"
    # instead of "R\x01" (the wedge seen after an aborted run).
    # Nothing the hub sends before our request can matter — drop it.
    await blink.drain(timeout=0)
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
    noted_quiet = False
    while True:
        if blink._buf:
            chunk = bytes(blink._buf)
            blink._buf = bytearray()
        else:
            chunk = await blink._link.read(timeout=30.0)
            if not chunk:
                # A quiet program is not a dead hub: robot code
                # legitimately prints nothing for minutes while it
                # drives (a 4-side square at bench speeds is ~25 s of
                # silence). Only a dropped BLE link ends the wait —
                # the user always has Ctrl-C and the stop button.
                if link.stats().get("connected") is True:
                    if not noted_quiet:
                        print("hub quiet but connected — program "
                              "likely still driving; waiting "
                              "(Ctrl-C to abort)", file=sys.stderr)
                        noted_quiet = True
                    continue
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


def rtc_sync_lines():
    """Hub-side lines that set the RTC from the host's UTC clock.

    The ESP32 has no NTP; its RTC starts at 2000-01-01 on power-up,
    which would make the per-line epoch-ms stamps in the run logs
    (``openbricks/log.py``) meaningless. Every connect path (run /
    upload / log) prepends these lines, so any run started after a
    connect carries real wall-clock stamps. The RTC is set in UTC —
    conversion to local time happens only on the host at display.
    Guarded because unix-MicroPython test runs have no ``machine``.
    """
    from datetime import datetime, timezone
    now = datetime.now(timezone.utc)
    return [
        "try:",
        "    import machine",
        "    machine.RTC().datetime("
        "(%d, %d, %d, %d, %d, %d, %d, 0))"
        % (now.year, now.month, now.day, now.weekday(),
           now.hour, now.minute, now.second),
        "except Exception:",
        "    pass",
    ]


# Payload bytes staged per raw-paste exec. The hub buffers each paste
# program in ONE contiguous, doubling allocation — the bench capture
# that originally forced chunking (1.19.2) showed 177 KB free with a
# max contiguous hole of 5.2 KB, aborting a 9.4 KB one-shot paste; a
# conservative 512-byte chunk kept the doubling peak (≤ ~4x repr
# expansion for pure binary) well under that hole, at the cost of a
# round trip per 512 bytes — 16 round trips for a typical 8 KB script,
# each paying BLE's connection-interval latency, making run/upload
# feel much slower.
#
# 1.20.0 enabled the S3's 8 MB PSRAM (GC heap ~234 KB -> megabytes),
# which makes fragmentation holes of that size a non-issue on current
# boards. Set equal to _MAX_SCRIPT_BYTES: any script within the
# supported size limit now stages in exactly ONE round trip (matching
# pre-1.19.2 speed), while the chunking loop stays in place as a
# safety net if _MAX_SCRIPT_BYTES is ever raised past a single
# comfortably-sized buffer.
_STAGE_CHUNK_BYTES = _MAX_SCRIPT_BYTES


# Chunks at or above this size go over the wire zlib-compressed,
# keystream-XORed, and base64-framed; the hub inverts all three.
# Rationale (bench, 2026-07-31, --debug capture): raw-paste flow
# control grants a 128-byte window, so staging is paced at one window
# per BLE ack round trip — a 7.9 KB script measured ~16.6 s. Python
# source deflates 3-4x and base64 re-inflates by only 1.33x, while
# the old ``repr()`` framing EXPANDED unicode-heavy source (every
# em-dash became a 6-char escape). Net on-wire: ~0.6x of the raw
# size → staging time drops proportionally. Tiny payloads
# (interactive ``-c`` snippets) stay in plain repr framing — the
# import + header overhead isn't worth it below this line. The
# firmware requirements (``deflate``/``hashlib`` extmods, enabled on
# every openbricks build and the unix test port; a flashed hub name)
# fail LOUDLY via the staged program's own exception if ever absent —
# no silent downgrade.
_COMPRESS_MIN_BYTES = 512

# WHY NOT STREAMED (1.31.0 tried, 1.33.0 reverted): staging the
# payload through the receiver's ``sys.stdin`` after the raw paste
# ends does NOT work on the hub — the pasted program executes
# instantly with empty stdout AND empty stderr, i.e. as if nothing
# was compiled, so the host's first payload chunk is answered by the
# end-of-execution 0x04 instead of an ack. It reproduced on two
# different firmware builds (1.32.0 and 1.32.1) and its host-side
# "verification" was bogus: the check fed the program from a FILE
# with the payload on a pipe, which is not the raw-paste path at
# all. The embedded form below is the version that has actually
# completed a staging on hardware (11.8 s for a 7.9 KB script at the
# old 128-byte window; the 1.32.x window bump is what makes it fast).
# Don't re-attempt streaming without a way to verify it on the real
# BLE raw-paste path first.

# The XOR layer's keystream: SHA-256 over key + nonce + the block's
# byte offset, 32 bytes per block. The key is the HUB NAME — which is
# broadcast in every BLE advertisement, so this is NOT confidentiality
# against anyone sniffing the link (they see the name too). What it
# provides: (a) the payload only decodes on the hub that actually
# carries the addressed name in its NVS — a mis-targeted staging
# fails loudly at decompress instead of silently landing on the wrong
# robot — and (b) casual-observer obfuscation of the source on the
# air. Real secrecy would need a per-hub SECRET (e.g. a future
# ``flash --secret`` NVS blob) swapped in as the key; the pipeline
# would not otherwise change. sha256 is C-backed on both sides and
# adds no dependencies, which is also why it beats pulling in an AES
# library for a public key.
_XOR_BLOCK = 32


def _keystream_xor(data, key, nonce):
    """XOR ``data`` with the sha256(key+nonce+offset) keystream.
    Symmetric — the staged hub-side program applies the identical
    derivation (via big-int XOR there, byte loop here; same bytes)."""
    import hashlib
    out = bytearray(len(data))
    for i in range(0, len(data), _XOR_BLOCK):
        ks = hashlib.sha256(
            key + nonce + i.to_bytes(4, "big")).digest()
        blk = data[i:i + _XOR_BLOCK]
        for j in range(len(blk)):
            out[i + j] = blk[j] ^ ks[j]
    return bytes(out)


def _compose_stage_chunk(target_path, chunk, first, hub_name):
    """One staging step: append ``chunk`` to ``target_path`` (create
    on the first). Large chunks ship deflate + hub-name-keyed XOR +
    base64 and are inverted hub-side (see ``_COMPRESS_MIN_BYTES`` /
    ``_XOR_BLOCK``); small ones use plain ``repr()`` framing. Both
    round-trip any bytes."""
    mode = "wb" if first else "ab"
    if len(chunk) >= _COMPRESS_MIN_BYTES:
        import base64
        import os
        import zlib
        nonce = os.urandom(8)
        sealed = _keystream_xor(
            zlib.compress(chunk, 9), hub_name.encode(), nonce)
        b64 = base64.b64encode(sealed)
        return (
            "import deflate, io, binascii, hashlib, openbricks\n"
            "_hn = openbricks.HUB_NAME\n"
            "if _hn is None:\n"
            "    raise OSError('hub name unset; cannot derive the "
            "staging key')\n"
            "_k = _hn.encode()\n"
            "_sn = %s\n"
            "_ct = binascii.a2b_base64(%s)\n"
            "_ks = b''.join([hashlib.sha256("
            "_k + _sn + (_i * %d).to_bytes(4, 'big')).digest() "
            "for _i in range((len(_ct) + %d) // %d)])[:len(_ct)]\n"
            "_pt = (int.from_bytes(_ct, 'big') ^ "
            "int.from_bytes(_ks, 'big')).to_bytes(len(_ct), 'big')\n"
            "_d = deflate.DeflateIO(io.BytesIO(_pt), deflate.ZLIB)\n"
            "with open(%r, %r) as f:\n    f.write(_d.read())\n"
            % (repr(nonce), repr(b64),
               _XOR_BLOCK, _XOR_BLOCK - 1, _XOR_BLOCK,
               target_path, mode)).encode()
    return ("with open(%r, %r) as f:\n    f.write(%s)\n"
            % (target_path, mode, repr(chunk))).encode()


def _compose_runner(target_path, remove_stale=None):
    """The fixed-size program that runs the staged program at
    ``target_path``: sync the RTC, then trigger the hub-side launcher.
    ``remove_stale`` names a sibling staging path to delete first —
    staging ``/program.mpy`` removes ``/program.py`` so the launcher's
    button-path resolution ("source wins when present") always runs
    the most recently staged program. Wrapping the launcher call in a
    try/except turns a ``KeyboardInterrupt`` (from a button-press
    stop) into a clean print instead of a raw traceback — the client
    is exiting anyway, and we'd rather not scare the user with the
    message that comes with an uncaught interrupt.
    """
    remove_lines = []
    if remove_stale is not None:
        remove_lines = [
            "import os",
            "try:",
            "    os.remove(%r)" % remove_stale,
            "except OSError:",
            "    pass",
        ]
    lines = rtc_sync_lines() + remove_lines + [
        # Firmware identity first, provenance suffix included —
        # ``firmware_label`` exists since 1.79.0; older firmware
        # falls back to the bare version (it cannot know its own
        # provenance).
        "import openbricks",
        "try:",
        "    print('firmware ' + openbricks.firmware_label())",
        "except AttributeError:",
        "    print('firmware ' + openbricks.__version__)",
        "from openbricks import launcher",
        "try:",
        "    launcher.run_program(%r)" % target_path,
        "except KeyboardInterrupt:",
        "    print('openbricks: stopped by button press.')",
        # Evidence in the stream, read BEFORE anything can reboot or
        # re-init the counters: which path delivered the stop. The
        # third field (hard_stops) > 0 proves the hard-tick path
        # fired; 0 with presses > 0 means the Python watcher (defence
        # in depth) delivered it instead. Guarded: the bindings are
        # firmware-only.
        "    try:",
        "        from _openbricks_native import motor_process as _hb",
        "        print('openbricks: hard button (presses, releases, "
        "hard_stops, armed):', _hb.hard_button_stats())",
        "    except (ImportError, AttributeError):",
        "        pass",
    ]
    return ("\n".join(lines) + "\n").encode()


async def _exec_step(blink, link, program, what):
    """Raw-paste one SMALL program and consume its whole output
    framing (stdout \x04 stderr \x04 prompt). Returns the step's
    stdout bytes. Raises ``RunError`` carrying the hub's stderr if
    the step failed (e.g. OSError from a full filesystem while
    staging)."""
    await _raw_paste_upload(blink, link, program)
    blink._step = "%s (reading step output)" % what
    out = await blink.read_until(_CTRL_D)    # stdout
    err = await blink.read_until(_CTRL_D)    # stderr
    prompt = await blink.read_exact(1, timeout=10.0)
    if err.strip():
        raise RunError("hub error during %s:\n%s"
                       % (what, err.decode("utf-8", "replace")))
    if prompt != b">":
        raise RunError("unexpected byte %r after %s" % (prompt, what))
    return out


async def _stage_file(blink, link, target_path, payload, hub_name):
    """Write ``payload`` to ``target_path`` in bounded chunks so peak
    hub RAM is O(_STAGE_CHUNK_BYTES) regardless of script size.
    ``hub_name`` keys the wire framing of large chunks — see
    ``_compose_stage_chunk``."""
    for i in range(0, len(payload), _STAGE_CHUNK_BYTES):
        chunk = payload[i:i + _STAGE_CHUNK_BYTES]
        program = _compose_stage_chunk(target_path, chunk,
                                       first=(i == 0), hub_name=hub_name)
        await _exec_step(
            blink, link, program,
            "staging %s (%d/%d bytes)" % (
                target_path, min(i + _STAGE_CHUNK_BYTES, len(payload)),
                len(payload)))
    if not payload:
        await _exec_step(
            blink, link,
            _compose_stage_chunk(target_path, b"", first=True,
                                 hub_name=hub_name),
            "staging %s (empty)" % target_path)


async def _run_async(name, script_path, scan_timeout, debug=False, command=None):
    if command is not None and script_path is not None:
        raise RunError("specify either SCRIPT or -c CODE, not both")
    if command is None and script_path is None:
        raise RunError("missing program: pass a SCRIPT path or -c CODE")

    if command is not None:
        # Inline code path. ``-c "print('x')"`` mirrors ``python -c``;
        # we wrap the same launcher.run_program flow as the file path
        # so cleanup (motor_process reset, KeyboardInterrupt handling)
        # is identical.
        user_bytes = command.encode("utf-8")
        display_name = "<inline>"
    else:
        try:
            with open(script_path, "rb") as f:
                user_bytes = f.read()
        except OSError as e:
            raise RunError("cannot read script %r: %s" % (script_path, e))
        display_name = os.path.basename(script_path)
    if len(user_bytes) > _MAX_SCRIPT_BYTES:
        raise RunError(
            "script is %d bytes, exceeding the %d-byte soft limit" % (
                len(user_bytes), _MAX_SCRIPT_BYTES))

    # Compile BEFORE any BLE work: a syntax error costs milliseconds
    # here instead of a scan + connect + upload round-trip. Whether
    # the .mpy or the source is actually staged is decided in-session
    # by the firmware-version probe.
    mpy_bytes = mpycompile.compile_source(user_bytes, display_name)

    # Route Ctrl-C through a CONTROLLED cancellation instead of a raw
    # KeyboardInterrupt: a KI lands at whatever await bleak happens
    # to be in, and under a notification flood the loop then tears
    # down with CoreBluetooth callbacks still in flight — a native
    # interpreter crash, not a traceback (macOS "Python quit
    # unexpectedly", bench 2026-08-13).
    loop = asyncio.get_running_loop()
    _sigint_installed = _install_sigint(
        loop, _make_sigint_handler(asyncio.current_task()))

    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        try:
            link = await NUSLink.connect(name, scan_timeout=scan_timeout,
                                         debug=debug)
        except NUSError as e:
            raise RunError(str(e))

        await _run_session(link, name, user_bytes, mpy_bytes)
    finally:
        if _sigint_installed:
            loop.remove_signal_handler(signal.SIGINT)


def _make_sigint_handler(task):
    """First SIGINT cancels ``task`` — the session's stream handler
    turns that cancellation into the verified robot-stop and clean
    teardown. Further presses only acknowledge: cleanup is already
    running, and letting them land as KeyboardInterrupt inside
    ``close()`` would recreate the native-crash class this routing
    exists to prevent."""
    count = [0]

    def _on_sigint():
        count[0] += 1
        if count[0] == 1:
            task.cancel()
        else:
            print("\nstopping — please wait ...", file=sys.stderr)

    return _on_sigint


def _install_sigint(loop, handler):
    """Route SIGINT on ``loop`` to ``handler``. Returns False where
    per-loop signal handlers don't exist (e.g. Windows) — there the
    raw-KeyboardInterrupt path remains, crash risk and all."""
    try:
        loop.add_signal_handler(signal.SIGINT, handler)
        return True
    except (NotImplementedError, RuntimeError):
        return False


async def _pick_staging(blink, link, mpy_target=_MPY_TARGET_PATH,
                        src_target=_TARGET_PATH):
    """Probe the hub's firmware version and return the staging plan
    ``(target_path, use_mpy, remove_stale)`` for the given slot pair
    (upload = the button's /program.*, run = its own /run.* — the
    two intents must not clobber each other). Firmware >= 1.92.0
    gets the host-compiled ``.mpy``; anything older (or a version
    the probe can't parse) gets source, announced — never
    silently."""
    out = await _exec_step(blink, link, _PROBE_VERSION_PROGRAM,
                           "probing firmware version")
    fw = _parse_fw_version(out.decode("utf-8", "replace"))
    if fw is not None and fw >= _MIN_MPY_FIRMWARE:
        return mpy_target, True, src_target
    shown = ".".join(str(p) for p in fw) if fw else "unknown"
    print("hub firmware %s predates precompiled programs — sending "
          "source (flash firmware >= 1.92.0 for faster starts)" % shown,
          file=sys.stderr)
    return src_target, False, None


async def _run_session(link, name, user_bytes, mpy_bytes):
    async with link:
        blink = _BufferedLink(link)
        blink._step = "scan + connect"
        await _enter_raw_repl(blink, link)
        try:
            target, use_mpy, remove_stale = await _pick_staging(
                blink, link)
            payload = mpy_bytes if use_mpy else user_bytes
            runner = _compose_runner(target, remove_stale)
            await _stage_file(blink, link, target, payload, name)
            await _raw_paste_upload(blink, link, runner)
            out = sys.stdout
            try:
                await _stream_output(blink, link, out)
            except (asyncio.CancelledError, KeyboardInterrupt):
                # Host-side Ctrl-C — stop the ROBOT before the link
                # drops, and VERIFY it stopped.
                #
                # KeyboardInterrupt is the one that actually arrives:
                # asyncio.run does not convert SIGINT to
                # CancelledError, it raises KeyboardInterrupt at the
                # await point. Catching only CancelledError meant this
                # never ran — the CLI printed "aborted." and exited
                # while the robot kept driving, which is the worst
                # possible response to someone reaching for Ctrl-C.
                #
                # The stop is _enter_raw_repl, not one bare Ctrl-C
                # write: a single injected interrupt can be eaten by
                # a scheduled callback on the hub (the file-top
                # comment — one eaten interrupt costs one retry), and
                # an unverified write makes "stopped" and "still
                # driving" indistinguishable at the terminal. Reaching
                # the raw-REPL banner IS the proof the program died.
                #
                # A SECOND Ctrl-C lands as another KeyboardInterrupt
                # at whatever await the stop sequence is in. Absorb it
                # and restart the stop (Ctrl-C/Ctrl-A are idempotent)
                # instead of letting it abandon a moving robot
                # mid-stop.
                stopped = False
                for _ in range(3):
                    try:
                        blink._step = "stopping the robot"
                        await _enter_raw_repl(blink, link)
                        stopped = True
                        break
                    except (asyncio.CancelledError, KeyboardInterrupt):
                        continue
                    except Exception:
                        break
                if stopped:
                    print("\nrobot stopped.", file=sys.stderr)
                else:
                    print("\nWARNING: could not confirm the robot "
                          "stopped — the interrupt was sent but the "
                          "hub never acknowledged it. If the robot is "
                          "still moving, press its hub button or cut "
                          "power.", file=sys.stderr)
                raise
        finally:
            # The restore is the same critical section: a Ctrl-C here
            # (or a dead link) previously vanished into ``pass`` and
            # left the hub with a dead idle loop — button presses
            # silently doing nothing until a power-cycle. Absorb
            # repeat interrupts, and NAME the failure instead of
            # leaving the silent trap.
            restored = False
            for _ in range(3):
                try:
                    await _restore_idle_loop(link)
                    restored = True
                    break
                except (asyncio.CancelledError, KeyboardInterrupt):
                    continue
                except Exception:
                    break
            if not restored:
                print("WARNING: could not re-arm the hub's idle loop "
                      "— the button may not start programs until the "
                      "hub is power-cycled.", file=sys.stderr)


def run(args):
    """Subcommand entry. ``args`` is an argparse Namespace."""
    debug = getattr(args, "debug", False)
    command = getattr(args, "inline_code", None)
    try:
        asyncio.run(_run_async(args.name, args.script, args.scan_timeout,
                               debug=debug, command=command))
    except RunError:
        raise
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    return 0
