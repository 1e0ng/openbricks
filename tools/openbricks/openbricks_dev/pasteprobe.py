# SPDX-License-Identifier: MIT
"""``openbricks paste-probe`` — measure what the hub's BLE input path
can actually absorb in one raw-paste burst.

Why this exists: two attempts to raise MicroPython's raw-paste
flow-control window (1.32.0 → 2048, 1.32.1 → 1024) both broke staging
on real hardware in different ways — a truncated program at 2048, a
mid-paste hang at 1024 — while the same firmware works at the stock
128. Raising the GATT rx buffer to 8 KB did not fix 1024, so the
limit is NOT simply that buffer, and no amount of desk reasoning
found it. This measures it.

The probe pastes padded no-op programs of increasing size through the
REAL raw-paste path and reports the largest that completes, plus how
each failure presented (truncated → the hub runs a fragment; hung →
the hub stops acking). Whatever the firmware advertises as its window
is what the host bursts, so running this on a build with a raised
window is what tells you whether that window is safe.

Run::

    openbricks paste-probe -n ls
    openbricks paste-probe -n ls --max 8192
"""

import asyncio
import sys
import time

from openbricks_dev._nus import NUSLink, NUSError
from openbricks_dev import run as run_mod


class PasteProbeError(Exception):
    pass


# Sizes to try, bytes of program text. Small enough to be quick,
# spread widely enough to bracket any plausible limit.
_SIZES = [128, 256, 512, 768, 1024, 1536, 2048, 3072, 4096, 6144, 8192]

_MARKER = "OBK-PASTE-OK"


def _padded_program(size):
    """A program of ~``size`` bytes that prints a marker. Padding is a
    comment, so compile cost stays trivial and only the TRANSFER is
    under test."""
    head = "print('%s')\n" % _MARKER
    pad = size - len(head)
    if pad <= 2:
        return head.encode()
    # ``#`` + newline framing keeps every line short (some paths care).
    body = []
    remaining = pad
    while remaining > 0:
        n = min(remaining, 70)
        body.append("#" + "x" * (n - 2))
        remaining -= n
    return (head + "\n".join(body) + "\n").encode()


def _hung(started):
    return ("HUNG during transfer after %.1fs — the hub stopped "
            "consuming and never sent a flow-control ack (bytes lost "
            "mid-burst)" % (time.monotonic() - started))


async def _try_size(blink, link, size, timeout):
    """Return (ok, detail). Never raises for a hub-side failure —
    the whole point is to characterise HOW it fails."""
    program = _padded_program(size)
    started = time.monotonic()
    try:
        await asyncio.wait_for(
            run_mod._raw_paste_upload(blink, link, program), timeout)
    except asyncio.TimeoutError:
        return False, _hung(started)
    except run_mod.RunError as e:
        # A stalled hub surfaces as the buffered link's own read
        # timeout, not asyncio's — that IS the hang (1.32.1's
        # symptom: consumption stops, no flow-control ack ever
        # arrives). Anything else is a genuine protocol error.
        if "timed out reading from hub" in str(e):
            return False, _hung(started)
        return False, "protocol error: %s" % str(e).splitlines()[0]

    try:
        out = await asyncio.wait_for(blink.read_until(run_mod._CTRL_D), timeout)
        err = await asyncio.wait_for(blink.read_until(run_mod._CTRL_D), timeout)
        prompt = await asyncio.wait_for(blink.read_exact(1), timeout)
    except (asyncio.TimeoutError, run_mod.RunError) as e:
        return False, "no complete reply after paste: %s" % e

    if prompt != b">":
        return False, "framing desync: prompt was %r" % prompt
    if err.strip():
        return False, ("hub raised: %s"
                       % err.decode("utf-8", "replace").strip().splitlines()[-1])
    if _MARKER.encode() not in out:
        return False, ("TRUNCATED — hub ran a fragment: no marker in "
                       "output %r (empty output means it compiled "
                       "nothing)" % out[:60])
    return True, "ok"


async def _probe_async(name, scan_timeout, max_size, timeout):
    print("connecting to %r ..." % name, file=sys.stderr)
    try:
        link = await NUSLink.connect(name, scan_timeout=scan_timeout)
    except NUSError as e:
        raise PasteProbeError(str(e))

    async with link:
        blink = run_mod._BufferedLink(link)
        await run_mod._enter_raw_repl(blink, link)
        print("hub advertises its window during each paste; bursting "
              "whatever it grants.\n")
        largest_ok = 0
        try:
            for size in _SIZES:
                if size > max_size:
                    break
                ok, detail = await _try_size(blink, link, size, timeout)
                print("  %6d bytes : %s" % (size, "OK" if ok else detail))
                if ok:
                    largest_ok = size
                else:
                    # Once it breaks it stays broken (the link is out
                    # of sync); stop rather than report noise.
                    break
        finally:
            await run_mod._restore_idle_loop(link)

        print()
        if largest_ok == 0:
            print("RESULT: even the smallest paste failed — the hub's "
                  "BLE input path is broken, not merely size-limited.")
        else:
            print("RESULT: largest paste that completed = %d bytes."
                  % largest_ok)
            print("        A raw-paste window is safe only if TWO windows "
                  "(the max the host may have in flight) fit under that,")
            print("        i.e. MICROPY_REPL_STDIN_BUFFER_MAX <= %d."
                  % largest_ok)
        return 0


def run(args):
    try:
        return asyncio.run(_probe_async(
            args.name, args.scan_timeout, args.max, args.timeout))
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
