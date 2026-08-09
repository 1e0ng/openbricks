# SPDX-License-Identifier: MIT
"""
Per-run log capture: tee ``print(...)`` output to a file on flash so
untethered runs can be inspected later via ``openbricks log``.

The launcher wraps every program execution with ``log.session()`` so
the user's ``print`` output streams to *both* the live USB / BLE
console (when one's listening) and a rotating file on flash. With
nobody listening on the live channel, the file is the only record.
``openbricks log`` reads the most recent files back over BLE.

Storage layout::

    /openbricks_logs/run_0.log
    /openbricks_logs/slot_1.log
    /openbricks_logs/slot_2.log

Each run gets the next index; the index lives in the file's header
line (``"<epoch> -- run_N --"``), NOT the filename. The
``MAX_RUNS`` slot files are reused in place (``run N`` overwrites
``slot_(N % MAX_RUNS)``) — the earlier delete+create rotation
churned littlefs directory metadata until every commit's allocator
traversal cost ~400 ms (bench 2026-08-09); truncate-reuse keeps
commits at fresh-filesystem cost. Flash usage stays bounded and
indices grow monotonically, exactly as before.

Every line is prefixed with a raw int64 **UTC Unix epoch in
milliseconds** (e.g. ``1783950123456 left ambient: 33``). No
formatting, no timezone on the hub — the host CLI converts to the
user's local time at display. The ESP32 RTC starts at 2000-01-01 on
power-up; the CLI syncs it from the host clock on every connect, so
runs started after any ``openbricks run`` / ``log`` / ``upload``
carry real wall-clock stamps (an unsynced run shows year-2000
dates, which is self-diagnosing).

The session is also bytes-capped: once a run's log file passes
``MAX_BYTES`` bytes, further writes are dropped from the file (the
live console still gets them). This keeps a runaway
``while True: print(...)`` from filling the entire flash partition.

Implementation note: MicroPython doesn't expose ``sys.stdout`` as a
re-bindable attribute on every port, so we tee at the
``builtins.print`` level instead. This catches every ``print(...)``
call — including ones with ``file=sys.stderr`` — but does not catch
direct ``sys.stdout.write()`` calls. User code on the firmware path
overwhelmingly goes through ``print()``, so this trade-off is fine.
The launcher additionally calls ``log.write_text(...)`` from its
exception handler so tracebacks are captured.
"""

import builtins
import os
import time


LOG_DIR    = "/openbricks_logs"
# 10, was 3. Three slots twice destroyed the evidence they existed to
# keep: an intermittent won't-start is diagnosed by comparing the
# FAILING run's log against a working one, but every diagnostic
# session (``openbricks run -c`` state dumps, bus scans) is itself a
# run that takes a slot — by the time the bench report arrived, the
# failing runs had been rotated out by the tools investigating them.
# Worst case 10 x 64 KB = 640 KB of a 16 MB flash.
MAX_RUNS   = 10
MAX_BYTES  = 64 * 1024

# Writes are ASYNCHRONOUS. ``print`` only appends to a RAM buffer; the
# file write and its flush happen off the hot path, driven by the
# launcher's Timer tick calling ``log.pump()`` (the same shape
# ``ble_repl`` uses for TX: buffer on write, drain on a scheduled
# callback, with the launcher tick as the liveness backstop).
#
# Why: ``flush()`` on littlefs forces a metadata commit, measured at
# ~60-90 ms on the ESP32 bench — PER LINE. The tee runs synchronously
# on the main thread between the user program's own bytecodes, so
# every ``print()`` stalled the robot for a tenth of a second (a
# 29-line ``dump_events`` took 1.9 s). Logging cost more than the work
# it was logging, and it distorted the timing of whatever the program
# was controlling.
#
# Durability is preserved where it matters. The buffer is committed:
#   * when the program ends (``__exit__``),
#   * when the stop button fires (launcher ``_fire_stop``),
#   * on every ``write_text`` — "started:", "stopped:", "Exception:"
#     and the launcher's button notes. Those are the crash-adjacent
#     lines a post-mortem needs, and they are rare enough that paying
#     a commit for each costs nothing measurable.
# So a hard reset (brownout, WDT, panic) can lose only ordinary
# ``print`` output since the last pump — never the run's framing.
#
# PENDING_MAX bounds the RAM buffer: past it, ``_append`` writes
# through synchronously rather than growing without limit. The pump
# runs every launcher tick, so reaching it takes a genuine print
# storm — and hitting it costs a write, never a dropped line.
PENDING_MAX = 4096

# MicroPython embedded ports count time from 2000-01-01 UTC; the unix
# port and CPython from 1970-01-01. Detect once so stored stamps are
# true Unix epoch regardless of runtime.
_EPOCH_OFFSET_MS = 946684800000 if time.gmtime(0)[0] == 2000 else 0


def _epoch_ms():
    """Current UTC Unix epoch in milliseconds (int)."""
    try:
        return time.time_ns() // 1000000 + _EPOCH_OFFSET_MS
    except AttributeError:
        # No time_ns on this runtime — whole-second resolution.
        return int(time.time()) * 1000 + _EPOCH_OFFSET_MS


# Resolved once at import; module-level so tests can patch either the
# function or these lookups (the fallback branch is otherwise dead on
# every test runtime — both MP and the CPython fakes provide ticks_*).
_TICKS_FN = getattr(time, "ticks_ms", None)
_DIFF_FN = getattr(time, "ticks_diff", None)


def _ticks_ms():
    """Monotonic ms — ``time.ticks_ms`` where available, wall-clock
    fallback otherwise."""
    if _TICKS_FN is not None:
        return _TICKS_FN()
    return int(time.time() * 1000)


def _ticks_diff(a, b):
    if _DIFF_FN is not None:
        return _DIFF_FN(a, b)
    return a - b


# ---- internal helpers ------------------------------------------------


def _ensure_log_dir():
    """Create LOG_DIR if it doesn't exist. Silent on EEXIST."""
    try:
        os.mkdir(LOG_DIR)
    except OSError:
        pass


def _list_existing():
    """Sorted list of ``(index, filename)`` for valid log files in
    LOG_DIR. Files that don't fit ``run_<int>.log`` are ignored."""
    try:
        entries = os.listdir(LOG_DIR)
    except OSError:
        return []
    out = []
    for name in entries:
        if not name.startswith("run_") or not name.endswith(".log"):
            continue
        idx_str = name[len("run_"):-len(".log")]
        try:
            idx = int(idx_str)
        except ValueError:
            continue
        out.append((idx, name))
    out.sort()
    return out


_SLOT_PREFIX = "slot_"


def _parse_header_line(line):
    """The run index from a slot file's header line
    (``"<epoch> -- run_N --"``), or ``None``."""
    parts = line.split()
    if (len(parts) == 4 and parts[1] == "--" and parts[3] == "--"
            and parts[2].startswith("run_")):
        try:
            return int(parts[2][4:])
        except ValueError:
            return None
    return None


def _slot_runs():
    """Sorted ``(index, filename)`` for slot files carrying a valid
    header. A truncated/corrupt slot (crash mid-write) parses to
    nothing and is simply skipped — its slot gets reused."""
    try:
        entries = os.listdir(LOG_DIR)
    except OSError:
        return []
    out = []
    for name in entries:
        if (not name.startswith(_SLOT_PREFIX)
                or not name.endswith(".log")):
            continue
        try:
            with open(LOG_DIR + "/" + name) as f:
                idx = _parse_header_line(f.readline())
        except (OSError, ValueError):
            continue
        if idx is not None:
            out.append((idx, name))
    out.sort()
    return out


def _next_run():
    """``(path, index)`` for the next run.

    The ``MAX_RUNS`` files are REUSED in place (truncate-on-open),
    never deleted and recreated: littlefs pays for directory churn
    forever — after ~70 delete+create rotation cycles every commit's
    allocator traversal crawled the accumulated metadata at ~400 ms
    per flush (bench run_68, 2026-08-09; reproduced on unix MP as
    8k+ block reads per commit vs ~40 on a fresh filesystem, and
    slot reuse returns it to ~40). The run INDEX keeps counting up —
    it lives in each slot's header line, not the filename."""
    _ensure_log_dir()
    runs = _slot_runs()
    legacy = _list_existing()
    next_idx = 0
    if runs:
        next_idx = runs[-1][0] + 1
    if legacy:
        # Numbering continues from pre-slot firmware's runs.
        next_idx = max(next_idx, legacy[-1][0] + 1)
    # One-time migration: legacy per-run files are exactly the churn
    # this scheme removes. Logs are ephemeral diagnostics; drop them.
    for idx, name in legacy:
        try:
            os.remove(LOG_DIR + "/" + name)
        except OSError:
            pass
    return ("%s/%s%d.log" % (LOG_DIR, _SLOT_PREFIX,
                             next_idx % MAX_RUNS),
            next_idx)


# ---- public session API ---------------------------------------------

# The session currently teeing prints (one program runs at a time).
# ``note()`` uses it to drop button-event lines into the run's log.
_ACTIVE = None


def note(text):
    """Write one stamped line to the active run's log file.

    No-op when no program is running (there is no file to write to).
    File only — the live console is deliberately not touched; callers
    that want a console message print one themselves. Used by the
    launcher so every button press that starts or stops a run leaves
    a timestamped entry in that run's log."""
    sess = _ACTIVE
    if sess is None:
        return
    if not text.endswith("\n"):
        text += "\n"
    sess.write_text(text)


class _LogSession:
    """Context manager. ``__enter__`` opens the next run log file and
    swaps in a wrapped ``builtins.print`` that writes to it; ``__exit__``
    restores ``builtins.print`` and closes the file."""

    def __init__(self):
        self._file          = None
        self._path          = None
        self._index         = None
        self._prev_print    = None
        self._budget        = [0]
        self._at_line_start = True
        # Stamped text not yet handed to the filesystem. A list of
        # str, joined at pump time — repeated ``+=`` on a str is
        # quadratic, and this buffer is appended to once per print.
        self._pending       = []
        self._pending_len   = 0
        # Instrumentation for the tick-starvation hunt: the slowest
        # single filesystem call this session, in ms. littlefs writes
        # occasionally trigger block erases/relocation, and a flash
        # erase suspends the CPU cache — a repeated ~100 ms-class
        # main-thread block is exactly what silently drops scheduler
        # ticks. The launcher's starvation note prints this so a
        # bench log can convict or exonerate the flash path without
        # another instrumented build.
        self._worst_write_ms = 0

    def pump(self, force=False):
        """Drain the RAM buffer to the file. Called off the print hot
        path — from the launcher's Timer tick — so ``print`` itself
        never touches flash.

        ``force`` additionally commits (``flush``), which is the
        expensive part on littlefs; the tick pumps without it and lets
        the filesystem cache batch, while program-end / stop-press /
        ``write_text`` force a real commit.

        Returns True if anything reached the file. Never raises: a
        logging failure must not propagate into the tick that owns the
        stop button, nor into the program being logged."""
        if self._file is None:
            return False
        try:
            t0 = _ticks_ms()
            if self._pending:
                text = "".join(self._pending)
                self._pending = []
                self._pending_len = 0
                self._file.write(text)
            elif not force:
                return False
            if force:
                self._file.flush()
            dt = _ticks_diff(_ticks_ms(), t0)
            if dt > self._worst_write_ms:
                self._worst_write_ms = dt
            return True
        except Exception:
            # Flash error — drop the bytes rather than wedge the tick.
            self._pending = []
            self._pending_len = 0
            return False

    def _append(self, payload, force=False):
        """Stamp, budget-check, and write ``payload`` to the file.

        Every new file line is prefixed with ``"<epoch_ms> "`` — a raw
        int64 UTC Unix-epoch-milliseconds number, converted to local
        time only by the host CLI at display. A ``print(..., end='')``
        continuation lands mid-line and is NOT re-stamped; blank lines
        carry no stamp. May raise on flash errors — callers decide
        whether that is swallowed."""
        if self._file is None or self._budget[0] >= MAX_BYTES:
            return
        stamp = "%d " % _epoch_ms()
        parts = payload.split("\n")
        out = []
        i = 0
        n = len(parts)
        while i < n:
            seg = parts[i]
            if seg:
                if self._at_line_start:
                    out.append(stamp)
                out.append(seg)
                self._at_line_start = False
            if i < n - 1:
                out.append("\n")
                self._at_line_start = True
            i += 1
        text = "".join(out)
        remaining = MAX_BYTES - self._budget[0]
        if len(text) > remaining:
            text = text[:remaining]
        self._budget[0] += len(text)
        # The hot path ends here: buffer only, no filesystem call.
        self._pending.append(text)
        self._pending_len += len(text)
        if force or self._pending_len >= PENDING_MAX:
            self.pump(force=force)

    def _make_tee_print(self, original_print):
        """Build the replacement ``print`` function."""
        def _tee_print(*args, **kwargs):
            original_print(*args, **kwargs)
            try:
                # Reproduce print's stringification: sep / end default
                # to " " and "\n". We don't honour file= here — every
                # print, including ones aimed at stderr, lands in the
                # log file too (which is the whole point).
                sep = kwargs.get("sep", " ")
                end = kwargs.get("end", "\n")
                self._append(sep.join(str(a) for a in args) + end)
            except Exception:
                # Flash error / OOM — drop the bytes; live print
                # already happened.
                pass
        return _tee_print

    def __enter__(self):
        try:
            self._path, self._index = _next_run()
            self._file = open(self._path, "w")
            # Header line: carries the run index (the filename no
            # longer does — slots are reused). Stamped like every
            # other line so dumps render it naturally.
            self._file.write("%d -- run_%d --\n"
                             % (_epoch_ms(), self._index))
        except Exception:
            self._file = None
            self._path = None
            return self

        self._prev_print = builtins.print
        self._budget = [0]
        self._at_line_start = True
        builtins.print = self._make_tee_print(self._prev_print)
        global _ACTIVE
        _ACTIVE = self
        return self

    def __exit__(self, exc_type, exc_value, exc_tb):
        global _ACTIVE
        if _ACTIVE is self:
            _ACTIVE = None
        if self._prev_print is not None:
            builtins.print = self._prev_print
            self._prev_print = None
        if self._file is not None:
            # Program ended — commit whatever print output is still
            # buffered before the file goes away.
            self.pump(force=True)
            try:
                self._file.close()
            except Exception:
                pass
            self._file = None
        return False   # do not suppress exceptions

    @property
    def path(self):
        """Absolute path of the file this session is writing to, or
        ``None`` if we couldn't open one."""
        return self._path

    def write_text(self, s):
        """Append text to the log file directly, bypassing ``print``
        (lines still get the epoch-ms stamp). Used by the launcher's
        exception handler so the traceback (which goes through
        ``sys.print_exception``, not ``print``) lands in the file
        too.

        Committed immediately, unlike ``print``: every caller of this
        is crash-adjacent ("started:", "stopped:", "Exception:", the
        launcher's button notes), so these lines must survive a reset
        that the buffered print output legitimately may not."""
        try:
            self._append(s, force=True)
        except Exception:
            pass


def worst_write_ms():
    """Slowest single filesystem call of the active session, in ms.
    0 when nothing has been written or no session is active. The
    launcher includes this in its tick-starvation note: if the two
    numbers are of the same order, littlefs (block erase under a
    write) owns the starvation; if this stays small while the gaps
    are large, the blocker is elsewhere."""
    sess = _ACTIVE
    if sess is None:
        return 0
    return sess._worst_write_ms


def pump():
    """Drain the active session's buffered output to its file.

    Called from the launcher's Timer tick, which is what makes logging
    asynchronous: ``print`` appends to RAM and returns, this moves the
    bytes to flash off the program's hot path. No-op when no program
    is running. Never raises — the tick that calls this also owns the
    stop button."""
    sess = _ACTIVE
    if sess is None:
        return False
    return sess.pump()


def flush():
    """Commit the active session's buffered output NOW.

    For the moments durability beats speed: the stop button firing, or
    any other point where the next thing that happens might be a reset.
    No-op when no program is running."""
    sess = _ACTIVE
    if sess is None:
        return False
    return sess.pump(force=True)


def session():
    """Construct a fresh :class:`_LogSession`.

    Use as a context manager::

        with log.session() as sess:
            run_user_program()
            # sess.write_text(extra) for non-print output if needed.
    """
    return _LogSession()


# ---- public read API (used by openbricks log) -------------------


def list_runs():
    """Return a list of ``(index, full_path)`` tuples, oldest first.
    Used by the on-hub helper that ``openbricks log`` invokes via
    raw-paste to enumerate available runs. Slot files are listed by
    their header index; legacy ``run_N.log`` files (pre-slot
    firmware) still appear until the next run's migration removes
    them."""
    out = [(idx, LOG_DIR + "/" + name)
           for idx, name in _list_existing()]
    out += [(idx, LOG_DIR + "/" + name)
            for idx, name in _slot_runs()]
    out.sort()
    return out


def read_run(index):
    """Read a single run's log by index. Raises ``OSError`` if no
    such run exists (the slot was reused by a newer run, or the
    index never happened)."""
    path = "%s/%s%d.log" % (LOG_DIR, _SLOT_PREFIX, index % MAX_RUNS)
    try:
        with open(path) as f:
            first = f.readline()
            if _parse_header_line(first) == index:
                return first + f.read()
    except OSError:
        pass
    # Legacy layout (pre-slot firmware) — or a clean OSError.
    return open("%s/run_%d.log" % (LOG_DIR, index)).read()
