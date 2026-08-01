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
    /openbricks_logs/run_1.log
    /openbricks_logs/run_2.log

Each run gets the next index; when the directory already holds
``MAX_RUNS`` files we delete the oldest before opening the new one.
Indices grow monotonically (run_9 is the tenth run ever; only the
newest ``MAX_RUNS`` files exist at a time), so flash usage is
bounded while filenames stay unambiguous across rotations.

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


def _next_run_path():
    """Allocate a path for the next run, evicting the oldest log if
    we'd exceed ``MAX_RUNS``. Returns the absolute path."""
    _ensure_log_dir()
    existing = _list_existing()
    while len(existing) >= MAX_RUNS:
        idx, name = existing.pop(0)
        try:
            os.remove(LOG_DIR + "/" + name)
        except OSError:
            break
    next_idx = 0
    if existing:
        next_idx = existing[-1][0] + 1
    return "%s/run_%d.log" % (LOG_DIR, next_idx)


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
        self._prev_print    = None
        self._budget        = [0]
        self._at_line_start = True
        # Stamped text not yet handed to the filesystem. A list of
        # str, joined at pump time — repeated ``+=`` on a str is
        # quadratic, and this buffer is appended to once per print.
        self._pending       = []
        self._pending_len   = 0

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
            if self._pending:
                text = "".join(self._pending)
                self._pending = []
                self._pending_len = 0
                self._file.write(text)
            elif not force:
                return False
            if force:
                self._file.flush()
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
            self._path = _next_run_path()
            self._file = open(self._path, "w")
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
    raw-paste to enumerate available runs."""
    return [(idx, LOG_DIR + "/" + name) for idx, name in _list_existing()]


def read_run(index):
    """Read a single run's log file by index. Raises ``OSError`` if
    no such run exists."""
    return open("%s/run_%d.log" % (LOG_DIR, index)).read()
