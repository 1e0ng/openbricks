# SPDX-License-Identifier: MIT
"""
Per-run log capture: tee ``print(...)`` output to a file on flash so
untethered runs can be inspected later via ``openbricks-dev log``.

The launcher wraps every program execution with ``log.session()`` so
the user's ``print`` output streams to *both* the live USB / BLE
console (when one's listening) and a rotating file on flash. With
nobody listening on the live channel, the file is the only record.
``openbricks-dev log`` reads the most recent files back over BLE.

Storage layout::

    /openbricks_logs/run_0.log
    /openbricks_logs/run_1.log
    /openbricks_logs/run_2.log

Each run gets the next index; when the directory already holds
``MAX_RUNS`` files we delete the oldest before opening the new one.
Indices are recycled rather than monotonically growing so flash
usage is bounded.

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


LOG_DIR    = "/openbricks_logs"
MAX_RUNS   = 3
MAX_BYTES  = 64 * 1024


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


class _LogSession:
    """Context manager. ``__enter__`` opens the next run log file and
    swaps in a wrapped ``builtins.print`` that writes to it; ``__exit__``
    restores ``builtins.print`` and closes the file."""

    def __init__(self):
        self._file       = None
        self._path       = None
        self._prev_print = None
        self._written    = 0

    def _make_tee_print(self, original_print, fp, budget_holder):
        """Build the replacement ``print`` function.

        ``budget_holder`` is a single-element list so the closure can
        mutate the running byte count without the (now-deprecated on
        MP) ``nonlocal`` keyword."""
        def _tee_print(*args, **kwargs):
            original_print(*args, **kwargs)
            if fp is None or budget_holder[0] >= MAX_BYTES:
                return
            try:
                # Reproduce print's stringification: sep / end default
                # to " " and "\n". We don't honour file= here — every
                # print, including ones aimed at stderr, lands in the
                # log file too (which is the whole point).
                sep = kwargs.get("sep", " ")
                end = kwargs.get("end", "\n")
                payload = sep.join(str(a) for a in args) + end
                remaining = MAX_BYTES - budget_holder[0]
                if len(payload) > remaining:
                    payload = payload[:remaining]
                fp.write(payload)
                fp.flush()
                budget_holder[0] += len(payload)
            except Exception:
                # Flash error / OOM — drop the byte; live print
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
        budget = [0]
        builtins.print = self._make_tee_print(
            self._prev_print, self._file, budget)
        self._budget = budget
        return self

    def __exit__(self, exc_type, exc_value, exc_tb):
        if self._prev_print is not None:
            builtins.print = self._prev_print
            self._prev_print = None
        if self._file is not None:
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
        """Append raw text to the log file directly, bypassing
        ``print``. Used by the launcher's exception handler so the
        traceback (which goes through ``sys.print_exception``, not
        ``print``) lands in the file too."""
        if self._file is None:
            return
        if self._budget[0] >= MAX_BYTES:
            return
        try:
            remaining = MAX_BYTES - self._budget[0]
            payload = s if len(s) <= remaining else s[:remaining]
            self._file.write(payload)
            self._file.flush()
            self._budget[0] += len(payload)
        except Exception:
            pass


def session():
    """Construct a fresh :class:`_LogSession`. Use as a context manager::

        with log.session() as sess:
            run_user_program()
            # sess.write_text(extra) for non-print output if needed.
    """
    return _LogSession()


# ---- public read API (used by openbricks-dev log) -------------------


def list_runs():
    """Return a list of ``(index, full_path)`` tuples, oldest first.
    Used by the on-hub helper that ``openbricks-dev log`` invokes via
    raw-paste to enumerate available runs."""
    return [(idx, LOG_DIR + "/" + name) for idx, name in _list_existing()]


def read_run(index):
    """Read a single run's log file by index. Raises ``OSError`` if
    no such run exists."""
    return open("%s/run_%d.log" % (LOG_DIR, index)).read()
