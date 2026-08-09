# SPDX-License-Identifier: MIT
"""Tests for openbricks.log — per-run print-tee + rotation."""

import tests._fakes  # noqa: F401

import builtins
import os
import unittest

from openbricks import log


_TEST_LOG_DIR = "/tmp/_obtest_logs"


def _wipe(dirpath):
    try:
        for name in os.listdir(dirpath):
            try:
                os.remove(dirpath + "/" + name)
            except OSError:
                pass
        try:
            os.rmdir(dirpath)
        except OSError:
            pass
    except OSError:
        pass


class _LogPathPatch(unittest.TestCase):
    """Common setUp / tearDown — redirect LOG_DIR to a tmp path and
    clean up after each test."""

    def setUp(self):
        _wipe(_TEST_LOG_DIR)
        self._prev_log_dir   = log.LOG_DIR
        self._prev_max_bytes = log.MAX_BYTES
        log.LOG_DIR = _TEST_LOG_DIR

    def tearDown(self):
        log.LOG_DIR   = self._prev_log_dir
        log.MAX_BYTES = self._prev_max_bytes
        _wipe(_TEST_LOG_DIR)


class SessionTeesPrintsToFile(_LogPathPatch):

    def test_session_writes_print_output_to_file(self):
        with log.session() as sess:
            print("hello sim")
            print("two")
        self.assertIsNotNone(sess.path)
        with open(sess.path) as f:
            data = f.read()
        self.assertIn("hello sim", data)
        self.assertIn("two", data)

    def test_session_does_not_swallow_exceptions(self):
        with self.assertRaises(ValueError):
            with log.session():
                raise ValueError("boom")

    def test_session_restores_print_after_exit(self):
        # MicroPython's ``builtins.print`` reports the cached original
        # even after a successful dict swap (the dispatch *does* go
        # through the wrapped version, but ``is`` compares against the
        # cached pointer). So we test behaviourally: prints inside the
        # session write to the file, prints after exit do not.
        with log.session() as sess:
            print("inside")
        with open(sess.path) as f:
            inside_contents = f.read()
        self.assertIn("inside", inside_contents)
        # After exit: opening sess.path again gives the same content —
        # subsequent prints don't append.
        print("outside")
        with open(sess.path) as f:
            self.assertEqual(f.read(), inside_contents)

    def test_print_with_kwargs_works_inside_session(self):
        # sep, end, multiple args — the wrapped print must handle
        # all of these. Verify by reading the file back.
        with log.session() as sess:
            print("a", "b", "c", sep="-", end="!")
        with open(sess.path) as f:
            self.assertIn("a-b-c!", f.read())

    def test_write_text_appends_raw_text(self):
        # Used by the launcher to capture tracebacks.
        with log.session() as sess:
            print("normal")
            sess.write_text("Exception: ValueError(boom)\n")
        with open(sess.path) as f:
            data = f.read()
        self.assertIn("normal", data)
        self.assertIn("Exception: ValueError(boom)", data)


class TimestampTests(_LogPathPatch):
    """Every file line starts with a raw int64 UTC epoch-ms stamp.

    The hub stores only the number (no formatting, no timezone) —
    the host CLI converts to local time at display. A ``print(...,
    end='')`` continuation must not re-stamp mid-line, and blank
    lines carry no stamp."""

    _TS = 1783950123456

    def setUp(self):
        _LogPathPatch.setUp(self)
        self._prev_epoch = log._epoch_ms
        ts = self._TS
        log._epoch_ms = lambda: ts

    def tearDown(self):
        log._epoch_ms = self._prev_epoch
        _LogPathPatch.tearDown(self)

    def _read(self, sess):
        # Every slot file opens with the run header line; the
        # assertions here target what the session WROTE after it.
        with open(sess.path) as f:
            first = f.readline()
            assert log._parse_header_line(first) is not None, first
            return f.read()

    def test_each_line_prefixed_with_epoch_ms(self):
        with log.session() as sess:
            print("hello")
            print("world")
        self.assertEqual(
            self._read(sess),
            "%d hello\n%d world\n" % (self._TS, self._TS))

    def test_multiline_print_stamps_every_line(self):
        with log.session() as sess:
            print("a\nb")
        self.assertEqual(
            self._read(sess), "%d a\n%d b\n" % (self._TS, self._TS))

    def test_end_continuation_not_restamped_mid_line(self):
        with log.session() as sess:
            print("left ", end="")
            print("right")
        self.assertEqual(
            self._read(sess), "%d left right\n" % self._TS)

    def test_blank_line_carries_no_stamp(self):
        with log.session() as sess:
            print("a")
            print()
            print("b")
        self.assertEqual(
            self._read(sess),
            "%d a\n\n%d b\n" % (self._TS, self._TS))

    def test_write_text_lines_are_stamped(self):
        with log.session() as sess:
            sess.write_text("Traceback:\n  boom\n")
        self.assertEqual(
            self._read(sess),
            "%d Traceback:\n%d   boom\n" % (self._TS, self._TS))

    def test_budget_counts_stamped_bytes(self):
        log.MAX_BYTES = 40
        with log.session() as sess:
            for _ in range(20):
                print("xxxxxxxxxx")
        data = self._read(sess)
        self.assertTrue(len(data) <= 40,
                        "stamps must count against MAX_BYTES: %d bytes"
                        % len(data))

    def test_epoch_ms_is_true_unix_epoch(self):
        # Sanity on the live clock (offset logic for 2000-based
        # embedded epochs): any real runtime must report a value
        # after 2017 and before 2100.
        now = self._prev_epoch()
        self.assertTrue(1500000000000 < now < 4102444800000,
                        "epoch-ms out of range: %d" % now)


class NoteTests(_LogPathPatch):
    """``log.note`` drops a stamped line into the active run's log —
    the launcher uses it so every button press that starts or stops a
    run leaves an entry."""

    _TS = 1783950123456

    def setUp(self):
        _LogPathPatch.setUp(self)
        self._prev_epoch = log._epoch_ms
        ts = self._TS
        log._epoch_ms = lambda: ts

    def tearDown(self):
        log._epoch_ms = self._prev_epoch
        _LogPathPatch.tearDown(self)

    def test_note_writes_stamped_line_into_active_session(self):
        with log.session() as sess:
            print("running")
            log.note("button pressed -> stop")
        with open(sess.path) as f:
            data = f.read()
        self.assertIn("%d button pressed -> stop\n" % self._TS, data)

    def test_note_without_active_session_is_noop(self):
        log.note("nobody home")   # must not raise

    def test_session_exit_clears_active(self):
        with log.session():
            pass
        self.assertIsNone(log._ACTIVE)
        log.note("late")          # must not raise, nothing to write to


class RotationTests(_LogPathPatch):
    """Slot reuse, not delete+create: littlefs charges directory
    churn forever (the 400 ms commits of bench 2026-08-09), so the
    MAX_RUNS files are truncated in place and the run index lives in
    each file's header line."""

    def test_first_run_uses_slot_0_with_run_0_header(self):
        with log.session() as sess:
            print("a")
        self.assertIsNotNone(sess.path)
        self.assertEqual(sess.path.split("/")[-1], "slot_0.log")
        with open(sess.path) as f:
            self.assertEqual(log._parse_header_line(f.readline()), 0)

    def test_subsequent_runs_increment_index(self):
        for _ in range(3):
            with log.session():
                pass
        self.assertEqual([idx for idx, _ in log.list_runs()],
                         [0, 1, 2])

    def test_run_past_capacity_reuses_the_oldest_slot(self):
        # Derived from MAX_RUNS, not a literal: the eviction contract
        # must hold at whatever capacity ships. Run MAX_RUNS wraps
        # onto slot_0, overwriting run 0 — same retention as the old
        # delete+create rotation, zero directory churn.
        for _ in range(log.MAX_RUNS + 1):
            with log.session():
                pass
        existing = os.listdir(_TEST_LOG_DIR)
        self.assertEqual(len(existing), log.MAX_RUNS)
        indices = [idx for idx, _ in log.list_runs()]
        self.assertEqual(indices,
                         list(range(1, log.MAX_RUNS + 1)))
        with self.assertRaises(OSError):
            log.read_run(0)                     # slot reused

    def test_slot_filenames_never_change(self):
        # THE fix: the same MAX_RUNS names forever — no
        # create/delete churn for littlefs to charge us for.
        for _ in range(log.MAX_RUNS * 2 + 3):
            with log.session():
                pass
        names = sorted(os.listdir(_TEST_LOG_DIR))
        self.assertEqual(
            names,
            sorted("slot_%d.log" % i for i in range(log.MAX_RUNS)))

    def test_legacy_run_files_are_migrated_away(self):
        # Files from pre-slot firmware are exactly the churn this
        # scheme removes: listed until the next run starts, removed
        # by its one-time migration.
        os.mkdir(_TEST_LOG_DIR)
        with open(_TEST_LOG_DIR + "/run_5.log", "w") as f:
            f.write("1783950123456 legacy line\n")
        self.assertEqual([idx for idx, _ in log.list_runs()], [5])
        self.assertIn("legacy line", log.read_run(5))
        with log.session():
            pass
        names = os.listdir(_TEST_LOG_DIR)
        # MP's unittest has no assertNotIn.
        self.assertFalse("run_5.log" in names, names)
        # The new run continues the numbering after the legacy run.
        self.assertEqual([idx for idx, _ in log.list_runs()], [6])

    def test_corrupt_slot_header_is_skipped(self):
        # Crash mid-write leaves a slot without a parseable header:
        # it must not break listing, and its slot gets reused.
        os.mkdir(_TEST_LOG_DIR)
        with open(_TEST_LOG_DIR + "/slot_3.log", "w") as f:
            f.write("garbage without a header\n")
        self.assertEqual(log.list_runs(), [])
        with log.session() as sess:
            pass
        self.assertEqual(sess.path.split("/")[-1], "slot_0.log")

    def test_capacity_survives_a_diagnostic_session(self):
        # The bench failure this bump answers: an intermittent
        # won't-start is diagnosed by READING the failing run's log,
        # but every diagnostic (state dump, bus scan) is itself a run
        # taking a slot. Twice the failing run was rotated out by the
        # tools investigating it. Capacity must fit a realistic
        # forensic session: several failing button presses plus a
        # handful of diagnostics, with the oldest failure still
        # readable.
        self.assertGreaterEqual(
            log.MAX_RUNS, 10,
            "MAX_RUNS shrank — diagnostics will again evict the "
            "failing runs they exist to explain")

    def test_max_bytes_truncates_runaway_logs(self):
        log.MAX_BYTES = 32
        with log.session() as sess:
            print("a" * 1000)   # blow past the budget
        with open(sess.path) as f:
            f.readline()        # the run header rides outside the budget
            data = f.read()
        # Allow a few bytes of leeway around the trailing newline.
        self.assertLessEqual(len(data), 32 + 4)


class ListAndReadRunsTests(_LogPathPatch):

    def test_list_runs_returns_indices(self):
        with log.session():
            print("first")
        with log.session():
            print("second")
        runs = log.list_runs()
        self.assertEqual([idx for idx, _ in runs], [0, 1])

    def test_read_run_reads_specific_index(self):
        with log.session():
            print("alpha")
        with log.session():
            print("beta")
        self.assertIn("alpha", log.read_run(0))
        self.assertIn("beta",  log.read_run(1))

    def test_read_run_missing_raises(self):
        with self.assertRaises(OSError):
            log.read_run(42)


class FilesystemErrorIsResilient(_LogPathPatch):
    """If LOG_DIR can't be created (file blocking it) the session
    silently degrades — print still works; sess.path is None."""

    def test_unwritable_log_dir_keeps_session_alive(self):
        blocker = "/tmp/_obtest_blocker"
        try:
            with open(blocker, "w") as f:
                f.write("blocking")
            log.LOG_DIR = blocker  # mkdir(blocker) → EEXIST; later
                                   # open() of f"{blocker}/run_0.log"
                                   # is ENOTDIR.
            with log.session() as sess:
                print("still works")
            self.assertIsNone(sess.path)
        finally:
            try:
                os.remove(blocker)
            except OSError:
                pass


class _CountingFile:
    """Wraps the real file object so a test can see exactly how many
    filesystem calls a print costs. Swapped in after ``__enter__`` so
    the session is otherwise completely real."""

    def __init__(self, inner):
        self._inner = inner
        self.writes = 0
        self.flushes = 0

    def write(self, s):
        self.writes += 1
        return self._inner.write(s)

    def flush(self):
        self.flushes += 1
        return self._inner.flush()

    def close(self):
        return self._inner.close()


class AsyncWriteTests(_LogPathPatch):
    """``print`` must not touch flash.

    A littlefs commit measured ~60-90 ms on the ESP32 bench, and it ran
    inline with every ``print()`` — on the main thread, between the
    user program's own bytecodes. Logging cost more than the work it
    logged and distorted the timing of whatever the robot was doing.
    """

    def _instrumented(self):
        sess = log.session()
        sess.__enter__()
        self.addCleanup(sess.__exit__, None, None, None)
        counter = _CountingFile(sess._file)
        sess._file = counter
        return sess, counter

    def test_print_touches_the_filesystem_zero_times(self):
        sess, counter = self._instrumented()
        for i in range(20):
            print("line %d" % i)
        self.assertEqual(counter.writes, 0)
        self.assertEqual(counter.flushes, 0)

    def test_pump_moves_buffered_output_to_the_file_without_committing(self):
        sess, counter = self._instrumented()
        print("buffered")
        self.assertTrue(sess.pump())
        self.assertEqual(counter.writes, 1)
        self.assertEqual(counter.flushes, 0)

    def test_pump_batches_a_print_storm_into_one_write(self):
        sess, counter = self._instrumented()
        for i in range(20):
            print("line %d" % i)
        sess.pump()
        self.assertEqual(counter.writes, 1)

    def test_forced_pump_commits(self):
        sess, counter = self._instrumented()
        print("buffered")
        sess.pump(force=True)
        self.assertEqual(counter.flushes, 1)

    def test_pump_without_a_file_is_a_no_op(self):
        # A session whose file never opened (flash full, bad path)
        # still gets pumped every tick — it must just say no.
        sess = log._LogSession()
        self.assertIsNone(sess._file)
        self.assertFalse(sess.pump())
        self.assertFalse(sess.pump(force=True))

    def test_forced_pump_with_empty_buffer_still_commits(self):
        sess, counter = self._instrumented()
        self.assertTrue(sess.pump(force=True))
        self.assertEqual(counter.writes, 0)
        self.assertEqual(counter.flushes, 1)

    def test_pump_with_nothing_buffered_is_a_no_op(self):
        sess, counter = self._instrumented()
        self.assertFalse(sess.pump())
        self.assertEqual(counter.writes, 0)

    def test_write_text_commits_immediately(self):
        # Crash-adjacent lines must not wait for a pump.
        sess, counter = self._instrumented()
        sess.write_text("Exception: OSError(19,)\n")
        self.assertEqual(counter.writes, 1)
        self.assertEqual(counter.flushes, 1)

    def test_pending_cap_writes_through_rather_than_growing(self):
        # Enough to cross PENDING_MAX several times over. The buffer
        # must drain itself rather than grow without bound, and no
        # line may be dropped to achieve that.
        sess, counter = self._instrumented()
        chunk = "x" * 200
        lines = (log.PENDING_MAX // len(chunk)) * 3
        for _ in range(lines):
            print(chunk)
        self.assertTrue(counter.writes >= 1)
        self.assertTrue(sess._pending_len < log.PENDING_MAX)
        sess.pump(force=True)
        with open(sess.path) as f:
            self.assertEqual(f.read().count(chunk), lines)

    def test_pump_survives_a_dead_file_and_drops_the_backlog(self):
        # A flash error must not wedge the tick that owns the stop
        # button, and must not leave bytes queued forever.
        sess, counter = self._instrumented()
        print("doomed")

        def _boom(_s):
            raise OSError(5)

        counter.write = _boom
        self.assertFalse(sess.pump())
        self.assertEqual(sess._pending_len, 0)


class WorstWriteTrackingTests(_LogPathPatch):
    """The tick-starvation instrumentation: pump() times each
    filesystem call and keeps the session's worst, so a starvation
    note can convict or exonerate littlefs without a special build."""

    def _session_with_clock(self, durations_ms):
        """A real session whose file 'write' advances a fake clock by
        the next duration in ``durations_ms``."""
        sess = log.session()
        sess.__enter__()
        self.addCleanup(sess.__exit__, None, None, None)
        clock = [0]
        queue = list(durations_ms)
        orig_ticks = log._ticks_ms
        log._ticks_ms = lambda: clock[0]
        self.addCleanup(setattr, log, "_ticks_ms", orig_ticks)
        inner = sess._file

        class _SlowFile:
            def write(self, s):
                clock[0] += queue.pop(0) if queue else 0
                return inner.write(s)

            def flush(self):
                return inner.flush()

            def close(self):
                return inner.close()

        sess._file = _SlowFile()
        return sess

    def test_worst_write_is_the_max_not_the_last(self):
        sess = self._session_with_clock([30, 250, 40])
        for _ in range(3):
            print("line")
            sess.pump()
        self.assertEqual(sess._worst_write_ms, 250)
        self.assertEqual(log.worst_write_ms(), 250)

    def test_fast_writes_report_small_numbers(self):
        sess = self._session_with_clock([2, 1])
        print("line")
        sess.pump()
        self.assertLessEqual(sess._worst_write_ms, 2)

    def test_no_session_reports_zero(self):
        self.assertEqual(log.worst_write_ms(), 0)

    def test_clock_fallbacks_without_ticks_functions(self):
        # The no-ticks_* branch is dead on every test runtime (MP and
        # the CPython fakes both provide them) — patch the resolved
        # lookups to cover the wall-clock fallback.
        orig_t, orig_d = log._TICKS_FN, log._DIFF_FN
        log._TICKS_FN, log._DIFF_FN = None, None
        try:
            self.assertTrue(isinstance(log._ticks_ms(), int))
            self.assertEqual(log._ticks_diff(7, 3), 4)
        finally:
            log._TICKS_FN, log._DIFF_FN = orig_t, orig_d


class BufferedOutputStillLandsTests(_LogPathPatch):
    """Speed must not cost content: everything printed still reaches
    the file by the time the run is over."""

    def test_program_end_commits_buffered_prints(self):
        with log.session() as sess:
            for i in range(50):
                print("line %d" % i)
        with open(sess.path) as f:
            data = f.read()
        self.assertIn("line 0", data)
        self.assertIn("line 49", data)

    def test_module_pump_and_flush_target_the_active_session(self):
        # pump() drains the RAM buffer into the file object; only
        # flush() guarantees another reader can see it (CPython buffers
        # at the file object, littlefs commits on sync). Assert each
        # against its own contract rather than conflating them.
        with log.session() as sess:
            print("via module pump")
            self.assertTrue(sess._pending_len > 0)
            self.assertTrue(log.pump())
            self.assertEqual(sess._pending_len, 0)
            print("via module flush")
            self.assertTrue(log.flush())
            with open(sess.path) as f:
                data = f.read()
            self.assertIn("via module pump", data)
            self.assertIn("via module flush", data)

    def test_module_pump_and_flush_are_no_ops_with_no_session(self):
        self.assertFalse(log.pump())
        self.assertFalse(log.flush())


if __name__ == "__main__":
    unittest.main()
