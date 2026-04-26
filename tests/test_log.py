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


class RotationTests(_LogPathPatch):

    def test_first_run_creates_run_0(self):
        with log.session() as sess:
            print("a")
        self.assertIsNotNone(sess.path)
        self.assertEqual(sess.path.split("/")[-1], "run_0.log")

    def test_subsequent_runs_increment_index(self):
        paths = []
        for _ in range(3):
            with log.session() as sess:
                paths.append(sess.path)
        names = [p.split("/")[-1] for p in paths]
        self.assertEqual(names, ["run_0.log", "run_1.log", "run_2.log"])

    def test_fourth_run_evicts_oldest(self):
        for _ in range(4):
            with log.session():
                pass
        existing = sorted(os.listdir(_TEST_LOG_DIR))
        self.assertEqual(existing, ["run_1.log", "run_2.log", "run_3.log"])

    def test_max_bytes_truncates_runaway_logs(self):
        log.MAX_BYTES = 32
        with log.session() as sess:
            print("a" * 1000)   # blow past the budget
        with open(sess.path) as f:
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


if __name__ == "__main__":
    unittest.main()
