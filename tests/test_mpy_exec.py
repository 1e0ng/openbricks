# SPDX-License-Identifier: MIT
"""Host-compiled ``.mpy`` program execution (1.92.0).

Three layers under test:

* the native ``exec_mpy`` loader — runs the checked-in fixtures
  (compiled by the CLI's pinned mpy-cross; see
  scripts/gen-mpy-fixtures.py) under a caller-supplied globals dict,
  exactly like the source path's ``exec(code, {"__name__":
  "__main__"})``. A MicroPython submodule bump that changes the
  bytecode format fails these loudly with ``ValueError: incompatible
  .mpy file`` — that is the compatibility canary, not an
  inconvenience;
* the launcher's staging resolution — source wins when present, the
  compiled sibling runs only when ``/program.py`` is absent, explicit
  paths pass through verbatim;
* the launcher's dispatch — a ``.mpy`` path routes to the native
  loader, everything else stays on the source-exec path.
"""

import tests._fakes  # noqa: F401

import os
import sys
import unittest

from openbricks import launcher


_HELLO = "tests/fixtures/mpy_hello.mpy"
_BOOM = "tests/fixtures/mpy_boom.mpy"

_TMP_SRC = "/tmp/_openbricks_mpy_test_program.py"
_TMP_MPY = "/tmp/_openbricks_mpy_test_program.mpy"


def _native_exec_mpy():
    try:
        from _openbricks_native import exec_mpy
        return exec_mpy
    except ImportError:
        return None   # CPython job: no user C module


def _remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


class ExecMpyLoaderTests(unittest.TestCase):
    """The C binding, against fixtures the pinned mpy-cross built."""

    def test_fixture_runs_under_the_supplied_globals(self):
        exec_mpy = _native_exec_mpy()
        if exec_mpy is None:
            self.skipTest("native exec_mpy only")
        g = {"__name__": "__main__"}
        exec_mpy(_HELLO, g)
        # The fixture stores what ``__name__`` it saw — exec parity
        # means "__main__", never the import machinery's module name.
        self.assertEqual(g["ran_as"], "__main__")

    def test_program_exception_propagates_with_globals_restored(self):
        exec_mpy = _native_exec_mpy()
        if exec_mpy is None:
            self.skipTest("native exec_mpy only")
        try:
            exec_mpy(_BOOM, {"__name__": "__main__"})
            self.fail("fixture must raise")
        except ValueError as e:
            self.assertIn("fixture-boom", str(e))
        # The nlr callback restored the caller's globals — this module
        # keeps executing with its own namespace intact.
        self.assertEqual(_BOOM, "tests/fixtures/mpy_boom.mpy")

    def test_missing_file_raises_oserror(self):
        exec_mpy = _native_exec_mpy()
        if exec_mpy is None:
            self.skipTest("native exec_mpy only")
        try:
            exec_mpy("/tmp/_openbricks_no_such.mpy", {})
            self.fail("must raise")
        except OSError:
            pass

    def test_non_dict_globals_raise_typeerror(self):
        exec_mpy = _native_exec_mpy()
        if exec_mpy is None:
            self.skipTest("native exec_mpy only")
        try:
            exec_mpy(_HELLO, 42)
            self.fail("must raise")
        except TypeError:
            pass


class ResolveProgramPathTests(unittest.TestCase):
    """Source wins when present; .mpy only fills absence. This is what
    keeps every mixed old/new CLI staging sequence running the MOST
    RECENTLY staged program on a filesystem with no timestamps."""

    def setUp(self):
        self._orig_src = launcher.DEFAULT_PROGRAM_PATH
        self._orig_mpy = launcher.MPY_PROGRAM_PATH
        launcher.DEFAULT_PROGRAM_PATH = _TMP_SRC
        launcher.MPY_PROGRAM_PATH = _TMP_MPY
        self.addCleanup(
            setattr, launcher, "DEFAULT_PROGRAM_PATH", self._orig_src)
        self.addCleanup(
            setattr, launcher, "MPY_PROGRAM_PATH", self._orig_mpy)
        self.addCleanup(_remove, _TMP_SRC)
        self.addCleanup(_remove, _TMP_MPY)
        _remove(_TMP_SRC)
        _remove(_TMP_MPY)

    def test_source_wins_when_both_exist(self):
        with open(_TMP_SRC, "w") as f:
            f.write("pass\n")
        with open(_TMP_MPY, "wb") as f:
            f.write(b"M\x06")
        self.assertEqual(
            launcher._resolve_program_path(_TMP_SRC), _TMP_SRC)

    def test_mpy_runs_when_source_absent(self):
        with open(_TMP_MPY, "wb") as f:
            f.write(b"M\x06")
        self.assertEqual(
            launcher._resolve_program_path(_TMP_SRC), _TMP_MPY)

    def test_neither_present_returns_the_default_unchanged(self):
        self.assertEqual(
            launcher._resolve_program_path(_TMP_SRC), _TMP_SRC)

    def test_explicit_path_passes_through_verbatim(self):
        with open(_TMP_MPY, "wb") as f:
            f.write(b"M\x06")
        self.assertEqual(
            launcher._resolve_program_path("/custom/boot.py"),
            "/custom/boot.py")


class ExecDispatchTests(unittest.TestCase):
    """A ``.mpy`` program path routes through the native loader with
    exec-parity globals; source keeps the source path."""

    def setUp(self):
        self.addCleanup(_remove, _TMP_MPY)

    def test_mpy_path_calls_the_native_loader(self):
        calls = []

        class _Mod:
            pass

        def _fake_exec_mpy(path, g):
            calls.append((path, g.get("__name__")))
        mod = _Mod()          # any attr-bearing object works as a
        mod.exec_mpy = _fake_exec_mpy   # sys.modules entry (MP + CPython)
        with open(_TMP_MPY, "wb") as f:
            f.write(b"M\x06")
        orig = sys.modules.get("_openbricks_native")
        sys.modules["_openbricks_native"] = mod
        try:
            launcher._exec_program_raw(_TMP_MPY, origin="test")
        finally:
            if orig is None:
                del sys.modules["_openbricks_native"]
            else:
                sys.modules["_openbricks_native"] = orig
        self.assertEqual(calls, [(_TMP_MPY, "__main__")])

    def test_missing_mpy_surfaces_oserror_before_the_log_session(self):
        try:
            launcher._exec_program_raw(
                "/tmp/_openbricks_no_such.mpy", origin="test")
            self.fail("must raise")
        except OSError:
            pass


if __name__ == "__main__":
    unittest.main()
