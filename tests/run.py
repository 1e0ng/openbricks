# SPDX-License-Identifier: MIT
"""Test runner for openbricks.

Runs every test module against the real C implementation in the
``_openbricks_native`` user_c_module. Two modes:

* **Orchestrator** — invoked with no arguments. Walks ``_TEST_MODULES``
  and spawns a fresh MicroPython subprocess per module. Module-level
  isolation means C-side state (the ``motor_process`` singleton, the
  fake ``Timer._instances`` list, observer buffers, etc.) is reset by
  process death between modules. Cross-module interactions that
  caused earlier segfaults can't happen.

* **Worker** — invoked with a module name as ``sys.argv[1]``. Runs
  just that module's tests via ``unittest.main``. This is what the
  orchestrator shells out to.

Invoke as::

    ./native/micropython/ports/unix/build-standard/micropython tests/run.py

or, equivalently on CPython (for non-native tests during iteration)::

    python3 tests/run.py
"""

import sys
import os


_tests_dir = sys.path[0]
_idx = _tests_dir.rfind("/")
_repo_root = _tests_dir[:_idx] if _idx > 0 else "."

# Replace the script-directory entry with the repo root so ``import
# tests.*`` and ``import openbricks.*`` resolve through their package
# layouts.
sys.path[0] = _repo_root
if "tests" in sys.modules:
    del sys.modules["tests"]

# Hardware fakes before any openbricks driver imports them.
import tests._fakes  # noqa: F401


_TEST_MODULES = [
    "tests.test_l298n",
    "tests.test_encoder",
    "tests.test_bno055",
    "tests.test_tcs34725",
    "tests.test_st3215",
    "tests.test_trajectory",
    "tests.test_observer",
    "tests.test_scheduler",
    "tests.test_jgb37_520",
    "tests.test_config",
    "tests.test_drivebase",
    "tests.test_hub",
    "tests.test_ssd1306",
    "tests.test_pcnt_encoder",
    "tests.test_mg370",
    "tests.test_tb6612",
]


def _run_orchestrator():
    """Spawn one MP subprocess per module. Each runs tests/run.py with
    the module name as argv[1] and inherits a clean C state."""
    py = sys.executable
    if py is None or not py:
        print("ERROR: sys.executable is empty; run tests/run.py under a named interpreter.")
        sys.exit(2)

    any_failed = False
    for name in _TEST_MODULES:
        print("\n=== %s ===" % name)
        # os.system forwards stdout/stderr; the child's exit code is in
        # the low byte of the return value (POSIX encoding).
        cmd = py + " " + _repo_root + "/tests/run.py " + name
        rc = os.system(cmd)
        # Non-zero return in any module means failure or crash.
        if rc != 0:
            any_failed = True
            print("*** %s: child exited non-zero (0x%x)" % (name, rc))

    print("\n--- openbricks test summary ---")
    if any_failed:
        print("FAIL: one or more modules had failures/errors.")
        sys.exit(1)
    print("all modules passed.")
    sys.exit(0)


def _run_worker(module_name):
    """Run tests for ``module_name`` in-process."""
    # Lazily add the unittest path so the orchestrator side doesn't
    # perturb module resolution of ``tests.*``.
    sys.path.insert(0, _repo_root + "/native/micropython/lib/micropython-lib/python-stdlib/unittest")
    import unittest
    tests._fakes._install_unittest_shims()
    mod = __import__(module_name, None, None, [""])
    result = unittest.main(module=mod)
    # Return non-zero if anything failed.
    if result is None:
        sys.exit(0)
    fails = len(getattr(result, "failures", []))
    errs = len(getattr(result, "errors", []))
    sys.exit(0 if (fails == 0 and errs == 0) else 1)


def main():
    if len(sys.argv) > 1:
        _run_worker(sys.argv[1])
    else:
        _run_orchestrator()


if __name__ == "__main__":
    main()
