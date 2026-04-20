# SPDX-License-Identifier: MIT
"""Test runner for openbricks.

Runs every test module against the real C implementation in the
``_openbricks_native`` user_c_module. Designed for both:

- **MicroPython unix port** (production test runtime) — invoked as
  ``native/micropython/ports/unix/build-standard/micropython tests/run.py``
  from the repo root (or any cwd — the runner computes paths from its
  own location).
- **CPython** (fallback for iteration on pure-Python drivers) —
  invoked as ``python3 tests/run.py``.

On CPython, tests that depend on ``_openbricks_native`` skip
gracefully because that module doesn't exist there.

No Python mirrors of the native logic exist — every native-dependent
test exercises the real C code.
"""

import sys

# MicroPython puts the script's directory at ``sys.path[0]`` when
# running ``micropython tests/run.py``. Replace it with the repo root
# (the parent of tests/) so ``import tests.*`` and ``import openbricks.*``
# resolve via the package layout. Also add the micropython-lib unittest
# path so ``import unittest`` works on the MP side.
_tests_dir = sys.path[0]
_idx = _tests_dir.rfind("/")
_repo_root = _tests_dir[:_idx] if _idx > 0 else "."

# Replace the script-directory entry with the repo root so ``import
# tests.*`` and ``import openbricks.*`` resolve through their package
# layouts.
sys.path[0] = _repo_root
if "tests" in sys.modules:
    # MP may have cached a namespace entry keyed on the old path.
    del sys.modules["tests"]

# Install hardware fakes BEFORE extending sys.path with
# micropython-lib's unittest — otherwise MP (observed on v1.28) fails
# to find ``tests`` past the newly-prepended unittest entry.
import tests._fakes  # noqa: F401

sys.path.insert(0, _repo_root + "/native/micropython/lib/micropython-lib/python-stdlib/unittest")
import unittest

# micropython-lib's unittest doesn't ship assertGreater / assertLess;
# fill them in so our tests can use the full familiar vocabulary.
tests._fakes._install_unittest_shims()


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
]


def _load_module(name):
    """Import ``name`` and return the module object, or None if a
    dependency is unavailable on this runtime."""
    try:
        return __import__(name, None, None, [""])
    except ImportError as e:
        print("SKIP %s: %s" % (name, e))
        return None


def main():
    total_failures = 0
    total_errors = 0
    total_run = 0

    for name in _TEST_MODULES:
        mod = _load_module(name)
        if mod is None:
            continue
        print("\n=== %s ===" % name)
        result = unittest.main(module=mod)
        if result is None:
            continue
        # micropython-lib's unittest.TestRunner.run returns a
        # TestResult-like object with these attributes.
        total_failures += len(getattr(result, "failures", []))
        total_errors   += len(getattr(result, "errors", []))
        total_run      += getattr(result, "testsRun", 0)

    print("\n--- openbricks test summary ---")
    print("run=%d, failures=%d, errors=%d" % (total_run, total_failures, total_errors))
    sys.exit(0 if (total_failures == 0 and total_errors == 0) else 1)


if __name__ == "__main__":
    main()
