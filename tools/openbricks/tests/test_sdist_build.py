# SPDX-License-Identifier: MIT
"""Regression test for the v0.10.0 publish failure.

The failure mode: ``python -m build`` runs ``setup.py`` inside an
isolated environment where the repo root's
``native/user_c_modules/openbricks/`` directory isn't reachable
(only the sdist's contents are). If the shared C cores aren't
bundled in the sdist via ``MANIFEST.in``, ``_sync_cores()`` raises
``"missing shared core source"`` and the wheel build aborts.

This test builds a fresh sdist with ``python setup.py sdist`` and
inspects the resulting tarball — every shared core (``*_core.c`` /
``*_core.h``) must be present in ``native/_shared/`` inside the
tarball, or the publish workflow will break the next time a tag is
pushed.

Skipped if ``setuptools`` can't run (extremely unlikely on a normal
sim install).
"""

import os
import subprocess
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path


_PKG_ROOT = Path(__file__).resolve().parent.parent


_REQUIRED_CORES = [
    "trajectory_core.c", "trajectory_core.h",
    "observer_core.c",   "observer_core.h",
    "motor_process_core.c", "motor_process_core.h",
    "servo_core.c",      "servo_core.h",
    "drivebase_core.c",  "drivebase_core.h",
]


def _build_sdist_into(tmpdir):
    """Run ``python -m build --sdist --outdir <tmpdir>`` in the
    package root. Returns the produced tarball path. Raises
    SkipTest if ``build`` isn't installed."""
    try:
        subprocess.run(
            [sys.executable, "-m", "build", "--sdist", "--outdir", tmpdir],
            cwd=str(_PKG_ROOT),
            check=True,
            capture_output=True,
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    except FileNotFoundError:
        raise unittest.SkipTest("python interpreter unavailable")
    except subprocess.CalledProcessError as e:
        raise AssertionError(
            "python -m build --sdist failed:\n" +
            e.stderr.decode("utf-8", "replace"))
    sdists = sorted(Path(tmpdir).glob("openbricks-*.tar.gz"))
    if not sdists:
        raise AssertionError(
            "no openbricks-*.tar.gz produced; got: %s" %
            list(Path(tmpdir).iterdir()))
    return sdists[-1]


def _sdist_relpaths(tarball):
    """Return paths inside the sdist with the leading
    ``openbricks-<version>/`` prefix stripped."""
    with tarfile.open(tarball) as tar:
        names = tar.getnames()
    rel = []
    for n in names:
        parts = n.split("/", 1)
        rel.append(parts[1] if len(parts) == 2 else parts[0])
    return rel


class SdistBundlesSharedCoresTests(unittest.TestCase):
    """``MANIFEST.in`` + ``_sync_cores()`` together must guarantee
    that every shared core lands in the sdist tarball — otherwise
    the publish workflow's wheel build fails inside its isolated
    environment (where ``../../native/user_c_modules/`` isn't
    reachable). Skipped if ``build`` isn't installed."""

    @classmethod
    def setUpClass(cls):
        try:
            import build  # noqa: F401
        except ImportError:
            raise unittest.SkipTest(
                "the ``build`` package is required for this test; "
                "install via ``pip install build`` (already in the "
                "[dev] extras of ``openbricks``)")

    def test_sdist_contains_every_shared_core(self):
        with tempfile.TemporaryDirectory() as tmp:
            sdist = _build_sdist_into(tmp)
            rel = _sdist_relpaths(sdist)

        missing = []
        for core in _REQUIRED_CORES:
            wanted = "native/_shared/" + core
            if wanted not in rel:
                missing.append(core)
        self.assertEqual(
            missing, [],
            "sdist is missing shared cores under native/_shared/: %s\n"
            "MANIFEST.in must include ``recursive-include "
            "native/_shared *.c *.h``." % missing)

    def test_sdist_contains_native_extension_source(self):
        # Same shape: ``openbricks_sim_native.c`` must be in the
        # sdist too — without it the wheel build can't compile the
        # extension at all.
        with tempfile.TemporaryDirectory() as tmp:
            sdist = _build_sdist_into(tmp)
            rel = _sdist_relpaths(sdist)
        self.assertIn("native/openbricks_sim_native.c", rel)


if __name__ == "__main__":
    unittest.main()
