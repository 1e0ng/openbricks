# SPDX-License-Identifier: MIT
"""
``setup.py`` for the ``openbricks_sim._native`` CPython extension.

Needed because we share algorithmic C sources with the firmware (they
live at ``native/user_c_modules/openbricks/*_core.c``, two levels up),
and setuptools refuses both absolute paths *and* relative paths that
reach upward out of the package root. The fix is a tiny pre-build
hook that copies the shared cores into a local ``native/_shared/``
directory so setuptools only ever sees in-tree sources.

Everything else (metadata, dependencies, scripts) still lives in
``pyproject.toml``; setuptools merges the two.

Build contexts
--------------

In a **repo checkout** (editable install, local ``python -m build``,
CI), the firmware sources at ``../../native/user_c_modules/openbricks``
exist and we copy them into ``native/_shared/`` fresh on every build.
That keeps the truth in one place: the firmware's user_c_modules.

In an **sdist build** (PyPI publish, isolated build environment), the
upstream firmware sources are NOT in the source tree — only what
made it into the sdist tarball is. ``MANIFEST.in`` ensures the
already-synced ``native/_shared/*.{c,h}`` files are bundled, so the
``_sync_cores()`` step is a no-op (the files already exist locally
and there's nothing upstream to copy from). The fallback path here
detects that case and proceeds without raising.
"""

from pathlib import Path
import shutil

from setuptools import setup, Extension


HERE        = Path(__file__).parent.resolve()
CORE_SRC    = (HERE / ".." / ".." / "native" / "user_c_modules" / "openbricks").resolve()
SHARED_DIR  = HERE / "native" / "_shared"


# Files to mirror from the firmware's user_c_modules into our local
# tree. Each entry is compiled into ``openbricks_sim._native`` below.
_CORE_FILES = [
    "trajectory_core.c",
    "trajectory_core.h",
    "observer_core.c",
    "observer_core.h",
    "motor_process_core.c",
    "motor_process_core.h",
    "servo_core.c",
    "servo_core.h",
    "drivebase_core.c",
    "drivebase_core.h",
]


def _sync_cores():
    """Refresh the shared core sources into ``native/_shared/``.

    Two valid build modes:

      * Repo checkout — the firmware sources at ``CORE_SRC`` exist;
        copy from there. This is the path used during development
        and CI test runs.
      * sdist build — ``CORE_SRC`` doesn't exist (only sdist contents
        are reachable); ``native/_shared/`` was pre-bundled by the
        sdist via MANIFEST.in. Verify the files are present and
        leave them alone.

    Anything else (no ``CORE_SRC`` *and* no pre-bundled ``_shared/``)
    is a broken checkout and we raise.
    """
    SHARED_DIR.mkdir(parents=True, exist_ok=True)
    src_available = CORE_SRC.is_dir()

    for fname in _CORE_FILES:
        src = CORE_SRC / fname
        dst = SHARED_DIR / fname
        if src_available and src.is_file():
            shutil.copyfile(src, dst)
            continue
        # No upstream — this is the sdist path. The file must already
        # be present locally (bundled via MANIFEST.in).
        if not dst.is_file():
            raise RuntimeError(
                "missing shared core source: cannot copy from " +
                str(src) + " (upstream unavailable) and " +
                str(dst) + " is not bundled either — broken checkout?")


_sync_cores()


setup(
    ext_modules=[
        Extension(
            "openbricks_sim._native",
            sources=[
                "native/openbricks_sim_native.c",
                # Shared cores compiled byte-identical with the
                # firmware (see ``_sync_cores`` above). Any future
                # ``*_core.c`` lands here.
                "native/_shared/trajectory_core.c",
                "native/_shared/observer_core.c",
                "native/_shared/motor_process_core.c",
                "native/_shared/servo_core.c",
                "native/_shared/drivebase_core.c",
            ],
            include_dirs=[
                "native/_shared",
            ],
        ),
    ],
)
