# SPDX-License-Identifier: MIT
"""
``setup.py`` for the ``openbricks_sim._native`` CPython extension.

Needed because we share algorithmic C sources with the firmware (they
live at ``native/user_c_modules/openbricks/*_core.c``, two levels up),
and setuptools refuses both absolute paths *and* relative paths that
reach upward out of the package root. The fix is a tiny pre-build
hook that copies the shared cores into a local ``native/_shared/``
directory so setuptools only ever sees in-tree sources. The copy is
reproducible and cached — it's re-done on every build but the bytes
are identical to the upstream.

Everything else (metadata, dependencies, scripts) still lives in
``pyproject.toml``; setuptools merges the two.
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
]


def _sync_cores():
    """Copy the shared core sources into ``native/_shared/`` so
    setuptools can compile them as in-tree files. Idempotent."""
    SHARED_DIR.mkdir(parents=True, exist_ok=True)
    for fname in _CORE_FILES:
        src = CORE_SRC / fname
        dst = SHARED_DIR / fname
        if not src.is_file():
            raise RuntimeError(
                "missing shared core source: " + str(src) +
                " — is the repo checkout complete?")
        shutil.copyfile(src, dst)


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
            ],
            include_dirs=[
                "native/_shared",
            ],
        ),
    ],
)
