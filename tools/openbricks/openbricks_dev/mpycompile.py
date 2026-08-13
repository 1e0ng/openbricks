# SPDX-License-Identifier: MIT
"""Host-side ``mpy-cross`` compilation for ``run`` and ``upload``.

Programs are cross-compiled to MicroPython persistent bytecode
(``.mpy``) on the host before any BLE connection is attempted, so:

* syntax errors surface in milliseconds with the compiler's
  file:line message — no scan, no connect, no upload;
* the hub skips the parse/compile step entirely at start
  (``exec_mpy`` loads the bytecode directly);
* tracebacks name the real source file and line (the compiler
  records the display name we pass with ``-s``) instead of the
  ``File "<string>"`` frames source-exec produced.

The ``mpy-cross`` binary comes from the pinned PyPI package (see
pyproject). Compatibility contract: bytecode-only ``.mpy`` files
carry the format major version (6) and no native-arch sub-version,
and the firmware's loader rejects a mismatch with
``ValueError: incompatible .mpy file`` — checked by the format
canary in tests/test_mpycompile.py, which must be revisited on
MicroPython submodule bumps.
"""

import os
import subprocess
import tempfile


class CompileError(Exception):
    pass


# .mpy header prefix the pinned firmware accepts: 'M' + format major
# version 6 (native/micropython/py/persistentcode.h MPY_VERSION).
MPY_HEADER_PREFIX = b"M\x06"


def _mpy_cross_binary():
    """Path to the pinned ``mpy-cross`` executable. Import deferred so
    ``openbricks --help`` doesn't pay for it."""
    try:
        import mpy_cross
    except ImportError:
        raise CompileError(
            "the mpy-cross package is not installed — reinstall the "
            "openbricks CLI (pipx upgrade openbricks) or "
            "pip install mpy-cross")
    return mpy_cross.mpy_cross


def compile_source(source_bytes, display_name):
    """Cross-compile Python source to ``.mpy`` bytes.

    ``display_name`` is what hub tracebacks will show as the file name
    (passed through ``mpy-cross -s``). Raises :class:`CompileError`
    carrying the compiler's message (which includes file and line for
    syntax errors).
    """
    with tempfile.TemporaryDirectory() as tmpdir:
        src = os.path.join(tmpdir, "program.py")
        out = os.path.join(tmpdir, "program.mpy")
        with open(src, "wb") as f:
            f.write(source_bytes)
        proc = subprocess.run(
            [_mpy_cross_binary(), "-o", out, "-s", display_name, src],
            capture_output=True, text=True)
        if proc.returncode != 0:
            message = (proc.stderr or proc.stdout or "").strip()
            raise CompileError(
                "mpy-cross failed:\n%s" % (message or
                                           "rc=%d" % proc.returncode))
        with open(out, "rb") as f:
            mpy_bytes = f.read()
    if not mpy_bytes.startswith(MPY_HEADER_PREFIX):
        # A header the firmware would reject must fail HERE, not as a
        # ValueError in the hub's run log after a full upload.
        raise CompileError(
            "mpy-cross produced format %r but the firmware expects "
            "%r — the pinned mpy-cross and firmware MicroPython have "
            "diverged; fix the pyproject pin" % (
                mpy_bytes[:2], MPY_HEADER_PREFIX))
    return mpy_bytes
