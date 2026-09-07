# SPDX-License-Identifier: MIT
"""One upload per hub per machine.

``openbricks run`` and ``openbricks upload`` both push a program to
the hub through the same raw-paste channel, and the operating system
shares a single BLE link between every process that connects to the
same peripheral (CoreBluetooth, BlueZ and WinRT all multiplex). Two
transfers started from two terminals therefore interleave their bytes
on the hub's REPL — a corrupted program, or both terminals reading
each other's output — and the hub, which sees ONE connection, cannot
tell them apart. The guard lives where the knowledge is: the host
holds a per-hub lock for the duration of a transfer, and a second
transfer to the same hub fails at once, before any scan, with
``an upload is ongoing``.

The lock is an OS-level advisory lock on a file in the temp directory
(``flock`` on POSIX, ``msvcrt.locking`` on Windows), so a CLI that
crashed or was killed mid-transfer never leaves a stale lock behind —
the kernel releases it with the process. The file itself persists and
carries the holder's PID for a curious human; the lock, not the
content, is the guard.
"""
import os
import re
import sys
import tempfile

# Tests point this at a scratch directory; None = the platform temp dir.
LOCK_DIR = None

MESSAGE = "an upload is ongoing"


class UploadInProgress(Exception):
    """Another ``openbricks run`` / ``upload`` is transferring to this
    hub from this machine."""

    def __init__(self, name):
        self.name = name
        super().__init__(
            "%s (another openbricks run/upload is transferring to %r; "
            "wait for it to finish)" % (MESSAGE, name))


def lock_path(name):
    """The lock file for hub ``name`` — one per hub, filesystem-safe."""
    safe = re.sub(r"[^A-Za-z0-9_.-]", "_", name)
    base = LOCK_DIR if LOCK_DIR is not None else tempfile.gettempdir()
    return os.path.join(base, "openbricks-upload-%s.lock" % safe)


def _try_lock(fd):
    """Non-blocking exclusive lock on ``fd``; True when taken."""
    os.lseek(fd, 0, os.SEEK_SET)
    if sys.platform == "win32":
        import msvcrt
        try:
            msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
            return True
        except OSError:
            return False
    import fcntl
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return True
    except OSError:
        return False


def _unlock(fd):
    os.lseek(fd, 0, os.SEEK_SET)
    if sys.platform == "win32":
        import msvcrt
        try:
            msvcrt.locking(fd, msvcrt.LK_UNLCK, 1)
        except OSError:
            pass
        return
    import fcntl
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    except OSError:
        pass


class UploadLock:
    """Hold the hub's transfer lock: ``acquire()`` raises
    :class:`UploadInProgress` at once when another process holds it;
    ``release()`` is idempotent. Usable as a context manager."""

    def __init__(self, name):
        self.name = name
        self.path = lock_path(name)
        self._fd = None

    def acquire(self):
        if self._fd is not None:
            return
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o644)
        if not _try_lock(fd):
            os.close(fd)
            raise UploadInProgress(self.name)
        try:
            os.ftruncate(fd, 0)
            os.write(fd, ("%d\n" % os.getpid()).encode())
        except OSError:
            pass                    # the note is a courtesy
        self._fd = fd

    def release(self):
        fd, self._fd = self._fd, None
        if fd is None:
            return
        _unlock(fd)
        os.close(fd)

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.release()
        return False
