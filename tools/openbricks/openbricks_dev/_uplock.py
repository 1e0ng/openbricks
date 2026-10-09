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

The lock is per MACHINE, not per account: the BLE link is shared by
every process on the host whatever its user. On Linux the temp
directory is the shared, sticky ``/tmp``, so the file is created
world-writable (0666, umask cleared), and a file another account made
read-only for us (an older CLI created it 0644) is locked through a
read-only descriptor — ``flock`` needs no write access, so the guard
is the same and only the PID note is lost. A lock file that cannot be
opened at all raises :class:`UploadLockError` naming its path.
"""
import os
import re
import sys
import tempfile

# Tests point this at a scratch directory; None = the platform temp dir.
LOCK_DIR = None

# Which locking primitive to use. A module flag rather than an inline
# platform check so the Windows branch is exercised (with a fake
# msvcrt) on every CI platform, not only where it happens to run.
_WINDOWS = sys.platform == "win32"

MESSAGE = "an upload is ongoing"


class UploadInProgress(Exception):
    """Another ``openbricks run`` / ``upload`` is transferring to this
    hub from this machine."""

    def __init__(self, name):
        self.name = name
        super().__init__(
            "%s (another openbricks run/upload is transferring to %r; "
            "wait for it to finish)" % (MESSAGE, name))


class UploadLockError(Exception):
    """The per-hub lock file cannot be opened, so the transfer cannot
    be guarded."""

    def __init__(self, path, err):
        self.path = path
        super().__init__(
            "cannot open the per-hub upload lock %s (%s) — remove the "
            "file (it may need its owner or root) and retry"
            % (path, err.strerror or err))


def _open_lock_file(path):
    """A descriptor on the lock file, created world-writable so any
    account on this machine can lock it later."""
    if _WINDOWS:
        try:
            return os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
        except OSError as e:
            raise UploadLockError(path, e)
    old_umask = os.umask(0)
    try:
        return os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    except PermissionError:
        # Another account's file we may not write: flock takes an
        # exclusive lock through a read-only descriptor just the same.
        try:
            return os.open(path, os.O_RDONLY)
        except OSError as e:
            raise UploadLockError(path, e)
    except OSError as e:
        raise UploadLockError(path, e)
    finally:
        os.umask(old_umask)


def lock_path(name):
    """The lock file for hub ``name`` — one per hub, filesystem-safe."""
    safe = re.sub(r"[^A-Za-z0-9_.-]", "_", name)
    base = LOCK_DIR if LOCK_DIR is not None else tempfile.gettempdir()
    return os.path.join(base, "openbricks-upload-%s.lock" % safe)


def _try_lock(fd):
    """Non-blocking exclusive lock on ``fd``; True when taken."""
    os.lseek(fd, 0, os.SEEK_SET)
    if _WINDOWS:
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
    if _WINDOWS:
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
    :class:`UploadInProgress` at once when another process holds it
    (and :class:`UploadLockError` when the lock file cannot be
    opened); ``release()`` is idempotent. Usable as a context
    manager."""

    def __init__(self, name):
        self.name = name
        self.path = lock_path(name)
        self._fd = None

    def acquire(self):
        if self._fd is not None:
            return
        fd = _open_lock_file(self.path)
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
