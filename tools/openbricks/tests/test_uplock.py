# SPDX-License-Identifier: MIT
"""The per-hub upload lock (3.10.0): a second ``openbricks run`` /
``upload`` to a hub that another process on this machine is still
transferring to fails at once with ``an upload is ongoing``."""

import os
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from openbricks_dev import _uplock


class UploadLockTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        patcher = patch.object(_uplock, "LOCK_DIR", self.dir)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_second_holder_is_refused_with_the_message(self):
        with _uplock.UploadLock("ls"):
            with self.assertRaises(_uplock.UploadInProgress) as cm:
                _uplock.UploadLock("ls").acquire()
        msg = str(cm.exception)
        self.assertTrue(msg.startswith("an upload is ongoing"), msg)
        self.assertIn("'ls'", msg)
        self.assertEqual(cm.exception.name, "ls")

    def test_release_frees_the_hub_and_is_idempotent(self):
        a = _uplock.UploadLock("ls")
        a.acquire()
        a.acquire()                 # re-entrant no-op for the holder
        a.release()
        b = _uplock.UploadLock("ls")
        b.acquire()
        b.release()
        a.release()                 # already released: harmless
        b.release()

    def test_context_manager_releases_on_error(self):
        try:
            with _uplock.UploadLock("ls"):
                raise ValueError("boom")
        except ValueError:
            pass
        with _uplock.UploadLock("ls"):
            pass

    def test_other_hubs_are_independent(self):
        with _uplock.UploadLock("ls"):
            with _uplock.UploadLock("RobotB"):
                pass

    def test_lock_path_is_per_hub_and_filesystem_safe(self):
        p = _uplock.lock_path("my hub/ü")
        self.assertTrue(p.startswith(self.dir), p)
        self.assertEqual(os.path.basename(p), "openbricks-upload-my_hub__.lock")
        self.assertEqual(_uplock.lock_path("ls"), _uplock.lock_path("ls"))
        self.assertNotEqual(_uplock.lock_path("ls"), _uplock.lock_path("lt"))

    def test_holder_pid_is_recorded_for_a_human(self):
        with _uplock.UploadLock("ls") as lock:
            with open(lock.path) as f:
                self.assertEqual(int(f.read().strip()), os.getpid())

    def test_pid_note_failure_does_not_cost_the_lock(self):
        # The note is a courtesy; the lock is the guard.
        with patch.object(os, "ftruncate", side_effect=OSError("ro")):
            lock = _uplock.UploadLock("ls")
            lock.acquire()
        try:
            with self.assertRaises(_uplock.UploadInProgress):
                _uplock.UploadLock("ls").acquire()
        finally:
            lock.release()

    def test_posix_unlock_failure_still_closes_the_descriptor(self):
        import fcntl
        lock = _uplock.UploadLock("ls")
        lock.acquire()
        with patch.object(fcntl, "flock", side_effect=OSError("gone")):
            lock.release()
        self.assertIsNone(lock._fd)
        with _uplock.UploadLock("ls"):        # the close freed it
            pass

    def _fake_msvcrt(self, refuse_lock=False, unlock_raises=False):
        calls = []

        class _Msvcrt:
            LK_NBLCK = 2
            LK_UNLCK = 0

            @staticmethod
            def locking(fd, mode, nbytes):
                calls.append((mode, nbytes))
                if mode == _Msvcrt.LK_NBLCK and refuse_lock:
                    raise OSError("locked by another process")
                if mode == _Msvcrt.LK_UNLCK and unlock_raises:
                    raise OSError("already unlocked")
        p = patch.dict(sys.modules, {"msvcrt": _Msvcrt})
        p.start()
        self.addCleanup(p.stop)
        q = patch.object(_uplock, "_WINDOWS", True)
        q.start()
        self.addCleanup(q.stop)
        return calls

    def test_windows_branch_locks_one_byte_non_blocking(self):
        calls = self._fake_msvcrt()
        with _uplock.UploadLock("ls"):
            pass
        self.assertEqual(calls, [(2, 1), (0, 1)])     # LK_NBLCK, LK_UNLCK

    def test_windows_branch_refusal_is_the_same_error(self):
        self._fake_msvcrt(refuse_lock=True)
        with self.assertRaises(_uplock.UploadInProgress):
            _uplock.UploadLock("ls").acquire()

    def test_windows_branch_unlock_failure_is_swallowed(self):
        self._fake_msvcrt(unlock_raises=True)
        lock = _uplock.UploadLock("ls")
        lock.acquire()
        lock.release()
        self.assertIsNone(lock._fd)

    def test_a_holder_in_another_process_is_seen_and_dies_with_it(self):
        # The real case: two terminals. And the lock is the kernel's,
        # so a killed CLI leaves nothing stale behind.
        code = ("import sys, time\n"
                "from openbricks_dev import _uplock\n"
                "_uplock.LOCK_DIR = sys.argv[1]\n"
                "lock = _uplock.UploadLock('ls')\n"
                "lock.acquire()\n"
                "print('held', flush=True)\n"
                "time.sleep(30)\n")
        proc = subprocess.Popen([sys.executable, "-c", code, self.dir],
                                stdout=subprocess.PIPE)
        try:
            self.assertEqual(proc.stdout.readline().strip(), b"held")
            with self.assertRaises(_uplock.UploadInProgress):
                _uplock.UploadLock("ls").acquire()
        finally:
            proc.kill()
            proc.wait()
        with _uplock.UploadLock("ls"):
            pass


if __name__ == "__main__":
    unittest.main()
