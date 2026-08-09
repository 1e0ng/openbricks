# SPDX-License-Identifier: MIT
"""Run-log rotation on a REAL littlefs — the 400 ms commit pin.

The delete+create rotation churned littlefs directory metadata
until every commit's allocator traversal crawled it: bench
2026-08-09 measured ~400 ms per log flush after ~70 rotation
cycles, reproduced mechanically as 8k+ block reads per commit
(vs ~40 fresh). Slot reuse keeps commits at fresh-filesystem cost;
this test pins that on a live littlefs. MicroPython-only — CPython
has no ``vfs.VfsLfs2`` and skips.
"""

import tests._fakes  # noqa: F401

import unittest

try:
    import vfs
    _HAVE_LFS = hasattr(vfs, "VfsLfs2")
except ImportError:
    vfs = None
    _HAVE_LFS = False


BLOCK = 4096
# 384 KiB: enough blocks to churn (the degradation showed by cycle
# ~12 regardless of device size), small enough that two tests'
# devices fit MP's default heap under the suite orchestrator.
NBLOCKS = 96


class _CountingDev:
    def __init__(self):
        self.data = bytearray(BLOCK * NBLOCKS)
        self.reads = 0

    def readblocks(self, block, buf, off=0):
        self.reads += 1
        addr = block * BLOCK + off
        buf[:] = self.data[addr:addr + len(buf)]

    def writeblocks(self, block, buf, off=0):
        addr = block * BLOCK + off
        self.data[addr:addr + len(buf)] = buf

    def ioctl(self, op, arg):
        if op == 4:
            return NBLOCKS
        if op == 5:
            return BLOCK
        if op == 6:
            addr = arg * BLOCK
            self.data[addr:addr + BLOCK] = b"\xff" * BLOCK
            return 0
        return None


@unittest.skipUnless(_HAVE_LFS, "littlefs only exists on MicroPython")
class SlotRotationOnLittlefsTests(unittest.TestCase):
    MOUNT = "/_obtest_lfs"

    def setUp(self):
        import gc
        gc.collect()          # the previous test's device is 384 KiB
        from openbricks import log
        self.log = log
        self.dev = _CountingDev()
        vfs.VfsLfs2.mkfs(self.dev)
        vfs.mount(vfs.VfsLfs2(self.dev), self.MOUNT)
        self._prev_dir = log.LOG_DIR
        log.LOG_DIR = self.MOUNT + "/openbricks_logs"

    def tearDown(self):
        self.log.LOG_DIR = self._prev_dir
        vfs.umount(self.MOUNT)

    def _one_run(self):
        """A session with committed lines — the write_text pattern
        that paid the 400 ms on the bench. Returns the worst
        per-commit block-read count of the run."""
        worst = 0
        with self.log.session() as sess:
            for _ in range(10):
                r0 = self.dev.reads
                sess.write_text("a line that gets committed\n")
                cost = self.dev.reads - r0
                if cost > worst:
                    worst = cost
        return worst

    def test_commit_cost_does_not_grow_with_rotation_cycles(self):
        first = self._one_run()
        for _ in range(30):
            worst = self._one_run()
        # Delete+create rotation degraded this ~10x and rising by
        # cycle 12 (8440 reads/commit on the 14 MiB repro). Slot
        # reuse holds steady-state at fresh-filesystem cost; 3x
        # covers littlefs's natural variance with margin.
        self.assertLessEqual(
            worst, max(3 * max(first, 1), 60),
            "per-commit block reads grew %d -> %d across rotation "
            "cycles — directory churn is back" % (first, worst))

    def test_rotation_leaves_only_slot_files(self):
        import os
        for _ in range(self.log.MAX_RUNS + 3):
            with self.log.session():
                pass
        names = sorted(os.listdir(self.log.LOG_DIR))
        self.assertEqual(
            names,
            sorted("slot_%d.log" % i
                   for i in range(self.log.MAX_RUNS)))


if __name__ == "__main__":
    unittest.main()
