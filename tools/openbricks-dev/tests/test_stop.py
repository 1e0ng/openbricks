# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev.stop`` with NUSLink stubbed."""

import argparse
import asyncio
import io
import unittest
from unittest.mock import patch

from openbricks_dev import stop as stop_mod
from openbricks_dev._nus import NUSError


class _FakeLink:
    def __init__(self, drain_chunk=b""):
        self._drain = drain_chunk
        self.writes = []
        self.closed = False

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.close()

    async def write(self, data):
        self.writes.append(bytes(data))

    async def read(self, timeout=None):
        return self._drain

    async def close(self):
        self.closed = True


def _args(name="RobotA", scan_timeout=5.0):
    return argparse.Namespace(name=name, scan_timeout=scan_timeout)


class StopTests(unittest.TestCase):
    def test_sends_single_ctrl_c(self):
        fake = _FakeLink()

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(stop_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch.object(stop_mod, "_DRAIN_DELAY_S", 0):
            rc = stop_mod.run(_args())

        self.assertEqual(rc, 0)
        # Exactly one write, exactly one byte, and that byte is Ctrl-C.
        self.assertEqual(fake.writes, [b"\x03"])
        self.assertTrue(fake.closed)

    def test_drain_output_is_printed(self):
        fake = _FakeLink(drain_chunk=b"Traceback (most recent call last):\r\n")

        async def _fake_connect(name, scan_timeout=5.0):
            return fake

        with patch.object(stop_mod.NUSLink, "connect", side_effect=_fake_connect), \
             patch.object(stop_mod, "_DRAIN_DELAY_S", 0), \
             patch("sys.stdout", new_callable=io.StringIO) as out:
            stop_mod.run(_args())

        self.assertIn("Traceback", out.getvalue())

    def test_connect_failure_raises_stop_error(self):
        async def _raise(name, scan_timeout=5.0):
            raise NUSError("no hub named 'Ghost' found")

        with patch.object(stop_mod.NUSLink, "connect", side_effect=_raise):
            with self.assertRaises(stop_mod.StopError):
                stop_mod.run(_args(name="Ghost"))


if __name__ == "__main__":
    unittest.main()
