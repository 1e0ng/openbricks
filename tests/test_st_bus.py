# SPDX-License-Identifier: MIT
"""Tests for the native SCS serial-bus core (``st_bus``).

Runs ONLY under unix MicroPython (tests/run.py) — the C module is the
subject. The golden vectors below are transcribed from
``openbricks/drivers/st3215.py``'s packet construction (the
hardware-proven reference the C core mirrors byte-for-byte); each
shows its arithmetic so a mismatch is auditable by eye.

The transaction machine's whole reason to exist is what the Python
driver can't do: never block. A dropped servo reply costs a counted
number of ticks, not a 50 ms wall-clock stall — pinned here via
explicit poll() ticks.
"""

import unittest

try:
    from _openbricks_native import st_bus as sb
except ImportError:
    sb = None


def _chk(body):
    return (~sum(body)) & 0xFF


def _reply(servo_id, err, payload=b""):
    body = bytes([servo_id, len(payload) + 2, err]) + payload
    return b"\xff\xff" + body + bytes([_chk(body)])


class _Base(unittest.TestCase):
    def setUp(self):
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()


class GoldenPacketTests(_Base):
    """TX bytes must match st3215.py's construction exactly."""

    def test_read_packet(self):
        # st3215.py::read — body = id, LEN=4, READ(0x02), reg, n.
        self.assertTrue(sb.start_read(1, 0x38, 2, 5))
        body = bytes([0x01, 0x04, 0x02, 0x38, 0x02])
        self.assertEqual(sb.take_tx(),
                         b"\xff\xff" + body + bytes([_chk(body)]))

    def test_write_packet(self):
        # st3215.py::write — body = id, LEN=len+3, WRITE(0x03), reg, data.
        self.assertTrue(sb.start_write(2, 0x2A, b"\x10\x27"))
        body = bytes([0x02, 0x05, 0x03, 0x2A, 0x10, 0x27])
        self.assertEqual(sb.take_tx(),
                         b"\xff\xff" + body + bytes([_chk(body)]))

    def test_ping_packet(self):
        # st3215.py::ping — body = id, 2, PING(0x01).
        self.assertTrue(sb.start_ping(3, 5))
        body = bytes([0x03, 0x02, 0x01])
        self.assertEqual(sb.take_tx(),
                         b"\xff\xff" + body + bytes([_chk(body)]))

    def test_sync_write_packet(self):
        # st3215.py::sync_write — 0xFE, LEN=(1+dlen)*n+4, 0x83, reg,
        # dlen, then (id, data) per servo.
        self.assertTrue(sb.start_sync_write(
            0x2A, 2, [(1, b"\x10\x27"), (2, b"\xf0\xd8")]))
        body = bytes([0xFE, (1 + 2) * 2 + 4, 0x83, 0x2A, 0x02,
                      0x01, 0x10, 0x27, 0x02, 0xF0, 0xD8])
        self.assertEqual(sb.take_tx(),
                         b"\xff\xff" + body + bytes([_chk(body)]))


class TransactionTests(_Base):
    def test_read_round_trip(self):
        sb.start_read(1, 0x38, 2, 5)
        sb.take_tx()
        sb.feed_rx(_reply(1, 0x00, b"\xd2\x04"))
        sb.poll()
        state, payload = sb.take_result()
        self.assertEqual(state, sb.DONE)
        self.assertEqual(payload, b"\xd2\x04")

    def test_reply_split_across_ticks_reassembles(self):
        # Bytes trickle in — a real 1 Mbps reply spans <1 tick, but
        # the machine must not care how the transport chunks it.
        sb.start_read(1, 0x38, 2, 10)
        sb.take_tx()
        full = _reply(1, 0x00, b"\xd2\x04")
        sb.feed_rx(full[:3]); sb.poll()
        self.assertEqual(sb.state(), sb.AWAIT_REPLY)
        sb.feed_rx(full[3:]); sb.poll()
        state, payload = sb.take_result()
        self.assertEqual(state, sb.DONE)
        self.assertEqual(payload, b"\xd2\x04")

    def test_timeout_costs_exactly_the_budgeted_ticks(self):
        # THE non-blocking guarantee: a silent servo costs N polls,
        # never a wall-clock stall.
        sb.start_read(1, 0x38, 2, 3)
        sb.take_tx()
        for _ in range(2):
            sb.poll()
            self.assertEqual(sb.state(), sb.AWAIT_REPLY)
        sb.poll()   # third tick: deadline
        state, _ = sb.take_result()
        self.assertEqual(state, sb.TIMEOUT)
        self.assertEqual(sb.stats()[1], 1)   # n_timeout

    def test_bad_checksum_is_rejected(self):
        sb.start_read(1, 0x38, 2, 5)
        sb.take_tx()
        good = _reply(1, 0x00, b"\xd2\x04")
        sb.feed_rx(good[:-1] + bytes([good[-1] ^ 0xFF]))
        sb.poll()
        state, _ = sb.take_result()
        self.assertEqual(state, sb.BAD_REPLY)
        self.assertEqual(sb.stats()[2], 1)   # n_bad

    def test_reply_from_wrong_servo_is_rejected(self):
        sb.start_read(1, 0x38, 2, 5)
        sb.take_tx()
        sb.feed_rx(_reply(2, 0x00, b"\xd2\x04"))   # id 2, expected 1
        sb.poll()
        state, _ = sb.take_result()
        self.assertEqual(state, sb.BAD_REPLY)

    def test_bus_rejects_start_while_awaiting(self):
        sb.start_read(1, 0x38, 2, 5)
        self.assertFalse(sb.start_read(2, 0x38, 2, 5))
        self.assertFalse(sb.start_ping(2, 5))

    def test_stale_rx_is_flushed_before_tx(self):
        # st3215.py's drain-before-TX rule: residue must never
        # mis-frame the next reply.
        sb.feed_rx(b"\xff\xff\x99garbage")
        sb.start_read(1, 0x38, 2, 5)
        sb.take_tx()
        sb.feed_rx(_reply(1, 0x00, b"\xd2\x04"))
        sb.poll()
        state, payload = sb.take_result()
        self.assertEqual(state, sb.DONE)
        self.assertEqual(payload, b"\xd2\x04")
        self.assertTrue(sb.stats()[3] >= 1)   # n_flush


class WriteSemanticsTests(_Base):
    def test_write_status_is_collected_and_discarded(self):
        sb.start_write(1, 0x2A, b"\x10\x27")
        sb.take_tx()
        sb.feed_rx(_reply(1, 0x00))
        sb.poll()
        state, payload = sb.take_result()
        self.assertEqual(state, sb.DONE)
        self.assertEqual(payload, b"")       # discarded, not surfaced

    def test_write_with_lost_status_still_succeeds(self):
        # Matches the Python driver: the command very likely landed;
        # status loss on a shared half-duplex line is not an error.
        sb.start_write(1, 0x2A, b"\x10\x27")
        sb.take_tx()
        for _ in range(3):
            sb.poll()
        state, _ = sb.take_result()
        self.assertEqual(state, sb.DONE)
        self.assertEqual(sb.stats()[1], 0)   # no timeout counted

    def test_broadcast_write_completes_immediately(self):
        sb.start_write(0xFE, 0x28, b"\x00")
        state, _ = sb.take_result()
        self.assertEqual(state, sb.DONE)

    def test_sync_write_completes_immediately(self):
        sb.start_sync_write(0x2A, 2, [(1, b"\x10\x27")])
        state, _ = sb.take_result()
        self.assertEqual(state, sb.DONE)


class GuardTests(_Base):
    def test_oversized_read_rejected(self):
        self.assertFalse(sb.start_read(1, 0x38, 200, 5))

    def test_take_result_when_idle_reports_idle(self):
        state, payload = sb.take_result()
        self.assertEqual(state, sb.IDLE)
        self.assertEqual(payload, b"")

    def test_sync_write_wrong_data_len_rejected(self):
        self.assertFalse(sb.start_sync_write(0x2A, 2, [(1, b"\x10")]))

    def test_public_reexport_includes_st_bus(self):
        # First bench contact failed on this: the firmware HAD st_bus
        # but openbricks._native's explicit re-export list didn't —
        # ImportError on the documented public path.
        from openbricks._native import st_bus as via_public
        self.assertIs(via_public, sb)

    def test_attach_uart_settles_before_first_packet(self):
        # Source pin: the 20 ms post-open settle. Dropping it bricks
        # the URT-2 until power-cycle (bench, 1.40.0 first contact;
        # same lesson as st3215.py's sleep).
        with open("native/user_c_modules/openbricks/st_bus.c") as f:
            src = f.read()
        self.assertIn("mp_hal_delay_ms(20)", src)

    def test_attach_uart_absent_off_firmware(self):
        # The real-UART backend needs the bus-uart patch (esp32 only);
        # unix/sim must not expose it — Python driver selection keys
        # off its presence.
        self.assertFalse(hasattr(sb, "attach_uart"))

    def test_zero_length_read_and_write_rejected(self):
        self.assertFalse(sb.start_read(1, 0x38, 0, 5))
        self.assertFalse(sb.start_write(1, 0x2A, b""))

    def test_too_many_sync_servos_rejected(self):
        five = [(i, b"\x00\x00") for i in range(1, 6)]
        self.assertFalse(sb.start_sync_write(0x2A, 2, five))
        self.assertFalse(sb.start_sync_write(0x2A, 2, []))

    def test_refused_tx_fails_the_transaction_not_the_bus(self):
        # Fill the capture ring exactly (the test backend refuses past
        # 256 B, modelling a full driver TX queue). A broadcast write
        # of one byte is 8 wire bytes -> 32 writes fill the ring; the
        # 33rd is refused. The refusal must fail THAT transaction
        # visibly (n_bad) and leave the bus IDLE — never wedged.
        for _ in range(32):
            self.assertTrue(sb.start_write(0xFE, 0x28, b"\x00"))
            sb.take_result()
        self.assertFalse(sb.start_write(0xFE, 0x28, b"\x00"))
        self.assertEqual(sb.stats()[2], 1)      # n_bad counted
        self.assertEqual(sb.state(), sb.IDLE)   # not wedged
        sb.take_tx()                            # drain the ring
        self.assertTrue(sb.start_ping(1, 5))    # bus usable again


if __name__ == "__main__":
    unittest.main()
