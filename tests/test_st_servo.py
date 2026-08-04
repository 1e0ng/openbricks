# SPDX-License-Identifier: MIT
"""Tests for the native servo-slot layer (``st_bus.servo_*``).

Runs under unix MicroPython only (the C module is the subject). The
pump driven here (``servo_pump``) is byte-identical to the firmware
hard tick's code path — same function — so what passes here is what
runs at 1 kHz on the hub.

The fake servo below answers by PARSING the wire: it replies to
whichever id was actually queried. Feeding replies blindly is wrong
by design — the bus core rejects wrong-id replies (tested in
test_st_bus) and the drain-before-TX rule then discards the rest.
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


def _pos_reply(servo_id, raw, speed=0, load=0):
    # Widened 6-byte feedback (1.50.0): pos + speed (b15 sign) +
    # load (b10 sign), matching the pump's read length.
    sp = (0x8000 | -speed) if speed < 0 else speed
    ld = (0x0400 | load) if load > 0 else -load   # b10 = positive
    return _reply(servo_id, 0, bytes([
        raw & 0xFF, (raw >> 8) & 0xFF,
        sp & 0xFF, (sp >> 8) & 0xFF,
        ld & 0xFF, (ld >> 8) & 0xFF]))


class _Wire:
    """Answer the captured TX like a healthy servo at ``raw`` counts."""

    def __init__(self):
        self.raw = 0
        self.tx_log = b""

    def pump(self):
        sb.servo_pump()
        tx = sb.take_tx()
        self.tx_log += tx
        if len(tx) >= 8 and tx[4] == 0x02:          # READ
            sb.feed_rx(_pos_reply(tx[2], self.raw))
        elif len(tx) >= 6 and tx[4] == 0x03 and tx[2] != 0xFE:  # WRITE
            sb.feed_rx(_reply(tx[2], 0))
        # SYNC WRITE (0x83) and broadcast: no reply by protocol.

    def settle(self, n=20):
        for _ in range(n):
            self.pump()


class _Base(unittest.TestCase):
    def setUp(self):
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()
        self.wire = _Wire()


class ConfigSequenceTests(_Base):
    def test_attach_configures_like_the_python_constructor(self):
        # st3215.py's sequence: op_mode=1 (wheel), torque=0 (fresh
        # motor coasts + releases a previous program's hold),
        # goal_acc.
        self.assertTrue(sb.servo_attach(0, 2, True, 45))
        self.wire.settle()
        tx = self.wire.tx_log
        i_mode = tx.find(bytes([0x21, 0x01]))
        i_torq = tx.find(bytes([0x28, 0x00]))
        i_acc  = tx.find(bytes([0x29, 45]))
        self.assertTrue(i_mode >= 0 and i_torq >= 0 and i_acc >= 0)
        self.assertTrue(i_mode < i_torq < i_acc)   # exact order

    def test_double_attach_same_slot_rejected(self):
        self.assertTrue(sb.servo_attach(0, 1, False, 0))
        self.assertFalse(sb.servo_attach(0, 2, False, 0))


class SpeedCommandTests(_Base):
    def setUp(self):
        super().setUp()
        # Bench mapping: left slot0 id2 inverted, right slot1 id1.
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.wire.settle()
        self.wire.tx_log = b""

    def test_encode_matches_the_python_driver(self):
        # st3215.py::_write_goal_speed_signed — sign-magnitude,
        # bit 15 = direction.
        self.assertEqual(sb.servo_encode(1000), 1000)
        self.assertEqual(sb.servo_encode(-1000), 1000 | 0x8000)
        self.assertEqual(sb.servo_encode(-40000), 0x7FFF | 0x8000)  # clamp

    def test_speeds_go_out_as_one_sync_write(self):
        sb.servo_run(0, 1000)      # inverted -> -1000 on the wire
        sb.servo_run(1, 800)
        self.wire.settle(6)
        tx = self.wire.tx_log
        self.assertIn(b"\x83", tx)                       # SYNC WRITE
        enc_l = sb.servo_encode(-1000)
        enc_r = sb.servo_encode(800)
        self.assertIn(bytes([2, enc_l & 0xFF, enc_l >> 8]), tx)
        self.assertIn(bytes([1, enc_r & 0xFF, enc_r >> 8]), tx)
        # Both wheels' setpoints share ONE speed packet (the
        # SyncServoGroup time-alignment lesson), and both torque-ons
        # share ONE torque packet (the atomic-stop rule) — identified
        # by their instr+reg+dlen signatures, since a bare 0x83 also
        # occurs inside encoded reverse speeds (0x8000|1000 = 0x83E8).
        self.assertEqual(tx.count(b"\x83\x2e\x02"), 1)   # speed sync
        self.assertEqual(tx.count(b"\x83\x28\x01"), 1)   # torque sync
        self.assertEqual(tx.count(b"\xff\xff\xfe"), 2)

    def test_run_implies_torque_on(self):
        sb.servo_run(1, 500)
        self.wire.settle(4)
        self.assertIn(bytes([0x28, 0x01]), self.wire.tx_log)

    def test_coast_cuts_torque_and_voids_stale_speed(self):
        sb.servo_run(1, 500)
        sb.servo_coast(1)
        self.wire.settle(6)
        tx = self.wire.tx_log
        # Torque-off rides the sync-torque frame: instr 0x83, reg
        # 0x28, dlen 1, then (id=1, value=0).
        i = tx.find(b"\x83\x28\x01")
        self.assertTrue(i >= 0, tx)
        self.assertEqual(tx[i + 3], 1)      # servo id
        self.assertEqual(tx[i + 4], 0)      # torque value: coast
        # The stale speed must NOT have gone out after the coast.
        enc = sb.servo_encode(500)
        self.assertEqual(tx.count(bytes([1, enc & 0xFF, enc >> 8])), 0)


class OdometryTests(_Base):
    def setUp(self):
        super().setUp()
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.wire.settle()          # primes have_raw at raw=0

    def _drive_to(self, raw, pumps=8):
        self.wire.raw = raw
        self.wire.settle(pumps)

    def test_unwrap_across_the_4096_boundary(self):
        # Primed at 0. 0->4000 unwraps SHORT WAY = -96 (not +4000);
        # then +90, +36 (across 4095->0), +90. Net +120.
        for raw in (4000, 4090, 30, 120):
            self._drive_to(raw)
        self.assertEqual(sb.servo_counts(1), 120)
        # Inverted slot reports the mirrored frame — command sign and
        # angle sign stay consistent, as DriveBase requires.
        self.assertEqual(sb.servo_counts(0), -120)

    def test_zero_read_failures_on_a_healthy_wire(self):
        for raw in (100, 200, 300):
            self._drive_to(raw)
        self.assertEqual(sb.servo_stats(0)[1], 0)
        self.assertEqual(sb.servo_stats(1)[1], 0)
        self.assertTrue(sb.servo_stats(0)[0] > 0)   # reads happened

    def test_silent_servo_counts_stale_not_wedge(self):
        # Detach the wire (stop answering reads): failures count,
        # stale grows, and the pump keeps cycling — never stuck.
        for _ in range(30):
            sb.servo_pump()          # no replies fed
        s0 = sb.servo_stats(0)
        s1 = sb.servo_stats(1)
        self.assertTrue(s0[1] + s1[1] > 0)          # failures counted
        self.assertTrue(s0[2] > 0 or s1[2] > 0)     # stale visible


class EStopTests(_Base):
    def test_torque_off_all_is_broadcast_and_immediate(self):
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.wire.settle()
        sb.servo_run(0, 1000)
        self.assertTrue(sb.torque_off_all())
        tx = sb.take_tx()
        # Broadcast id 0xFE, torque register, value 0.
        self.assertIn(bytes([0xFE]), tx)
        self.assertIn(bytes([0x28, 0x00]), tx[-16:])
        # Staged speed voided: pumping afterwards must not re-drive.
        self.wire.settle(6)
        enc = sb.servo_encode(-1000)
        self.assertEqual(
            self.wire.tx_log.count(bytes([2, enc & 0xFF, enc >> 8])), 0)

    def test_torque_off_all_jumps_an_inflight_transaction(self):
        sb.servo_attach(0, 2, True, 45)
        self.wire.settle()
        sb.servo_pump()              # starts a read; leave it hanging
        self.assertTrue(sb.torque_off_all())   # must not wait/queue


class ProgramBoundaryResetTests(_Base):
    def test_launcher_boundary_reset_frees_previous_programs_claims(self):
        # run_program calls _reset_motor_process before every
        # program; since 1.43.2 that also resets st_bus runtime state
        # — the principled fix for "slot attach failed until
        # power-cycle" (same precedent as motor_process.reset: a new
        # program must not inherit the previous one's native state).
        from openbricks import launcher
        self.assertTrue(sb.servo_attach(0, 2, True, 45))
        self.assertFalse(sb.servo_attach(0, 2, True, 45))  # claimed
        launcher._reset_motor_process()
        self.assertTrue(sb.servo_attach(0, 2, True, 45))   # freed

    def test_reset_runtime_disables_the_drivebase(self):
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.wire.settle()
        sb.db_config(0, 1, 88.0, 136.0, 400.0)
        sb.reset_runtime()
        # Slots free again AND the drivebase no longer drives.
        self.assertTrue(sb.servo_attach(0, 2, True, 45))
        sb.db_straight(100.0, 100.0)   # config gone: ignored by pump
        self.wire.settle(20)


class ManualCoexistenceTests(_Base):
    def test_manual_result_is_not_stolen_by_the_pump(self):
        # The pump only consumes results of transactions IT started.
        # Manual op goes first — once the pump is running, attached
        # servos keep the bus busy with feedback and a manual start
        # is (correctly) refused; see the test below.
        sb.servo_attach(0, 2, True, 45)
        self.assertTrue(sb.start_ping(9, 5))
        sb.take_tx()
        sb.feed_rx(_reply(9, 0))
        sb.servo_pump()              # polls; must leave result alone
        state, payload = sb.take_result()
        self.assertEqual(state, sb.DONE)

    def test_manual_start_refused_while_pump_owns_the_bus(self):
        # With slots attached, feedback fills every idle gap — manual
        # transactions lose the race by design (probe mode and drive
        # mode don't mix). The refusal must be visible, not silent
        # corruption.
        sb.servo_attach(0, 2, True, 45)
        self.wire.settle()
        self.assertFalse(sb.start_ping(9, 5))


class EStopIntegrationTests(_Base):
    def test_engage_reaches_the_native_bus(self):
        # The stop button's engage() must torque-off motors the
        # NATIVE pump drives — the Python bus objects can't reach a
        # natively-owned UART. Broadcast bytes must land on the wire.
        from openbricks import estop
        sb.servo_attach(0, 2, True, 45)
        self.wire.settle()
        sb.servo_run(0, 1000)
        sb.take_tx()
        estop.engage()
        try:
            tx = sb.take_tx()
            self.assertIn(b"\xff\xff\xfe", tx)          # broadcast frame
            self.assertIn(bytes([0x28, 0x00]), tx)       # torque off
        finally:
            estop.clear()


if __name__ == "__main__":
    unittest.main()
