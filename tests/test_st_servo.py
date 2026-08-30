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
    """Answer the captured TX like a healthy servo at ``raw`` counts.

    Register-aware (1.91.0): non-broadcast writes commit into a
    per-servo register map, and reads of anything but the feedback
    burst answer FROM that map — so the config sequence's op-mode
    read-back (reg 0x21) sees what was actually written. Two seams
    model misbehaving servos: ``drop_writes`` eats the next N writes
    whole (no ACK — transport loss), ``apply_writes=False`` ACKs
    every write while committing none of them (the cold-boot EEPROM
    drop that left a bench wheel in the wrong mode, 2026-08-30)."""

    def __init__(self):
        self.raw = 0
        self.tx_log = b""
        self.regs = {}              # (servo_id, reg) -> byte
        self.apply_writes = True
        self.drop_writes = 0

    def pump(self):
        sb.servo_pump()
        tx = sb.take_tx()
        self.tx_log += tx
        if len(tx) >= 8 and tx[4] == 0x02:          # READ
            reg, nbytes = tx[5], tx[6]
            if reg == 0x38:                          # feedback burst
                sb.feed_rx(_pos_reply(tx[2], self.raw))
            else:
                payload = bytes(self.regs.get((tx[2], reg + i), 0)
                                for i in range(nbytes))
                sb.feed_rx(_reply(tx[2], 0, payload))
        elif len(tx) >= 6 and tx[4] == 0x03 and tx[2] != 0xFE:  # WRITE
            if self.drop_writes:
                self.drop_writes -= 1
                return                               # eaten: no ACK
            if self.apply_writes:
                for i, b in enumerate(tx[6:-1]):
                    self.regs[(tx[2], tx[5] + i)] = b
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

    def test_lost_config_write_retries_the_same_register(self):
        # The op_mode ACK is eaten once; the pump must reissue reg
        # 0x21, not advance past it — a skipped op_mode is a servo in
        # position mode receiving speed sync-writes.
        self.assertTrue(sb.servo_attach(0, 2, False, 45))
        self.wire.drop_writes = 1              # first write: silence
        self.wire.settle(30)
        tx = self.wire.tx_log
        # Count WRITE packets (instr 0x03) — the verify READ's bytes
        # (0x02 0x21 0x01) would otherwise match the bare pattern.
        self.assertEqual(tx.count(bytes([0x03, 0x21, 0x01])), 2)
        self.assertEqual(sb.servo_write_stats(0), (1, 0))   # counted
        self.assertTrue(sb.servo_stats(0)[0] > 0)           # then lives

    def test_config_reads_op_mode_back_before_going_live(self):
        # ACKs prove transport, not application: the sequence must
        # END with a read of reg 0x21 (1 byte), after the writes, and
        # only then may feedback polling begin.
        self.assertTrue(sb.servo_attach(0, 2, False, 45))
        self.wire.settle()
        tx = self.wire.tx_log
        i_verify = tx.find(bytes([0x02, 0x21, 0x01]))   # READ 0x21 x1
        i_acc = tx.find(bytes([0x29, 45]))
        self.assertTrue(i_verify >= 0)
        self.assertTrue(i_verify > i_acc)               # after writes
        self.assertTrue(sb.servo_stats(0)[0] > 0)       # then lives

    def test_acked_but_unapplied_mode_write_latches_with_evidence(self):
        # The cold-boot servo: every write ACKed, none committed —
        # op_mode reads back 0, so the slot must latch config_failed
        # WITH the mismatch evidence (never "check your wiring": the
        # wiring just carried eight perfect ACKs), and feedback reads
        # must never start for it.
        self.assertTrue(sb.servo_attach(0, 2, False, 45))
        self.wire.apply_writes = False
        self.wire.settle(400)      # 8 rounds incl. 25-pump cooldowns
        _fails, failed, mismatch, got = sb.servo_config_state(0)
        self.assertEqual(failed, 1)
        self.assertEqual(mismatch, 1)
        self.assertEqual(got, 0)                    # the mode it read
        self.assertEqual(sb.servo_write_stats(0)[1], 1)   # latched
        self.assertEqual(sb.servo_stats(0)[0], 0)   # never went live

    def test_mode_applied_on_a_later_round_heals(self):
        # A servo that finishes its power-on init mid-retry: the
        # cooldown-paced re-runs pick it up and the slot goes live
        # with no latch and no leftover mismatch evidence.
        self.assertTrue(sb.servo_attach(0, 2, False, 45))
        self.wire.apply_writes = False
        self.wire.settle(40)               # at least one failed round
        self.wire.apply_writes = True      # servo boot completes
        self.wire.settle(300)
        _fails, failed, mismatch, _got = sb.servo_config_state(0)
        self.assertEqual(failed, 0)
        self.assertEqual(mismatch, 0)
        self.assertEqual(sb.servo_write_stats(0)[1], 0)
        self.assertTrue(sb.servo_stats(0)[0] > 0)


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

    def test_coast_then_run_immediately_still_drives(self):
        # stop(); run_speed(...) faster than the pump ships the coast:
        # the pending torque-off is superseded. Unfixed, the wire got
        # torque-off then the new speed — motor limp with a live goal
        # speed, and nothing ever re-staged torque.
        sb.servo_run(1, 500)
        self.wire.settle(6)
        self.wire.tx_log = b""
        sb.servo_coast(1)
        sb.servo_run(1, 400)                # before any pump
        self.wire.settle(6)
        tx = self.wire.tx_log
        i = tx.find(b"\x83\x28\x01")
        self.assertTrue(i >= 0, tx)
        self.assertEqual(tx[i + 4], 1)      # torque ON ships, not off
        self.assertEqual(tx.count(b"\x83\x28\x01"), 1)   # ONE sync
        enc = sb.servo_encode(400)
        self.assertIn(bytes([1, enc & 0xFF, enc >> 8]), tx)

    def test_a_coasted_motor_stops_costing_the_driving_one_reads(self):
        # Heat heuristic end-to-end: both drive, then slot 1 coasts.
        # A parked motor must drop to the cold rotation (one read in
        # OB_SSERVO_COLD_EVERY), not keep splitting the bus 50/50 —
        # the 1.59.x bandwidth regression, which a stale target_steps
        # quietly reintroduced for coast (but not brake).
        sb.servo_run(0, 300)
        sb.servo_run(1, 300)
        self.wire.settle(10)
        sb.servo_coast(1)
        self.wire.settle(6)                 # coast ships
        self.wire.tx_log = b""
        self.wire.settle(160)
        tx = self.wire.tx_log
        reads_driving = tx.count(b"\xff\xff\x02\x04\x02\x38")
        reads_coasted = tx.count(b"\xff\xff\x01\x04\x02\x38")
        self.assertTrue(reads_driving > 0 and reads_coasted > 0,
                        (reads_driving, reads_coasted))
        self.assertTrue(reads_driving > 3 * reads_coasted,
                        (reads_driving, reads_coasted))


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
        # Slots free again AND the drivebase no longer drives: arming
        # without a config RAISES (1.65.1) instead of indexing
        # st_moves[-1] and arming a zero-geometry controller.
        self.assertTrue(sb.servo_attach(0, 2, True, 45))
        try:
            sb.db_straight(100.0, 100.0)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("db_config" in str(e), e)
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


class _RegWire(_Wire):
    """A servo with real registers: WRITEs land in ``regs``, READs of
    any register answer from it (position reads keep the widened
    6-byte reply the pump expects)."""

    def __init__(self):
        _Wire.__init__(self)
        self.regs = {}
        self.mute = 0        # swallow this many replies (lost on wire)

    def pump(self):
        sb.servo_pump()
        tx = sb.take_tx()
        self.tx_log += tx
        if not tx:
            return
        if self.mute > 0:
            self.mute -= 1
            return
        if len(tx) >= 8 and tx[4] == 0x02:               # READ reg n
            reg, n = tx[5], tx[6]
            if reg == 0x38:
                sb.feed_rx(_pos_reply(tx[2], self.raw))
            else:
                v = self.regs.get(reg, 0)
                payload = bytes([(v >> (8 * i)) & 0xFF
                                 for i in range(n)])
                sb.feed_rx(_reply(tx[2], 0, payload))
        elif len(tx) >= 6 and tx[4] == 0x03 and tx[2] != 0xFE:  # WRITE
            reg = tx[5]
            v = 0
            for i, b in enumerate(tx[6:-1]):
                v |= b << (8 * i)
            self.regs[reg] = v
            sb.feed_rx(_reply(tx[2], 0))


class UserRegisterTxnTests(_Base):
    """servo_user_write / servo_user_read / servo_user_poll — the
    duty_limit path: one-deep staged transactions shipped by the pump
    once config is done, resolved by verified ACK or a latched
    failure after CONFIG_TRIES losses (a loss is never silent)."""

    REG = 0x30    # torque limit — what the feature exists for

    def setUp(self):
        _Base.setUp(self)
        self.wire = _RegWire()

    def _configured(self, slot=0, sid=2):
        sb.servo_attach(slot, sid, False, 45)
        self.wire.settle()

    def test_write_lands_and_polls_done(self):
        self._configured()
        self.assertTrue(sb.servo_user_write(0, self.REG, 300, 2))
        self.wire.settle(10)
        st, _ = sb.servo_user_poll(0)
        self.assertEqual(st, 1)
        self.assertEqual(self.wire.regs[self.REG], 300)

    def test_read_returns_the_register_value(self):
        self._configured()
        self.wire.regs[self.REG] = 777
        self.assertTrue(sb.servo_user_read(0, self.REG, 2))
        self.wire.settle(10)
        st, val = sb.servo_user_poll(0)
        self.assertEqual(st, 1)
        self.assertEqual(val, 777)

    def test_poll_with_nothing_staged(self):
        self._configured()
        st, _ = sb.servo_user_poll(0)
        self.assertEqual(st, -2)

    def test_stage_on_unattached_slot_rejected(self):
        self.assertFalse(sb.servo_user_write(1, self.REG, 300, 2))

    def test_second_stage_while_pending_rejected(self):
        self._configured()
        self.assertTrue(sb.servo_user_write(0, self.REG, 300, 2))
        self.assertFalse(sb.servo_user_write(0, self.REG, 400, 2))

    def test_lost_ack_retries_until_it_lands(self):
        self._configured()
        self.wire.mute = 2                   # first two acks lost
        self.assertTrue(sb.servo_user_write(0, self.REG, 300, 2))
        self.wire.settle(40)                 # timeouts + retries
        st, _ = sb.servo_user_poll(0)
        self.assertEqual(st, 1)
        self.assertEqual(self.wire.regs[self.REG], 300)

    def test_dead_servo_latches_failure_then_allows_restage(self):
        self._configured()
        self.wire.mute = 10 ** 6             # never answers again
        self.assertTrue(sb.servo_user_write(0, self.REG, 300, 2))
        self.wire.settle(120)                # 8 tries x timeout window
        st, _ = sb.servo_user_poll(0)
        self.assertEqual(st, -1)
        # The latch is consumed: a new transaction may be staged.
        self.wire.mute = 0
        self.assertTrue(sb.servo_user_write(0, self.REG, 500, 2))
        self.wire.settle(10)
        st, _ = sb.servo_user_poll(0)
        self.assertEqual(st, 1)

    def test_user_txn_waits_for_config(self):
        # Staged before the config sequence finishes: the wire must
        # see op_mode/torque/goal_acc first, the user write after.
        sb.servo_attach(0, 2, False, 45)
        self.assertTrue(sb.servo_user_write(0, self.REG, 300, 2))
        self.wire.settle(20)
        regs_in_order = []
        log, i = self.wire.tx_log, 0
        while i + 6 <= len(log):
            if log[i] == 0xFF and log[i + 1] == 0xFF and log[i + 4] == 0x03:
                regs_in_order.append(log[i + 5])
                i += 6
            else:
                i += 1
        self.assertIn(self.REG, regs_in_order)
        self.assertTrue(regs_in_order.index(self.REG)
                        > regs_in_order.index(0x21),
                        regs_in_order)

    def test_poll_on_unattached_slot(self):
        st, _ = sb.servo_user_poll(3)
        self.assertEqual(st, -2)

    def test_poll_while_pending_reports_pending(self):
        self._configured()
        self.assertTrue(sb.servo_user_write(0, self.REG, 300, 2))
        st, _ = sb.servo_user_poll(0)     # staged, not yet on the wire
        self.assertEqual(st, 0)

    def test_lost_read_retries_until_it_lands(self):
        self._configured()
        self.wire.regs[self.REG] = 640
        self.wire.mute = 2
        self.assertTrue(sb.servo_user_read(0, self.REG, 2))
        self.wire.settle(40)
        st, val = sb.servo_user_poll(0)
        self.assertEqual(st, 1)
        self.assertEqual(val, 640)

    def test_dead_servo_latches_read_failure(self):
        self._configured()
        self.wire.mute = 10 ** 6
        self.assertTrue(sb.servo_user_read(0, self.REG, 2))
        self.wire.settle(120)
        st, _ = sb.servo_user_poll(0)
        self.assertEqual(st, -1)

    def test_detach_mid_flight_drops_the_result(self):
        # Detach while the user WRITE's status is still on the wire:
        # the in-flight routing is cleared, and the late reply must
        # not be attributed to whatever reuses the slot.
        self._configured()
        self.assertTrue(sb.servo_user_write(0, self.REG, 300, 2))
        sb.servo_pump()                    # txn goes on the wire
        tx = sb.take_tx()
        self.assertTrue(len(tx) > 0)
        sb.servo_detach(0)
        sb.feed_rx(_reply(2, 0))           # the late status arrives
        self.wire.settle(5)                # consumed, routed nowhere
        st, _ = sb.servo_user_poll(0)
        self.assertEqual(st, -2)           # slot empty, nothing staged

    def test_detach_mid_flight_read_drops_too(self):
        self._configured()
        self.wire.regs[self.REG] = 500
        self.assertTrue(sb.servo_user_read(0, self.REG, 2))
        sb.servo_pump()
        self.assertTrue(len(sb.take_tx()) > 0)
        sb.servo_detach(0)
        sb.feed_rx(_reply(2, 0, bytes([0xF4, 0x01])))
        self.wire.settle(5)
        # Reattaching works and the slot is clean.
        self.assertTrue(sb.servo_attach(0, 2, False, 45))
        self.wire.settle()
        st, _ = sb.servo_user_poll(0)
        self.assertEqual(st, -2)

    def test_user_txn_outranks_speed_syncs(self):
        self._configured()
        sb.servo_run(0, 1000)                # torque + speed staged
        self.assertTrue(sb.servo_user_write(0, self.REG, 300, 2))
        self.wire.settle(10)
        log = self.wire.tx_log
        user_at = log.find(bytes([0x03, self.REG]))
        speed_at = log.find(bytes([0x83, 0x2E]))
        self.assertTrue(user_at >= 0, "user write never shipped")
        self.assertTrue(speed_at < 0 or user_at < speed_at,
                        (user_at, speed_at))


if __name__ == "__main__":
    unittest.main()
