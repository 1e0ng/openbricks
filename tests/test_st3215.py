# SPDX-License-Identifier: MIT
"""Tests for the ST-3215 serial bus servo driver."""

import tests._fakes  # noqa: F401

import time
import unittest

from openbricks.drivers import st3215 as st3215_mod
from openbricks.drivers.st3215 import ST3215, ST3215Motor, SyncServoGroup


_REG_MIN_ANGLE     = 0x09
_REG_MAX_ANGLE     = 0x0B
_REG_OP_MODE       = 0x21
_REG_TORQUE        = 0x28
_REG_GOAL_POSITION = 0x2A
_REG_GOAL_ACC      = 0x29
_REG_PRESENT_SPEED = 0x3A
_REG_PRESENT_LOAD  = 0x3C
_REG_GOAL_SPEED    = 0x2E
_REG_PRESENT_POS   = 0x38
_REG_TORQUE_LIMIT  = 0x30
_HEADER            = b"\xFF\xFF"

_MODE_WHEEL = 1
_MODE_STEP  = 3


def _checksum(body):
    s = 0
    for b in body:
        s += b
    return (~s) & 0xFF


def _decode_signed_step(data):
    """Decode a step-mode goal-position write (sign-magnitude, bit 15
    = direction) back into a signed encoder-count step."""
    v = data[0] | (data[1] << 8)
    mag = v & 0x7FFF
    return -mag if (v & 0x8000) else mag


class TestST3215(unittest.TestCase):
    def setUp(self):
        # The bus registry is class-level state; reset it so each test sees
        # a fresh UART.
        ST3215._buses = {}

    def test_constructor_coasts_torque_off(self):
        # Pybricks-consistent: constructing the servo cuts torque, so
        # an arm left holding by a previous program goes limp until
        # the first move_to.
        servo = ST3215(servo_id=1)
        tx = servo._bus._uart._tx_log
        self.assertEqual(len(tx), 1)
        torque_body = bytes([1, 4, 0x03, _REG_TORQUE, 0])
        self.assertEqual(
            tx[0], _HEADER + torque_body + bytes([_checksum(torque_body)]))

    def test_move_to_writes_goal_position_packet(self):
        servo = ST3215(servo_id=1, uart_id=1, tx=14, rx=6)
        baseline = len(servo._bus._uart._tx_log)
        servo.move_to(180, wait=False)
        tx = servo._bus._uart._tx_log[baseline:]

        # A fresh servo coasts; move_to first re-enables torque, then
        # writes the goal position.
        torque_body = bytes([1, 4, 0x03, _REG_TORQUE, 1])
        self.assertEqual(
            tx[0], _HEADER + torque_body + bytes([_checksum(torque_body)]))

        # 180 deg on a 0..4095 raw range maps to int(4095 * 180 / 360) = 2047.
        raw = 2047
        data = bytes([raw & 0xFF, (raw >> 8) & 0xFF])
        params = bytes([_REG_GOAL_POSITION]) + data
        length = len(data) + 3
        body = bytes([1, length, 0x03]) + params
        expected = _HEADER + body + bytes([_checksum(body)])
        self.assertEqual(tx[1], expected)

    def test_second_move_to_skips_redundant_torque_write(self):
        servo = ST3215(servo_id=1)
        servo.move_to(90, wait=False)
        baseline = len(servo._bus._uart._tx_log)
        servo.move_to(180, wait=False)
        # Torque is cached on — the second move is exactly one packet.
        self.assertEqual(len(servo._bus._uart._tx_log) - baseline, 1)

    def test_move_to_with_speed_writes_two_packets(self):
        servo = ST3215(servo_id=2)
        baseline = len(servo._bus._uart._tx_log)
        servo.move_to(90, speed=500, wait=False)

        # After the one-time torque enable: goal speed, then goal
        # position.
        tx = servo._bus._uart._tx_log[baseline + 1:]
        self.assertEqual(len(tx), 2)

        speed = 500
        speed_body = bytes([2, 5, 0x03, _REG_GOAL_SPEED,
                            speed & 0xFF, (speed >> 8) & 0xFF])
        self.assertEqual(tx[0], _HEADER + speed_body + bytes([_checksum(speed_body)]))

    def test_checksum_is_ones_complement_of_body_sum(self):
        servo = ST3215(servo_id=1)
        servo.move_to(0, wait=False)
        packet = servo._bus._uart._tx_log[-1]
        # packet = 0xFF 0xFF <body> <chk>
        body = packet[2:-1]
        chk = packet[-1]
        self.assertEqual(chk, _checksum(body))

    def test_ping_emits_ping_instruction(self):
        servo = ST3215(servo_id=7)
        baseline = len(servo._bus._uart._tx_log)
        servo.ping()
        packet = servo._bus._uart._tx_log[baseline]
        body = packet[2:-1]
        # Instruction byte is at offset 2 within body: [id, length, instr].
        self.assertEqual(body[0], 7)      # servo id
        self.assertEqual(body[2], 0x01)   # PING

    def test_angle_returns_none_when_bus_is_silent(self):
        # The fake UART returns no RX data, so the read times out and angle()
        # reports None rather than a bogus zero.
        servo = ST3215(servo_id=1)
        self.assertIsNone(servo.angle())

    def test_tx_drains_stale_rx_bytes_first(self):
        # Bug: half-duplex bus echo or a timed-out previous reply can
        # leave bytes in the RX FIFO. Without draining before TX, the
        # next read() pulls those stale bytes — symptom on real
        # hardware: ping returns True (any 6 bytes pass the length
        # check) but read() returns None (stale bytes don't form a
        # valid reply). _tx must drain first.
        servo = ST3215(servo_id=1)
        # Pre-stuff the RX buffer with stale junk + the actual valid
        # reply we expect to see for a present-position read.
        # Real reply layout for a 2-byte read: FF FF ID LEN ERR D0 D1 CHK.
        valid_reply = b"\xFF\xFF\x01\x04\x00\x39\x05\xBC"
        servo._bus._uart._rx_buf = b"\xAA\x55\xDE\xAD" + valid_reply
        # The read should drain the junk in _tx, send the read packet,
        # then read the valid reply that we placed AFTER the junk.
        # Since the fake UART has only one buffer, draining empties
        # everything — so we restage the valid reply post-drain via
        # a write-side hook on the fake.
        original_write = servo._bus._uart.write
        def write_then_stage(data):
            servo._bus._uart._rx_buf = valid_reply
            return original_write(data)
        servo._bus._uart.write = write_then_stage
        # Read should now succeed against the valid reply (D0=0x39 D1=0x05).
        result = servo._bus.read(servo._id, _REG_PRESENT_POS, 2)
        self.assertEqual(result, b"\x39\x05")

    def _stage_reply(self, servo, reply):
        """Stage ``reply`` to arrive after the next TX (the fake UART
        drain-before-TX empties the buffer, so pre-stuffing alone is
        not enough — same trick as the stale-RX test)."""
        uart = servo._bus._uart
        if not hasattr(uart, "_pristine_write"):
            uart._pristine_write = uart.write

        def write_then_stage(data):
            uart._rx_buf = reply
            return uart._pristine_write(data)
        uart.write = write_then_stage

    def test_read_rejects_a_wrong_sender(self):
        # Reads feed the angle accumulator's wrap heuristic and the
        # step-park detector; a reply from the WRONG servo (the
        # two-conversations-on-one-wire signature) must read as "no
        # reply", not as data.
        servo = ST3215(servo_id=1)
        body = bytes([2, 4, 0, 0x39, 0x05])         # id 2, not 1
        self._stage_reply(servo,
                          _HEADER + body + bytes([_checksum(body)]))
        self.assertIsNone(servo._bus.read(1, _REG_PRESENT_POS, 2))

    def test_read_rejects_a_corrupt_checksum(self):
        servo = ST3215(servo_id=1)
        body = bytes([1, 4, 0, 0x39, 0x05])
        good = _HEADER + body + bytes([_checksum(body)])
        self._stage_reply(servo, good[:-1] + bytes([good[-1] ^ 0xFF]))
        self.assertIsNone(servo._bus.read(1, _REG_PRESENT_POS, 2))

    def test_ping_requires_the_right_sender_and_frame(self):
        # "Any 6 bytes" also matched stale residue and other servos'
        # replies — reporting a present servo that wasn't.
        servo = ST3215(servo_id=1)
        body = bytes([1, 2, 0])
        self._stage_reply(servo,
                          _HEADER + body + bytes([_checksum(body)]))
        self.assertTrue(servo.ping())
        wrong = bytes([3, 2, 0])
        self._stage_reply(servo,
                          _HEADER + wrong + bytes([_checksum(wrong)]))
        self.assertFalse(servo.ping())

    def test_buses_are_shared_per_uart(self):
        # Two servos on the same UART params share one _SCServoBus instance.
        s1 = ST3215(servo_id=1, uart_id=2, tx=14, rx=6)
        s2 = ST3215(servo_id=2, uart_id=2, tx=14, rx=6)
        self.assertIs(s1._bus, s2._bus)

        # A different UART id gets a separate bus.
        s3 = ST3215(servo_id=3, uart_id=1, tx=14, rx=6)
        self.assertIsNot(s1._bus, s3._bus)


class TestWriteAcknowledgement(unittest.TestCase):
    """A write that never landed must never look like one that did.

    The driver used to send a register write and discard the servo's
    status packet. On this hardware that is not a cosmetic gap:
    goal-speed 0 means MAXIMUM SPEED, so a lost speed write is a
    runaway rather than a small error (bench 2026-08-04, 697 dps
    against a commanded 200).
    """

    def setUp(self):
        ST3215._buses = {}

    def _motor(self):
        return ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)

    def test_silent_servo_raises_naming_the_register(self):
        m = self._motor()
        m._bus._uart._ack_scs_write = lambda pkt: None   # servo goes mute
        try:
            m.run_speed(100)
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("no acknowledgement" in msg, msg)
        self.assertTrue("servo id 1" in msg, msg)
        # run_speed enables torque before it writes a speed, so the
        # FIRST unacknowledged write is the one reported — failing at
        # the first lost packet beats reporting the last one.
        self.assertTrue("0x28" in msg, msg)
        self.assertTrue("verify_writes=False" in msg, msg)

    def test_error_flags_in_the_ack_are_raised(self):
        m = self._motor()
        uart = m._bus._uart

        def erroring_ack(packet):
            if len(packet) < 6 or packet[4] != 0x03 or packet[2] == 0xFE:
                return
            body = bytes([packet[2], 2, 0x20])           # error flags set
            chk = 0
            for b in body:
                chk += b
            uart._rx_buf += b"\xff\xff" + body + bytes([(~chk) & 0xFF])

        uart._ack_scs_write = erroring_ack
        try:
            m.run_speed(100)
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("error flags 0x20" in str(e), str(e))

    def test_wrong_responder_id_is_raised(self):
        # Two servos sharing an id, or replies arriving out of step.
        m = self._motor()
        uart = m._bus._uart

        def impostor_ack(packet):
            if len(packet) < 6 or packet[4] != 0x03 or packet[2] == 0xFE:
                return
            body = bytes([9, 2, 0])                      # id 9 answers
            chk = 0
            for b in body:
                chk += b
            uart._rx_buf += b"\xff\xff" + body + bytes([(~chk) & 0xFF])

        uart._ack_scs_write = impostor_ack
        try:
            m.run_speed(100)
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("servo id 9" in str(e), str(e))

    def test_corrupt_ack_checksum_is_raised(self):
        m = self._motor()
        uart = m._bus._uart

        def corrupt_ack(packet):
            if len(packet) < 6 or packet[4] != 0x03 or packet[2] == 0xFE:
                return
            uart._rx_buf += b"\xff\xff" + bytes([packet[2], 2, 0, 0x00])

        uart._ack_scs_write = corrupt_ack
        try:
            m.run_speed(100)
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("checksum" in str(e), str(e))

    def test_broadcast_writes_expect_no_reply(self):
        # The protocol defines no answer to id 0xFE, so verification
        # must skip it — the e-stop broadcast must never raise.
        m = self._motor()
        m._bus._uart._ack_scs_write = lambda pkt: None
        m._bus.write(0xFE, 0x28, bytes([0]))             # must not raise

    def test_verify_writes_false_is_an_explicit_opt_out(self):
        m = self._motor()
        m._bus.verify_writes = False
        m._bus._uart._ack_scs_write = lambda pkt: None
        m.run_speed(100)                                  # must not raise


def _decode_write(packet):
    """Pull (servo_id, register, data_bytes) out of an SCServo write packet."""
    assert packet.startswith(_HEADER)
    body = packet[2:-1]
    sid, length, instr = body[0], body[1], body[2]
    assert instr == 0x03   # WRITE
    register = body[3]
    data     = bytes(body[4:])
    return sid, register, data


def _writes_to(packets, register):
    """Filter a UART tx log down to writes targeting one register."""
    out = []
    for pkt in packets:
        sid, reg, data = _decode_write(pkt)
        if reg == register:
            out.append((sid, data))
    return out


class TestST3215Motor(unittest.TestCase):
    def setUp(self):
        ST3215._buses = {}

    def test_constructor_switches_servo_into_wheel_mode(self):
        m = ST3215Motor(servo_id=1)
        mode_writes = _writes_to(m._bus._uart._tx_log, _REG_OP_MODE)
        self.assertEqual(mode_writes, [(1, bytes([1]))])   # 1 = wheel

    def test_constructor_coasts_torque_off(self):
        # Pybricks-consistent: a constructed motor coasts until its
        # first motion command. The explicit 0 also releases a hold
        # left behind by a previous program on the same power session.
        m = ST3215Motor(servo_id=2)
        torque_writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE)
        self.assertEqual(torque_writes, [(2, bytes([0]))])

    def test_first_run_speed_enables_torque(self):
        m = ST3215Motor(servo_id=2)
        m.run_speed(100)
        torque_writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE)
        self.assertEqual(torque_writes, [(2, bytes([0])), (2, bytes([1]))])
        # Second command: torque cached on, no third write.
        m.run_speed(200)
        torque_writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE)
        self.assertEqual(len(torque_writes), 2)

    def test_run_speed_writes_signed_magnitude_to_goal_speed(self):
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        m.run_speed(50)   # → magnitude = 500, sign bit clear
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        self.assertEqual(speed_writes[-1], (3, bytes([500 & 0xFF, 500 >> 8])))

    def test_run_speed_negative_sets_high_bit(self):
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        m.run_speed(-50)
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        # magnitude 500, plus the direction bit at bit 15 of the 16-bit value
        v = 500 | 0x8000
        self.assertEqual(speed_writes[-1], (3, bytes([v & 0xFF, (v >> 8) & 0xFF])))

    def test_run_speed_clamps_to_max_dps(self):
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=100.0)
        m.run_speed(99999)
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        # Clamped to 100 dps × 10 steps/dps = 1000
        self.assertEqual(speed_writes[-1], (3, bytes([1000 & 0xFF, 1000 >> 8])))

    def test_invert_flips_run_speed_direction(self):
        # Both servos default to uart_id=1 → they share a _SCServoBus,
        # so the tx_log holds packets from both. Filter by servo_id to
        # isolate each motor's last command.
        m_plain = ST3215Motor(servo_id=4, steps_per_dps=10.0, max_dps=1000.0)
        m_inv   = ST3215Motor(servo_id=5, steps_per_dps=10.0, max_dps=1000.0,
                              invert=True)
        m_plain.run_speed(50)
        m_inv.run_speed(50)
        all_speed_writes = _writes_to(m_plain._bus._uart._tx_log,
                                      _REG_GOAL_SPEED)
        plain = next(d for sid, d in reversed(all_speed_writes) if sid == 4)
        inv   = next(d for sid, d in reversed(all_speed_writes) if sid == 5)
        # plain should have sign bit clear; inv should have it set
        self.assertEqual(plain[1] & 0x80, 0)
        self.assertEqual(inv[1] & 0x80, 0x80)

    def test_brake_writes_zero_speed(self):
        m = ST3215Motor(servo_id=6)
        m.brake()
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        self.assertEqual(speed_writes[-1], (6, bytes([0, 0])))

    def test_coast_disables_torque(self):
        m = ST3215Motor(servo_id=7)
        m.run_speed(100)   # enables torque
        m.coast()
        torque_writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE)
        self.assertEqual(torque_writes[-1], (7, bytes([0])))

    def test_angle_accumulates_across_positive_wrap(self):
        # Synthesise the bus reads on the 12-bit (0..4095) absolute-
        # position register. 3800 → 100 means the wheel moved +396
        # counts the short way (across the 0/4095 boundary), not
        # -3700 the long way. The wrap heuristic must pick the
        # short-arc interpretation.
        m = ST3215Motor(servo_id=8)

        # MicroPython doesn't allow attribute access on closures, so the
        # queue lives in an enclosing list (mutated via .pop()).
        queue = [3800, 100]

        def fake_read(servo_id, register, nbytes):
            assert servo_id == 8 and register == _REG_PRESENT_POS and nbytes == 2
            v = queue.pop(0) & 0xFFFF
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        m._bus.read = fake_read

        first  = m.angle()
        second = m.angle()
        # First read = absolute baseline at 3800 counts.
        self.assertAlmostEqual(first, 3800 * 360.0 / 4096, places=2)
        # Wrap correction: delta = 100 - 3800 = -3700, < -2048, so
        # delta += 4096 → +396. accum = 3800 + 396 = 4196.
        expected_total = (3800 + 396) * 360.0 / 4096
        self.assertAlmostEqual(second, expected_total, places=2)

    def test_reset_angle_zeroes_the_reading(self):
        m = ST3215Motor(servo_id=9)

        # 12-bit absolute position; pin all reads to the same raw value
        # so reset_angle observes a stable baseline.
        queue = [2000, 2000, 2000]

        def fake_read(servo_id, register, nbytes):
            v = queue[0] if len(queue) == 1 else queue.pop(0)
            v &= 0xFFFF
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        m._bus.read = fake_read

        before = m.angle()
        self.assertGreater(before, 0)
        m.reset_angle(0)
        after = m.angle()
        self.assertAlmostEqual(after, 0.0, places=2)


class TestGoalAccRamp(unittest.TestCase):
    """The hardware acceleration ramp (goal-acc register 0x29, unit
    100 encoder steps/s²): the SERVO slews speed changes, so direct
    ``run_speed()`` — the line follower, user code — honours the same
    1500 deg/s² default as the DriveBase profile. Bench origin: the
    default-acceleration retune didn't affect ``run_speed`` because
    the serial drivers had no acceleration home at all."""

    def setUp(self):
        ST3215._buses = {}

    def _acc_writes(self, log):
        return _writes_to(log, _REG_GOAL_ACC)

    def test_constructor_writes_default_acc(self):
        # 1500 deg/s² × 11.378 steps/deg = 17067 steps/s² → 171
        # register units of 100 steps/s².
        m = ST3215Motor(servo_id=1)
        accs = self._acc_writes(m._bus._uart._tx_log)
        self.assertEqual(len(accs), 1)
        self.assertEqual(accs[0], (1, bytes([171])))

    def test_custom_accel_encodes_in_register_units(self):
        m = ST3215Motor(servo_id=2, steps_per_dps=10.0, accel_dps2=500.0)
        accs = self._acc_writes(m._bus._uart._tx_log)
        self.assertEqual(accs[-1], (2, bytes([50])))   # 500×10/100

    def test_zero_accel_disables_the_ramp(self):
        # Register 0 = unlimited (the servo's power-on default).
        m = ST3215Motor(servo_id=3, accel_dps2=0.0)
        accs = self._acc_writes(m._bus._uart._tx_log)
        self.assertEqual(accs[-1], (3, bytes([0])))

    def test_huge_accel_clamps_to_one_byte(self):
        m = ST3215Motor(servo_id=4, accel_dps2=100000.0)
        accs = self._acc_writes(m._bus._uart._tx_log)
        self.assertEqual(accs[-1], (4, bytes([254])))

    def test_brake_is_ramped_no_acc_bypass(self):
        # Uniform rule (1.19.1, reverting 1.18.1): brake() is a speed
        # transition like any other — the goal-acc ramp stays in
        # force, no acc-register bracket around the zero-speed write.
        m = ST3215Motor(servo_id=6)
        m.run_speed(120)
        base = len(m._bus._uart._tx_log)
        m.brake()
        log = m._bus._uart._tx_log[base:]
        self.assertEqual(_writes_to(log, _REG_GOAL_ACC), [])
        self.assertEqual(_writes_to(log, _REG_GOAL_SPEED),
                         [(6, bytes([0, 0]))])

    def test_stop_coasts_via_torque_cut(self):
        # Pybricks Motor.stop() = spin freely: inherited from the
        # Motor base, lands on coast() -> torque register 0.
        m = ST3215Motor(servo_id=9)
        m.run_speed(120)
        base = len(m._bus._uart._tx_log)
        m.stop()
        log = m._bus._uart._tx_log[base:]
        self.assertEqual(_writes_to(log, _REG_TORQUE), [(9, bytes([0]))])
        self.assertEqual(_writes_to(log, _REG_GOAL_SPEED), [])

    def test_estop_coast_path_is_a_torque_cut_not_a_speed_write(self):
        # The safety stop must stay instant: coast() (the e-stop's
        # per-motor action) cuts the torque register and never
        # touches goal speed or goal acc — the ramp cannot slow it.
        m = ST3215Motor(servo_id=7)
        m.run_speed(120)
        base = len(m._bus._uart._tx_log)
        m.coast()
        log = m._bus._uart._tx_log[base:]
        self.assertEqual(_writes_to(log, _REG_TORQUE), [(7, bytes([0]))])
        self.assertEqual(_writes_to(log, _REG_GOAL_ACC), [])
        self.assertEqual(_writes_to(log, _REG_GOAL_SPEED), [])

    def test_run_speed_packets_unchanged_by_the_ramp(self):
        # The ramp lives in the servo: goal-speed writes are byte-for-
        # byte what they were (no host-side slewing crept in).
        a = ST3215Motor(servo_id=5, steps_per_dps=10.0, max_dps=1000.0)
        b = ST3215Motor(servo_id=5, steps_per_dps=10.0, max_dps=1000.0,
                        accel_dps2=0.0)
        base_a = len(a._bus._uart._tx_log)
        a.run_speed(300)
        pk_a = _writes_to(a._bus._uart._tx_log[base_a:], _REG_GOAL_SPEED)
        ST3215._buses = {}
        b2 = ST3215Motor(servo_id=5, steps_per_dps=10.0, max_dps=1000.0,
                         accel_dps2=0.0)
        base_b = len(b2._bus._uart._tx_log)
        b2.run_speed(300)
        pk_b = _writes_to(b2._bus._uart._tx_log[base_b:], _REG_GOAL_SPEED)
        self.assertEqual(pk_a, pk_b)


class TestST3215MotorRunAngle(unittest.TestCase):
    """``run_angle`` drives the servo in STEP mode (op_mode=3) where the
    goal-position register is a SIGNED RELATIVE step, so moves past one
    full turn work without the single-turn 0/4095 boundary that capped
    the old position-mode (op_mode=0) implementation.

    Bench finding (examples/st3032_stepmode_probe.py): in step mode the
    present-position register reads *remaining counts to target*,
    counting down to ~0 as the move completes (sign-magnitude). So
    done-detection reads that register (``_read_step_remaining``), and
    the shaft-angle accumulator is advanced by the counts travelled.
    These tests drive the state machine by patching that register.
    """

    def setUp(self):
        ST3215._buses = {}
        # Give the fake servo a REGISTER FILE: a read returns whatever
        # was last written to that register. run_angle verifies its
        # goal-speed / goal-acc writes by reading them back (a lost
        # speed write means FULL SPEED on real hardware, bench
        # 2026-08-04), so the fake has to model storage, not just
        # swallow writes.
        from openbricks.drivers.st3215 import _SCServoBus
        self._bus_cls = _SCServoBus
        self._real_read = _SCServoBus.read

        def register_file_read(bus, servo_id, register, nbytes):
            for packet in reversed(bus._uart._tx_log):
                if (len(packet) >= 6 + nbytes and packet[2] == servo_id
                        and packet[4] == 0x03 and packet[5] == register):
                    return bytes(packet[6:6 + nbytes])
            return None

        _SCServoBus.read = register_file_read
        self._register_file_read = register_file_read

    def tearDown(self):
        self._bus_cls.read = self._real_read

    def _patch_remaining(self, motor, sequence):
        """Make successive ``motor._read_step_remaining()`` calls return
        the signed remaining-to-target counts in ``sequence`` (last
        value sticks). A converging move is e.g. [N, N, 0]: the run_angle
        bus-alive probe consumes one, then the await/poll loop sees the
        move 'started' (|rem|>tol) and then 'parked' (|rem|<=tol)."""
        values = list(sequence)

        def fake():
            return values.pop(0) if len(values) > 1 else values[0]
        motor._read_step_remaining = fake

    # --- step-mode setup ---------------------------------------------------

    def test_run_angle_enters_step_mode_and_zeroes_angle_limits(self):
        # First run_angle must unlock multi-turn: write min/max angle
        # limit = 0 and switch op_mode to step (3).
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)

        log = m._bus._uart._tx_log[baseline:]
        self.assertEqual(_writes_to(log, _REG_MIN_ANGLE), [(1, bytes([0, 0]))])
        self.assertEqual(_writes_to(log, _REG_MAX_ANGLE), [(1, bytes([0, 0]))])
        mode_writes = _writes_to(log, _REG_OP_MODE)
        self.assertEqual(mode_writes[0][1], bytes([_MODE_STEP]))

    def test_run_angle_zeroes_limits_only_once_across_calls(self):
        # The angle-limit registers are EEPROM; only the first run_angle
        # should write them. A second call must not re-touch them.
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        m.run_angle(deg_per_s=200, target_angle=90)
        baseline = len(m._bus._uart._tx_log)
        self._patch_remaining(m, [1024, 1024, 0])
        m.run_angle(deg_per_s=200, target_angle=90)
        log = m._bus._uart._tx_log[baseline:]
        self.assertEqual(_writes_to(log, _REG_MIN_ANGLE), [])
        self.assertEqual(_writes_to(log, _REG_MAX_ANGLE), [])

    def test_already_zeroed_limits_cost_no_eeprom_write(self):
        # THE FIRST-MOVE BUG (bench 2026-08-04). The guard flag is per
        # INSTANCE, so every program run used to rewrite these EEPROM
        # registers on its first run_angle even though a previous run
        # had already zeroed them — burning an EEPROM cycle per run
        # and, worse, leaving the servo busy right before the
        # goal-speed write. That made the first move of every run
        # behave differently from every later move: measured 697 dps
        # against a commanded 200, with goal-acc reading 0 afterwards
        # despite the constructor writing 171.
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        # Servo already reports limits 0/0 (a servo that has run
        # run_angle before — i.e. every run after the very first).
        m._bus.read = lambda sid, reg, n: (
            b"\x00\x00" if reg in (_REG_MIN_ANGLE, _REG_MAX_ANGLE)
            else self._register_file_read(m._bus, sid, reg, n))
        baseline = len(m._bus._uart._tx_log)
        self._patch_remaining(m, [1024, 1024, 0])
        m.run_angle(deg_per_s=200, target_angle=90)
        log = m._bus._uart._tx_log[baseline:]
        self.assertEqual(_writes_to(log, _REG_MIN_ANGLE), [])
        self.assertEqual(_writes_to(log, _REG_MAX_ANGLE), [])

    def test_run_angle_reasserts_goal_acc_every_move(self):
        # goal-acc read back 0 on the bench despite the constructor
        # writing it, so the move no longer trusts that write to have
        # survived — the documented uniform-acceleration rule has to
        # hold in step mode too.
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0,
                        accel_dps2=1500.0)
        baseline = len(m._bus._uart._tx_log)
        self._patch_remaining(m, [1024, 1024, 0])
        m.run_angle(deg_per_s=200, target_angle=90)
        log = m._bus._uart._tx_log[baseline:]
        self.assertEqual(_writes_to(log, _REG_GOAL_ACC),
                         [(1, bytes([m._encode_goal_acc()]))])

    def test_servo_that_wont_store_goal_acc_still_moves(self):
        # Bench 2026-08-04: an ST-3032 acknowledges the goal-acc write
        # and still reports 0 — the ramp is not settable on that unit.
        # Refusing every move over an acceleration the servo won't
        # store would ground a working robot to enforce a preference,
        # so this warns and proceeds. goal-SPEED keeps the strict
        # treatment (see the next test); the two failures cost
        # different amounts.
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        m._bus.read = lambda sid, reg, n: (
            b"\x00" if reg == _REG_GOAL_ACC
            else self._register_file_read(m._bus, sid, reg, n))
        m.run_angle(deg_per_s=200, target_angle=90)   # must not raise
        self.assertTrue(m._acc_mismatch_warned)

    def test_lost_speed_write_is_refused_not_run_at_full_speed(self):
        # goal_speed 0 means MAXIMUM SPEED on a Feetech servo, and
        # _SCServoBus.write never checked the ACK — so a dropped speed
        # write used to mean the move ran flat out. Verified now, and
        # a servo that won't take the setting is refused loudly.
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        # Servo accepts the packet but the register never changes.
        m._bus.read = lambda sid, reg, n: (
            b"\x00\x00"[:n] if reg == _REG_GOAL_SPEED
            else self._register_file_read(m._bus, sid, reg, n))
        try:
            m.run_angle(deg_per_s=200, target_angle=90)
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("servo id 1" in msg, msg)
        self.assertTrue("FULL SPEED" in msg, msg)

    def test_drivebase_wheel_motor_keeps_stock_limits(self):
        # A motor used purely for wheel/velocity work (never run_angle)
        # must NOT have its angle limits zeroed — that would make its
        # present-position reads multi-turn and break DriveBase odometry.
        m = ST3215Motor(servo_id=1)
        m.run_speed(100)
        m.brake()
        m.coast()
        self.assertEqual(_writes_to(m._bus._uart._tx_log, _REG_MIN_ANGLE), [])
        self.assertEqual(_writes_to(m._bus._uart._tx_log, _REG_MAX_ANGLE), [])

    # --- relative step encoding -------------------------------------------

    def test_run_angle_writes_one_signed_relative_step(self):
        m = ST3215Motor(servo_id=2, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)   # +90° = +1024 counts
        steps = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        self.assertEqual(len(steps), 1)
        self.assertEqual(_decode_signed_step(steps[0][1]), 1024)

    def test_run_angle_negative_target_sets_direction_bit(self):
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [-1024, -1024, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=-90)
        steps = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        self.assertEqual(_decode_signed_step(steps[0][1]), -1024)

    def test_run_angle_past_one_full_turn_is_a_single_step(self):
        # The headline fix: 540° (1.5 turns) is ONE relative step of
        # 6144 counts — no chunking, no boundary. The old single-turn
        # implementation could not express this at all.
        m = ST3215Motor(servo_id=4, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [6144, 6144, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=540)
        steps = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        self.assertEqual(len(steps), 1)
        self.assertEqual(_decode_signed_step(steps[0][1]), 6144)

    def test_run_angle_invert_flips_step_direction(self):
        m = ST3215Motor(servo_id=5, steps_per_dps=10.0, max_dps=1000.0,
                        invert=True)
        # +90° asked; invert means command -1024 counts to the hardware.
        self._patch_remaining(m, [-1024, -1024, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)
        steps = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        self.assertEqual(_decode_signed_step(steps[0][1]), -1024)

    def test_run_angle_huge_move_splits_into_seven_turn_steps(self):
        # > 7 turns can't fit one step. 8 turns = 2880° = 32768 counts →
        # first step 28672 (7 turns), then 4096 (the remaining turn),
        # each issued as the await/poll loop sees the previous one park.
        m = ST3215Motor(servo_id=6, steps_per_dps=10.0, max_dps=1000.0)
        # remaining seq: probe, step1 started, step1 parked, step2 started,
        # step2 parked.
        self._patch_remaining(m, [28672, 28672, 0, 4096, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=2880)
        steps = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        self.assertEqual([_decode_signed_step(s[1]) for s in steps],
                         [28672, 4096])

    def test_run_angle_writes_goal_speed_clamped_to_register_range(self):
        m = ST3215Motor(servo_id=7, steps_per_dps=10.0, max_dps=10000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        baseline = len(m._bus._uart._tx_log)
        # 5000 dps × 10 = 50000 steps — exceeds the 0x7FFF register cap.
        m.run_angle(deg_per_s=5000, target_angle=90, then="brake")
        speed_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                  _REG_GOAL_SPEED)
        v_cruise = speed_writes[0][1][0] | (speed_writes[0][1][1] << 8)
        self.assertEqual(v_cruise, 0x7FFF)
        # then="brake" also writes goal_speed=0 at the end.
        self.assertEqual(speed_writes[-1][1], bytes([0, 0]))

    # --- angle accumulation across a step move ----------------------------

    def test_angle_tracks_executed_step_counts(self):
        # The headline measurement fix: after a step move, angle() must
        # reflect the counts travelled (step register can't be read as a
        # position in step mode, so the accumulator is advanced by the
        # executed counts). +90° from a zeroed baseline reads back +90°.
        m = ST3215Motor(servo_id=8, steps_per_dps=10.0, max_dps=1000.0)
        # Establish a wheel-mode baseline and zero it.
        m._read_present_pos = lambda: 500
        m.reset_angle(0)
        self.assertAlmostEqual(m.angle(), 0.0, places=1)
        # Now a +90° step (1024 counts) that parks exactly (remaining→0).
        self._patch_remaining(m, [1024, 1024, 0])
        m.run_angle(deg_per_s=200, target_angle=90, then="hold")
        self.assertAlmostEqual(m.angle(), 90.0, places=1)

    def test_angle_tracks_negative_step_with_invert(self):
        m = ST3215Motor(servo_id=9, steps_per_dps=10.0, max_dps=1000.0,
                        invert=True)
        m._read_present_pos = lambda: 500
        m.reset_angle(0)
        # User asks -90°; with invert the hardware step is +1024, but
        # angle() must still report the user-frame -90°.
        self._patch_remaining(m, [1024, 1024, 0])
        m.run_angle(deg_per_s=200, target_angle=-90, then="hold")
        self.assertAlmostEqual(m.angle(), -90.0, places=1)

    # --- end-state (then=) ------------------------------------------------

    def test_run_angle_default_coasts_and_stays_in_step_mode(self):
        # then="coast" cuts torque and leaves op_mode in step (no flip
        # back to wheel — the next run_speed/brake restores it lazily).
        m = ST3215Motor(servo_id=10, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)   # default coast
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertEqual(len(mode_writes), 1)
        self.assertEqual(mode_writes[0][1], bytes([_MODE_STEP]))
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes[-1][1], bytes([0]))   # torque cut

    def test_run_angle_then_brake_restores_wheel_mode_and_zero_speed(self):
        m = ST3215Motor(servo_id=11, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90, then="brake")
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertEqual(mode_writes[0][1], bytes([_MODE_STEP]))    # into step
        self.assertEqual(mode_writes[-1][1], bytes([_MODE_WHEEL]))  # back to wheel
        speed_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_SPEED)
        self.assertEqual(speed_writes[-1][1], bytes([0, 0]))

    def test_run_angle_then_hold_stays_in_step_mode_torque_on(self):
        m = ST3215Motor(servo_id=12, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        m.brake()   # enable torque; motors now coast at construction
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90, then="hold")
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertEqual(len(mode_writes), 1)
        self.assertEqual(mode_writes[0][1], bytes([_MODE_STEP]))
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes, [])   # torque not cut

    def test_run_angle_then_invalid_raises_value_error(self):
        m = ST3215Motor(servo_id=13, steps_per_dps=10.0, max_dps=1000.0)
        with self.assertRaises(ValueError):
            m.run_angle(deg_per_s=200, target_angle=90, then="freewheel")

    def test_run_angle_zero_target_is_a_noop(self):
        m = ST3215Motor(servo_id=14)
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=0)
        self.assertEqual(len(m._bus._uart._tx_log), baseline)

    def test_run_angle_silent_bus_raises_and_writes_no_step(self):
        # If the bus-alive probe read fails (servo silent) run_angle
        # bails before commanding a move it can't track — no step
        # write, no end-state dispatch (torque stays as it was) — and
        # it RAISES naming the servo. The old quiet return made an
        # unplugged servo look exactly like a completed move.
        m = ST3215Motor(servo_id=15, steps_per_dps=10.0, max_dps=1000.0)
        m._read_step_remaining = lambda: None
        baseline = len(m._bus._uart._tx_log)
        try:
            m.run_angle(deg_per_s=200, target_angle=90)
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("servo id 15" in msg, msg)
        self.assertTrue("wiring" in msg, msg)
        steps = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        self.assertEqual(steps, [])
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes, [])

    def test_run_angle_step_that_never_parks_reports_not_succeeds(self):
        # The step register stops counting down and the budget
        # expires. This used to fall through and return True — a
        # stalled motor reporting a fully successful move. Now it
        # reports loudly and returns False (raise_on_stall opts into
        # fatal, tested below).
        m = ST3215Motor(servo_id=16, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 512, 512])   # frozen partway
        seen = []
        orig = ST3215Motor._report_stall
        ST3215Motor._report_stall = staticmethod(seen.append)
        try:
            result = m.run_angle(deg_per_s=1000, target_angle=90)
        finally:
            ST3215Motor._report_stall = orig
        self.assertFalse(result, "a stall must return False, not True")
        self.assertTrue(seen and "gave up" in seen[0], seen)
        self.assertTrue("servo id 16" in seen[0], seen[0])

    def test_run_angle_stall_raises_when_asked(self):
        m = ST3215Motor(servo_id=17, steps_per_dps=10.0, max_dps=1000.0,
                        raise_on_stall=True)
        self._patch_remaining(m, [1024, 512, 512])
        try:
            m.run_angle(deg_per_s=1000, target_angle=90)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("gave up" in str(e), str(e))

    def test_done_polling_detects_a_frozen_step(self):
        # THE documented wait=False pattern: while not m.done(): ...
        # A jammed motor froze the register and done() returned False
        # forever. The poller now runs the same idle rule as the
        # blocking path.
        m = ST3215Motor(servo_id=18, steps_per_dps=10.0, max_dps=1000.0)
        m._stall_idle_ms = 30
        self._patch_remaining(m, [1024, 512, 512])   # started, frozen
        m.run_angle(deg_per_s=1000, target_angle=90, wait=False)
        seen = []
        orig = ST3215Motor._report_stall
        ST3215Motor._report_stall = staticmethod(seen.append)
        try:
            for _ in range(200):
                if m.done():
                    break
                time.sleep_ms(5)
            else:
                self.fail("done() never detected the frozen step")
        finally:
            ST3215Motor._report_stall = orig
        self.assertTrue(seen and "stopped counting down" in seen[0],
                        seen)
        self.assertTrue(m.done())        # pending cleared

    def test_done_polling_raises_on_a_bus_that_goes_silent(self):
        # Persistent silence mid-move is a wiring fault, not a stall:
        # polling it forever hides the loss.
        m = ST3215Motor(servo_id=19, steps_per_dps=10.0, max_dps=1000.0)
        m._stall_idle_ms = 30
        self._patch_remaining(m, [1024, 512])
        m.run_angle(deg_per_s=1000, target_angle=90, wait=False)
        m._read_step_remaining = lambda: None        # bus dies
        try:
            for _ in range(200):
                if m.done():
                    self.fail("silent bus must raise, not complete")
                time.sleep_ms(5)
            self.fail("done() never raised on the silent bus")
        except OSError as e:
            msg = str(e)
        self.assertTrue("SILENT" in msg, msg)
        self.assertTrue("servo id 19" in msg, msg)

    # --- wait=False / done() ----------------------------------------------

    def test_done_returns_true_when_no_move_pending(self):
        m = ST3215Motor(servo_id=20)
        self.assertTrue(m.done())

    def test_run_angle_wait_false_returns_immediately_and_done_tracks(self):
        # wait=False writes the step and returns; done() reads the
        # remaining register and reports False until it counts down.
        m = ST3215Motor(servo_id=21, steps_per_dps=10.0, max_dps=1000.0)
        # remaining: probe(1024), done#1 started(512), done#2 parked(0).
        self._patch_remaining(m, [1024, 512, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90, wait=False)
        steps = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        self.assertEqual(_decode_signed_step(steps[0][1]), 1024)
        self.assertFalse(m.done())   # 512 remaining — still moving
        self.assertTrue(m.done())    # 0 — parked
        self.assertTrue(m.done())    # stays done

    def test_run_angle_wait_false_ignores_stale_zero_at_kickoff(self):
        # If the remaining register still reads ~0 from the previous
        # move when done() is first called, it must NOT report done — the
        # started latch waits for the register to go large first.
        m = ST3215Motor(servo_id=25, steps_per_dps=10.0, max_dps=1000.0)
        # probe(2), done#1 stale 0 (not started!), done#2 started 800,
        # done#3 parked 0.
        self._patch_remaining(m, [2, 0, 800, 0])
        m.run_angle(deg_per_s=200, target_angle=90, wait=False)
        self.assertFalse(m.done())   # stale 0 — latch not armed, NOT done
        self.assertFalse(m.done())   # 800 — moving, now armed
        self.assertTrue(m.done())    # 0 — really parked

    def test_run_angle_wait_false_defers_then_dispatch_until_done(self):
        # then="coast" must cut torque only AFTER the move parks, not at
        # run_angle() return time (else it kills the move).
        m = ST3215Motor(servo_id=22, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        m.brake()   # enable torque; motors now coast at construction
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90, wait=False, then="coast")
        # No torque write yet — coast deferred.
        self.assertEqual(_writes_to(m._bus._uart._tx_log[baseline:],
                                    _REG_TORQUE), [])
        self.assertFalse(m.done())   # started
        self.assertTrue(m.done())    # parked → coast runs now
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes[-1][1], bytes([0]))

    def test_run_speed_supersedes_pending_run_angle_wait_false(self):
        m = ST3215Motor(servo_id=23, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024])   # never reaches 0 → pending
        m.run_angle(deg_per_s=200, target_angle=90, wait=False)
        self.assertFalse(m.done())   # pending kicked off (started)
        m.run_speed(50)              # supersede
        self.assertTrue(m.done())    # pending dropped

    # --- mode restoration after coast / hold ------------------------------

    def test_run_speed_after_coast_re_enables_torque(self):
        m = ST3215Motor(servo_id=30, steps_per_dps=10.0, max_dps=1000.0)
        m.coast()
        baseline = len(m._bus._uart._tx_log)
        m.run_speed(50)
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes[-1][1], bytes([1]))

    def test_brake_after_coast_re_enables_torque(self):
        m = ST3215Motor(servo_id=31)
        m.coast()
        baseline = len(m._bus._uart._tx_log)
        m.brake()
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes[-1][1], bytes([1]))

    def test_run_speed_after_hold_restores_wheel_mode(self):
        # hold() leaves the servo in position (or step) mode; the next
        # run_speed must flip back to wheel mode before writing goal_speed.
        m = ST3215Motor(servo_id=32, steps_per_dps=10.0, max_dps=1000.0)
        m._read_present_pos = lambda: 500
        m.hold()
        baseline = len(m._bus._uart._tx_log)
        m.run_speed(50)
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertEqual(mode_writes[-1][1], bytes([_MODE_WHEEL]))

    def test_hold_on_wheel_motor_uses_single_turn_no_limit_zeroing(self):
        # A motor that never ran run_angle (e.g. a DriveBase wheel) must
        # hold WITHOUT zeroing its angle limits — otherwise its present-
        # position reads would go multi-turn and break wheel odometry.
        m = ST3215Motor(servo_id=33, steps_per_dps=10.0, max_dps=1000.0)
        m._read_present_pos = lambda: 500
        baseline = len(m._bus._uart._tx_log)
        m.hold()
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertEqual(mode_writes[-1][1], bytes([0]))   # single-turn position
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        goal = pos_writes[-1][1][0] | (pos_writes[-1][1][1] << 8)
        self.assertEqual(goal, 500)                        # anchored at present
        self.assertEqual(_writes_to(m._bus._uart._tx_log[baseline:],
                                    _REG_MIN_ANGLE), [])
        self.assertEqual(_writes_to(m._bus._uart._tx_log[baseline:],
                                    _REG_MAX_ANGLE), [])

    def test_hold_after_run_angle_uses_step_mode_zero_step(self):
        # Once run_angle has switched the motor into multi-turn step
        # mode, hold() keeps it there with a zero-count step.
        m = ST3215Motor(servo_id=34, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_remaining(m, [1024, 1024, 0])
        m.run_angle(deg_per_s=200, target_angle=90)   # zeroes limits, step mode
        baseline = len(m._bus._uart._tx_log)
        m.hold()
        steps = _writes_to(m._bus._uart._tx_log[baseline:], _REG_GOAL_POSITION)
        self.assertEqual(_decode_signed_step(steps[-1][1]), 0)
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertFalse(bytes([_MODE_WHEEL]) in [w[1] for w in mode_writes])


class TestPybricksParityFeedback(unittest.TestCase):
    """speed()/load()/stalled()/dc() — the Pybricks Motor surface
    served from the servo's own feedback registers (present-speed
    0x3A, sign bit 15; present-load 0x3C, sign bit 10 per the
    Feetech SCServo SDK)."""

    def setUp(self):
        ST3215._buses = {}

    def _motor(self, reads=None, **kw):
        kw.setdefault("steps_per_dps", 10.0)
        m = ST3215Motor(servo_id=1, **kw)
        reads = dict(reads or {})

        def fake_read(servo_id, register, nbytes):
            v = reads.get(register)
            if v is None:
                return None
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        m._bus.read = fake_read
        return m

    def test_speed_scales_and_signs(self):
        m = self._motor({_REG_PRESENT_SPEED: 500})
        self.assertEqual(m.speed(), 50.0)
        m = self._motor({_REG_PRESENT_SPEED: 500 | 0x8000})
        self.assertEqual(m.speed(), -50.0)

    def test_speed_invert_flips(self):
        m = self._motor({_REG_PRESENT_SPEED: 500}, invert=True)
        self.assertEqual(m.speed(), -50.0)

    def test_speed_silent_bus_returns_none(self):
        m = self._motor({})
        self.assertIsNone(m.speed())

    def test_load_scales_by_model_stall_torque(self):
        # 500/1000 of stall. ST-3215 stall = 2940 mNm -> 1470.
        # Bit 10 = POSITIVE (bench-pinned 2026-08-03; the Feetech
        # SDK decode was inverted).
        m = self._motor({_REG_PRESENT_LOAD: 500 | 0x400})
        self.assertEqual(m.load(), 1470.0)
        m = self._motor({_REG_PRESENT_LOAD: 500})
        self.assertEqual(m.load(), -1470.0)

    def test_load_st3032_uses_its_own_stall_torque(self):
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=2, steps_per_dps=10.0)
        m._bus.read = lambda sid, reg, n: bytes([0xF4, 0x05])  # 500|b10
        self.assertEqual(m.load(), 490.0)   # 500/1000 x 980, positive

    def test_stalled_true_when_loaded_and_slow(self):
        m = self._motor({_REG_PRESENT_LOAD: 850,        # 85 % of stall
                         _REG_PRESENT_SPEED: 50})       # 5 dps
        self.assertTrue(m.stalled())

    def test_not_stalled_when_moving_or_unloaded(self):
        m = self._motor({_REG_PRESENT_LOAD: 850,
                         _REG_PRESENT_SPEED: 3000})     # 300 dps
        self.assertFalse(m.stalled())
        m = self._motor({_REG_PRESENT_LOAD: 100,        # 10 %
                         _REG_PRESENT_SPEED: 50})
        self.assertFalse(m.stalled())

    def test_stalled_silent_bus_raises(self):
        # A silent bus must not read as "not stalled" — that would
        # spin run_until_stalled forever.
        m = self._motor({})
        with self.assertRaises(OSError):
            m.stalled()

    def test_dc_maps_duty_onto_max_dps(self):
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=600.0)
        m.dc(50)
        speeds = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        self.assertEqual(speeds[-1], (3, bytes([
            3000 & 0xFF, 3000 >> 8])))   # 300 dps x 10 steps

    def test_run_is_degrees_per_second_not_power(self):
        # THE breaking change pin: run(300) must command 300 dps
        # (Pybricks), not 300 %-clamped power.
        m = ST3215Motor(servo_id=4, steps_per_dps=10.0, max_dps=600.0)
        m.run(300)
        speeds = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        self.assertEqual(speeds[-1], (4, bytes([
            3000 & 0xFF, 3000 >> 8])))


class TestSyncServoGroup(unittest.TestCase):
    def setUp(self):
        ST3215._buses = {}

    def test_constructor_rejects_servos_on_different_buses(self):
        s1 = ST3215Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        s2 = ST3215Motor(servo_id=2, uart_id=2, tx=14, rx=6)
        with self.assertRaises(ValueError):
            SyncServoGroup([s1, s2])

    def test_constructor_rejects_empty_list(self):
        with self.assertRaises(ValueError):
            SyncServoGroup([])

    def test_set_goal_speeds_emits_one_sync_write_packet(self):
        s1 = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        s2 = ST3215Motor(servo_id=2, steps_per_dps=10.0, max_dps=1000.0)
        group = SyncServoGroup([s1, s2])

        # Motors coast at construction; the group's first command
        # restores torque per member (tested separately). Pre-warm so
        # the baseline captures the steady-state one-packet contract.
        group.set_goal_speeds([0, 0])
        # Drain constructor + pre-warm packets so we examine only the
        # sync write.
        baseline = len(s1._bus._uart._tx_log)
        group.set_goal_speeds([50, -50])

        new_packets = s1._bus._uart._tx_log[baseline:]
        self.assertEqual(len(new_packets), 1)
        pkt = new_packets[0]

        # Header + body + checksum.
        self.assertTrue(pkt.startswith(_HEADER))
        body = pkt[2:-1]
        # Body layout: ID(0xFE) | LEN | INSTR(0x83) | ADDR | DATA_LEN |
        #              ID1 | D1_lo | D1_hi | ID2 | D2_lo | D2_hi
        self.assertEqual(body[0], 0xFE)            # broadcast
        self.assertEqual(body[2], 0x83)            # SYNC WRITE
        self.assertEqual(body[3], _REG_GOAL_SPEED) # register
        self.assertEqual(body[4], 2)               # data_len
        # Servo 1: speed = +50 dps × 10 steps/dps = 500 (no sign bit)
        self.assertEqual(body[5], 1)
        self.assertEqual(body[6], 500 & 0xFF)
        self.assertEqual(body[7], (500 >> 8) & 0xFF)
        # Servo 2: speed = -50 dps → magnitude 500 + sign bit
        v2 = 500 | 0x8000
        self.assertEqual(body[8], 2)
        self.assertEqual(body[9],  v2 & 0xFF)
        self.assertEqual(body[10], (v2 >> 8) & 0xFF)

    def test_set_goal_speeds_respects_per_servo_invert(self):
        s1 = ST3215Motor(servo_id=10, steps_per_dps=10.0, max_dps=1000.0)
        s2 = ST3215Motor(servo_id=11, steps_per_dps=10.0, max_dps=1000.0,
                         invert=True)
        group = SyncServoGroup([s1, s2])
        group.set_goal_speeds([0, 0])   # pre-warm: first call enables torque
        baseline = len(s1._bus._uart._tx_log)
        group.set_goal_speeds([50, 50])   # both commanded forward
        body = s1._bus._uart._tx_log[baseline:][0][2:-1]
        # s1 (no invert) → sign bit clear in high byte
        self.assertEqual(body[7] & 0x80, 0)
        # s2 (invert=True) → sign bit set in high byte
        self.assertEqual(body[10] & 0x80, 0x80)

    def test_set_goal_speeds_packet_length_field_matches_payload(self):
        servos = [ST3215Motor(servo_id=i + 1) for i in range(4)]
        group = SyncServoGroup(servos)
        group.set_goal_speeds([0, 0, 0, 0])   # pre-warm: enables torque
        baseline = len(servos[0]._bus._uart._tx_log)
        group.set_goal_speeds([0, 0, 0, 0])
        pkt = servos[0]._bus._uart._tx_log[baseline:][0]
        body = pkt[2:-1]
        # LEN = 4 + N × (1 + data_len) = 4 + 4 × 3 = 16
        self.assertEqual(body[1], 16)
        # Total body = ID + LEN + INSTR + ADDR + DATA_LEN + 4×(ID+2bytes) = 5 + 12 = 17
        self.assertEqual(len(body), 17)

    def test_set_goal_speeds_count_must_match_servo_count(self):
        s1 = ST3215Motor(servo_id=1)
        s2 = ST3215Motor(servo_id=2)
        group = SyncServoGroup([s1, s2])
        with self.assertRaises(ValueError):
            group.set_goal_speeds([100])      # too few
        with self.assertRaises(ValueError):
            group.set_goal_speeds([1, 2, 3])  # too many

    def test_set_goal_speeds_rejects_position_mode_servos(self):
        # Position-mode ST3215 doesn't have ``_encode_goal_speed`` —
        # SyncServoGroup should refuse rather than write nonsense.
        s_pos   = ST3215(servo_id=1)
        s_wheel = ST3215Motor(servo_id=2)
        group = SyncServoGroup([s_pos, s_wheel])
        with self.assertRaises(TypeError):
            group.set_goal_speeds([100, 100])

    def test_first_group_command_enables_torque_on_members(self):
        # Motors coast at construction; a goal-speed write to a
        # torque-off servo is silently ignored by the hardware. The
        # group's first command must therefore restore torque per
        # member before the sync write — and only once (cached).
        s1 = ST3215Motor(servo_id=1)
        s2 = ST3215Motor(servo_id=2)
        group = SyncServoGroup([s1, s2])
        baseline = len(s1._bus._uart._tx_log)
        group.set_goal_speeds([50, 50])
        new = s1._bus._uart._tx_log[baseline:]
        torque_writes = [p for p in new if p[2 + 3] == _REG_TORQUE]
        self.assertEqual(len(torque_writes), 2)   # one per member
        # Second command: no more torque packets, just the sync write.
        baseline = len(s1._bus._uart._tx_log)
        group.set_goal_speeds([60, 60])
        self.assertEqual(len(s1._bus._uart._tx_log) - baseline, 1)

    def test_sync_write_does_not_read_response(self):
        # Broadcast writes: no per-servo reply. Ensure we don't block
        # on the RX path waiting for one.
        s1 = ST3215Motor(servo_id=1)
        s2 = ST3215Motor(servo_id=2)
        group = SyncServoGroup([s1, s2])
        # Pre-warm: the first group command restores torque via
        # per-servo register writes (which do drain a status byte);
        # steady-state sync writes must not touch the RX path.
        group.set_goal_speeds([0, 0])
        # Track _rx calls — sync_write must not call into them.
        original_rx = s1._bus._rx
        rx_calls = [0]

        def counting_rx(*args, **kwargs):
            rx_calls[0] += 1
            return original_rx(*args, **kwargs)
        s1._bus._rx = counting_rx
        before = rx_calls[0]
        group.set_goal_speeds([100, 100])
        self.assertEqual(rx_calls[0], before)


class TestDutyLimit(unittest.TestCase):
    """run_until_stalled(duty_limit=...) — a temporary cap written to
    the RAM torque-limit register (0x30), restored afterwards (stall,
    error, or Ctrl-C alike), with stall detection scaled to the cap:
    under a 30 % cap the load can never reach 80 % of FULL stall, so
    the unscaled threshold would spin forever."""

    def setUp(self):
        ST3215._buses = {}

    def _motor(self, reads=None, **kw):
        kw.setdefault("steps_per_dps", 10.0)
        m = ST3215Motor(servo_id=1, **kw)
        reads = dict(reads or {})

        def fake_read(servo_id, register, nbytes):
            v = reads.get(register)
            if v is None:
                return None
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        m._bus.read = fake_read
        return m

    def test_push_caps_and_returns_previous(self):
        m = self._motor({_REG_TORQUE_LIMIT: 800})
        prev = m._duty_limit_push(30)
        self.assertEqual(prev, 800)
        writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE_LIMIT)
        self.assertEqual(writes[-1], (1, bytes([0x2C, 0x01])))  # 300
        self.assertEqual(m._duty_limit_raw, 300)

    def test_pop_restores_exactly_what_push_read(self):
        # A servo with a custom cap (800, not the factory 1000) gets
        # its own cap back — not a hardcoded "full torque".
        m = self._motor({_REG_TORQUE_LIMIT: 800})
        prev = m._duty_limit_push(30)
        m._duty_limit_pop(prev)
        writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE_LIMIT)
        self.assertEqual(writes[-1], (1, bytes([0x20, 0x03])))  # 800
        self.assertEqual(m._duty_limit_raw, 800)

    def test_duty_limit_validation(self):
        m = self._motor({_REG_TORQUE_LIMIT: 1000})
        for bad in (0, -5, 150, "abc", None):
            try:
                m._duty_limit_push(bad)
                self.fail("expected ValueError for %r" % (bad,))
            except ValueError:
                pass
        # Validation fires BEFORE any bus traffic.
        self.assertEqual(
            _writes_to(m._bus._uart._tx_log, _REG_TORQUE_LIMIT), [])

    def test_stalled_threshold_scales_with_the_cap(self):
        # 26 % of stall, barely moving: not stalled at full torque
        # (threshold 80 %), stalled under a 30 % cap (threshold 24 %).
        m = self._motor({_REG_PRESENT_LOAD: 260 | 0x400,
                         _REG_PRESENT_SPEED: 50})
        self.assertFalse(m.stalled())
        m._duty_limit_raw = 300
        self.assertTrue(m.stalled())

    def test_run_until_stalled_caps_runs_and_restores(self):
        m = self._motor({_REG_TORQUE_LIMIT: 1000,
                         _REG_PRESENT_LOAD: 280 | 0x400,   # 28 %
                         _REG_PRESENT_SPEED: 30,           # 3 dps
                         _REG_PRESENT_POS: 1000})
        got = m.run_until_stalled(200, duty_limit=30)
        self.assertTrue(got is not None)
        writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE_LIMIT)
        self.assertEqual(writes[0][1], bytes([0x2C, 0x01]))   # cap 300
        self.assertEqual(writes[-1][1], bytes([0xE8, 0x03]))  # back to 1000
        self.assertEqual(m._duty_limit_raw, 1000)
        # Default then="coast": the last torque write is off.
        torque = _writes_to(m._bus._uart._tx_log, _REG_TORQUE)
        self.assertEqual(torque[-1][1], bytes([0]))

    def test_cap_is_restored_when_the_run_dies(self):
        # Silent bus mid-run: stalled() raises, and the finally still
        # writes the previous limit back — the cap must never outlive
        # the call.
        m = self._motor({_REG_TORQUE_LIMIT: 700})
        with self.assertRaises(OSError):
            m.run_until_stalled(200, duty_limit=50)
        writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE_LIMIT)
        self.assertEqual(writes[-1][1], bytes([0xBC, 0x02]))  # 700
        self.assertEqual(m._duty_limit_raw, 700)

    def test_silent_bus_on_the_limit_read_raises_named(self):
        m = self._motor({})
        try:
            m._duty_limit_push(30)
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("0x30" in str(e), e)
            self.assertTrue("servo id 1" in str(e), e)


if __name__ == "__main__":
    # Keep the linter quiet about the unused module import used for reloading.
    assert st3215_mod is not None
    unittest.main()
