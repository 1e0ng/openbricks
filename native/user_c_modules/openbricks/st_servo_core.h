// SPDX-License-Identifier: MIT
//
// st_servo_core — serial-bus servo slots + the per-tick bus scheduler.
//
// Pure C, hard-tick-safe, integer-only. The C rewrite of the control-
// relevant slice of drivers/st3215.py's wheel mode, mirroring its
// hardware-proven behaviors: the wheel-mode init sequence (op_mode=1,
// torque=0 so a fresh motor coasts, goal_acc ramp), sign-magnitude
// speed encoding (bit 15 = direction), and 12-bit present-position
// unwrapped into a multi-turn accumulator.
//
// Split of responsibilities:
//   * This core owns WHAT to do next on the bus (a small planner:
//     staged torque/config/speed commands first, then round-robin
//     position feedback) and how to fold results back into slot
//     state. It never touches I/O.
//   * st_bus owns the wire (ob_bus_t) and calls into this core from
//     the hard tick: when the bus goes idle, ask for the next op;
//     when a transaction finishes, hand back the result.
//
// Units: speeds are signed encoder steps/second (the servo's native
// unit; Python converts dps <-> steps with its existing constants so
// this core stays float-free). Positions are signed accumulated
// encoder counts (4096/rev).

#pragma once

#include <stddef.h>
#include <stdint.h>

#define OB_SSERVO_SLOTS      4

// SCS register map (st3215.py's _REG_* — the subset this core uses).
#define OB_SREG_OP_MODE      0x21
#define OB_SREG_TORQUE       0x28
#define OB_SREG_GOAL_ACC     0x29
#define OB_SREG_GOAL_SPEED   0x2E
#define OB_SREG_PRESENT_POS  0x38

// Feedback-read deadline, in bus polls (hard ticks). 5 ms at 1 kHz.
#define OB_SSERVO_READ_TICKS 5

// One turn in this many reads goes to a parked ("cold") slot. The
// rest go to slots that are actually driving. With two wheels under
// a drivebase and two idle task motors this keeps the wheels at
// ~7/8 of the odometry rate they had when they were alone on the
// bus, instead of the half they get under a flat round-robin.
#define OB_SSERVO_COLD_EVERY 8

// How many times a lost/refused config write is retried before the
// slot is latched config_failed and stops issuing ops. Transient
// status loss on the half-duplex line costs one retry; a servo that
// fails this many CONSECUTIVE writes is absent or broken, and letting
// it retry forever would hog the bus (config outranks speed syncs and
// feedback reads) and starve every healthy slot behind it. Python
// surfaces the latch through servo_write_stats.
#define OB_SSERVO_CONFIG_TRIES 8

// Feedback read width: present-position (0x38), present-speed
// (0x3A) and present-load (0x3C) are CONTIGUOUS, so one 6-byte read
// returns all three for ~the wire cost of the old 2-byte position
// read (reply grows 4 bytes ≈ 40 µs at 1 Mbps) — no extra
// transactions, no odometry-rate dilution.
#define OB_SSERVO_FEEDBACK_LEN 6

// What the planner wants done on the (idle) bus next.
typedef enum {
    OB_SOP_NONE = 0,        // nothing pending
    OB_SOP_WRITE,           // single-register write to one servo
    OB_SOP_SYNC_SPEED,      // sync-write goal_speed to every dirty slot
    OB_SOP_SYNC_TORQUE,     // sync-write torque to every pending slot —
                            // both drivebase wheels coast at the same
                            // packet boundary, not one write apart
    OB_SOP_READ_POS,        // read present-position of one slot
    OB_SOP_USER_WRITE,      // user-staged register write (duty_limit)
    OB_SOP_USER_READ,       // user-staged register read (limit restore)
} ob_sservo_op_kind_t;

typedef struct {
    ob_sservo_op_kind_t kind;
    // OB_SOP_WRITE:
    uint8_t  id;
    uint8_t  reg;
    uint8_t  data[2];
    uint8_t  data_len;
    // OB_SOP_SYNC_SPEED (2 bytes/slot) / OB_SOP_SYNC_TORQUE (1 byte/slot):
    uint8_t  sync_ids[OB_SSERVO_SLOTS];
    uint8_t  sync_data[OB_SSERVO_SLOTS * 2];
    uint8_t  sync_n;
    // OB_SOP_READ_POS:
    int      slot;
} ob_sservo_op_t;

typedef struct {
    uint8_t  in_use;
    uint8_t  id;
    uint8_t  invert;

    // Wheel-mode init: staged writes issued before anything else for
    // this slot (op_mode=1, torque=0, goal_acc) — st3215.py's
    // constructor sequence, one packet per idle-bus tick.
    uint8_t  config_step;   // 0..2 pending, 3 = configured
    uint8_t  goal_acc;      // register byte, pre-encoded by Python

    // Command staging (Python writes under the bus lock).
    int32_t  target_steps;  // signed steps/s
    uint8_t  target_dirty;
    int8_t   torque_cmd;    // -1 none pending, 0 = coast, 1 = on
    uint8_t  torque_on;     // last torque value WRITTEN to the wire
    // Feedback.
    uint16_t last_raw;      // last 12-bit reading
    uint8_t  have_raw;
    int32_t  accum;         // unwrapped counts (signed, multi-turn)
    // Present-speed / present-load from the widened 6-byte read,
    // decoded to MOTOR-frame signed values (invert applied by the
    // accessors, like counts). Valid once have_feedback is set.
    int32_t  speed_steps;   // signed steps/s (reg sign-magnitude b15)
    int32_t  load_raw;      // signed 0.1%-of-stall units (b10 sign)
    uint8_t  have_feedback;
    uint32_t reads_ok;
    uint32_t reads_failed;  // timeouts/bad replies on feedback
    uint32_t stale;         // consecutive failures (0 after success)
    // Write-side accounting — the read counters' counterpart, so a
    // lost command is as visible as a lost reply.
    uint32_t writes_failed; // config writes that timed out / errored
    uint8_t  config_fails;  // consecutive failures of the CURRENT step
    uint8_t  config_failed; // latched: gave up after CONFIG_TRIES
    // Poll priority. A slot that is DRIVING needs its odometry now;
    // a parked one does not, and polling both alike splits the bus
    // evenly — which halved the drivebase's feedback rate the moment
    // two task motors joined the bus. Set by the pump each tick.
    uint8_t  hot;
    // One-deep user register transaction (run_until_stalled's
    // duty_limit torque cap and its read-back/restore, on a bus the
    // native pump owns). Python stages it, the planner ships it once
    // config is done, and the verified result — or CONFIG_TRIES
    // consecutive losses — resolves it. Python polls the outcome; a
    // loss is a latched failure, never a silent shrug.
    uint8_t  user_kind;     // 0 none, 1 write, 2 read
    uint8_t  user_reg;
    uint8_t  user_len;      // 1 or 2 bytes
    uint16_t user_val;      // write value in / read result out
    uint8_t  user_done;     // result ready, waiting for Python's poll
    uint8_t  user_failed;   // latched after CONFIG_TRIES losses
    uint8_t  user_fails;    // consecutive losses of the current txn
} ob_sservo_slot_t;

typedef struct {
    ob_sservo_slot_t slots[OB_SSERVO_SLOTS];
    int      rr_next;       // round-robin cursor, DRIVING slots
    int      rr_cold;       // separate cursor for parked slots, so a
                            // cold slot's turn never perturbs the
                            // rotation between the driving ones
    int      read_in_flight; // slot whose POS read is on the bus, or -1
    int      write_in_flight; // slot whose config WRITE is on the bus,
                              // or -1 — its step advances only when
                              // the verified ACK comes back
    int      user_in_flight;  // slot whose USER txn is on the bus, or -1
    // Cold slots still get an occasional turn: skipping them
    // entirely would let angle()/speed() drift arbitrarily stale
    // while still reporting themselves fresh.
    uint8_t  cold_tick;
    uint8_t  last_was_sync;  // fairness: a sync must be followed by a
                             // read before the next sync — a
                             // drivebase re-staging speeds every tick
                             // otherwise starves feedback entirely
                             // and the control loop runs on frozen
                             // odometry (caught by the first
                             // drivebase sim run)
} ob_sservo_t;

void ob_sservo_init(ob_sservo_t *s);

// Claim/configure a slot. Returns 0, or -1 on bad args / slot in use.
int  ob_sservo_attach(ob_sservo_t *s, int slot, uint8_t id,
                      uint8_t invert, uint8_t goal_acc);
void ob_sservo_detach(ob_sservo_t *s, int slot);

// Stage commands (call under the bus lock). Speeds in signed steps/s;
// invert is applied here, matching the Python driver's site.
int  ob_sservo_set_speed(ob_sservo_t *s, int slot, int32_t steps_per_s);
int  ob_sservo_coast(ob_sservo_t *s, int slot);

// Encoded 16-bit sign-magnitude goal-speed value (bit 15 = direction)
// — byte-identical to st3215.py::_encode_goal_speed's output for the
// same signed steps value. Exposed for tests.
uint16_t ob_sservo_encode_speed(int32_t steps_per_s);

// Planner: fill *op with the next bus operation, or OB_SOP_NONE.
// Priority: pending torque > config sequence > dirty speeds (one
// sync-write covers all) > round-robin feedback read.
void ob_sservo_next_op(ob_sservo_t *s, ob_sservo_op_t *op);

// Next readable slot of the requested kind (1 = driving, 0 = parked)
// from that kind's own cursor, or -1. Exposed for tests.
int ob_sservo_pick_read(ob_sservo_t *s, int want_hot);

// Result routing. For reads: payload is the 2-byte present-position
// little-endian (12-bit) — folds into the unwrap accumulator. ok=0
// counts the failure and bumps ``stale``.
void ob_sservo_read_result(ob_sservo_t *s, int ok,
                           const uint8_t *payload, uint8_t len);

// Result routing for single-servo WRITES (the config sequence). ok=1
// (verified ACK) advances the slot's config_step; ok=0 counts the
// failure and retries, latching config_failed after
// OB_SSERVO_CONFIG_TRIES consecutive losses. No-op when no write is
// in flight (sync-writes are broadcast: no reply by protocol).
void ob_sservo_write_result(ob_sservo_t *s, int ok);

// Op-issued bookkeeping: the caller reports which op it actually
// started (planner state like dirty flags advance HERE, not in
// next_op, so a refused bus start retries the same op — and
// config_step advances even later, at write_result, so a LOST config
// write retries too).
void ob_sservo_op_started(ob_sservo_t *s, const ob_sservo_op_t *op);

// Stage a user register transaction on a configured slot (call under
// the bus lock). kind: 1 = write ``val``, 2 = read. len: 1 or 2
// bytes. Returns 0; -1 on bad args / unconfigured slot; -2 when a
// previous transaction is still unresolved (poll it first).
int ob_sservo_user_stage(ob_sservo_t *s, int slot, int kind,
                         uint8_t reg, uint16_t val, uint8_t len);

// Poll the staged transaction: 1 = done (read value in *val_out; the
// stage is cleared, a new one may be staged), 0 = still pending,
// -1 = failed after CONFIG_TRIES losses (latched; cleared by this
// poll), -2 = nothing staged / bad slot.
int ob_sservo_user_poll(ob_sservo_t *s, int slot, uint16_t *val_out);

// Result routing for user transactions (mirrors write_result /
// read_result: verified outcome only, retries ride next_op).
void ob_sservo_user_write_result(ob_sservo_t *s, int ok);
void ob_sservo_user_read_result(ob_sservo_t *s, int ok,
                                const uint8_t *payload, uint8_t len);

// Signed unwrapped position in encoder counts.
int32_t ob_sservo_counts(const ob_sservo_t *s, int slot);

// Present-speed (signed steps/s) and present-load (signed 0.1 % of
// stall), USER frame (slot invert applied, same rule as counts).
// Valid only while ob_sservo_feedback_fresh; both 0 before the
// first widened read lands.
int32_t  ob_sservo_speed_steps(const ob_sservo_t *s, int slot);
int32_t  ob_sservo_load_raw(const ob_sservo_t *s, int slot);
// 1 when speed/load have ever been decoded AND the last read
// succeeded (stale == 0).
int      ob_sservo_feedback_fresh(const ob_sservo_t *s, int slot);
