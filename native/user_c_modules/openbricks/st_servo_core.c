// SPDX-License-Identifier: MIT
//
// st_servo_core — implementation. See the header for the contract and
// drivers/st3215.py for the hardware-proven reference behaviors.

#include "st_servo_core.h"

#include <string.h>

#define MODE_WHEEL 1


void ob_sservo_init(ob_sservo_t *s) {
    memset(s, 0, sizeof(*s));
    s->read_in_flight = -1;
    s->write_in_flight = -1;
    s->user_in_flight = -1;
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        s->slots[i].torque_cmd = -1;
    }
}


int ob_sservo_attach(ob_sservo_t *s, int slot, uint8_t id,
                     uint8_t invert, uint8_t goal_acc) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS || s->slots[slot].in_use) {
        return -1;
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    memset(sl, 0, sizeof(*sl));
    sl->in_use   = 1;
    sl->id       = id;
    sl->invert   = invert;
    sl->goal_acc = goal_acc;
    sl->torque_cmd = -1;
    // config_step 0..2: op_mode, torque(coast), goal_acc — the
    // st3215.py constructor sequence, including the fresh-motor-
    // coasts rule (writing torque 0, not merely skipping it, also
    // releases a hold left by a previous program).
    sl->config_step = 0;
    return 0;
}


void ob_sservo_detach(ob_sservo_t *s, int slot) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS) {
        return;
    }
    if (s->read_in_flight == slot) {
        s->read_in_flight = -1;
    }
    if (s->write_in_flight == slot) {
        s->write_in_flight = -1;
    }
    if (s->user_in_flight == slot) {
        s->user_in_flight = -1;
    }
    memset(&s->slots[slot], 0, sizeof(s->slots[slot]));
    s->slots[slot].torque_cmd = -1;
}


int ob_sservo_user_stage(ob_sservo_t *s, int slot, int kind,
                         uint8_t reg, uint16_t val, uint8_t len) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS || !s->slots[slot].in_use) {
        return -1;
    }
    if ((kind != 1 && kind != 2) || len < 1 || len > 2) {
        return -1;
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    if (sl->user_kind != 0) {
        return -2;              // unresolved transaction; poll it first
    }
    sl->user_kind   = (uint8_t)kind;
    sl->user_reg    = reg;
    sl->user_len    = len;
    sl->user_val    = val;
    sl->user_done   = 0;
    sl->user_failed = 0;
    sl->user_fails  = 0;
    return 0;
}


int ob_sservo_user_poll(ob_sservo_t *s, int slot, uint16_t *val_out) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS || !s->slots[slot].in_use) {
        return -2;
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    if (sl->user_kind == 0) {
        return -2;
    }
    if (sl->user_failed) {
        sl->user_kind = 0;      // latch consumed: a new stage may follow
        return -1;
    }
    if (!sl->user_done) {
        return 0;
    }
    if (val_out != NULL) {
        *val_out = sl->user_val;
    }
    sl->user_kind = 0;
    return 1;
}


int ob_sservo_set_speed(ob_sservo_t *s, int slot, int32_t steps_per_s) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS || !s->slots[slot].in_use) {
        return -1;
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    if (sl->invert) {
        steps_per_s = -steps_per_s;
    }
    sl->target_steps = steps_per_s;
    sl->target_dirty = 1;
    // A speed command implies torque on (the Python driver's
    // _ensure_torque_on on every motion path) — but only when the
    // wire doesn't already have it: the native drivebase calls
    // set_speed EVERY tick, and unconditionally re-staging torque
    // starved the sync-writes behind an endless stream of priority-1
    // torque packets (caught by the first drivebase sim run).
    //
    // A PENDING coast (torque_cmd == 0, not yet shipped) must be
    // superseded too: torque_on still reads 1 in that window, but the
    // wire is about to get torque-off — without the second clause a
    // stop() followed immediately by run() shipped the coast, then
    // the new speed, and the motor sat limp with a live goal speed.
    if (!sl->torque_on || sl->torque_cmd == 0) {
        sl->torque_cmd = 1;
    }
    return 0;
}


int ob_sservo_coast(ob_sservo_t *s, int slot) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS || !s->slots[slot].in_use) {
        return -1;
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    sl->torque_cmd = 0;      // torque off = instant coast (e-stop rule)
    sl->target_dirty = 0;    // a stale speed must not re-drive it
    sl->target_steps = 0;    // ...and must not keep the slot HOT: the
                             // pump's heat heuristic reads a non-zero
                             // target as "commanded to keep turning",
                             // so a coasted-and-parked task motor was
                             // taking a full share of the read
                             // rotation forever — the exact bandwidth
                             // OB_SSERVO_COLD_EVERY exists to protect
    return 0;
}


uint16_t ob_sservo_encode_speed(int32_t steps_per_s) {
    // Sign-magnitude, bit 15 = direction — st3215.py::
    // _write_goal_speed_signed byte-identical.
    uint32_t mag = (steps_per_s < 0) ? (uint32_t)(-steps_per_s)
                                     : (uint32_t)steps_per_s;
    if (mag > 0x7FFF) {
        mag = 0x7FFF;
    }
    uint16_t v = (uint16_t)mag;
    if (steps_per_s < 0) {
        v |= 0x8000;
    }
    return v;
}


// Next readable slot of the requested kind from that kind's own
// round-robin cursor. ``want_hot``: 1 = driving slots, 0 = parked.
// Each kind has its own cursor so a parked slot's occasional turn
// cannot bias the rotation between two driving wheels.
int ob_sservo_pick_read(ob_sservo_t *s, int want_hot) {
    int start = want_hot ? s->rr_next : s->rr_cold;
    for (int k = 0; k < OB_SSERVO_SLOTS; k++) {
        int i = (start + k) % OB_SSERVO_SLOTS;
        ob_sservo_slot_t *sl = &s->slots[i];
        if (!sl->in_use || sl->config_step < 3) {
            continue;
        }
        if ((sl->hot != 0) == (want_hot != 0)) {
            return i;
        }
    }
    return -1;
}


void ob_sservo_next_op(ob_sservo_t *s, ob_sservo_op_t *op) {
    memset(op, 0, sizeof(*op));
    op->kind = OB_SOP_NONE;
    op->slot = -1;

    // 1. Torque changes first — coast is the e-stop-adjacent path and
    //    must never queue behind feedback reads. ONE sync-write
    //    covers every pending slot, so a drivebase stop releases
    //    both wheels at the same packet boundary (previously one
    //    single-servo write each, a bus transaction apart). Torque
    //    commands are one-shot, so this deliberately skips the
    //    sync/read fairness rule below: they can't starve anything.
    uint8_t tn = 0;
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        ob_sservo_slot_t *sl = &s->slots[i];
        if (sl->in_use && sl->config_step >= 3 && sl->torque_cmd >= 0) {
            op->sync_ids[tn] = sl->id;
            op->sync_data[tn] = (uint8_t)sl->torque_cmd;
            tn++;
        }
    }
    if (tn > 0) {
        op->kind = OB_SOP_SYNC_TORQUE;
        op->sync_n = tn;
        return;
    }

    // 2. Config sequences. A config_failed slot issues nothing more:
    //    retrying a dead servo forever would hog the bus (config
    //    outranks speeds and reads) and starve every healthy slot.
    //    Python sees the latch via servo_write_stats and the absence
    //    of odometry.
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        ob_sservo_slot_t *sl = &s->slots[i];
        if (!sl->in_use || sl->config_step >= 3 || sl->config_failed) {
            continue;
        }
        op->kind = OB_SOP_WRITE;
        op->slot = i;
        op->id = sl->id;
        op->data_len = 1;
        switch (sl->config_step) {
            case 0:
                op->reg = OB_SREG_OP_MODE;
                op->data[0] = MODE_WHEEL;
                break;
            case 1:
                op->reg = OB_SREG_TORQUE;
                op->data[0] = 0;
                break;
            default:
                op->reg = OB_SREG_GOAL_ACC;
                op->data[0] = sl->goal_acc;
                break;
        }
        return;
    }

    // 2.5 User register transactions (run_until_stalled's duty_limit
    //     cap and its restore) — rare one-shots staged by Python,
    //     which is blocked polling for the outcome. After config (a
    //     half-configured servo must finish its init first); before
    //     the speed syncs, because a drivebase re-staging speeds
    //     every tick plus rule 3's sync/read fairness dance would
    //     otherwise starve the transaction indefinitely.
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        ob_sservo_slot_t *sl = &s->slots[i];
        if (!sl->in_use || sl->config_step < 3 || sl->user_kind == 0
            || sl->user_done || sl->user_failed) {
            continue;
        }
        op->slot = i;
        op->id = sl->id;
        op->reg = sl->user_reg;
        op->data_len = sl->user_len;
        if (sl->user_kind == 1) {
            op->kind = OB_SOP_USER_WRITE;
            op->data[0] = (uint8_t)(sl->user_val & 0xFF);
            op->data[1] = (uint8_t)(sl->user_val >> 8);
        } else {
            op->kind = OB_SOP_USER_READ;
        }
        return;
    }

    // 3. Dirty speeds — one sync-write covers every dirty slot, so
    //    both wheels of a drivebase get their setpoints at the same
    //    packet boundary (the Python driver's SyncServoGroup lesson).
    uint8_t n = 0;
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        ob_sservo_slot_t *sl = &s->slots[i];
        if (sl->in_use && sl->config_step >= 3 && sl->target_dirty) {
            uint16_t v = ob_sservo_encode_speed(sl->target_steps);
            op->sync_ids[n] = sl->id;
            op->sync_data[n * 2]     = (uint8_t)(v & 0xFF);
            op->sync_data[n * 2 + 1] = (uint8_t)(v >> 8);
            n++;
        }
    }
    if (n > 0 && !s->last_was_sync) {
        op->kind = OB_SOP_SYNC_SPEED;
        op->sync_n = n;
        return;
    }
    op->sync_n = 0;   // sync deferred for fairness (or none dirty)

    // 4. Feedback: round-robin present-position reads, WEIGHTED.
    //
    // A flat rotation gives a parked task motor exactly as much bus
    // as a wheel steering a heading loop, so putting two task motors
    // on the bus halved the drivebase's odometry rate (~220 Hz per
    // wheel -> ~110). Slots that are driving get the bus; parked
    // ones get one turn in OB_SSERVO_COLD_EVERY — enough that
    // angle()/speed() never drift arbitrarily stale, little enough
    // that they cost the wheels almost nothing.
    int want_cold = 0;
    if (++s->cold_tick >= OB_SSERVO_COLD_EVERY) {
        s->cold_tick = 0;
        want_cold = 1;
    }
    int pick = ob_sservo_pick_read(s, want_cold ? 0 : 1);
    if (pick < 0) {
        // Nothing of the preferred kind. Fall back to the OTHER kind
        // explicitly, so the rotation is driven by that kind's own
        // cursor. Falling straight through to "any" scanned from the
        // hot cursor while a cold read advanced the cold one — with
        // every slot parked (exactly the state while a script is
        // still constructing its motors) the hot cursor never moved
        // and one slot took 7 reads in 8, starving the rest.
        pick = ob_sservo_pick_read(s, want_cold ? 1 : 0);
    }
    // No third fallback: every readable slot is either hot or cold,
    // so asking for both has already covered all of them.
    if (pick >= 0) {
        op->kind = OB_SOP_READ_POS;
        op->slot = pick;
        op->id = s->slots[pick].id;
    }
}


void ob_sservo_op_started(ob_sservo_t *s, const ob_sservo_op_t *op) {
    switch (op->kind) {
        case OB_SOP_WRITE:
            // Post-config torque rides OB_SOP_SYNC_TORQUE; the only
            // single writes left are the config sequence. The step
            // does NOT advance here: a write is done when its
            // verified ACK comes back (ob_sservo_write_result), not
            // when its bytes leave — advancing at TX let a lost
            // op_mode write mark a servo "configured" while it was
            // still in position mode, receiving speed sync-writes.
            s->write_in_flight = op->slot;
            break;
        case OB_SOP_USER_WRITE:
        case OB_SOP_USER_READ:
            s->user_in_flight = op->slot;
            break;
        case OB_SOP_SYNC_SPEED:
            s->last_was_sync = 1;
            for (uint8_t k = 0; k < op->sync_n; k++) {
                for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
                    if (s->slots[i].in_use
                        && s->slots[i].id == op->sync_ids[k]) {
                        s->slots[i].target_dirty = 0;
                    }
                }
            }
            break;
        case OB_SOP_SYNC_TORQUE:
            // Not counted for sync/read fairness — one-shot commands.
            for (uint8_t k = 0; k < op->sync_n; k++) {
                for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
                    ob_sservo_slot_t *sl = &s->slots[i];
                    if (sl->in_use && sl->id == op->sync_ids[k]) {
                        sl->torque_cmd = -1;
                        sl->torque_on = (op->sync_data[k] != 0);
                    }
                }
            }
            break;
        case OB_SOP_READ_POS:
            s->last_was_sync = 0;    // fairness satisfied
            s->read_in_flight = op->slot;
            if (s->slots[op->slot].hot) {
                s->rr_next = (op->slot + 1) % OB_SSERVO_SLOTS;
            } else {
                s->rr_cold = (op->slot + 1) % OB_SSERVO_SLOTS;
            }
            break;
        default:
            break;
    }
}


void ob_sservo_read_result(ob_sservo_t *s, int ok,
                           const uint8_t *payload, uint8_t len) {
    int slot = s->read_in_flight;
    s->read_in_flight = -1;
    if (slot < 0 || slot >= OB_SSERVO_SLOTS) {
        return;
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    if (!sl->in_use) {
        return;
    }
    if (!ok || len < 2) {
        sl->reads_failed++;
        sl->stale++;
        return;
    }
    if (len >= OB_SSERVO_FEEDBACK_LEN) {
        // Present-speed: sign-magnitude, bit 15 = negative.
        uint16_t sp = (uint16_t)(payload[2] | (payload[3] << 8));
        sl->speed_steps = (sp & 0x8000) ? -(int32_t)(sp & 0x7FFF)
                                        : (int32_t)sp;
        // Present-load: 0.1%-of-stall magnitude, bit 10 = POSITIVE.
        // Bench-measured 2026-08-03 (ST-3032, both directions): the
        // servo sets bit 10 while pushing in its positive direction —
        // the OPPOSITE of the Feetech SDK's decode, which read a
        // forward-driving motor as negative torque. The Pybricks
        // contract (load sign matches speed sign when driving) pins
        // this decode.
        uint16_t ld = (uint16_t)(payload[4] | (payload[5] << 8));
        sl->load_raw = (ld & 0x0400) ? (int32_t)(ld & 0x03FF)
                                     : -(int32_t)(ld & 0x03FF);
        sl->have_feedback = 1;
    }
    uint16_t raw = (uint16_t)((payload[0] | (payload[1] << 8)) & 0x0FFF);
    if (sl->have_raw) {
        // Wrap-correct delta into [-2048, 2047]: the 12-bit position
        // wraps at 4096 every revolution (st3215.py's angle()
        // heuristic, integer form).
        int32_t delta = (int32_t)((raw - sl->last_raw + 2048) & 0x0FFF)
                        - 2048;
        sl->accum += delta;
    } else {
        sl->have_raw = 1;
    }
    sl->last_raw = raw;
    sl->reads_ok++;
    sl->stale = 0;
}


void ob_sservo_write_result(ob_sservo_t *s, int ok) {
    int slot = s->write_in_flight;
    s->write_in_flight = -1;
    if (slot < 0 || slot >= OB_SSERVO_SLOTS) {
        return;             // no single write in flight (sync/none)
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    if (!sl->in_use || sl->config_step >= 3) {
        return;             // detached mid-flight, or already done
    }
    if (!ok) {
        sl->writes_failed++;
        if (++sl->config_fails >= OB_SSERVO_CONFIG_TRIES) {
            sl->config_failed = 1;
        }
        return;             // step unchanged: next_op reissues it
    }
    if (sl->config_step == 1) {
        sl->torque_on = 0;  // the torque(coast) write is CONFIRMED on
                            // the wire, not merely transmitted
    }
    sl->config_step++;
    sl->config_fails = 0;
}


void ob_sservo_user_write_result(ob_sservo_t *s, int ok) {
    int slot = s->user_in_flight;
    s->user_in_flight = -1;
    if (slot < 0 || slot >= OB_SSERVO_SLOTS
        || !s->slots[slot].in_use || s->slots[slot].user_kind != 1) {
        return;                 // detached/abandoned mid-flight
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    if (!ok) {
        sl->writes_failed++;
        if (++sl->user_fails >= OB_SSERVO_CONFIG_TRIES) {
            sl->user_failed = 1;
        }
        return;                 // still staged: next_op reissues it
    }
    sl->user_done = 1;
    sl->user_fails = 0;
}


void ob_sservo_user_read_result(ob_sservo_t *s, int ok,
                                const uint8_t *payload, uint8_t len) {
    int slot = s->user_in_flight;
    s->user_in_flight = -1;
    if (slot < 0 || slot >= OB_SSERVO_SLOTS
        || !s->slots[slot].in_use || s->slots[slot].user_kind != 2) {
        return;                 // detached/abandoned mid-flight
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    if (!ok || len < sl->user_len) {
        if (++sl->user_fails >= OB_SSERVO_CONFIG_TRIES) {
            sl->user_failed = 1;
        }
        return;
    }
    sl->user_val = payload[0];
    if (sl->user_len > 1) {
        sl->user_val |= (uint16_t)(payload[1] << 8);
    }
    sl->user_done = 1;
    sl->user_fails = 0;
}


int32_t ob_sservo_counts(const ob_sservo_t *s, int slot) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS) {
        return 0;
    }
    // Invert flips the REPORTED frame too, so command sign and angle
    // sign stay consistent (the Python driver applies invert on both
    // paths; a drivebase depends on that symmetry).
    const ob_sservo_slot_t *sl = &s->slots[slot];
    return sl->invert ? -sl->accum : sl->accum;
}


int32_t ob_sservo_speed_steps(const ob_sservo_t *s, int slot) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS) {
        return 0;
    }
    const ob_sservo_slot_t *sl = &s->slots[slot];
    return sl->invert ? -sl->speed_steps : sl->speed_steps;
}


int32_t ob_sservo_load_raw(const ob_sservo_t *s, int slot) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS) {
        return 0;
    }
    const ob_sservo_slot_t *sl = &s->slots[slot];
    return sl->invert ? -sl->load_raw : sl->load_raw;
}


int ob_sservo_feedback_fresh(const ob_sservo_t *s, int slot) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS) {
        return 0;
    }
    const ob_sservo_slot_t *sl = &s->slots[slot];
    return sl->in_use && sl->have_feedback && sl->stale == 0;
}
