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
    memset(&s->slots[slot], 0, sizeof(s->slots[slot]));
    s->slots[slot].torque_cmd = -1;
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
    // _ensure_torque_on on every motion path).
    sl->torque_cmd = 1;
    return 0;
}


int ob_sservo_coast(ob_sservo_t *s, int slot) {
    if (slot < 0 || slot >= OB_SSERVO_SLOTS || !s->slots[slot].in_use) {
        return -1;
    }
    ob_sservo_slot_t *sl = &s->slots[slot];
    sl->torque_cmd = 0;      // torque off = instant coast (e-stop rule)
    sl->target_dirty = 0;    // a stale speed must not re-drive it
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


void ob_sservo_next_op(ob_sservo_t *s, ob_sservo_op_t *op) {
    memset(op, 0, sizeof(*op));
    op->kind = OB_SOP_NONE;
    op->slot = -1;

    // 1. Torque changes first — coast is the e-stop-adjacent path and
    //    must never queue behind feedback reads.
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        ob_sservo_slot_t *sl = &s->slots[i];
        if (sl->in_use && sl->config_step >= 3 && sl->torque_cmd >= 0) {
            op->kind = OB_SOP_WRITE;
            op->slot = i;
            op->id = sl->id;
            op->reg = OB_SREG_TORQUE;
            op->data[0] = (uint8_t)sl->torque_cmd;
            op->data_len = 1;
            return;
        }
    }

    // 2. Config sequences.
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        ob_sservo_slot_t *sl = &s->slots[i];
        if (!sl->in_use || sl->config_step >= 3) {
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
    if (n > 0) {
        op->kind = OB_SOP_SYNC_SPEED;
        op->sync_n = n;
        return;
    }

    // 4. Feedback: round-robin present-position reads.
    for (int k = 0; k < OB_SSERVO_SLOTS; k++) {
        int i = (s->rr_next + k) % OB_SSERVO_SLOTS;
        ob_sservo_slot_t *sl = &s->slots[i];
        if (sl->in_use && sl->config_step >= 3) {
            op->kind = OB_SOP_READ_POS;
            op->slot = i;
            op->id = sl->id;
            return;
        }
    }
}


void ob_sservo_op_started(ob_sservo_t *s, const ob_sservo_op_t *op) {
    switch (op->kind) {
        case OB_SOP_WRITE: {
            ob_sservo_slot_t *sl = &s->slots[op->slot];
            if (sl->config_step < 3) {
                sl->config_step++;
            } else if (op->reg == OB_SREG_TORQUE) {
                sl->torque_cmd = -1;
            }
            break;
        }
        case OB_SOP_SYNC_SPEED:
            for (uint8_t k = 0; k < op->sync_n; k++) {
                for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
                    if (s->slots[i].in_use
                        && s->slots[i].id == op->sync_ids[k]) {
                        s->slots[i].target_dirty = 0;
                    }
                }
            }
            break;
        case OB_SOP_READ_POS:
            s->read_in_flight = op->slot;
            s->rr_next = (op->slot + 1) % OB_SSERVO_SLOTS;
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
