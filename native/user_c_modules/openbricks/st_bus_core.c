// SPDX-License-Identifier: MIT
//
// st_bus_core — implementation. See the header for the contract and
// st3215.py::_SCServoBus for the hardware-proven reference this
// mirrors byte-for-byte on the wire.

#include "st_bus_core.h"

#include <string.h>

#define SCS_HDR0        0xFF
#define SCS_HDR1        0xFF
#define SCS_INSTR_PING  0x01
#define SCS_INSTR_READ  0x02
#define SCS_INSTR_WRITE 0x03
#define SCS_INSTR_SYNCW 0x83
#define SCS_BROADCAST   0xFE

// Single-servo writes elicit a 6-byte status reply (unless
// broadcast). It is collected — leaving it in the RX path would
// mis-frame the NEXT transaction's reply, the stale-residue failure
// st3215.py's drain rule exists for — AND verified: sender, checksum
// and error flags, like st3215.py::_check_write_ack. A status that
// never arrives is a TIMEOUT, not a shrug; the caller retries.
// Short deadline: status arrives fast or not at all.
#define WRITE_STATUS_LEN    6
#define WRITE_STATUS_TICKS  3


uint8_t ob_bus_checksum(const uint8_t *body, size_t len) {
    unsigned s = 0;
    for (size_t i = 0; i < len; i++) {
        s += body[i];
    }
    return (uint8_t)(~s) & 0xFF;
}


void ob_bus_init(ob_bus_t *b, ob_bus_io_t io) {
    memset(b, 0, sizeof(*b));
    b->io = io;
    b->state = OB_BUS_IDLE;
}


static int bus_send(ob_bus_t *b, const uint8_t *body, size_t body_len,
                    uint8_t expect_id, uint8_t expect_len,
                    uint8_t want_reply, int timeout_ticks) {
    if (b->state == OB_BUS_AWAIT_REPLY) {
        return -1;      // mid-transaction; caller retries next tick
    }
    uint8_t pkt[OB_BUS_TX_BUF];
    if (body_len + 3 > sizeof(pkt)) {
        return -1;
    }
    pkt[0] = SCS_HDR0;
    pkt[1] = SCS_HDR1;
    memcpy(&pkt[2], body, body_len);
    pkt[2 + body_len] = ob_bus_checksum(body, body_len);

    b->io.rx_flush(b->io.ctx);
    size_t total = body_len + 3;
    int sent = b->io.tx(b->io.ctx, pkt, total);
    if (sent != (int)total) {
        // TX path refused bytes — treat as a failed transaction
        // rather than blocking. Diagnosable via n_bad.
        b->n_bad++;
        b->state = OB_BUS_IDLE;
        return -1;
    }
    b->expect_id  = expect_id;
    b->expect_len = expect_len;
    b->want_reply = want_reply;
    b->ticks_left = timeout_ticks;
    b->rx_have    = 0;
    b->state      = (expect_len > 0) ? OB_BUS_AWAIT_REPLY : OB_BUS_DONE;
    if (b->state == OB_BUS_DONE) {
        b->payload_len = 0;
        b->n_ok++;
    }
    return 0;
}


int ob_bus_start_read(ob_bus_t *b, uint8_t id, uint8_t reg,
                      uint8_t nbytes, int timeout_ticks) {
    if (nbytes == 0 || nbytes > OB_BUS_MAX_READ) {
        return -1;
    }
    // Body mirrors st3215.py::read — id, LEN=4, READ, reg, n.
    uint8_t body[5] = { id, 4, SCS_INSTR_READ, reg, nbytes };
    return bus_send(b, body, sizeof(body), id,
                    (uint8_t)(6 + nbytes), 1, timeout_ticks);
}


int ob_bus_start_write(ob_bus_t *b, uint8_t id, uint8_t reg,
                       const uint8_t *data, uint8_t len) {
    if (len == 0 || len > OB_BUS_MAX_READ) {
        return -1;
    }
    // Body mirrors st3215.py::write — id, LEN=len+3, WRITE, reg, data.
    uint8_t body[4 + OB_BUS_MAX_READ];
    body[0] = id;
    body[1] = (uint8_t)(len + 3);
    body[2] = SCS_INSTR_WRITE;
    body[3] = reg;
    memcpy(&body[4], data, len);
    // Broadcast writes get no status reply by protocol.
    if (id == SCS_BROADCAST) {
        return bus_send(b, body, 4 + len, id, 0, 0, 0);
    }
    // Collect-and-discard the 6-byte status (see WRITE_STATUS_LEN).
    return bus_send(b, body, 4 + len, id, WRITE_STATUS_LEN, 0,
                    WRITE_STATUS_TICKS);
}


int ob_bus_start_ping(ob_bus_t *b, uint8_t id, int timeout_ticks) {
    uint8_t body[3] = { id, 2, SCS_INSTR_PING };
    return bus_send(b, body, sizeof(body), id, 6, 1, timeout_ticks);
}


int ob_bus_start_sync_write(ob_bus_t *b, uint8_t reg, uint8_t data_len,
                            const uint8_t *ids, const uint8_t *data,
                            uint8_t n) {
    if (n == 0 || n > OB_BUS_MAX_SERVOS || data_len == 0) {
        return -1;
    }
    // Body mirrors st3215.py::sync_write —
    // 0xFE, LEN, SYNC_WRITE, reg, data_len, then (id, data...) per servo.
    size_t per = (size_t)1 + data_len;
    size_t body_len = 5 + n * per;
    uint8_t body[OB_BUS_TX_BUF - 3];
    if (body_len > sizeof(body)) {
        return -1;
    }
    body[0] = SCS_BROADCAST;
    body[1] = (uint8_t)(per * n + 4);
    body[2] = SCS_INSTR_SYNCW;
    body[3] = reg;
    body[4] = data_len;
    for (uint8_t i = 0; i < n; i++) {
        body[5 + i * per] = ids[i];
        memcpy(&body[5 + i * per + 1], &data[i * data_len], data_len);
    }
    // Broadcast: no reply by protocol.
    return bus_send(b, body, body_len, SCS_BROADCAST, 0, 0, 0);
}


void ob_bus_poll(ob_bus_t *b) {
    if (b->state != OB_BUS_AWAIT_REPLY) {
        return;
    }
    // Pull whatever arrived since last tick.
    if (b->rx_have < b->expect_len) {
        int got = b->io.rx(b->io.ctx, &b->rx_buf[b->rx_have],
                           (size_t)(b->expect_len - b->rx_have));
        if (got > 0) {
            b->rx_have = (uint8_t)(b->rx_have + got);
        }
    }
    if (b->rx_have >= b->expect_len) {
        // Frame complete — validate. Reply: FF FF ID LEN ERR .. CHK.
        const uint8_t *r = b->rx_buf;
        uint8_t ok = (r[0] == SCS_HDR0) && (r[1] == SCS_HDR1)
                     && (r[2] == b->expect_id);
        if (ok) {
            uint8_t chk = ob_bus_checksum(&r[2],
                                          (size_t)(b->expect_len - 3));
            ok = (chk == r[b->expect_len - 1]);
        }
        if (!ok) {
            b->n_bad++;
            b->state = OB_BUS_BAD_REPLY;
            return;
        }
        if (r[4] != 0) {
            // The servo itself reports a fault (overload, over-heat,
            // over-voltage...). The frame is authentic, so record the
            // flags either way. A WRITE acknowledged with error flags
            // did not do what was asked — fail it, like the reference
            // driver (st3215.py::_check_write_ack) raises. A READ's
            // payload is still valid data (a latched overload flag
            // doesn't corrupt present-position), and failing every
            // read would make a protection flag look like a dead bus.
            b->last_err = r[4];
            b->n_err++;
            if (!b->want_reply) {
                b->state = OB_BUS_SERVO_ERR;
                return;
            }
        }
        if (b->want_reply) {
            b->payload_len = (uint8_t)(b->expect_len - 6);
            memcpy(b->payload, &r[5], b->payload_len);
        } else {
            b->payload_len = 0;     // status collected + discarded
        }
        b->n_ok++;
        b->state = OB_BUS_DONE;
        return;
    }
    if (--b->ticks_left <= 0) {
        // A lost reply is a loss, write status and read payload
        // alike. This used to count a write whose status never came
        // as OK ("very likely still landed") — but a silently lost
        // config write leaves a servo in the wrong op-mode with zero
        // diagnostic, and speed sync-writes then move it in a mode
        // nobody configured. Register writes are idempotent, so the
        // caller retries; see st_servo_core's config sequence.
        b->n_timeout++;
        b->state = OB_BUS_TIMEOUT;
    }
}


ob_bus_state_t ob_bus_take_result(ob_bus_t *b, uint8_t *out,
                                  uint8_t *out_len) {
    ob_bus_state_t s = b->state;
    if (s != OB_BUS_DONE && s != OB_BUS_TIMEOUT && s != OB_BUS_BAD_REPLY
        && s != OB_BUS_SERVO_ERR) {
        return OB_BUS_IDLE;
    }
    if (out_len != NULL) {
        *out_len = 0;
    }
    if (s == OB_BUS_DONE && out != NULL && b->payload_len > 0) {
        memcpy(out, b->payload, b->payload_len);
        if (out_len != NULL) {
            *out_len = b->payload_len;
        }
    }
    b->state = OB_BUS_IDLE;
    b->rx_have = 0;
    return s;
}
