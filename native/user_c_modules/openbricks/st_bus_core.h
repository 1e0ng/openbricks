// SPDX-License-Identifier: MIT
//
// st_bus_core — Feetech SCS serial-bus protocol + transaction state
// machine, pure C, hard-tick-safe.
//
// This is the C rewrite of the wire layer in
// ``openbricks/drivers/st3215.py`` (the hardware-proven reference —
// packet layout, checksum, response framing and the drain-before-TX
// rule all mirror it; golden-vector tests pin byte equality). The
// difference is HOW it waits: the Python driver blocks up to 50 ms
// per response, which is 50 lost control cycles when a servo drops a
// reply. This core never blocks — callers start a transaction, then
// poll it once per tick; deadlines are counted in ticks.
//
// Context contract: designed to run on the hard tick (esp_timer
// service task — see esp32-openbricks-hard-tick.patch). No
// MicroPython headers, no ESP-IDF headers, no allocation, no
// blocking. I/O is injected as non-blocking function pointers:
// the firmware backs them with IDF UART shims (separate patch),
// unix tests back them with Python-fed ring buffers.

#pragma once

#include <stddef.h>
#include <stdint.h>

// Injected byte I/O. All calls must be non-blocking:
//   tx        — queue bytes for transmit, return count accepted
//   rx        — read available bytes, return count read (0 = none)
//   rx_flush  — discard anything pending in the receive path
// (the Python driver's drain-before-TX rule: stale echo/residue
// otherwise mis-frames the next reply — see st3215.py::_tx).
typedef struct {
    int  (*tx)(void *ctx, const uint8_t *buf, size_t len);
    int  (*rx)(void *ctx, uint8_t *buf, size_t maxlen);
    void (*rx_flush)(void *ctx);
    void *ctx;
} ob_bus_io_t;

// SCS packet limits. Longest packet we ever build is a SYNC WRITE to
// OB_BUS_MAX_SERVOS servos; longest reply is a read of
// OB_BUS_MAX_READ bytes (header 2 + id + len + err + data + chk).
#define OB_BUS_MAX_SERVOS   4
#define OB_BUS_MAX_READ     8
#define OB_BUS_TX_BUF       64
#define OB_BUS_RX_BUF       (6 + OB_BUS_MAX_READ)

// Transaction states, readable from Python for diagnostics.
typedef enum {
    OB_BUS_IDLE = 0,        // no transaction; ready to start one
    OB_BUS_AWAIT_REPLY,     // sent, collecting response bytes
    OB_BUS_DONE,            // reply validated; payload available
    OB_BUS_TIMEOUT,         // deadline passed with no valid reply
    OB_BUS_BAD_REPLY,       // framing/checksum/id mismatch
    OB_BUS_SERVO_ERR,       // authentic reply, but the servo's error
                            // flags are set — the write it acknowledges
                            // did not do what was asked
} ob_bus_state_t;

typedef struct {
    ob_bus_io_t     io;
    ob_bus_state_t  state;

    // In-flight transaction bookkeeping.
    uint8_t         expect_id;      // servo the reply must come from
    uint8_t         expect_len;     // total reply bytes expected
    uint8_t         want_reply;     // 0 = fire-and-forget (status discarded)
    int             ticks_left;     // countdown to timeout
    uint8_t         rx_buf[OB_BUS_RX_BUF];
    uint8_t         rx_have;

    // Completed-read payload (valid when state == OB_BUS_DONE and
    // the transaction was a read).
    uint8_t         payload[OB_BUS_MAX_READ];
    uint8_t         payload_len;

    // Cumulative diagnostics, readable from Python: enough to answer
    // "is this bus healthy" without a logic analyzer.
    uint32_t        n_ok;
    uint32_t        n_timeout;
    uint32_t        n_bad;
    uint32_t        n_err;          // replies carrying servo error flags
    uint8_t         last_err;       // most recent non-zero ERR byte
} ob_bus_t;

void ob_bus_init(ob_bus_t *b, ob_bus_io_t io);

// Start transactions. Return 0 on success, -1 if the bus is mid-
// transaction (caller retries next tick) or arguments are invalid.
// All of them flush RX and transmit immediately.
//
// timeout_ticks: how many ob_bus_poll calls before OB_BUS_TIMEOUT.
// At the 1 kHz hard tick, 5 covers the ST-3032's reply latency with
// margin while costing at most 5 control cycles on a dropped reply
// (the Python driver's blocking 50 ms cost 50).
int ob_bus_start_read(ob_bus_t *b, uint8_t id, uint8_t reg,
                      uint8_t nbytes, int timeout_ticks);
int ob_bus_start_write(ob_bus_t *b, uint8_t id, uint8_t reg,
                       const uint8_t *data, uint8_t len);
int ob_bus_start_ping(ob_bus_t *b, uint8_t id, int timeout_ticks);

// SYNC WRITE: one broadcast packet, same register on N servos, no
// reply by protocol. ids/data laid out as in st3215.py::sync_write.
int ob_bus_start_sync_write(ob_bus_t *b, uint8_t reg, uint8_t data_len,
                            const uint8_t *ids,
                            const uint8_t *data /* n*data_len */,
                            uint8_t n);

// Advance the in-flight transaction by one tick: pull RX bytes,
// validate framing/checksum when complete, count down the deadline.
// Call exactly once per hard tick. Safe to call in every state
// (IDLE/DONE/... are no-ops).
void ob_bus_poll(ob_bus_t *b);

// Consume a terminal state (DONE/TIMEOUT/BAD_REPLY) back to IDLE.
// Returns the state it consumed, or OB_BUS_IDLE if nothing to
// consume. Payload of a DONE read is copied to ``out`` (may be NULL
// to discard); returns via *out_len how many bytes were written.
ob_bus_state_t ob_bus_take_result(ob_bus_t *b, uint8_t *out,
                                  uint8_t *out_len);

// The SCS checksum over ``body`` (id..params), byte-identical to
// st3215.py::_checksum. Exposed for tests.
uint8_t ob_bus_checksum(const uint8_t *body, size_t len);
