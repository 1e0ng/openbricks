// SPDX-License-Identifier: MIT
// Native tests for st_bus_core — SCS protocol + transaction machine.
// Edge cases the Python-driven suite can't set up directly, under
// ASan/UBSan.

#include "harness.h"
#include "st_bus_core.h"

// ---- fake io: byte-exact capture + scripted rx ----

static uint8_t io_tx[512];
static size_t  io_tx_len;
static uint8_t io_rx[64];
static size_t  io_rx_len, io_rx_pos;
static int     io_refuse_tx;
static int     io_flushes;

static int f_tx(void *ctx, const uint8_t *buf, size_t len) {
    (void)ctx;
    if (io_refuse_tx) {
        return 0;
    }
    memcpy(&io_tx[io_tx_len], buf, len);
    io_tx_len += len;
    return (int)len;
}
static int f_rx(void *ctx, uint8_t *buf, size_t maxlen) {
    (void)ctx;
    size_t avail = io_rx_len - io_rx_pos;
    size_t n = avail < maxlen ? avail : maxlen;
    memcpy(buf, &io_rx[io_rx_pos], n);
    io_rx_pos += n;
    return (int)n;
}
static void f_flush(void *ctx) {
    (void)ctx;
    io_rx_len = io_rx_pos = 0;
    io_flushes++;
}

static ob_bus_t bus;

static void io_reset(void) {
    io_tx_len = 0;
    io_rx_len = io_rx_pos = 0;
    io_refuse_tx = 0;
    io_flushes = 0;
    ob_bus_io_t io = { f_tx, f_rx, f_flush, NULL };
    ob_bus_init(&bus, io);
}

static void feed_reply(uint8_t id, uint8_t err,
                       const uint8_t *payload, uint8_t plen) {
    uint8_t body[3 + 8];
    body[0] = id;
    body[1] = (uint8_t)(plen + 2);
    body[2] = err;
    memcpy(&body[3], payload, plen);
    io_rx[io_rx_len++] = 0xFF;
    io_rx[io_rx_len++] = 0xFF;
    memcpy(&io_rx[io_rx_len], body, (size_t)(3 + plen));
    io_rx_len += (size_t)(3 + plen);
    io_rx[io_rx_len++] = ob_bus_checksum(body, (size_t)(3 + plen));
}

// ---- tests ----

TEST(checksum_overflow_bytes) {
    // Sums past 0xFF must wrap through the mask, not UB. 0xFF-heavy
    // bodies are real on this bus (broadcast id 0xFE, big params).
    uint8_t body[] = { 0xFE, 0xFF, 0xFF, 0xFF, 0xFF };
    unsigned s = 0xFE + 0xFF * 4;
    CHECK_EQ_INT(ob_bus_checksum(body, sizeof(body)),
                 (uint8_t)(~s) & 0xFF);
}

TEST(read_round_trip_payload_exact) {
    io_reset();
    CHECK_EQ_INT(ob_bus_start_read(&bus, 1, 0x38, 2, 5), 0);
    // TX bytes: FF FF 01 04 02 38 02 CHK — byte-golden.
    uint8_t expect_body[] = { 0x01, 0x04, 0x02, 0x38, 0x02 };
    CHECK_EQ_INT(io_tx_len, 8);
    CHECK(memcmp(io_tx, "\xff\xff", 2) == 0);
    CHECK(memcmp(&io_tx[2], expect_body, 5) == 0);
    CHECK_EQ_INT(io_tx[7], ob_bus_checksum(expect_body, 5));

    uint8_t pl[] = { 0xD2, 0x04 };
    feed_reply(1, 0, pl, 2);
    ob_bus_poll(&bus);
    uint8_t out[8];
    uint8_t out_len = 0;
    CHECK_EQ_INT(ob_bus_take_result(&bus, out, &out_len), OB_BUS_DONE);
    CHECK_EQ_INT(out_len, 2);
    CHECK(out[0] == 0xD2 && out[1] == 0x04);
    CHECK_EQ_INT(io_flushes, 1);          // drain-before-TX rule
}

TEST(timeout_is_exactly_budgeted) {
    io_reset();
    CHECK_EQ_INT(ob_bus_start_read(&bus, 1, 0x38, 2, 3), 0);
    ob_bus_poll(&bus);
    ob_bus_poll(&bus);
    CHECK_EQ_INT(bus.state, OB_BUS_AWAIT_REPLY);
    ob_bus_poll(&bus);
    CHECK_EQ_INT(bus.state, OB_BUS_TIMEOUT);
    CHECK_EQ_INT(bus.n_timeout, 1);
}

TEST(refused_tx_fails_visibly_not_wedged) {
    io_reset();
    io_refuse_tx = 1;
    CHECK_EQ_INT(ob_bus_start_ping(&bus, 1, 5), -1);
    CHECK_EQ_INT(bus.state, OB_BUS_IDLE);
    CHECK_EQ_INT(bus.n_bad, 1);
    io_refuse_tx = 0;
    CHECK_EQ_INT(ob_bus_start_ping(&bus, 1, 5), 0);   // usable again
}

TEST(wrong_id_and_bad_checksum_rejected) {
    io_reset();
    ob_bus_start_read(&bus, 1, 0x38, 2, 5);
    uint8_t pl[] = { 0x00, 0x00 };
    feed_reply(2, 0, pl, 2);              // wrong servo
    ob_bus_poll(&bus);
    CHECK_EQ_INT(ob_bus_take_result(&bus, NULL, NULL), OB_BUS_BAD_REPLY);

    ob_bus_start_read(&bus, 1, 0x38, 2, 5);
    feed_reply(1, 0, pl, 2);
    io_rx[io_rx_len - 1] ^= 0xFF;         // corrupt checksum
    ob_bus_poll(&bus);
    CHECK_EQ_INT(ob_bus_take_result(&bus, NULL, NULL), OB_BUS_BAD_REPLY);
    CHECK_EQ_INT(bus.n_bad, 2);
}

TEST(write_status_lost_is_not_an_error) {
    io_reset();
    uint8_t data[] = { 0x10, 0x27 };
    CHECK_EQ_INT(ob_bus_start_write(&bus, 1, 0x2A, data, 2), 0);
    for (int i = 0; i < 5; i++) {
        ob_bus_poll(&bus);
    }
    CHECK_EQ_INT(ob_bus_take_result(&bus, NULL, NULL), OB_BUS_DONE);
    CHECK_EQ_INT(bus.n_timeout, 0);       // half-duplex reality rule
}

TEST(sync_write_layout_and_broadcast_immediacy) {
    io_reset();
    uint8_t ids[] = { 1, 2 };
    uint8_t data[] = { 0x10, 0x27, 0xF0, 0xD8 };
    CHECK_EQ_INT(ob_bus_start_sync_write(&bus, 0x2A, 2, ids, data, 2), 0);
    CHECK_EQ_INT(bus.state, OB_BUS_DONE); // no reply by protocol
    // Body: FE LEN 83 2A 02 01 10 27 02 F0 D8
    uint8_t expect[] = { 0xFE, (1 + 2) * 2 + 4, 0x83, 0x2A, 0x02,
                         0x01, 0x10, 0x27, 0x02, 0xF0, 0xD8 };
    CHECK(memcmp(&io_tx[2], expect, sizeof(expect)) == 0);
}

TEST(guards_reject_bad_sizes) {
    io_reset();
    CHECK_EQ_INT(ob_bus_start_read(&bus, 1, 0x38, 0, 5), -1);
    CHECK_EQ_INT(ob_bus_start_read(&bus, 1, 0x38, OB_BUS_MAX_READ + 1, 5), -1);
    uint8_t d = 0;
    CHECK_EQ_INT(ob_bus_start_write(&bus, 1, 0x2A, &d, 0), -1);
    uint8_t ids[1] = { 1 };
    CHECK_EQ_INT(ob_bus_start_sync_write(&bus, 0x2A, 2, ids, &d, 0), -1);
    CHECK_EQ_INT(ob_bus_start_sync_write(&bus, 0x2A, 2, ids, &d,
                                         OB_BUS_MAX_SERVOS + 1), -1);
}

TEST(busy_bus_rejects_new_start) {
    io_reset();
    CHECK_EQ_INT(ob_bus_start_read(&bus, 1, 0x38, 2, 5), 0);
    CHECK_EQ_INT(ob_bus_start_ping(&bus, 2, 5), -1);
}

int main(void) {
    RUN(checksum_overflow_bytes);
    RUN(read_round_trip_payload_exact);
    RUN(timeout_is_exactly_budgeted);
    RUN(refused_tx_fails_visibly_not_wedged);
    RUN(wrong_id_and_bad_checksum_rejected);
    RUN(write_status_lost_is_not_an_error);
    RUN(sync_write_layout_and_broadcast_immediacy);
    RUN(guards_reject_bad_sizes);
    RUN(busy_bus_rejects_new_start);
    return harness_exit("st_bus_core");
}
