// SPDX-License-Identifier: MIT
//
// st_bus — MicroPython binding for st_bus_core, with a built-in TEST
// backend.
//
// What ships here on purpose: everything needed to exercise the SCS
// protocol core from Python with byte-exact fidelity — a captured TX
// stream and a feedable RX stream, both static C ring buffers. The
// unix test suite replays st3215.py's known-good packets against it
// (golden vectors) and simulates servo replies, so the protocol +
// transaction state machine are fully proven before any UART or hard
// tick is involved. The firmware I/O backend (IDF UART shims via a
// build-time patch) and the hard-tick pump arrive in the next arc PR
// and swap in through the same ob_bus_io_t seam.

#include "py/runtime.h"
#include "py/objstr.h"

#include "st_bus_core.h"

#include <string.h>

// ---- test I/O backend: static rings fed/drained from Python ----

#define TEST_TX_CAP 256
#define TEST_RX_CAP 64

typedef struct {
    uint8_t tx_buf[TEST_TX_CAP];
    size_t  tx_len;
    uint8_t rx_buf[TEST_RX_CAP];
    size_t  rx_len;
    size_t  rx_pos;
    uint32_t n_flush;
} test_io_t;

static test_io_t test_io;
static ob_bus_t  test_bus;
static bool      test_inited;

static int test_tx(void *ctx, const uint8_t *buf, size_t len) {
    test_io_t *io = ctx;
    if (io->tx_len + len > TEST_TX_CAP) {
        return 0;   // refuse — lets tests exercise the TX-refused path
    }
    memcpy(&io->tx_buf[io->tx_len], buf, len);
    io->tx_len += len;
    return (int)len;
}

static int test_rx(void *ctx, uint8_t *buf, size_t maxlen) {
    test_io_t *io = ctx;
    size_t avail = io->rx_len - io->rx_pos;
    size_t n = (avail < maxlen) ? avail : maxlen;
    if (n > 0) {
        memcpy(buf, &io->rx_buf[io->rx_pos], n);
        io->rx_pos += n;
    }
    return (int)n;
}

static void test_rx_flush(void *ctx) {
    test_io_t *io = ctx;
    io->rx_len = 0;
    io->rx_pos = 0;
    io->n_flush++;
}

// ---- Python-facing methods (module-singleton style, like
//      motor_process: self argument ignored) ----

static ob_bus_t *bus_get(void) {
    if (!test_inited) {
        ob_bus_io_t io = {
            .tx = test_tx, .rx = test_rx,
            .rx_flush = test_rx_flush, .ctx = &test_io,
        };
        memset(&test_io, 0, sizeof(test_io));
        ob_bus_init(&test_bus, io);
        test_inited = true;
    }
    return &test_bus;
}

static mp_obj_t sb_test_reset(mp_obj_t self_in) {
    (void)self_in;
    test_inited = false;
    bus_get();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_test_reset_obj, sb_test_reset);

static mp_obj_t sb_start_read(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    int r = ob_bus_start_read(bus_get(),
                              (uint8_t)mp_obj_get_int(args[1]),
                              (uint8_t)mp_obj_get_int(args[2]),
                              (uint8_t)mp_obj_get_int(args[3]),
                              mp_obj_get_int(args[4]));
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_start_read_obj, 5, 5, sb_start_read);

static mp_obj_t sb_start_write(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    mp_buffer_info_t data;
    mp_get_buffer_raise(args[3], &data, MP_BUFFER_READ);
    int r = ob_bus_start_write(bus_get(),
                               (uint8_t)mp_obj_get_int(args[1]),
                               (uint8_t)mp_obj_get_int(args[2]),
                               data.buf, (uint8_t)data.len);
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_start_write_obj, 4, 4, sb_start_write);

static mp_obj_t sb_start_ping(mp_obj_t self_in, mp_obj_t id_in, mp_obj_t t_in) {
    (void)self_in;
    int r = ob_bus_start_ping(bus_get(), (uint8_t)mp_obj_get_int(id_in),
                              mp_obj_get_int(t_in));
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_3(sb_start_ping_obj, sb_start_ping);

static mp_obj_t sb_start_sync_write(size_t n_args, const mp_obj_t *args) {
    // (self, reg, data_len, [(id, bytes), ...])
    (void)n_args;
    uint8_t reg = (uint8_t)mp_obj_get_int(args[1]);
    uint8_t dlen = (uint8_t)mp_obj_get_int(args[2]);
    size_t n;
    mp_obj_t *items;
    mp_obj_get_array(args[3], &n, &items);
    if (n == 0 || n > OB_BUS_MAX_SERVOS) {
        return mp_obj_new_bool(false);
    }
    uint8_t ids[OB_BUS_MAX_SERVOS];
    uint8_t data[OB_BUS_MAX_SERVOS * OB_BUS_MAX_READ];
    for (size_t i = 0; i < n; i++) {
        size_t two;
        mp_obj_t *pair;
        mp_obj_get_array(items[i], &two, &pair);
        if (two != 2) {
            return mp_obj_new_bool(false);
        }
        ids[i] = (uint8_t)mp_obj_get_int(pair[0]);
        mp_buffer_info_t d;
        mp_get_buffer_raise(pair[1], &d, MP_BUFFER_READ);
        if (d.len != dlen || dlen > OB_BUS_MAX_READ) {
            return mp_obj_new_bool(false);
        }
        memcpy(&data[i * dlen], d.buf, dlen);
    }
    int r = ob_bus_start_sync_write(bus_get(), reg, dlen, ids, data,
                                    (uint8_t)n);
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_start_sync_write_obj, 4, 4, sb_start_sync_write);

static mp_obj_t sb_poll(mp_obj_t self_in) {
    (void)self_in;
    ob_bus_poll(bus_get());
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_poll_obj, sb_poll);

static mp_obj_t sb_state(mp_obj_t self_in) {
    (void)self_in;
    return MP_OBJ_NEW_SMALL_INT((int)bus_get()->state);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_state_obj, sb_state);

static mp_obj_t sb_take_result(mp_obj_t self_in) {
    (void)self_in;
    uint8_t payload[OB_BUS_MAX_READ];
    uint8_t plen = 0;
    ob_bus_state_t s = ob_bus_take_result(bus_get(), payload, &plen);
    mp_obj_t tuple[2] = {
        MP_OBJ_NEW_SMALL_INT((int)s),
        mp_obj_new_bytes(payload, plen),
    };
    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_take_result_obj, sb_take_result);

static mp_obj_t sb_take_tx(mp_obj_t self_in) {
    (void)self_in;
    (void)bus_get();
    mp_obj_t out = mp_obj_new_bytes(test_io.tx_buf, test_io.tx_len);
    test_io.tx_len = 0;
    return out;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_take_tx_obj, sb_take_tx);

static mp_obj_t sb_feed_rx(mp_obj_t self_in, mp_obj_t data_in) {
    (void)self_in;
    (void)bus_get();
    mp_buffer_info_t d;
    mp_get_buffer_raise(data_in, &d, MP_BUFFER_READ);
    if (test_io.rx_len + d.len > TEST_RX_CAP) {
        return mp_obj_new_bool(false);
    }
    memcpy(&test_io.rx_buf[test_io.rx_len], d.buf, d.len);
    test_io.rx_len += d.len;
    return mp_obj_new_bool(true);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_feed_rx_obj, sb_feed_rx);

static mp_obj_t sb_stats(mp_obj_t self_in) {
    (void)self_in;
    ob_bus_t *b = bus_get();
    mp_obj_t tuple[4] = {
        mp_obj_new_int_from_uint(b->n_ok),
        mp_obj_new_int_from_uint(b->n_timeout),
        mp_obj_new_int_from_uint(b->n_bad),
        mp_obj_new_int_from_uint(test_io.n_flush),
    };
    return mp_obj_new_tuple(4, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_stats_obj, sb_stats);

// ---- module-singleton plumbing ----

typedef struct {
    mp_obj_base_t base;
} st_bus_obj_t;

static const mp_rom_map_elem_t st_bus_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_test_reset),       MP_ROM_PTR(&sb_test_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_read),       MP_ROM_PTR(&sb_start_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_write),      MP_ROM_PTR(&sb_start_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_ping),       MP_ROM_PTR(&sb_start_ping_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_sync_write), MP_ROM_PTR(&sb_start_sync_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_poll),             MP_ROM_PTR(&sb_poll_obj) },
    { MP_ROM_QSTR(MP_QSTR_state),            MP_ROM_PTR(&sb_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_take_result),      MP_ROM_PTR(&sb_take_result_obj) },
    { MP_ROM_QSTR(MP_QSTR_take_tx),          MP_ROM_PTR(&sb_take_tx_obj) },
    { MP_ROM_QSTR(MP_QSTR_feed_rx),          MP_ROM_PTR(&sb_feed_rx_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),            MP_ROM_PTR(&sb_stats_obj) },
    // States as constants so tests read like the header enum.
    { MP_ROM_QSTR(MP_QSTR_IDLE),        MP_ROM_INT(OB_BUS_IDLE) },
    { MP_ROM_QSTR(MP_QSTR_AWAIT_REPLY), MP_ROM_INT(OB_BUS_AWAIT_REPLY) },
    { MP_ROM_QSTR(MP_QSTR_DONE),        MP_ROM_INT(OB_BUS_DONE) },
    { MP_ROM_QSTR(MP_QSTR_TIMEOUT),     MP_ROM_INT(OB_BUS_TIMEOUT) },
    { MP_ROM_QSTR(MP_QSTR_BAD_REPLY),   MP_ROM_INT(OB_BUS_BAD_REPLY) },
};
static MP_DEFINE_CONST_DICT(st_bus_locals_dict, st_bus_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    st_bus_type,
    MP_QSTR_st_bus,
    MP_TYPE_FLAG_NONE,
    locals_dict, &st_bus_locals_dict
    );

const mp_obj_base_t st_bus_singleton = { &st_bus_type };
