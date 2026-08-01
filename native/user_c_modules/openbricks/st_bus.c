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
#include "py/mphal.h"

#include "st_bus_core.h"
#include "st_servo_core.h"

#include <string.h>

// Hard-tick pump entry, called from motor_process's dispatcher.
// Prototype here (not a header) because it's the single cross-file
// symbol of this module and the coverage build runs with
// -Werror=missing-prototypes.
void ob_st_bus_hard_poll(void);

#if defined(MICROPY_OPENBRICKS_BUS_UART) && MICROPY_OPENBRICKS_BUS_UART
#include <stdatomic.h>

// IDF UART shims from the esp32-openbricks-bus-uart patch (port
// land; this module cannot include IDF headers).
extern int ob_bus_uart_open(int uart_num, int baud, int tx_pin, int rx_pin);
extern int ob_bus_uart_tx(int uart_num, const uint8_t *buf, size_t len);
extern int ob_bus_uart_rx(int uart_num, uint8_t *buf, size_t maxlen);
extern void ob_bus_uart_flush_rx(int uart_num);
#endif

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

// ---- servo slots + the shared bus pump ----
//
// One iteration of the drive loop: poll the in-flight transaction,
// route a finished result to whoever started it, and — when the bus
// is idle — start the planner's next op. The SAME function runs from
// the firmware hard tick (1 kHz, esp_timer task) and from the Python
// ``servo_pump()`` binding (unix tests drive it against the test
// rings), so the whole scheduler is provable off-hardware.
//
// Ownership rule: the pump only consumes results of transactions IT
// started (``tick_txn``); results of manually started transactions
// (start_ping & co) are left for Python's take_result. Manual
// traffic and attached servos share the bus politely, but mixing
// them mid-drive is a probe-mode-vs-drive-mode smell.
static ob_sservo_t sservo;
static bool sservo_inited;
static uint8_t tick_txn;        // pump-started transaction in flight
static uint8_t tick_txn_is_read;

static ob_sservo_t *sservo_get(void) {
    if (!sservo_inited) {
        ob_sservo_init(&sservo);
        sservo_inited = true;
    }
    return &sservo;
}

static void servo_pump_locked(ob_bus_t *b) {
    ob_bus_poll(b);
    if (tick_txn
        && (b->state == OB_BUS_DONE || b->state == OB_BUS_TIMEOUT
            || b->state == OB_BUS_BAD_REPLY)) {
        uint8_t payload[OB_BUS_MAX_READ];
        uint8_t plen = 0;
        ob_bus_state_t st = ob_bus_take_result(b, payload, &plen);
        if (tick_txn_is_read) {
            ob_sservo_read_result(sservo_get(), st == OB_BUS_DONE,
                                  payload, plen);
        }
        tick_txn = 0;
        tick_txn_is_read = 0;
    }
    if (b->state != OB_BUS_IDLE) {
        return;
    }
    ob_sservo_op_t op;
    ob_sservo_next_op(sservo_get(), &op);
    int started = 0;
    switch (op.kind) {
        case OB_SOP_WRITE:
            started = (ob_bus_start_write(b, op.id, op.reg,
                                          op.data, op.data_len) == 0);
            break;
        case OB_SOP_SYNC_SPEED:
            started = (ob_bus_start_sync_write(b, OB_SREG_GOAL_SPEED, 2,
                                               op.sync_ids, op.sync_data,
                                               op.sync_n) == 0);
            break;
        case OB_SOP_READ_POS:
            started = (ob_bus_start_read(b, op.id, OB_SREG_PRESENT_POS, 2,
                                         OB_SSERVO_READ_TICKS) == 0);
            break;
        default:
            return;
    }
    if (started) {
        tick_txn = 1;
        tick_txn_is_read = (op.kind == OB_SOP_READ_POS);
        ob_sservo_op_started(sservo_get(), &op);
    }
}

// ---- Python-facing methods (module-singleton style, like
//      motor_process: self argument ignored) ----

#if defined(MICROPY_OPENBRICKS_BUS_UART) && MICROPY_OPENBRICKS_BUS_UART
// ---- firmware I/O backend: the real UART via the patch shims ----
//
// The bus state machine is touched from TWO contexts once attached:
// Python bindings (main task) start transactions and take results;
// the hard tick (esp_timer task) polls. A C11 spinlock serializes
// them — every critical section is microseconds (memcpy + ring ops,
// no waiting inside), so spinning is cheaper than any handoff.
static atomic_flag bus_lock = ATOMIC_FLAG_INIT;
static int  fw_uart_num = -1;

static void bus_take(void)    { while (atomic_flag_test_and_set(&bus_lock)) { } }
static void bus_release(void) { atomic_flag_clear(&bus_lock); }

static int fw_tx(void *ctx, const uint8_t *buf, size_t len) {
    (void)ctx;
    return ob_bus_uart_tx(fw_uart_num, buf, len);
}
static int fw_rx(void *ctx, uint8_t *buf, size_t maxlen) {
    (void)ctx;
    return ob_bus_uart_rx(fw_uart_num, buf, maxlen);
}
static void fw_rx_flush(void *ctx) {
    (void)ctx;
    ob_bus_uart_flush_rx(fw_uart_num);
}

// Called from motor_process's hard-tick dispatcher, esp_timer task
// context: poll the in-flight transaction. Pure C throughout.
void ob_st_bus_hard_poll(void) {
    if (fw_uart_num < 0) {
        return;
    }
    bus_take();
    servo_pump_locked(&test_bus);
    bus_release();
}
#else
// No firmware UART on this build: the pump entry still exists so
// motor_process's dispatcher links everywhere, but does nothing.
static void bus_take(void)    { }
static void bus_release(void) { }
void ob_st_bus_hard_poll(void) { }
#endif

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
    sservo_inited = false;
    tick_txn = 0;
    tick_txn_is_read = 0;
    bus_get();
    sservo_get();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_test_reset_obj, sb_test_reset);

static mp_obj_t sb_start_read(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    bus_take();
    int r = ob_bus_start_read(bus_get(),
                              (uint8_t)mp_obj_get_int(args[1]),
                              (uint8_t)mp_obj_get_int(args[2]),
                              (uint8_t)mp_obj_get_int(args[3]),
                              mp_obj_get_int(args[4]));
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_start_read_obj, 5, 5, sb_start_read);

static mp_obj_t sb_start_write(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    mp_buffer_info_t data;
    mp_get_buffer_raise(args[3], &data, MP_BUFFER_READ);
    bus_take();
    int r = ob_bus_start_write(bus_get(),
                               (uint8_t)mp_obj_get_int(args[1]),
                               (uint8_t)mp_obj_get_int(args[2]),
                               data.buf, (uint8_t)data.len);
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_start_write_obj, 4, 4, sb_start_write);

static mp_obj_t sb_start_ping(mp_obj_t self_in, mp_obj_t id_in, mp_obj_t t_in) {
    (void)self_in;
    bus_take();
    int r = ob_bus_start_ping(bus_get(), (uint8_t)mp_obj_get_int(id_in),
                              mp_obj_get_int(t_in));
    bus_release();
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
    bus_take();
    int r = ob_bus_start_sync_write(bus_get(), reg, dlen, ids, data,
                                    (uint8_t)n);
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_start_sync_write_obj, 4, 4, sb_start_sync_write);

static mp_obj_t sb_poll(mp_obj_t self_in) {
    (void)self_in;
    bus_take();
    ob_bus_poll(bus_get());
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_poll_obj, sb_poll);

static mp_obj_t sb_state(mp_obj_t self_in) {
    (void)self_in;
    bus_take();
    int st = (int)bus_get()->state;
    bus_release();
    return MP_OBJ_NEW_SMALL_INT(st);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_state_obj, sb_state);

static mp_obj_t sb_take_result(mp_obj_t self_in) {
    (void)self_in;
    uint8_t payload[OB_BUS_MAX_READ];
    uint8_t plen = 0;
    bus_take();
    ob_bus_state_t s = ob_bus_take_result(bus_get(), payload, &plen);
    bus_release();
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

#if defined(MICROPY_OPENBRICKS_BUS_UART) && MICROPY_OPENBRICKS_BUS_UART
static mp_obj_t sb_attach_uart(size_t n_args, const mp_obj_t *args) {
    // (self, uart_id, baud, tx, rx) — open the real UART through the
    // patch shims and point the bus at it. From here the hard tick
    // polls; Python starts transactions and takes results.
    (void)n_args;
    int uart_num = mp_obj_get_int(args[1]);
    if (ob_bus_uart_open(uart_num, mp_obj_get_int(args[2]),
                         mp_obj_get_int(args[3]),
                         mp_obj_get_int(args[4])) != 0) {
        return mp_obj_new_bool(false);
    }
    // Post-open settle, learned twice now: the ESP32-S3 UART takes
    // ~10 ms to produce clean TX after (re)configuration, and the
    // first packets otherwise leave as malformed bits — the servo
    // stays silent and the URT-2 adapter SULKS until its rail is
    // power-cycled (bench-confirmed on first native-bus contact,
    // 1.40.0; st3215.py:122-131 documents the identical failure and
    // carries the identical sleep). Main-thread context here, so a
    // blocking delay is legal — this is setup, not the hot path.
    mp_hal_delay_ms(20);
    ob_bus_io_t io = {
        .tx = fw_tx, .rx = fw_rx, .rx_flush = fw_rx_flush, .ctx = NULL,
    };
    bus_take();
    ob_bus_init(&test_bus, io);
    test_inited = true;      // bus_get() must not re-init to test io
    fw_uart_num = uart_num;  // set LAST: hard poll keys off it
    bus_release();
    return mp_obj_new_bool(true);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_attach_uart_obj, 5, 5, sb_attach_uart);
#endif

// ---- servo-slot bindings ----

static mp_obj_t sb_servo_attach(size_t n_args, const mp_obj_t *args) {
    // (self, slot, id, invert, goal_acc)
    (void)n_args;
    bus_take();
    int r = ob_sservo_attach(sservo_get(), mp_obj_get_int(args[1]),
                             (uint8_t)mp_obj_get_int(args[2]),
                             mp_obj_is_true(args[3]) ? 1 : 0,
                             (uint8_t)mp_obj_get_int(args[4]));
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_servo_attach_obj, 5, 5, sb_servo_attach);

static mp_obj_t sb_servo_detach(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    bus_take();
    ob_sservo_detach(sservo_get(), mp_obj_get_int(slot_in));
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_detach_obj, sb_servo_detach);

static mp_obj_t sb_servo_run(mp_obj_t self_in, mp_obj_t slot_in,
                             mp_obj_t steps_in) {
    // Signed encoder steps/s (Python converts dps; core is int-only).
    (void)self_in;
    bus_take();
    int r = ob_sservo_set_speed(sservo_get(), mp_obj_get_int(slot_in),
                                (int32_t)mp_obj_get_int(steps_in));
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_3(sb_servo_run_obj, sb_servo_run);

static mp_obj_t sb_servo_coast(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    bus_take();
    int r = ob_sservo_coast(sservo_get(), mp_obj_get_int(slot_in));
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_coast_obj, sb_servo_coast);

static mp_obj_t sb_servo_counts(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    bus_take();
    int32_t c = ob_sservo_counts(sservo_get(), mp_obj_get_int(slot_in));
    bus_release();
    return mp_obj_new_int(c);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_counts_obj, sb_servo_counts);

static mp_obj_t sb_servo_stats(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    int slot = mp_obj_get_int(slot_in);
    bus_take();
    ob_sservo_t *sv = sservo_get();
    uint32_t ok = 0, failed = 0, stale = 0;
    if (slot >= 0 && slot < OB_SSERVO_SLOTS) {
        ok     = sv->slots[slot].reads_ok;
        failed = sv->slots[slot].reads_failed;
        stale  = sv->slots[slot].stale;
    }
    bus_release();
    mp_obj_t t[3] = {
        mp_obj_new_int_from_uint(ok),
        mp_obj_new_int_from_uint(failed),
        mp_obj_new_int_from_uint(stale),
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_stats_obj, sb_servo_stats);

static mp_obj_t sb_servo_pump(mp_obj_t self_in) {
    // One pump iteration — the hard tick's exact code path, callable
    // from Python. On firmware with the tick running this is a
    // harmless extra iteration; on unix it IS the drive loop, which
    // is how the scheduler is tested off-hardware.
    (void)self_in;
    bus_take();
    servo_pump_locked(bus_get());
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_servo_pump_obj, sb_servo_pump);

static mp_obj_t sb_servo_encode(mp_obj_t self_in, mp_obj_t steps_in) {
    (void)self_in;
    return mp_obj_new_int(
        ob_sservo_encode_speed((int32_t)mp_obj_get_int(steps_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_encode_obj, sb_servo_encode);

static mp_obj_t sb_torque_off_all(mp_obj_t self_in) {
    // E-stop path: broadcast torque-off NOW, jumping any in-flight
    // transaction (an emergency stop must not queue behind feedback
    // reads). Consuming whatever was in flight is acceptable damage:
    // the pump's next iteration recovers, and the stale-RX flush
    // before TX (st3215.py's drain rule) re-frames the bus.
    (void)self_in;
    uint8_t off = 0;
    bus_take();
    ob_bus_t *b = bus_get();
    if (b->state == OB_BUS_AWAIT_REPLY) {
        b->state = OB_BUS_IDLE;   // abandon; flush-before-TX re-frames
        tick_txn = 0;
        tick_txn_is_read = 0;
    }
    int r = ob_bus_start_write(b, 0xFE, OB_SREG_TORQUE, &off, 1);
    // Broadcast completes immediately; consume so the pump can go on.
    if (r == 0) {
        ob_bus_take_result(b, NULL, NULL);
    }
    // Void every staged command so nothing re-drives the motors.
    ob_sservo_t *sv = sservo_get();
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        if (sv->slots[i].in_use) {
            sv->slots[i].target_dirty = 0;
            sv->slots[i].torque_cmd = -1;
        }
    }
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_torque_off_all_obj, sb_torque_off_all);

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
    { MP_ROM_QSTR(MP_QSTR_servo_attach),     MP_ROM_PTR(&sb_servo_attach_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_detach),     MP_ROM_PTR(&sb_servo_detach_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_run),        MP_ROM_PTR(&sb_servo_run_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_coast),      MP_ROM_PTR(&sb_servo_coast_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_counts),     MP_ROM_PTR(&sb_servo_counts_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_stats),      MP_ROM_PTR(&sb_servo_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_pump),       MP_ROM_PTR(&sb_servo_pump_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_encode),     MP_ROM_PTR(&sb_servo_encode_obj) },
    { MP_ROM_QSTR(MP_QSTR_torque_off_all),   MP_ROM_PTR(&sb_torque_off_all_obj) },
    #if defined(MICROPY_OPENBRICKS_BUS_UART) && MICROPY_OPENBRICKS_BUS_UART
    { MP_ROM_QSTR(MP_QSTR_attach_uart),      MP_ROM_PTR(&sb_attach_uart_obj) },
    #endif
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
