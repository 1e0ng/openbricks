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
#include "st_move_core.h"
#include "drivebase_core.h"
#include "motor_process.h"

#include <string.h>

// Hard-tick pump entry, called from motor_process's dispatcher.
// Prototype here (not a header) because it's the single cross-file
// symbol of this module and the coverage build runs with
// -Werror=missing-prototypes.
void ob_st_bus_hard_poll(void);
void ob_st_bus_estop_from_tick(void);

#if defined(MICROPY_OPENBRICKS_BUS_UART) && MICROPY_OPENBRICKS_BUS_UART
#include <stdatomic.h>

// IDF UART shims from the esp32-openbricks-bus-uart patch (port
// land; this module cannot include IDF headers).
extern uint32_t ob_hard_ticks_ms(void);   // hard-tick patch clock
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
static uint8_t tick_txn_is_swrite;  // single-servo write (config) —
                                    // its result must reach
                                    // ob_sservo_write_result, and ONLY
                                    // its result: routing a sync's
                                    // completion there would
                                    // misattribute it to whatever
                                    // write_in_flight last held

static ob_sservo_t *sservo_get(void) {
    if (!sservo_inited) {
        ob_sservo_init(&sservo);
        sservo_inited = true;
    }
    return &sservo;
}

// ---- native 2-DOF drivebase on serial-bus slots ----
//
// The existing, hardware-proven drivebase_core controller drives
// serial-bus wheels through two BRIDGE servo structs: the controller
// only ever touches observer.pos_hat (read) and target_dps (write)
// — verified against drivebase_core.c — so the pump syncs slot
// odometry into pos_hat before the tick and slot speed targets out
// of target_dps after it. Zero churn to the controller; the same
// math that passed the encoder path's asymmetric-friction gate runs
// here, at the ~490 Hz feedback rate the wire test measured.
#define ST_DB_DEG_PER_COUNT (360.0 / 4096.0)
#define ST_DB_STEPS_PER_DEG (4096.0 / 360.0)

static ob_servo_t st_db_bridge_l, st_db_bridge_r;
static ob_drivebase_t st_db;
static bool st_db_active;
// Arbitration between the drivebase and per-slot moves (1.46.0):
// the db writes its slots' speed targets only while it OWNS a move
// — from db_straight/db_turn until db_stop/db_disable. Yielded, it
// keeps its odometry synced (so the next move arms from the true
// pose) but stays silent, which is what lets an adopted motor's
// run_angle/hold — and plain run_speed after a stop — own the wheel.
// Pre-1.46.0 the configured db wrote hold targets forever, which
// also torqued the wheels stiff from the moment of construction.
static bool st_db_writing;
// Heading source for gyro mode: 0 = Python override (db_set_heading,
// the BNO055 pump), 1 = the hard-tick yaw integrator
// (openbricks_hard_yaw_deg — the raw-IMU/ICM-45686 path). With
// source 1 the db tick pulls heading EVERY millisecond in C; the
// reference is captured at source selection (enable-is-the-frame
// rule, same as the Python engine's).
static uint8_t st_db_gyro_source;
static ob_float_t st_db_yaw_ref;
// Dead-wheel fault latch. A wheel that stops answering feedback
// reads is CATASTROPHIC for the coupled controller, not merely
// invisible: its odometry freezes, so the diff error grows without
// bound and the P term winds that wheel's command toward the rail
// (bench-reproduced: 8468 steps/s commanded on the silent wheel
// while the live one sat at 6). If the motor is alive but simply
// not REPORTING — a broken feedback line, the classic failure — the
// robot takes off. So the tick stops driving as soon as a wheel
// goes quiet, and latches which one for Python to report.
//
// Threshold is in consecutive failed reads. Each failed read costs
// OB_SSERVO_READ_TICKS (5) pump iterations and the two wheels share
// the round-robin, so 20 is roughly 200 ms of silence at the 1 kHz
// tick — long enough to ride out bus noise, short enough that a
// runaway never gets going.
#define ST_DB_STALE_FAULT 20
#define ST_DB_FAULT_LEFT  0x01
#define ST_DB_FAULT_RIGHT 0x02
static uint8_t st_db_fault;
static int st_db_slot_l = -1, st_db_slot_r = -1;
static uint32_t st_db_now_ms;      // clock fed by the pump caller

// Per-slot position moves (run_angle / hold on adopted motors).
static ob_smove_t st_moves[OB_SSERVO_SLOTS];

static void st_moves_reset_all(void) {
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        ob_smove_init(&st_moves[i]);
    }
}

static void st_db_sync_bridges_locked(void) {
    ob_sservo_t *sv = sservo_get();
    // Slot counts -> wheel degrees. ob_sservo_counts applies the
    // slot's invert, so both bridges live in the same wheel frame the
    // encoder path uses (frame-consistency tests pin this).
    st_db_bridge_l.observer.pos_hat =
        (ob_float_t)(ob_sservo_counts(sv, st_db_slot_l) * ST_DB_DEG_PER_COUNT);
    st_db_bridge_r.observer.pos_hat =
        (ob_float_t)(ob_sservo_counts(sv, st_db_slot_r) * ST_DB_DEG_PER_COUNT);
}

static void st_db_tick_locked(void) {
    if (!st_db_active) {
        return;
    }
    st_db_sync_bridges_locked();
    if (!st_db_writing) {
        return;                    // configured but yielded
    }
    ob_sservo_t *sv = sservo_get();
    // Silent-wheel guard, BEFORE the control law: a frozen odometry
    // reading would otherwise wind that wheel's command to the rail.
    // A config_failed wheel is the attach-time flavour of the same
    // death: it never gets feedback reads AT ALL (the planner skips
    // unconfigured slots), so its stale counter would sit at 0
    // forever — fault it immediately, don't wait for a climb that
    // cannot happen.
    if (sv->slots[st_db_slot_l].stale >= ST_DB_STALE_FAULT
        || sv->slots[st_db_slot_l].config_failed) {
        st_db_fault |= ST_DB_FAULT_LEFT;
    }
    if (sv->slots[st_db_slot_r].stale >= ST_DB_STALE_FAULT
        || sv->slots[st_db_slot_r].config_failed) {
        st_db_fault |= ST_DB_FAULT_RIGHT;
    }
    if (st_db_fault) {
        ob_sservo_set_speed(sv, st_db_slot_l, 0);
        ob_sservo_set_speed(sv, st_db_slot_r, 0);
        ob_drivebase_stop(&st_db);
        st_db_writing = false;     // yield; Python raises on the fault
        return;
    }
    if (st_db.use_gyro && st_db_gyro_source == 1) {
        st_db.heading_override_wheel_deg = ob_drivebase_body_to_wheel_diff(
            &st_db,
            (ob_float_t)openbricks_hard_yaw_deg() - st_db_yaw_ref);
    }
    ob_drivebase_tick(&st_db, (long)st_db_now_ms);
    // Wheel-degree targets -> slot steps/s. set_speed re-applies the
    // slot invert, mirroring how counts un-applied it above.
    ob_sservo_set_speed(sv, st_db_slot_l,
        (int32_t)(st_db_bridge_l.target_dps * ST_DB_STEPS_PER_DEG));
    ob_sservo_set_speed(sv, st_db_slot_r,
        (int32_t)(st_db_bridge_r.target_dps * ST_DB_STEPS_PER_DEG));
}

static void st_moves_tick_locked(void) {
    ob_sservo_t *sv = sservo_get();
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        if (!sv->slots[i].in_use || st_moves[i].state == OB_SMOVE_IDLE) {
            continue;
        }
        // Dead-feedback guard, same rule as the drivebase's
        // ST_DB_STALE_FAULT: feeding the profile a FROZEN position
        // winds ff + kp*err toward the rail at rising speed for as
        // long as Python's ~1 s stall detector takes to notice — the
        // db path bounded this at ~200 ms and called it mandatory;
        // the per-slot moves relied on Python alone. Halt the move
        // and zero the wheel in C; Python's stall/dead-bus check
        // then names the fault from the stats.
        if (sv->slots[i].stale >= ST_DB_STALE_FAULT) {
            ob_smove_init(&st_moves[i]);
            ob_sservo_set_speed(sv, i, 0);
            continue;
        }
        ob_float_t cmd = ob_smove_tick(&st_moves[i], (long)st_db_now_ms,
                                       (ob_float_t)ob_sservo_counts(sv, i));
        ob_sservo_set_speed(sv, i, (int32_t)cmd);
    }
}

// Which slots are DRIVING right now. Only st_bus knows: the servo
// core can't see per-slot moves or who the drivebase owns.
static void st_mark_hot_locked(void) {
    ob_sservo_t *sv = sservo_get();
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        uint8_t hot = 0;
        if (st_moves[i].state != OB_SMOVE_IDLE) {
            hot = 1;                       // run_angle / hold in flight
        } else if (st_db_active && st_db_writing
                   && (i == st_db_slot_l || i == st_db_slot_r)) {
            hot = 1;                       // a drivebase move is running
        } else if (sv->slots[i].target_steps != 0) {
            hot = 1;                       // commanded to keep turning
        }
        sv->slots[i].hot = hot;
    }
}

static void servo_pump_locked(ob_bus_t *b) {
    st_db_tick_locked();
    st_mark_hot_locked();
    st_moves_tick_locked();
    ob_bus_poll(b);
    if (tick_txn
        && (b->state == OB_BUS_DONE || b->state == OB_BUS_TIMEOUT
            || b->state == OB_BUS_BAD_REPLY
            || b->state == OB_BUS_SERVO_ERR)) {
        uint8_t payload[OB_BUS_MAX_READ];
        uint8_t plen = 0;
        ob_bus_state_t st = ob_bus_take_result(b, payload, &plen);
        if (tick_txn_is_read) {
            ob_sservo_read_result(sservo_get(), st == OB_BUS_DONE,
                                  payload, plen);
        } else if (tick_txn_is_swrite) {
            ob_sservo_write_result(sservo_get(), st == OB_BUS_DONE);
        }
        tick_txn = 0;
        tick_txn_is_read = 0;
        tick_txn_is_swrite = 0;
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
        case OB_SOP_SYNC_TORQUE:
            started = (ob_bus_start_sync_write(b, OB_SREG_TORQUE, 1,
                                               op.sync_ids, op.sync_data,
                                               op.sync_n) == 0);
            break;
        case OB_SOP_READ_POS:
            started = (ob_bus_start_read(b, op.id, OB_SREG_PRESENT_POS,
                                         OB_SSERVO_FEEDBACK_LEN,
                                         OB_SSERVO_READ_TICKS) == 0);
            break;
        default:
            return;
    }
    if (started) {
        tick_txn = 1;
        tick_txn_is_read = (op.kind == OB_SOP_READ_POS);
        tick_txn_is_swrite = (op.kind == OB_SOP_WRITE);
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
// E-stop from the hard tick itself (the hard button's stop path):
// abandon any in-flight transaction, broadcast torque-off, void all
// staged speeds. Pure C, same task as the pump (dispatcher calls
// this BEFORE the pump), so the spinlock nests sequentially.
void ob_st_bus_estop_from_tick(void) {
    if (fw_uart_num < 0) {
        return;
    }
    bus_take();
    ob_bus_t *b = &test_bus;
    if (b->state == OB_BUS_AWAIT_REPLY) {
        b->state = OB_BUS_IDLE;
        tick_txn = 0;
        tick_txn_is_read = 0;
        tick_txn_is_swrite = 0;
        // The abandoned transaction's slot must not soak up the NEXT
        // single-write's result.
        sservo_get()->write_in_flight = -1;
    }
    uint8_t off = 0;
    if (ob_bus_start_write(b, 0xFE, OB_SREG_TORQUE, &off, 1) == 0) {
        ob_bus_take_result(b, NULL, NULL);
    }
    st_db_active = false;
    st_db_writing = false;
    // st_db_fault deliberately NOT cleared: a latched dead-wheel
    // diagnosis must survive the button stop that follows it, or a
    // post-mortem db_fault() reads healthy (diagnostics must not
    // destroy evidence). The next db_straight/db_turn clears it.
    st_moves_reset_all();
    ob_sservo_t *sv = sservo_get();
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        if (sv->slots[i].in_use) {
            sv->slots[i].target_dirty = 0;
            sv->slots[i].torque_cmd = -1;
            sv->slots[i].torque_on = 0;
        }
    }
    bus_release();
}

void ob_st_bus_hard_poll(void) {
    if (fw_uart_num < 0) {
        return;
    }
    bus_take();
    st_db_now_ms = ob_hard_ticks_ms();
    servo_pump_locked(&test_bus);
    bus_release();
}
#else
// No firmware UART on this build: the pump entry still exists so
// motor_process's dispatcher links everywhere, but does nothing.
static void bus_take(void)    { }
static void bus_release(void) { }
void ob_st_bus_hard_poll(void) { }
void ob_st_bus_estop_from_tick(void) { }
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
    st_db_gyro_source = 0;
    st_db_active = false;
    st_db_writing = false;
    st_db_fault = 0;
    st_moves_reset_all();
    st_db_slot_l = st_db_slot_r = -1;
    st_db_now_ms = 0;
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

// uart_num() -> the UART the native bus has taken over, or -1.
//
// This is the authority on "who owns those pins". A servo driver that
// opened its own machine.UART on a natively-owned bus would put two
// drivers on one wire: the hard tick's replies land in the other
// driver's RX buffer and get consumed as the wrong packet's answer
// (bench 2026-08-04 — a write to id 4 acknowledged by id 1). It has
// to be asked in C rather than remembered in Python, because the
// attached UART deliberately SURVIVES a program boundary while
// reset_runtime clears the slots: a fresh program's Python state
// would say "nobody owns it" while the IDF driver still does.
static mp_obj_t sb_uart_num(mp_obj_t self_in) {
    (void)self_in;
#if defined(MICROPY_OPENBRICKS_BUS_UART) && MICROPY_OPENBRICKS_BUS_UART
    return mp_obj_new_int(fw_uart_num);
#else
    return mp_obj_new_int(-1);   // test backend owns no real UART
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_uart_num_obj, sb_uart_num);

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

// slot_of(servo_id) -> the slot already driving that servo, or -1.
//
// One physical servo, one slot. Without this, re-running a script in
// the same boot (or building a second engine) would claim a SECOND
// slot for a servo that already has one, and a 4-motor robot on 4
// slots would run out.
static mp_obj_t sb_servo_slot_of(mp_obj_t self_in, mp_obj_t id_in) {
    (void)self_in;
    uint8_t want = (uint8_t)mp_obj_get_int(id_in);
    int found = -1;
    bus_take();
    ob_sservo_t *sv = sservo_get();
    for (int i = 0; i < OB_SSERVO_SLOTS; i++) {
        if (sv->slots[i].in_use && sv->slots[i].id == want) {
            found = i;
            break;
        }
    }
    bus_release();
    return mp_obj_new_int(found);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_slot_of_obj, sb_servo_slot_of);

static mp_obj_t sb_servo_detach(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    int slot = mp_obj_get_int(slot_in);
    bus_take();
    if (slot >= 0 && slot < OB_SSERVO_SLOTS) {
        ob_smove_stop(&st_moves[slot]);
    }
    ob_sservo_detach(sservo_get(), slot);
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_detach_obj, sb_servo_detach);

static mp_obj_t sb_servo_run(mp_obj_t self_in, mp_obj_t slot_in,
                             mp_obj_t steps_in) {
    // Signed encoder steps/s (Python converts dps; core is int-only).
    (void)self_in;
    int slot = mp_obj_get_int(slot_in);
    bus_take();
    if (slot >= 0 && slot < OB_SSERVO_SLOTS) {
        ob_smove_stop(&st_moves[slot]);   // new command wins
    }
    int r = ob_sservo_set_speed(sservo_get(), slot,
                                (int32_t)mp_obj_get_int(steps_in));
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_3(sb_servo_run_obj, sb_servo_run);

static mp_obj_t sb_servo_coast(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    int slot = mp_obj_get_int(slot_in);
    bus_take();
    if (slot >= 0 && slot < OB_SSERVO_SLOTS) {
        ob_smove_stop(&st_moves[slot]);   // new command wins
    }
    int r = ob_sservo_coast(sservo_get(), slot);
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_coast_obj, sb_servo_coast);

// ---- per-slot position moves (run_angle / hold, 1.46.0) ----

// Common gate: the slot must be attached with live odometry (a move
// armed before the first feedback read would anchor to counts=0 and
// slam the shaft toward a wrong absolute position), and the db must
// not currently own the slot.
static bool smove_slot_ready_locked(int slot) {
    ob_sservo_t *sv = sservo_get();
    if (slot < 0 || slot >= OB_SSERVO_SLOTS || !sv->slots[slot].in_use
        || !sv->slots[slot].have_raw) {
        return false;
    }
    if (st_db_active && st_db_writing
        && (slot == st_db_slot_l || slot == st_db_slot_r)) {
        return false;
    }
    return true;
}

static mp_obj_t sb_servo_move(size_t n_args, const mp_obj_t *args) {
    // (self, slot, delta_counts, speed_cps, accel_cps2) -> bool
    (void)n_args;
    int slot = mp_obj_get_int(args[1]);
    bus_take();
    bool ok = smove_slot_ready_locked(slot);
    if (ok) {
        ob_smove_start(&st_moves[slot], (long)st_db_now_ms,
                       (ob_float_t)ob_sservo_counts(sservo_get(), slot),
                       (ob_float_t)mp_obj_get_float(args[2]),
                       (ob_float_t)mp_obj_get_float(args[3]),
                       (ob_float_t)mp_obj_get_float(args[4]));
    }
    bus_release();
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_servo_move_obj, 5, 5, sb_servo_move);

static mp_obj_t sb_servo_hold(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    int slot = mp_obj_get_int(slot_in);
    bus_take();
    bool ok = smove_slot_ready_locked(slot);
    if (ok) {
        ob_smove_hold_at(&st_moves[slot],
                         (ob_float_t)ob_sservo_counts(sservo_get(), slot));
    }
    bus_release();
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_hold_obj, sb_servo_hold);

static mp_obj_t sb_servo_move_done(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    int slot = mp_obj_get_int(slot_in);
    bus_take();
    bool d = slot >= 0 && slot < OB_SSERVO_SLOTS
             && ob_smove_is_done(&st_moves[slot]);
    bus_release();
    return mp_obj_new_bool(d);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_move_done_obj, sb_servo_move_done);

static mp_obj_t sb_servo_counts(mp_obj_t self_in, mp_obj_t slot_in) {
    (void)self_in;
    bus_take();
    int32_t c = ob_sservo_counts(sservo_get(), mp_obj_get_int(slot_in));
    bus_release();
    return mp_obj_new_int(c);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_counts_obj, sb_servo_counts);

static mp_obj_t sb_servo_feedback(mp_obj_t self_in, mp_obj_t slot_in) {
    // (speed_steps_per_s, load_raw_0p1pct, fresh) — user frame (slot
    // invert applied), from the widened 6-byte feedback read.
    // fresh==False means the values are stale (bus silent) or no
    // widened read has landed yet; callers surface that as None /
    // OSError, never as a silent 0.
    (void)self_in;
    int slot = mp_obj_get_int(slot_in);
    bus_take();
    ob_sservo_t *sv = sservo_get();
    mp_obj_t t[3] = {
        mp_obj_new_int(ob_sservo_speed_steps(sv, slot)),
        mp_obj_new_int(ob_sservo_load_raw(sv, slot)),
        mp_obj_new_bool(ob_sservo_feedback_fresh(sv, slot)),
    };
    bus_release();
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_feedback_obj, sb_servo_feedback);

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

static mp_obj_t sb_servo_write_stats(mp_obj_t self_in, mp_obj_t slot_in) {
    // The write-side counterpart of servo_stats: (writes_failed,
    // config_failed). A separate binding rather than a wider tuple so
    // existing servo_stats unpackers keep working.
    (void)self_in;
    int slot = mp_obj_get_int(slot_in);
    bus_take();
    ob_sservo_t *sv = sservo_get();
    uint32_t failed = 0, latched = 0;
    if (slot >= 0 && slot < OB_SSERVO_SLOTS) {
        failed  = sv->slots[slot].writes_failed;
        latched = sv->slots[slot].config_failed;
    }
    bus_release();
    mp_obj_t t[2] = {
        mp_obj_new_int_from_uint(failed),
        mp_obj_new_int_from_uint(latched),
    };
    return mp_obj_new_tuple(2, t);
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_servo_write_stats_obj,
                                 sb_servo_write_stats);

static mp_obj_t sb_servo_pump(size_t n_args, const mp_obj_t *args) {
    // One pump iteration — the hard tick's exact code path, callable
    // from Python. On firmware with the tick running this is a
    // harmless extra iteration; on unix it IS the drive loop, which
    // is how the scheduler is tested off-hardware. The optional
    // ``now_ms`` argument is the unix suite's drivebase clock (the
    // firmware path feeds ob_hard_ticks_ms instead — NOT the
    // motor_process clock, which advances on the starvable scheduler
    // path).
    bus_take();
    if (n_args == 2) {
        st_db_now_ms = (uint32_t)mp_obj_get_int_truncated(args[1]);
    }
    servo_pump_locked(bus_get());
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_servo_pump_obj, 1, 2, sb_servo_pump);

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
            sv->slots[i].torque_on = 0;   // next run re-arms torque
        }
    }
    bus_release();
    return mp_obj_new_bool(r == 0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_torque_off_all_obj, sb_torque_off_all);

// ---- native drivebase bindings ----

static mp_obj_t sb_db_config(size_t n_args, const mp_obj_t *args) {
    // (self, slot_l, slot_r, wheel_diameter_mm, axle_track_mm,
    //  accel_dps2)
    (void)n_args;
    bus_take();
    st_db_slot_l = mp_obj_get_int(args[1]);
    st_db_slot_r = mp_obj_get_int(args[2]);
    memset(&st_db_bridge_l, 0, sizeof(st_db_bridge_l));
    memset(&st_db_bridge_r, 0, sizeof(st_db_bridge_r));
    ob_drivebase_init(&st_db, &st_db_bridge_l, &st_db_bridge_r,
                      (ob_float_t)mp_obj_get_float(args[3]),
                      (ob_float_t)mp_obj_get_float(args[4]),
                      OB_DRIVEBASE_DEFAULT_KP_SUM,
                      OB_DRIVEBASE_DEFAULT_KP_DIFF);
    st_db.accel_dps2 = (ob_float_t)mp_obj_get_float(args[5]);
    st_db_active = true;
    // Configured but yielded: the db starts writing at its first
    // move, so a freshly constructed DriveBase leaves the wheels
    // free instead of torquing them into a hold at pose zero.
    st_db_writing = false;
    st_db_fault = 0;
    ob_smove_init(&st_moves[st_db_slot_l]);
    ob_smove_init(&st_moves[st_db_slot_r]);
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_db_config_obj, 6, 6, sb_db_config);

static mp_obj_t sb_db_disable(mp_obj_t self_in) {
    (void)self_in;
    bus_take();
    st_db_active = false;
    st_db_writing = false;
    st_db_fault = 0;
    st_db_slot_l = st_db_slot_r = -1;
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_db_disable_obj, sb_db_disable);

static mp_obj_t sb_db_straight(mp_obj_t self_in, mp_obj_t mm_in,
                               mp_obj_t mm_s_in) {
    (void)self_in;
    bus_take();
    // New command wins: the db takes its slots back from any
    // per-slot move, and arms from freshly synced odometry (the
    // yielded tick doesn't run ob_drivebase_tick, so sync here).
    st_db_sync_bridges_locked();
    ob_smove_stop(&st_moves[st_db_slot_l]);
    ob_smove_stop(&st_moves[st_db_slot_r]);
    st_db_fault = 0;      // retry re-detects within ~200 ms
    st_db_writing = true;
    ob_drivebase_straight(&st_db, (long)st_db_now_ms,
                          (ob_float_t)mp_obj_get_float(mm_in),
                          (ob_float_t)mp_obj_get_float(mm_s_in));
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(sb_db_straight_obj, sb_db_straight);

static mp_obj_t sb_db_turn(mp_obj_t self_in, mp_obj_t deg_in,
                           mp_obj_t dps_in) {
    (void)self_in;
    bus_take();
    st_db_sync_bridges_locked();
    ob_smove_stop(&st_moves[st_db_slot_l]);
    ob_smove_stop(&st_moves[st_db_slot_r]);
    st_db_fault = 0;      // retry re-detects within ~200 ms
    st_db_writing = true;
    ob_drivebase_turn(&st_db, (long)st_db_now_ms,
                      (ob_float_t)mp_obj_get_float(deg_in),
                      (ob_float_t)mp_obj_get_float(dps_in));
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(sb_db_turn_obj, sb_db_turn);

// Take the wheels away from the coupled controller: capture the
// frame if a move was aborted mid-flight, halt the profiles, and
// yield so the tick stops re-asserting the db's own hold. Shared by
// db_stop and db_move_wheels — ONE implementation, because the
// capture rule below is subtle enough that two copies would drift.
static void st_db_abort_capture_and_yield_locked(void) {
    // Hold capture is ABORT-ONLY. Two cases:
    //
    // * Move NOT done (mid-move abort): the holds still carry
    //   move-START values (they only update at profile expiry), so
    //   without a fresh capture the next arm would baseline from a
    //   stale pose — the original lurch bug. Capture measured.
    //
    // * Move done (the normal per-move stop in DriveBase's flow):
    //   the holds are already end-locked to the ABSOLUTE targets.
    //   Re-capturing MEASURED here re-baselines the gyro frame at
    //   every segment boundary, banking each turn's arrival residual
    //   instead of letting the next move correct it — bench
    //   2026-08-02: +7.6 deg after one gyro square (~+1.9/turn), the
    //   exact pre-1.25.0 per-move-re-baselining failure mode. The
    //   1.45.0 one-class flow stops after EVERY move, which is what
    //   armed this. Keep the absolute holds.
    if (!ob_drivebase_is_done(&st_db)) {
        st_db.fwd_hold = (st_db_bridge_l.observer.pos_hat
                          + st_db_bridge_r.observer.pos_hat) / (ob_float_t)2.0;
        st_db.turn_hold = st_db.use_gyro
            ? st_db.heading_override_wheel_deg
            : (st_db_bridge_l.observer.pos_hat
               - st_db_bridge_r.observer.pos_hat) / (ob_float_t)2.0;
    }
    ob_drivebase_stop(&st_db);
    st_db_writing = false;
}

// db_move_wheels(left_steps_per_s, right_steps_per_s) — drive the two
// wheels at independent speeds, both staged inside this one critical
// section so they leave in a single sync-write packet. This is the
// drivebase-owned replacement for building a SyncServoGroup over the
// wheels: after adoption the motors' MicroPython UART is gone (the
// IDF driver owns the pins), so a SyncServoGroup physically cannot
// reach them any more.
//
// A direct wheel command supersedes everything: the coupled move,
// and any per-slot run_angle/hold. Returns False if the drivebase
// has no slots configured.
static mp_obj_t sb_db_move_wheels(mp_obj_t self_in, mp_obj_t l_in,
                                  mp_obj_t r_in) {
    (void)self_in;
    int32_t l = (int32_t)mp_obj_get_int(l_in);
    int32_t r = (int32_t)mp_obj_get_int(r_in);
    bool ok = false;
    bus_take();
    st_db_abort_capture_and_yield_locked();
    if (st_db_slot_l >= 0) {
        ob_sservo_t *sv = sservo_get();
        ob_smove_stop(&st_moves[st_db_slot_l]);
        ob_smove_stop(&st_moves[st_db_slot_r]);
        // Both dirty before the lock drops -> the planner emits them
        // together (ob_sservo_next_op batches every dirty slot).
        ob_sservo_set_speed(sv, st_db_slot_l, l);
        ob_sservo_set_speed(sv, st_db_slot_r, r);
        ok = true;
    }
    bus_release();
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_3(sb_db_move_wheels_obj, sb_db_move_wheels);

static mp_obj_t sb_db_stop(size_t n_args, const mp_obj_t *args) {
    // db_stop([then_mode]) — without the arg: yield only (the abort
    // paths; the caller dispatches the end-state itself). With it
    // (0 = coast, 1 = brake, 2 = hold) the COMPLETE stop is staged
    // inside this one critical section, so both wheels reach their
    // end-state together: coast rides one sync-torque packet, brake
    // one sync-speed packet, and hold captures both poses at the
    // same instant. The Python layer previously dispatched per
    // motor, releasing the wheels one bus transaction apart.
    // Returns False only for mode 2 before slot odometry is live
    // (a hold would anchor to counts=0 and slam the shaft).
    int mode = (n_args >= 2) ? mp_obj_get_int(args[1]) : -1;
    bool ok = true;
    bus_take();
    // The abort-only hold capture and the yield that hands the
    // wheels to the motor layer for the caller's ``then=`` both live
    // in this helper, with the full reasoning.
    st_db_abort_capture_and_yield_locked();
    if (st_db_slot_l >= 0) {
        ob_sservo_t *sv = sservo_get();
        ob_sservo_set_speed(sv, st_db_slot_l, 0);
        ob_sservo_set_speed(sv, st_db_slot_r, 0);
        if (mode == 0 || mode == 1) {
            // New command wins: kill any per-slot move too.
            ob_smove_stop(&st_moves[st_db_slot_l]);
            ob_smove_stop(&st_moves[st_db_slot_r]);
            if (mode == 0) {
                ob_sservo_coast(sv, st_db_slot_l);
                ob_sservo_coast(sv, st_db_slot_r);
            }
            // mode 1 (brake): the zero-speed staging above IS the
            // brake — one sync-write, torque stays on, the servos'
            // goal_acc ramps to zero (the uniform-accel rule).
        } else if (mode == 2) {
            if (sv->slots[st_db_slot_l].have_raw
                && sv->slots[st_db_slot_r].have_raw) {
                ob_smove_hold_at(&st_moves[st_db_slot_l],
                    (ob_float_t)ob_sservo_counts(sv, st_db_slot_l));
                ob_smove_hold_at(&st_moves[st_db_slot_r],
                    (ob_float_t)ob_sservo_counts(sv, st_db_slot_r));
            } else {
                ok = false;
            }
        }
    }
    bus_release();
    return mp_obj_new_bool(ok);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sb_db_stop_obj, 1, 2, sb_db_stop);

// db_fault() -> bitmask: 0x01 left wheel silent, 0x02 right wheel
// silent, 0 healthy. Latched by the tick; cleared when a new move is
// armed, so a retry re-detects within ~200 ms rather than bricking
// the drivebase until reconstruction.
static mp_obj_t sb_db_fault(mp_obj_t self_in) {
    (void)self_in;
    bus_take();
    uint8_t f = st_db_fault;
    bus_release();
    return mp_obj_new_int_from_uint(f);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_db_fault_obj, sb_db_fault);

static mp_obj_t sb_db_gyro_source(mp_obj_t self_in, mp_obj_t mode_in) {
    // 0 = Python db_set_heading override (default); 1 = hard-tick
    // yaw integrator. Selecting 1 captures the current yaw as the
    // frame reference.
    (void)self_in;
    int mode = mp_obj_get_int(mode_in);
    bus_take();
    st_db_gyro_source = (mode == 1) ? 1 : 0;
    if (st_db_gyro_source == 1) {
        st_db_yaw_ref = (ob_float_t)openbricks_hard_yaw_deg();
    }
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_db_gyro_source_obj, sb_db_gyro_source);

static mp_obj_t sb_db_done(mp_obj_t self_in) {
    (void)self_in;
    bus_take();
    bool d = ob_drivebase_is_done(&st_db);
    bus_release();
    return mp_obj_new_bool(d);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_db_done_obj, sb_db_done);

static mp_obj_t sb_db_set_accel(mp_obj_t self_in, mp_obj_t dps2_in) {
    // settings(acceleration=...) parity for the serial-native path:
    // retune the trajectory accel without re-running db_config (which
    // would reset the pose holds).
    (void)self_in;
    bus_take();
    st_db.accel_dps2 = (ob_float_t)mp_obj_get_float(dps2_in);
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_db_set_accel_obj, sb_db_set_accel);

static mp_obj_t sb_db_use_gyro(mp_obj_t self_in, mp_obj_t on_in) {
    (void)self_in;
    bus_take();
    bool on = mp_obj_is_true(on_in);
    if (on && !st_db.use_gyro) {
        ob_drivebase_gyro_frame_reset(&st_db);
    }
    st_db.use_gyro = on;
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_db_use_gyro_obj, sb_db_use_gyro);

static mp_obj_t sb_db_set_heading(mp_obj_t self_in, mp_obj_t body_deg_in) {
    // The Python outer gyro loop: body-degree heading delta from the
    // IMU, converted to the wheel-degree differential the controller
    // expects. Called at ~50 Hz from a soft timer — a float store
    // under the lock, nothing more.
    (void)self_in;
    bus_take();
    st_db.heading_override_wheel_deg = ob_drivebase_body_to_wheel_diff(
        &st_db, (ob_float_t)mp_obj_get_float(body_deg_in));
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(sb_db_set_heading_obj, sb_db_set_heading);

static mp_obj_t sb_reset_runtime(mp_obj_t self_in) {
    // Program-boundary reset, called by launcher.run_program before
    // every program (the motor_process.reset precedent): a new
    // program must not inherit the previous one\'s servo slots or
    // drivebase config — that inheritance is why a second run of the
    // same script died with "slot attach failed" until a power-cycle.
    // Hardware state (the attached UART) survives: it\'s environment,
    // not program state, and attach_uart re-configures idempotently.
    (void)self_in;
    bus_take();
    st_db_gyro_source = 0;
    st_db_active = false;
    st_db_writing = false;
    // The fault latch IS cleared here (unlike the estop path, which
    // preserves it as evidence): a program boundary is where the
    // previous run's diagnosis has been read — leaving it latched
    // made the NEXT program's first db_fault() report a wheel fault
    // that belonged to the last one.
    st_db_fault = 0;
    st_moves_reset_all();
    st_db_slot_l = st_db_slot_r = -1;
    ob_sservo_init(sservo_get());
    tick_txn = 0;
    tick_txn_is_read = 0;
    tick_txn_is_swrite = 0;
    // ABANDON any transaction the previous program left in flight.
    //
    // The hard tick pumps right up to the instant a program ends, so
    // the bus is usually mid-transaction here. The pump only consumes
    // results IT started (``tick_txn``) — and that flag was just
    // cleared — so a reply left un-taken holds the bus in a non-IDLE
    // state that nothing ever clears, and ``servo_pump_locked``
    // returns early forever: no config writes, no feedback reads, no
    // slot ever serviced.
    //
    // Bench 2026-08-05: a task motor attached at program start
    // reported "0 replies, 0 failed reads" — not a dead servo (that
    // shows failed reads CLIMBING), but a pump that never ran.
    // ``attach_uart`` hid this for years by re-initialising the bus,
    // so only a program that constructed its DriveBase FIRST ever
    // recovered; 1.57.0 let a task motor get there first.
    test_bus.state = OB_BUS_IDLE;
    bus_release();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sb_reset_runtime_obj, sb_reset_runtime);

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
    { MP_ROM_QSTR(MP_QSTR_servo_slot_of),    MP_ROM_PTR(&sb_servo_slot_of_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_run),        MP_ROM_PTR(&sb_servo_run_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_coast),      MP_ROM_PTR(&sb_servo_coast_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_move),       MP_ROM_PTR(&sb_servo_move_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_hold),       MP_ROM_PTR(&sb_servo_hold_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_move_done),  MP_ROM_PTR(&sb_servo_move_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_counts),     MP_ROM_PTR(&sb_servo_counts_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_feedback),   MP_ROM_PTR(&sb_servo_feedback_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_stats),      MP_ROM_PTR(&sb_servo_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_write_stats), MP_ROM_PTR(&sb_servo_write_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_pump),       MP_ROM_PTR(&sb_servo_pump_obj) },
    { MP_ROM_QSTR(MP_QSTR_servo_encode),     MP_ROM_PTR(&sb_servo_encode_obj) },
    { MP_ROM_QSTR(MP_QSTR_torque_off_all),   MP_ROM_PTR(&sb_torque_off_all_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_runtime),    MP_ROM_PTR(&sb_reset_runtime_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_config),        MP_ROM_PTR(&sb_db_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_disable),       MP_ROM_PTR(&sb_db_disable_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_straight),      MP_ROM_PTR(&sb_db_straight_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_turn),          MP_ROM_PTR(&sb_db_turn_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_stop),          MP_ROM_PTR(&sb_db_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_move_wheels),   MP_ROM_PTR(&sb_db_move_wheels_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_fault),         MP_ROM_PTR(&sb_db_fault_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_done),          MP_ROM_PTR(&sb_db_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_gyro_source),  MP_ROM_PTR(&sb_db_gyro_source_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_use_gyro),      MP_ROM_PTR(&sb_db_use_gyro_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_set_accel),     MP_ROM_PTR(&sb_db_set_accel_obj) },
    { MP_ROM_QSTR(MP_QSTR_db_set_heading),   MP_ROM_PTR(&sb_db_set_heading_obj) },
    #if defined(MICROPY_OPENBRICKS_BUS_UART) && MICROPY_OPENBRICKS_BUS_UART
    { MP_ROM_QSTR(MP_QSTR_attach_uart),      MP_ROM_PTR(&sb_attach_uart_obj) },
    #endif
    // Unguarded: drivers ASK whether a UART is natively owned before
    // opening their own, and must get a truthful -1 on builds with no
    // UART backend rather than an AttributeError they'd have to guess
    // the meaning of.
    { MP_ROM_QSTR(MP_QSTR_uart_num),         MP_ROM_PTR(&sb_uart_num_obj) },
    // States as constants so tests read like the header enum.
    { MP_ROM_QSTR(MP_QSTR_IDLE),        MP_ROM_INT(OB_BUS_IDLE) },
    { MP_ROM_QSTR(MP_QSTR_AWAIT_REPLY), MP_ROM_INT(OB_BUS_AWAIT_REPLY) },
    { MP_ROM_QSTR(MP_QSTR_DONE),        MP_ROM_INT(OB_BUS_DONE) },
    { MP_ROM_QSTR(MP_QSTR_TIMEOUT),     MP_ROM_INT(OB_BUS_TIMEOUT) },
    { MP_ROM_QSTR(MP_QSTR_BAD_REPLY),   MP_ROM_INT(OB_BUS_BAD_REPLY) },
    { MP_ROM_QSTR(MP_QSTR_SERVO_ERR),   MP_ROM_INT(OB_BUS_SERVO_ERR) },
};
static MP_DEFINE_CONST_DICT(st_bus_locals_dict, st_bus_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    st_bus_type,
    MP_QSTR_st_bus,
    MP_TYPE_FLAG_NONE,
    locals_dict, &st_bus_locals_dict
    );

const mp_obj_base_t st_bus_singleton = { &st_bus_type };
