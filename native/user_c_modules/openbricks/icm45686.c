// SPDX-License-Identifier: MIT
//
// icm45686 — MicroPython binding for the TDK ICM-45686 raw 6-axis
// IMU as the hard-tick heading source.
//
// Wire logic lives in icm45686_core (injected-transfer, unix-tested);
// the SPI itself comes from the esp32-openbricks-spi-shim patch
// (port land; this module cannot include IDF headers). Once
// ``config`` succeeds, a hard-tick consumer reads the 13-byte
// accel+gyro burst every millisecond (~13 us at 8 MHz), feeds
// gyro-Z into motor_process's yaw integrator, and caches all six
// axes for Python-side ``read()`` — Python NEVER touches the SPI
// after config (single-caller contract of the shim).

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"

#include "icm45686_core.h"
#include "motor_process.h"

typedef struct {
    mp_obj_base_t base;
} icm_obj_t;

extern const mp_obj_type_t icm45686_type;
const mp_obj_base_t icm45686_singleton = { &icm45686_type };

#if defined(MICROPY_OPENBRICKS_SPI_SHIM) && MICROPY_OPENBRICKS_SPI_SHIM \
    && defined(MICROPY_OPENBRICKS_HARD_TICK) && MICROPY_OPENBRICKS_HARD_TICK

extern int ob_spi_open(int host, int sck, int mosi, int miso, int cs,
                       int hz, int mode);
extern int ob_spi_close(void);
extern int ob_spi_txn(const uint8_t *tx, uint8_t *rx, int len);
extern uint32_t ob_hard_ticks_ms(void);

static int spi_txn_adapter(void *ctx, const uint8_t *tx,
                           uint8_t *rx, int len) {
    (void)ctx;
    return ob_spi_txn(tx, rx, len);
}

static volatile int16_t  icm_accel[3];
static volatile int16_t  icm_gyro[3];
static volatile uint32_t icm_reads_ok;
static volatile uint32_t icm_read_errs;
static volatile uint8_t  icm_configured;
static uint32_t          icm_last_ms;

static void icm_hard_tick(void) {
    if (!icm_configured) {
        return;
    }
    int16_t a[3], g[3];
    if (ob_icm_read_burst(spi_txn_adapter, NULL, a, g) != 0) {
        icm_read_errs++;
        return;
    }
    for (int i = 0; i < 3; i++) {
        icm_accel[i] = a[i];
        icm_gyro[i]  = g[i];
    }
    icm_reads_ok++;
    uint32_t now = ob_hard_ticks_ms();
    uint32_t dt = now - icm_last_ms;
    icm_last_ms = now;
    if (dt == 0 || dt > 100) {
        return;    // sub-ms tick or a hiccup: skip this integration
    }
    openbricks_hard_yaw_feed_c(
        (double)dt, (double)g[2] / OB_ICM_GYRO_LSB_PER_DPS);
}

static mp_obj_t icm_config(size_t n_args, const mp_obj_t *pos_args,
                           mp_map_t *kw_args) {
    enum { ARG_sck, ARG_mosi, ARG_miso, ARG_cs, ARG_host, ARG_hz,
           ARG_mode, ARG_scale };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_sck,   MP_ARG_REQUIRED | MP_ARG_INT, {0} },
        { MP_QSTR_mosi,  MP_ARG_REQUIRED | MP_ARG_INT, {0} },
        { MP_QSTR_miso,  MP_ARG_REQUIRED | MP_ARG_INT, {0} },
        { MP_QSTR_cs,    MP_ARG_REQUIRED | MP_ARG_INT, {0} },
        { MP_QSTR_host,  MP_ARG_INT, {.u_int = 1} },       // SPI2
        { MP_QSTR_hz,    MP_ARG_INT, {.u_int = 8000000} },
        { MP_QSTR_mode,  MP_ARG_INT, {.u_int = 3} },       // silicon-verified
        { MP_QSTR_scale, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
                     MP_ARRAY_SIZE(allowed), allowed, args);

    if (icm_configured) {
        // Program re-run in the same boot: the previous
        // construction's 1 kHz consumer still owns the bus, and a
        // config transaction from the MP task would race it (the
        // 1.84.0 second-run EIO / who_am_i-mismatch). Pause the
        // consumer, let any in-flight burst drain, and rebuild the
        // SPI so new pins/clock/mode take effect too.
        icm_configured = 0;
        mp_hal_delay_ms(3);
        ob_spi_close();
    }
    if (ob_spi_open(args[ARG_host].u_int, args[ARG_sck].u_int,
                    args[ARG_mosi].u_int, args[ARG_miso].u_int,
                    args[ARG_cs].u_int, args[ARG_hz].u_int,
                    args[ARG_mode].u_int) != 0) {
        mp_raise_OSError(MP_EIO);
    }
    if (ob_icm_whoami_ok(spi_txn_adapter, NULL) != 0) {
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("ICM-45686 not found (who_am_i "
                                   "mismatch — wiring/mode?)"));
    }
    for (int i = 0; i < ob_icm_init_len; i++) {
        if (ob_icm_write_reg(spi_txn_adapter, NULL,
                             ob_icm_init_seq[i].reg,
                             ob_icm_init_seq[i].val) != 0) {
            mp_raise_OSError(MP_EIO);
        }
        mp_hal_delay_ms(ob_icm_init_seq[i].delay_ms);
    }
    double scale = (args[ARG_scale].u_obj == MP_OBJ_NULL)
                   ? -1.0    // top-mounted default: CW-positive frame
                   : mp_obj_get_float(args[ARG_scale].u_obj);
    openbricks_hard_yaw_configure_c(scale);
    icm_last_ms = ob_hard_ticks_ms();
    icm_reads_ok = icm_read_errs = 0;
    icm_configured = 1;
    openbricks_hard_imu_install(icm_hard_tick);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(icm_config_obj, 1, icm_config);

static mp_obj_t icm_read(mp_obj_t self_in) {
    // Cached last hard-tick sample (Python must not touch the SPI).
    (void)self_in;
    if (!icm_configured) {
        mp_raise_msg(&mp_type_OSError,
                     MP_ERROR_TEXT("icm45686 not configured"));
    }
    mp_obj_t t[6] = {
        mp_obj_new_float((mp_float_t)icm_accel[0] / OB_ICM_ACCEL_LSB_PER_G),
        mp_obj_new_float((mp_float_t)icm_accel[1] / OB_ICM_ACCEL_LSB_PER_G),
        mp_obj_new_float((mp_float_t)icm_accel[2] / OB_ICM_ACCEL_LSB_PER_G),
        mp_obj_new_float((mp_float_t)icm_gyro[0] / OB_ICM_GYRO_LSB_PER_DPS),
        mp_obj_new_float((mp_float_t)icm_gyro[1] / OB_ICM_GYRO_LSB_PER_DPS),
        mp_obj_new_float((mp_float_t)icm_gyro[2] / OB_ICM_GYRO_LSB_PER_DPS),
    };
    return mp_obj_new_tuple(6, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(icm_read_obj, icm_read);

static mp_obj_t icm_stats(mp_obj_t self_in) {
    (void)self_in;
    mp_obj_t t[3] = {
        mp_obj_new_int_from_uint(icm_reads_ok),
        mp_obj_new_int_from_uint(icm_read_errs),
        mp_obj_new_bool(icm_configured),
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(icm_stats_obj, icm_stats);

static mp_obj_t icm_available(mp_obj_t self_in) {
    (void)self_in;
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_1(icm_available_obj, icm_available);

#else   // no spi-shim / hard-tick firmware: inert stubs

static mp_obj_t icm_config(size_t n_args, const mp_obj_t *pos_args,
                           mp_map_t *kw_args) {
    (void)n_args; (void)pos_args; (void)kw_args;
    mp_raise_msg(&mp_type_OSError,
                 MP_ERROR_TEXT("icm45686 requires the spi-shim + "
                               "hard-tick firmware"));
}
static MP_DEFINE_CONST_FUN_OBJ_KW(icm_config_obj, 1, icm_config);

static mp_obj_t icm_read(mp_obj_t self_in) {
    (void)self_in;
    mp_raise_msg(&mp_type_OSError,
                 MP_ERROR_TEXT("icm45686 not available on this build"));
}
static MP_DEFINE_CONST_FUN_OBJ_1(icm_read_obj, icm_read);

static mp_obj_t icm_stats(mp_obj_t self_in) {
    (void)self_in;
    mp_obj_t t[3] = {
        mp_obj_new_int(0), mp_obj_new_int(0), mp_const_false,
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(icm_stats_obj, icm_stats);

static mp_obj_t icm_available(mp_obj_t self_in) {
    (void)self_in;
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_1(icm_available_obj, icm_available);

#endif

// Byte-order self-check, available on EVERY build (the core needs
// no SPI — transfers are injected): decode a canned little-endian
// frame and hand back (rc, a0, a1, a2, g0, g1, g2) raw for the
// test suite to pin. Every failure mode lands in the tuple — a
// wrong command byte poisons the payload with 0xFF, so the values
// can't accidentally match — because a branch here could only fire
// on a bug and would otherwise never execute. This is the
// off-hardware witness of what first silicon contact established
// (2026-08-10): the 45686 stores low byte first.
static const uint8_t icm_selftest_frame[13] = {
    0x00,                                     // command-byte echo slot
    0x02, 0x01,  0x04, 0x03,  0x06, 0x05,     // accel 258, 772, 1286
    0xFE, 0xFF,  0x2C, 0x01,  0xFB, 0xFF,     // gyro  -2, 300, -5
};

static int icm_selftest_txn(void *ctx, const uint8_t *tx,
                            uint8_t *rx, int len) {
    (void)ctx;
    int cmd_ok = (len == 13
                  && tx[0] == (OB_ICM_REG_ACCEL_DATA | OB_ICM_READ_FLAG));
    for (int i = 0; i < len; i++) {
        rx[i] = cmd_ok ? icm_selftest_frame[i] : 0xFF;
    }
    return 0;
}

static mp_obj_t icm_selftest(mp_obj_t self_in) {
    (void)self_in;
    int16_t a[3], g[3];
    int rc = ob_icm_read_burst(icm_selftest_txn, NULL, a, g);
    mp_obj_t t[7] = {
        mp_obj_new_int(rc),
        mp_obj_new_int(a[0]), mp_obj_new_int(a[1]), mp_obj_new_int(a[2]),
        mp_obj_new_int(g[0]), mp_obj_new_int(g[1]), mp_obj_new_int(g[2]),
    };
    return mp_obj_new_tuple(7, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(icm_selftest_obj, icm_selftest);

static const mp_rom_map_elem_t icm_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_config),    MP_ROM_PTR(&icm_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),      MP_ROM_PTR(&icm_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),     MP_ROM_PTR(&icm_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_available), MP_ROM_PTR(&icm_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_selftest),  MP_ROM_PTR(&icm_selftest_obj) },
};
static MP_DEFINE_CONST_DICT(icm_locals, icm_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    icm45686_type,
    MP_QSTR_icm45686,
    MP_TYPE_FLAG_NONE,
    locals_dict, &icm_locals
);
