// SPDX-License-Identifier: MIT
//
// openbricks — bno055.c
//
// Bosch BNO055 9-DOF IMU with onboard sensor fusion. Ported to C from
// the original ``openbricks/drivers/bno055.py`` to match the rest of
// the hot-path components: the drivebase tick reads ``heading()`` every
// 1 kHz when ``use_gyro`` is on, so eliminating the Python frame per
// read gives the 2-DOF controller a cleaner latency profile.
//
// I²C hardware access itself stays the limiting factor (~125 µs per
// heading read at 400 kHz), but removing the Python dispatch + struct
// unpack + signed-int reconstruction takes ~30-50 µs off the critical
// path. We keep the driver target-agnostic by calling ``i2c.readfrom_mem``
// / ``i2c.writeto_mem`` via the MP object API — same pattern the other
// openbricks C modules use, so the unix MP test binary (with our fake
// I²C) exercises this code unchanged.

#include <stdbool.h>

#include "py/obj.h"
#include "py/runtime.h"


// --- register map (BNO055 page 0) ---
#define REG_CHIP_ID         0x00
#define REG_ACC_DATA_X_LSB  0x08
#define REG_GYR_DATA_X_LSB  0x14
#define REG_EULER_H_LSB     0x1A
#define REG_UNIT_SEL        0x3B
#define REG_OPR_MODE        0x3D
#define REG_PWR_MODE        0x3E
#define REG_SYS_TRIGGER     0x3F

#define MODE_CONFIG         0x00
#define MODE_NDOF           0x0C

#define EXPECTED_CHIP_ID    0xA0


extern const mp_obj_type_t openbricks_bno055_type;

typedef struct _bno055_obj_t {
    mp_obj_base_t base;
    mp_obj_t i2c;
    mp_int_t addr;

    // Cached bound methods — avoids mp_load_attr on every read.
    mp_obj_t readfrom_mem;
    mp_obj_t writeto_mem;
} bno055_obj_t;


// --- low-level I/O helpers ---

static mp_int_t read_u8(bno055_obj_t *self, mp_int_t reg) {
    // i2c.readfrom_mem(addr, reg, 1)
    mp_obj_t args[3] = {
        MP_OBJ_NEW_SMALL_INT(self->addr),
        MP_OBJ_NEW_SMALL_INT(reg),
        MP_OBJ_NEW_SMALL_INT(1),
    };
    mp_obj_t buf = mp_call_function_n_kw(self->readfrom_mem, 3, 0, args);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(buf, &bi, MP_BUFFER_READ);
    return ((const uint8_t *)bi.buf)[0];
}

static void write_u8(bno055_obj_t *self, mp_int_t reg, mp_int_t value) {
    // i2c.writeto_mem(addr, reg, bytes([value]))
    uint8_t data = (uint8_t)value;
    mp_obj_t buf = mp_obj_new_bytes(&data, 1);
    mp_obj_t args[3] = {
        MP_OBJ_NEW_SMALL_INT(self->addr),
        MP_OBJ_NEW_SMALL_INT(reg),
        buf,
    };
    (void)mp_call_function_n_kw(self->writeto_mem, 3, 0, args);
}

// Read n bytes starting at ``reg`` into ``out``. Caller owns the buffer.
static void read_block(bno055_obj_t *self, mp_int_t reg, size_t n, uint8_t *out) {
    mp_obj_t args[3] = {
        MP_OBJ_NEW_SMALL_INT(self->addr),
        MP_OBJ_NEW_SMALL_INT(reg),
        MP_OBJ_NEW_SMALL_INT((mp_int_t)n),
    };
    mp_obj_t buf = mp_call_function_n_kw(self->readfrom_mem, 3, 0, args);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(buf, &bi, MP_BUFFER_READ);
    const uint8_t *src = (const uint8_t *)bi.buf;
    size_t copy = bi.len < n ? bi.len : n;
    for (size_t i = 0; i < copy; i++) {
        out[i] = src[i];
    }
    // Zero-pad if the fake/peripheral returned fewer bytes than expected.
    for (size_t i = copy; i < n; i++) {
        out[i] = 0;
    }
}

static inline int16_t sint16_le(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}


// --- time.sleep_ms, routed via Python so the virtual-clock test fake
//     can short-circuit it and tests don't spend ~700 ms in real sleeps
//     during each BNO055 construction. ---

static mp_obj_t cached_sleep_ms = MP_OBJ_NULL;

static void sleep_ms(mp_int_t ms) {
    if (cached_sleep_ms == MP_OBJ_NULL) {
        mp_obj_t time_mod = mp_import_name(MP_QSTR_time, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
        cached_sleep_ms = mp_load_attr(time_mod, MP_QSTR_sleep_ms);
    }
    mp_call_function_1(cached_sleep_ms, MP_OBJ_NEW_SMALL_INT(ms));
}


// --- constructor ---
//
// BNO055(i2c, address=0x28)

static mp_obj_t bno055_make_new(const mp_obj_type_t *type,
                                size_t n_args, size_t n_kw,
                                const mp_obj_t *all_args) {
    enum { ARG_i2c, ARG_address };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_i2c,     MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_address, MP_ARG_INT,                   {.u_int = 0x28} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed), allowed, parsed);

    bno055_obj_t *self = mp_obj_malloc(bno055_obj_t, type);
    self->i2c         = parsed[ARG_i2c].u_obj;
    self->addr        = parsed[ARG_address].u_int;
    self->readfrom_mem = mp_load_attr(self->i2c, MP_QSTR_readfrom_mem);
    self->writeto_mem  = mp_load_attr(self->i2c, MP_QSTR_writeto_mem);

    // Sanity-check chip ID.
    mp_int_t chip_id = read_u8(self, REG_CHIP_ID);
    if (chip_id != EXPECTED_CHIP_ID) {
        mp_raise_msg_varg(&mp_type_OSError,
                          MP_ERROR_TEXT("BNO055 not found at 0x%02x (got chip id 0x%02x)"),
                          (unsigned)self->addr, (unsigned)chip_id);
    }

    // Switch to CONFIG before touching settings.
    write_u8(self, REG_OPR_MODE, MODE_CONFIG);
    sleep_ms(25);

    // Reset and wait for the chip to come back.
    write_u8(self, REG_SYS_TRIGGER, 0x20);
    sleep_ms(650);
    while (read_u8(self, REG_CHIP_ID) != EXPECTED_CHIP_ID) {
        sleep_ms(10);
    }

    // Normal power mode.
    write_u8(self, REG_PWR_MODE, 0x00);
    sleep_ms(10);

    // Units: degrees for Euler, deg/s for gyro, m/s² for accel.
    // bit 7=0 Windows orientation, bit 2=0 °C, bit 1=0 deg, bit 0=0 m/s².
    write_u8(self, REG_UNIT_SEL, 0x00);

    // Engage 9-DOF fusion.
    write_u8(self, REG_SYS_TRIGGER, 0x00);
    write_u8(self, REG_OPR_MODE, MODE_NDOF);
    sleep_ms(25);

    return MP_OBJ_FROM_PTR(self);
}


// --- Python-facing methods ---

static mp_obj_t bno055_heading(mp_obj_t self_in) {
    bno055_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t buf[6];
    read_block(self, REG_EULER_H_LSB, 6, buf);
    // Euler scaling: 16 LSB per degree. BNO055 reports 0..360; wrap to
    // signed [-180, 180) for the ``heading()`` specifically.
    mp_float_t h = (mp_float_t)sint16_le(buf) / (mp_float_t)16.0;
    if (h > (mp_float_t)180.0) {
        h -= (mp_float_t)360.0;
    }
    return mp_obj_new_float(h);
}
static MP_DEFINE_CONST_FUN_OBJ_1(bno055_heading_obj, bno055_heading);

static mp_obj_t bno055_euler(mp_obj_t self_in) {
    bno055_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t buf[6];
    read_block(self, REG_EULER_H_LSB, 6, buf);
    mp_obj_t tup[3] = {
        mp_obj_new_float((mp_float_t)sint16_le(buf + 0) / (mp_float_t)16.0),
        mp_obj_new_float((mp_float_t)sint16_le(buf + 2) / (mp_float_t)16.0),
        mp_obj_new_float((mp_float_t)sint16_le(buf + 4) / (mp_float_t)16.0),
    };
    return mp_obj_new_tuple(3, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_1(bno055_euler_obj, bno055_euler);

static mp_obj_t bno055_read_vec(bno055_obj_t *self, mp_int_t reg, mp_float_t scale) {
    uint8_t buf[6];
    read_block(self, reg, 6, buf);
    mp_obj_t tup[3] = {
        mp_obj_new_float((mp_float_t)sint16_le(buf + 0) / scale),
        mp_obj_new_float((mp_float_t)sint16_le(buf + 2) / scale),
        mp_obj_new_float((mp_float_t)sint16_le(buf + 4) / scale),
    };
    return mp_obj_new_tuple(3, tup);
}

static mp_obj_t bno055_angular_velocity(mp_obj_t self_in) {
    bno055_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return bno055_read_vec(self, REG_GYR_DATA_X_LSB, (mp_float_t)16.0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(bno055_angular_velocity_obj, bno055_angular_velocity);

static mp_obj_t bno055_acceleration(mp_obj_t self_in) {
    bno055_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return bno055_read_vec(self, REG_ACC_DATA_X_LSB, (mp_float_t)100.0);
}
static MP_DEFINE_CONST_FUN_OBJ_1(bno055_acceleration_obj, bno055_acceleration);


// --- type ---

static const mp_rom_map_elem_t bno055_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_heading),           MP_ROM_PTR(&bno055_heading_obj) },
    { MP_ROM_QSTR(MP_QSTR_euler),             MP_ROM_PTR(&bno055_euler_obj) },
    { MP_ROM_QSTR(MP_QSTR_angular_velocity),  MP_ROM_PTR(&bno055_angular_velocity_obj) },
    { MP_ROM_QSTR(MP_QSTR_acceleration),      MP_ROM_PTR(&bno055_acceleration_obj) },
};
static MP_DEFINE_CONST_DICT(bno055_locals_dict, bno055_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_bno055_type,
    MP_QSTR_BNO055,
    MP_TYPE_FLAG_NONE,
    make_new,    bno055_make_new,
    locals_dict, &bno055_locals_dict
);
