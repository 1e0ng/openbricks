// SPDX-License-Identifier: MIT
// icm45686_core — implementation. See header for the contract.

#include "icm45686_core.h"

// betaflight's proven bring-up order: soft reset, sensors off, both
// sensors to low-noise mode, then rate/scale configs. Values:
//   PWR_MGMT0  0x0F = gyro LN | accel LN
//   ACCEL_CFG0 0x14 = 16 g full scale, 1.6 kHz ODR
//   GYRO_CFG0  0x13 = 2000 dps full scale, 6.4 kHz ODR
const ob_icm_reg_write_t ob_icm_init_seq[] = {
    { OB_ICM_REG_MISC2,      0x02, 20 },   // soft reset
    { OB_ICM_REG_PWR_MGMT0,  0x00,  1 },   // sensors off
    { OB_ICM_REG_PWR_MGMT0,  0x0F,  1 },   // gyro+accel low-noise
    { OB_ICM_REG_ACCEL_CFG0, 0x14, 10 },
    { OB_ICM_REG_GYRO_CFG0,  0x13, 35 },
};
const int ob_icm_init_len =
    (int)(sizeof(ob_icm_init_seq) / sizeof(ob_icm_init_seq[0]));


int ob_icm_write_reg(ob_icm_txn_t txn, void *ctx,
                     uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    uint8_t rx[2];
    return txn(ctx, tx, rx, 2);
}


int ob_icm_read_reg(ob_icm_txn_t txn, void *ctx,
                    uint8_t reg, uint8_t *out) {
    uint8_t tx[2] = { (uint8_t)(reg | OB_ICM_READ_FLAG), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    int r = txn(ctx, tx, rx, 2);
    if (r != 0) {
        return r;
    }
    *out = rx[1];
    return 0;
}


int ob_icm_whoami_ok(ob_icm_txn_t txn, void *ctx) {
    uint8_t v = 0;
    if (ob_icm_read_reg(txn, ctx, OB_ICM_REG_WHO_AM_I, &v) != 0) {
        return -1;
    }
    return (v == OB_ICM_WHO_AM_I_VAL) ? 0 : -1;
}


int ob_icm_read_burst(ob_icm_txn_t txn, void *ctx,
                      int16_t accel[3], int16_t gyro[3]) {
    uint8_t tx[13] = { OB_ICM_REG_ACCEL_DATA | OB_ICM_READ_FLAG };
    uint8_t rx[13] = { 0 };
    int r = txn(ctx, tx, rx, 13);
    if (r != 0) {
        return r;
    }
    // Little-endian: low byte at the lower address. The 45686
    // breaks with the 42688 family here — silicon-verified
    // 2026-08-10 (big-endian decode read gravity as 12.4 g;
    // swapped, |a| = 1.0095 g on a resting chip).
    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)((rx[2 + i * 2] << 8) | rx[1 + i * 2]);
        gyro[i]  = (int16_t)((rx[8 + i * 2] << 8) | rx[7 + i * 2]);
    }
    return 0;
}
