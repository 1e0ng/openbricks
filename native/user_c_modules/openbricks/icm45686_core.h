// SPDX-License-Identifier: MIT
//
// icm45686_core — register logic for the TDK ICM-45686 raw 6-axis
// IMU, the hard-tick heading source.
//
// Pure C with INJECTED SPI transfer (the st_bus_core pattern): the
// firmware binding hands it ob_spi_txn from the spi-shim patch;
// tests hand it a scripted transfer. Register map and init sequence
// derive from two hardware-proven references — betaflight's
// accgyro_spi_icm456xx.c and Zephyr's icm45686 driver — per the
// check-upstream-first rule. IREG-based filter tuning (AAF/LPF) is
// deliberately omitted: power-on defaults serve a 1 kHz consumer;
// revisit only if bench noise says otherwise.
//
// Silicon-verified 2026-08-10: SPI mode 3 (WHO_AM_I + 1000 err-free
// burst reads/s on first contact) and LITTLE-endian data (low byte
// at the lower address). Endianness is configurable on this part —
// DS-000577 §15: default little (SREG_DATA_ENDIAN_SEL=0); the
// register-table naming (X1 = [15:8]) assumes the big-endian mode
// betaflight selects through an IREG write we deliberately skip —
// so this map stays valid ONLY while init never touches SREG_CTRL.
// Gravity magnitude 1.0095 g only under the little-endian decode.

#pragma once

#include <stdint.h>

// Register map (betaflight/Zephyr, ICM-456xx direct bank).
#define OB_ICM_REG_ACCEL_DATA   0x00   // 12-byte accel+gyro burst base
#define OB_ICM_REG_GYRO_DATA    0x06
#define OB_ICM_REG_PWR_MGMT0    0x10
#define OB_ICM_REG_ACCEL_CFG0   0x1B
#define OB_ICM_REG_GYRO_CFG0    0x1C
#define OB_ICM_REG_WHO_AM_I     0x72
#define OB_ICM_REG_MISC2        0x7F

#define OB_ICM_WHO_AM_I_VAL     0xE9
#define OB_ICM_READ_FLAG        0x80

// Configured full scales (the init sequence below): gyro 2000 dps,
// accel 16 g — betaflight's proven bring-up values.
#define OB_ICM_GYRO_LSB_PER_DPS 16.384
#define OB_ICM_ACCEL_LSB_PER_G  2048.0

// Injected SPI transfer: full duplex, ``len`` bytes each way,
// returns 0 on success.
typedef int (*ob_icm_txn_t)(void *ctx, const uint8_t *tx,
                            uint8_t *rx, int len);

// Init sequence, walked by the binding (delays can't live in a pure
// core): write ``val`` to ``reg``, then wait ``delay_ms``.
typedef struct {
    uint8_t  reg;
    uint8_t  val;
    uint16_t delay_ms;
} ob_icm_reg_write_t;

extern const ob_icm_reg_write_t ob_icm_init_seq[];
extern const int ob_icm_init_len;

// Single register write/read through the injected transfer.
int ob_icm_write_reg(ob_icm_txn_t txn, void *ctx,
                     uint8_t reg, uint8_t val);
int ob_icm_read_reg(ob_icm_txn_t txn, void *ctx,
                    uint8_t reg, uint8_t *out);

// WHO_AM_I check: 0 = confirmed ICM-45686, -1 = wrong/absent chip.
int ob_icm_whoami_ok(ob_icm_txn_t txn, void *ctx);

// 13-byte burst from ACCEL_DATA: decode accel[3] + gyro[3] raw
// int16 (little-endian). Returns 0 on success.
int ob_icm_read_burst(ob_icm_txn_t txn, void *ctx,
                      int16_t accel[3], int16_t gyro[3]);
