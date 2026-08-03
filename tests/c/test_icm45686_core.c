// SPDX-License-Identifier: MIT
// Native tests for icm45686_core — scripted SPI transfers pin the
// wire protocol (read flag, init order, burst decode) before the
// part ever arrives.

#include <string.h>

#include "harness.h"
#include "icm45686_core.h"

// Scripted transfer: records TX, plays back a canned RX.
static uint8_t last_tx[16];
static int     last_len;
static uint8_t canned_rx[16];
static int     fail_next;

static int fake_txn(void *ctx, const uint8_t *tx, uint8_t *rx, int len) {
    (void)ctx;
    if (fail_next) {
        fail_next = 0;
        return -1;
    }
    memcpy(last_tx, tx, (size_t)len);
    last_len = len;
    memcpy(rx, canned_rx, (size_t)len);
    return 0;
}

TEST(init_sequence_shape) {
    // Reset FIRST, sensors off before on, configs after power. The
    // exact values are betaflight's proven bring-up set.
    CHECK_EQ_INT(ob_icm_init_len, 5);
    CHECK_EQ_INT(ob_icm_init_seq[0].reg, OB_ICM_REG_MISC2);
    CHECK_EQ_INT(ob_icm_init_seq[0].val, 0x02);
    CHECK(ob_icm_init_seq[0].delay_ms >= 10);
    CHECK_EQ_INT(ob_icm_init_seq[1].reg, OB_ICM_REG_PWR_MGMT0);
    CHECK_EQ_INT(ob_icm_init_seq[1].val, 0x00);
    CHECK_EQ_INT(ob_icm_init_seq[2].reg, OB_ICM_REG_PWR_MGMT0);
    CHECK_EQ_INT(ob_icm_init_seq[2].val, 0x0F);
    CHECK_EQ_INT(ob_icm_init_seq[3].reg, OB_ICM_REG_ACCEL_CFG0);
    CHECK_EQ_INT(ob_icm_init_seq[4].reg, OB_ICM_REG_GYRO_CFG0);
    CHECK_EQ_INT(ob_icm_init_seq[4].val, 0x13);
}

TEST(write_clears_read_flag_read_sets_it) {
    ob_icm_write_reg(fake_txn, NULL, 0x1C, 0x13);
    CHECK_EQ_INT(last_tx[0], 0x1C);          // bit 7 clear on write
    CHECK_EQ_INT(last_tx[1], 0x13);
    CHECK_EQ_INT(last_len, 2);
    uint8_t v;
    ob_icm_read_reg(fake_txn, NULL, 0x72, &v);
    CHECK_EQ_INT(last_tx[0], 0x72 | 0x80);   // bit 7 set on read
}

TEST(whoami_accepts_e9_rejects_else) {
    canned_rx[1] = 0xE9;
    CHECK_EQ_INT(ob_icm_whoami_ok(fake_txn, NULL), 0);
    canned_rx[1] = 0x47;                     // a 42688 would differ
    CHECK_EQ_INT(ob_icm_whoami_ok(fake_txn, NULL), -1);
    fail_next = 1;                           // bus dead
    CHECK_EQ_INT(ob_icm_whoami_ok(fake_txn, NULL), -1);
}

TEST(burst_decodes_big_endian_signed) {
    memset(canned_rx, 0, sizeof(canned_rx));
    // accel X = 0x0102, gyro X = -2 (0xFFFE), gyro Z = 0x7FFF.
    canned_rx[1] = 0x01; canned_rx[2] = 0x02;      // accel X
    canned_rx[7] = 0xFF; canned_rx[8] = 0xFE;      // gyro X
    canned_rx[11] = 0x7F; canned_rx[12] = 0xFF;    // gyro Z
    int16_t a[3], g[3];
    CHECK_EQ_INT(ob_icm_read_burst(fake_txn, NULL, a, g), 0);
    CHECK_EQ_INT(last_tx[0], OB_ICM_REG_ACCEL_DATA | 0x80);
    CHECK_EQ_INT(last_len, 13);
    CHECK_EQ_INT(a[0], 0x0102);
    CHECK_EQ_INT(g[0], -2);
    CHECK_EQ_INT(g[2], 0x7FFF);
}

TEST(burst_failure_propagates) {
    int16_t a[3], g[3];
    fail_next = 1;
    CHECK_EQ_INT(ob_icm_read_burst(fake_txn, NULL, a, g), -1);
}

TEST(scale_constants_match_the_configured_full_scales) {
    // GYRO_CFG0 0x13 = 2000 dps -> 32768/2000; ACCEL 0x14 = 16 g.
    CHECK(OB_ICM_GYRO_LSB_PER_DPS > 16.383
          && OB_ICM_GYRO_LSB_PER_DPS < 16.385);
    CHECK_EQ_INT((int)OB_ICM_ACCEL_LSB_PER_G, 2048);
}

int main(void) {
    RUN(init_sequence_shape);
    RUN(write_clears_read_flag_read_sets_it);
    RUN(whoami_accepts_e9_rejects_else);
    RUN(burst_decodes_big_endian_signed);
    RUN(burst_failure_propagates);
    RUN(scale_constants_match_the_configured_full_scales);
    return harness_exit("icm45686_core");
}
