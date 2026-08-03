#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# Build and run the native C unit tests for the user_c_modules pure
# cores (tests/c/). Every binary runs under ASan + UBSan — the whole
# point of testing the cores OUTSIDE MicroPython: a silent buffer
# overflow that would quietly corrupt the MP heap aborts loudly here,
# and struct-level edge states (exact unwrap boundaries, 2^32 clock
# wraps) are set directly instead of being coaxed through Python.
#
# The binding-driven suite (tests/*.py on the unix MP build) remains
# the integration truth; this is the layer below it.
#
# Usage: scripts/run_c_tests.sh

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CORES="${ROOT}/native/user_c_modules/openbricks"
TESTS="${ROOT}/tests/c"
OUT="${ROOT}/tests/c/build"
mkdir -p "${OUT}"

CC="${CC:-cc}"
CFLAGS="-std=c11 -Wall -Wextra -Werror -O1 -g \
        -fsanitize=address,undefined -fno-sanitize-recover=all \
        -I${CORES} -I${TESTS}"

fail=0
run_one() {
    local name="$1"; shift
    echo "== ${name} =="
    # -lm last: GNU ld resolves libraries left-to-right, and
    # trajectory_core's sqrt needs libm on Linux (macOS's libSystem
    # bundles it, which is why the gap only shows in CI).
    # shellcheck disable=SC2086
    ${CC} ${CFLAGS} "$@" -o "${OUT}/${name}" -lm
    "${OUT}/${name}" || fail=1
}

run_one test_motor_process_core \
    "${TESTS}/test_motor_process_core.c" \
    "${CORES}/motor_process_core.c"

run_one test_st_bus_core \
    "${TESTS}/test_st_bus_core.c" \
    "${CORES}/st_bus_core.c"

run_one test_st_servo_core \
    "${TESTS}/test_st_servo_core.c" \
    "${CORES}/st_servo_core.c"

run_one test_st_button_core \
    "${TESTS}/test_st_button_core.c" \
    "${CORES}/st_button_core.c"

run_one test_st_move_core \
    "${TESTS}/test_st_move_core.c" \
    "${CORES}/st_move_core.c" \
    "${CORES}/trajectory_core.c"

run_one test_imu_yaw_core \
    "${TESTS}/test_imu_yaw_core.c" \
    "${CORES}/imu_yaw_core.c"

run_one test_icm45686_core \
    "${TESTS}/test_icm45686_core.c" \
    "${CORES}/icm45686_core.c"

if [ "${fail}" -ne 0 ]; then
    echo "C unit tests: FAILED"
    exit 1
fi
echo "C unit tests: all suites passed (ASan+UBSan clean)"
