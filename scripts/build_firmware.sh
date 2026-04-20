#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# Build the openbricks firmware image.
#
# Usage:
#   ./scripts/build_firmware.sh esp32       # default
#   ./scripts/build_firmware.sh esp32s3     # ESP32-S3 (Xtensa LX7, native USB)
#
# Requirements:
#   * ``native/micropython`` submodule initialised:
#         git submodule update --init --recursive native/micropython
#   * ESP-IDF 5.x on PATH via $IDF_PATH + ``get_idf`` (see docs/build.md).

set -euo pipefail

PLATFORM="${1:-esp32}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NATIVE_DIR="${REPO_ROOT}/native"
MICROPY_DIR="${NATIVE_DIR}/micropython"
BOARDS_DIR="${NATIVE_DIR}/boards"
USER_C_MODULES="${NATIVE_DIR}/user_c_modules/openbricks/micropython.cmake"

if [[ ! -d "${MICROPY_DIR}" ]] || [[ -z "$(ls -A "${MICROPY_DIR}" 2>/dev/null)" ]]; then
    echo "error: ${MICROPY_DIR} is empty or missing."
    echo "       run: git submodule update --init --recursive native/micropython"
    exit 1
fi

case "${PLATFORM}" in
    esp32|esp32s3)
        BOARD="openbricks_${PLATFORM}"
        if [[ ! -d "${BOARDS_DIR}/${BOARD}" ]]; then
            echo "error: board ${BOARD} not found in ${BOARDS_DIR}"
            exit 1
        fi
        if [[ -z "${IDF_PATH:-}" ]]; then
            echo "error: \$IDF_PATH is not set. Install ESP-IDF 5.x and source export.sh."
            echo "       see docs/build.md"
            exit 1
        fi

        PORT_DIR="${MICROPY_DIR}/ports/esp32"
        echo ">>> building mpy-cross"
        make -C "${MICROPY_DIR}/mpy-cross" -j

        echo ">>> building firmware image (board=${BOARD})"
        make -C "${PORT_DIR}" \
             BOARD="${BOARD}" \
             BOARD_DIR="${BOARDS_DIR}/${BOARD}" \
             USER_C_MODULES="${USER_C_MODULES}" \
             -j

        echo
        echo "done — firmware at ${PORT_DIR}/build-${BOARD}/firmware.bin"
        ;;
    *)
        echo "usage: $0 [esp32|esp32s3]"
        exit 1
        ;;
esac
