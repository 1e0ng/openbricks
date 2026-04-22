#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# Build the openbricks firmware image.
#
# One image is reused across every hub — per-hub identity (the BLE
# advertising / addressing name) is set at **flash** time, not build
# time. See ``scripts/flash_firmware.sh --name``.
#
# Usage:
#   ./scripts/build_firmware.sh esp32                    # default
#   ./scripts/build_firmware.sh esp32s3                  # ESP32-S3
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

# Apply any patches from native/patches/ to the MP submodule. Each
# ``*.patch`` is git-apply'd (idempotently — if it's already applied we
# skip it) before the build starts. See native/patches/README.md.
PATCHES_DIR="${NATIVE_DIR}/patches"
if [[ -d "${PATCHES_DIR}" ]]; then
    shopt -s nullglob
    for patch in "${PATCHES_DIR}"/*.patch; do
        name="$(basename "${patch}")"
        if git -C "${MICROPY_DIR}" apply --check --reverse "${patch}" 2>/dev/null; then
            echo ">>> patch already applied: ${name} (skipping)"
        elif git -C "${MICROPY_DIR}" apply --check "${patch}" 2>/dev/null; then
            echo ">>> applying patch: ${name}"
            git -C "${MICROPY_DIR}" apply "${patch}"
        else
            echo "warning: patch ${name} neither applies cleanly nor is already applied."
            echo "         MP master may have moved past it. Inspect manually."
        fi
    done
    shopt -u nullglob
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
        echo "       flash with: scripts/flash_firmware.sh --name NAME --port /dev/ttyUSB0"
        ;;
    *)
        echo "usage: $0 [esp32|esp32s3]"
        exit 1
        ;;
esac
