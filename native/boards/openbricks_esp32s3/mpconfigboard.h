// SPDX-License-Identifier: MIT
// Board-specific macro overrides for the openbricks ESP32-S3 firmware.

#define MICROPY_HW_BOARD_NAME "openbricks (ESP32-S3)"
#define MICROPY_HW_MCU_NAME   "ESP32S3"

// WiFi disabled in sdkconfig.board — also drop MP's network-wlan
// wrapper so the firmware doesn't carry a dead ``import network`` path.
// BLE stays on.
#define MICROPY_PY_NETWORK        (0)
#define MICROPY_PY_NETWORK_WLAN   (0)

// Mirror upstream MicroPython's ESP32_GENERIC_S3: enable UART REPL
// alongside USB-Serial-JTAG so the same image works whether the
// user's USB-C cable goes to the chip's native USB pins (GPIO 19/20,
// USB-Serial-JTAG) or to a USB-UART converter (CH340 / CP210x)
// connected to UART0 (GPIO 43/44). The mphalport.c output path
// writes to BOTH transports when both are enabled — silent failure
// of one doesn't block the other.
#define MICROPY_HW_ENABLE_UART_REPL  (1)
