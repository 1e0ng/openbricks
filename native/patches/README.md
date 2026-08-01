# patches

Small patches applied to the `native/micropython` submodule at
firmware-build time by `scripts/build_firmware.sh`. They live here
(instead of in a fork) so we stay on upstream MicroPython master
by default — the patches are the narrow set of local changes we
haven't (yet) upstreamed.

`build_firmware.sh` applies each `*.patch` with `git apply --check`
first; if the patch is already in the working tree (e.g. a previous
build applied it, or a newer MP master already contains the fix) we
skip re-applying and don't abort the build.

## Current patches

| Patch | Purpose | Upstream status |
|---|---|---|
| `modbluetooth_nimble-esp-idf-5.5.patch` | Guard MP's `extern uint16_t ble_hs_max_*` declarations with `#ifndef` so they don't collide with ESP-IDF 5.5's macro-form definitions of the same names in `ble_hs_priv.h`. Without this, the ESP32 firmware build with BLE enabled fails at the qstr-gen pass with `error: expected ')' before '->' token`. | Not upstreamed yet — consider sending a PR to micropython/micropython. |
| `esp32-stdin-ringbuf-configurable.patch` | Make the ESP32 port's `stdin_ringbuf` size a board-overridable `MICROPY_HW_STDIN_RINGBUF_LEN` (`#ifndef`-guarded, default = upstream's 260). Currently INERT: both boards run the stock window since 1.33.1 (the 1.32.x raised-window releases broke BLE staging on hardware). Kept for a future bump backed by an `openbricks paste-probe` measurement; if a board raises `MICROPY_REPL_STDIN_BUFFER_MAX`, the ring must be ≥ 2× it or UART/USB pastes (mpremote — `openbricks flash` uses it) silently drop bytes. `tests/test_board_config.py::RawPasteWindowTests` pins the invariant. | Good upstream candidate — a 3-line configurability hook with upstream-identical defaults. |
| `esp32-openbricks-hard-tick.patch` | Adds `ports/esp32/openbricks_hard_tick.c` (+ cmake entry + `MICROPY_OPENBRICKS_HARD_TICK` config macro): one periodic C hook on the **esp_timer service task**, below the Python scheduler — the dispatch context machine.Timer callbacks don't have (those ride `mp_sched_schedule`'s droppable queue). Contract: the hook must not touch Python objects / GC / mp_* VM calls. Consumer: `_openbricks_native` (extern symbols gated on the macro; unix/sim fall back to the scheduler tick). Foundation for native serial-bus motor control. | openbricks-specific plumbing — not an upstream candidate in this form. |
| `esp32-uart-repl-tx-nonblocking.patch` | Board-opt-in non-blocking UART stdout: `MICROPY_HW_UART_REPL_TX_RING` (`#ifndef`-guarded, default 0 = upstream's blocking busy-wait, which upstream itself flags with `// TODO add a timeout`). When set, `uart_stdout_tx_strn` copies into a static ring and returns; the existing UART ISR drains ring → FIFO via `TXFIFO_EMPTY`. Rationale: blocking TX made every `print()` pay wire time (~5.1 ms/line at 115200) on the calling thread — even with nothing attached to the UART pins. Ring-full drops the remainder on the wired console only (the USB-Serial-JTAG path already has the same give-up-when-undrained contract). Both boards set 2048. Functional coverage: the `qemu-smoke` CI job boots the patched image and asserts the banner arrives over UART0, which exercises exactly this path. | Plausible upstream candidate — solves upstream's own TODO, inert at default config. |
