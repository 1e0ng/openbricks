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
