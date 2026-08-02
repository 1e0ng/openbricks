// SPDX-License-Identifier: MIT
//
// st_button_core — program-button handling for the hard tick.
//
// Why this exists: the launcher's button watcher is a machine.Timer
// soft callback — it rides micropython.schedule on the MP task
// (core 1) and inherits every scheduler blackout (bench-measured
// gaps to 981 ms). The PCNT latch keeps presses from being LOST,
// but ACTING on a stop press waited for Python. Sampled from the
// hard tick (esp_timer task, core 0) instead, the stop chain is
// bounded: press -> detected within one tick -> stop actions fire
// from C, regardless of what Python is doing.
//
// Split of responsibilities:
//   * This core: sampling, debounce, and the press state machine —
//     pure C, GPIO read injected, unix/c-unit testable.
//   * The binding (motor_process.c's dispatcher): calls
//     ob_button_tick each hard tick and maps events to actions —
//     STOP: mp_sched_keyboard_interrupt (ISR-safe by design) + the
//     native bus's torque-off; START: an atomic flag the launcher's
//     main-thread idle loop consumes (only the main thread can exec
//     a program; start is not latency-critical).
//   * The Python launcher: arms/disarms around program runs and
//     keeps its existing scheduler-path handling as defence in
//     depth (classic-bus robots also need the Python e-stop for
//     their torque-off).
//
// Context contract: everything here runs on the esp_timer task. No
// MicroPython, no IDF headers, no allocation, no blocking.

#pragma once

#include <stdint.h>

// Debounce: a level change must hold this many consecutive ticks
// (1 kHz) to be believed. The launcher's 50 ms poll needed 2 polls
// (100 ms); at 1 kHz, 20 ms of stability rejects the same contact
// chatter with 5x less latency.
#define OB_BUTTON_DEBOUNCE_TICKS 20

typedef enum {
    OB_BUTTON_NONE = 0,
    OB_BUTTON_PRESSED,      // debounced press edge (down)
    OB_BUTTON_RELEASED,     // debounced release edge (up)
} ob_button_event_t;

typedef struct {
    // Injected sampler: returns nonzero when the (active-low) button
    // reads PRESSED. The firmware backend is a GPIO-read shim; tests
    // inject a scripted level.
    int  (*read_pressed)(void *ctx);
    void *ctx;

    uint8_t  stable_pressed;    // debounced level
    uint8_t  raw_last;
    uint16_t raw_count;         // consecutive ticks at raw_last
    uint32_t n_presses;         // cumulative debounced press edges
    uint32_t n_releases;
} ob_button_t;

void ob_button_init(ob_button_t *b,
                    int (*read_pressed)(void *ctx), void *ctx);

// One hard tick: sample, debounce, return the edge event (if any).
ob_button_event_t ob_button_tick(ob_button_t *b);
