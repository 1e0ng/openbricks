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

// Debounce: N-of-M majority vote over a sliding sample window with
// hysteresis, NOT consecutive-sample stability. The original rule
// (20 CONSECUTIVE ticks at the new level) was defeated by real
// contact chatter: one sub-ms glitch anywhere in the hold restarted
// the count, and the bench sampler missed 1-of-4 presses outright
// (2026-08-03, hard counters frozen while the chatter-immune PCNT
// edge counter caught the same press). Majority voting absorbs
// flicker: a press registers when >= ON_THRESH of the last WINDOW
// 1 kHz samples read pressed (clean press: 15 ms to fire, FASTER
// than the old 20), and releases only when <= OFF_THRESH read
// pressed — the 10-sample hysteresis gap means mid-hold chatter
// cannot toggle the state.
#define OB_BUTTON_WINDOW       20
#define OB_BUTTON_ON_THRESH    15
#define OB_BUTTON_OFF_THRESH    5

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

    uint32_t window;            // last WINDOW samples, bit 0 = newest
    uint8_t  win_count;         // pressed samples in the window
    uint8_t  stable_pressed;    // debounced level (hysteresis state)
    uint8_t  raw_last;          // most recent raw sample (diagnostics)
    uint32_t n_presses;         // cumulative debounced press edges
    uint32_t n_releases;
} ob_button_t;

void ob_button_init(ob_button_t *b,
                    int (*read_pressed)(void *ctx), void *ctx);

// One hard tick: sample, vote, return the edge event (if any).
ob_button_event_t ob_button_tick(ob_button_t *b);
