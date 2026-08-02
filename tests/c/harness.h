// SPDX-License-Identifier: MIT
//
// Minimal C test harness for the user_c_modules pure cores.
//
// The cores are MicroPython-free POD C by design (see each header's
// "No MicroPython headers" note) — so they compile as plain host
// executables, which is what buys these tests their value over the
// binding-driven suite in tests/*.py:
//   * every run is under -fsanitize=address,undefined (a silent
//     buffer overflow aborts loudly here; under MP it corrupts the
//     heap quietly),
//   * edge states are set directly on the structs instead of being
//     coaxed through Python,
//   * cc + run is milliseconds, and lldb/gdb attach trivially.
//
// One executable per core; scripts/run_c_tests.sh builds and runs
// them all. No framework dependency on purpose.

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int t_checks;
static int t_failures;

#define CHECK(cond) do { \
        t_checks++; \
        if (!(cond)) { \
            t_failures++; \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

#define CHECK_EQ_INT(a, b) do { \
        long long va = (long long)(a), vb = (long long)(b); \
        t_checks++; \
        if (va != vb) { \
            t_failures++; \
            fprintf(stderr, "FAIL %s:%d: %s == %s (%lld != %lld)\n", \
                    __FILE__, __LINE__, #a, #b, va, vb); \
        } \
    } while (0)

#define TEST(name) static void name(void)
#define RUN(name) do { \
        fprintf(stderr, "  %s\n", #name); \
        name(); \
    } while (0)

static int harness_exit(const char *suite) {
    if (t_failures) {
        fprintf(stderr, "%s: %d/%d checks FAILED\n", suite,
                t_failures, t_checks);
        return 1;
    }
    fprintf(stderr, "%s: %d checks ok\n", suite, t_checks);
    return 0;
}
