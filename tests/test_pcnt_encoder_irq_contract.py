# SPDX-License-Identifier: MIT
"""
Regression guard for the firmware 1.4.0 IRQ-handler-must-clear-flags
contract.

Looking at upstream ``ports/esp32/esp32_pcnt.c::esp32_pcnt_value()``:

    while (true) {
        pcnt_get_counter_value(...);
        if (self->irq && self->irq->flags && handler != none) {
            // The handler must call irq.flags() to clear
            // self->irq->base.flags, otherwise this will be an
            // infinite loop.
            mp_call_function_1(handler, ...);
            continue;
        }
        break;
    }

— ``pcnt.value()`` synchronously re-invokes the IRQ handler until
the flags are cleared. Our handler calls ``pcnt.value()`` to drain
the counter, so without an up-front flag-clear the first
``pcnt_read_raw`` inside the handler recurses infinitely (handler
→ value() → handler → value() → ...) and locks up the chip.

Hardware confirmed: a few wheel rotations in
``prob_pcnt_by_hand.py`` were enough to trip the threshold IRQ; the
chip went silent (BLE alive but no further script output) until the
30-second openbricks-run read timeout. Fix is to call
``irq.methods->info(irq, MP_IRQ_INFO_FLAGS)`` before draining.

This file is a static guard on the C source — if anyone refactors
the handler and forgets the flag-clear, the test fails before the
firmware reaches hardware.
"""

import os
import re
import unittest


_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
_PCNT_C    = os.path.join(
    _REPO_ROOT, "native", "user_c_modules", "openbricks", "pcnt_encoder.c")


class IRQDispatcherContractTests(unittest.TestCase):

    def setUp(self):
        with open(_PCNT_C) as f:
            self.src = f.read()

    def test_dispatcher_clears_irq_flags(self):
        """The dispatcher must call ``info(..., MP_IRQ_INFO_FLAGS)`` —
        the only API that atomically clears ``self->irq->flags`` so
        the synchronous-flush loop inside ``pcnt.value()`` exits."""
        # Find the dispatcher body — between its definition and the
        # closing brace of the function.
        m = re.search(
            r"_pcnt_encoder_irq_dispatch\([^)]*\)\s*\{(?P<body>.*?)\n\}",
            self.src, re.DOTALL,
        )
        self.assertIsNotNone(m, "dispatcher function not found in source")
        body = m.group("body")
        self.assertIn(
            "MP_IRQ_INFO_FLAGS", body,
            "dispatcher must call info(..., MP_IRQ_INFO_FLAGS) to clear "
            "the IRQ flags — without it, pcnt.value() recurses into the "
            "handler infinitely (esp32_pcnt_value runs a while-true that "
            "re-invokes the handler until flags are cleared)."
        )

    def test_flags_clear_happens_before_pcnt_drain(self):
        """The clear has to happen BEFORE ``pcnt_drain_to_accum`` is
        called, because the drain itself goes through ``pcnt.value()``
        — which is what triggers the flush loop. A late clear would
        still recurse on the first drain."""
        m = re.search(
            r"_pcnt_encoder_irq_dispatch\([^)]*\)\s*\{(?P<body>.*?)\n\}",
            self.src, re.DOTALL,
        )
        body = m.group("body")
        flags_idx = body.find("MP_IRQ_INFO_FLAGS")
        drain_idx = body.find("pcnt_drain_to_accum")
        self.assertGreater(flags_idx, -1)
        self.assertGreater(drain_idx, -1)
        self.assertLess(
            flags_idx, drain_idx,
            "flags must be cleared (info(..., MP_IRQ_INFO_FLAGS)) BEFORE "
            "pcnt_drain_to_accum runs; otherwise the drain's pcnt.value() "
            "call sees flags still set and recurses into us."
        )


if __name__ == "__main__":
    unittest.main()
