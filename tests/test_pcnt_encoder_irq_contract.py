# SPDX-License-Identifier: MIT
"""
Regression guards for the PCNTEncoder IRQ-handler reentrancy traps.

History:

* 1.4.0 added a threshold IRQ that called ``pcnt.value()`` to drain
  the counter. ``pcnt.value()`` runs a synchronous-flush loop that
  re-invokes the handler whenever ``irq.flags`` is set — so even
  with flags cleared up front, a hardware IRQ arriving during the
  handler's drain step reentered us. Hung the chip on the first
  threshold crossing.

* 1.4.1 cleared ``irq.flags()`` at the top of the handler. Still
  hung — clear-then-drain doesn't help if hardware fires during
  the drain.

* 1.4.2 redesigned: configure min/max for hardware auto-reset,
  add the known limit value (±PCNT_LIMIT) to ``accum`` from the
  handler. The handler no longer touches ``pcnt.value()`` at all,
  so reentrancy can't happen.

These tests guard the 1.4.2 contract: the handler must NOT call
the helper that touches pcnt.value() (``pcnt_drain_to_accum``,
which we removed), AND must still atomically clear flags via
``info(MP_IRQ_INFO_FLAGS)`` so the flush loop in the OUTER
``pcnt.value()`` (which is what dispatched us) breaks out.
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

    def test_dispatcher_does_not_call_pcnt_value(self):
        """The 1.4.2 redesign: the handler must NOT call back into
        ``pcnt.value()`` (which is what re-dispatches us via the
        upstream synchronous-flush loop). Verify by checking the
        dispatcher body contains no reference to ``pcnt_read_raw``,
        ``pcnt_write_raw_zero``, or ``pcnt_drain_to_accum`` — the
        helpers from 1.4.0/1.4.1 that all funneled through
        pcnt.value()."""
        m = re.search(
            r"_pcnt_encoder_irq_dispatch\([^)]*\)\s*\{(?P<body>.*?)\n\}",
            self.src, re.DOTALL,
        )
        self.assertIsNotNone(m)
        body = m.group("body")
        for forbidden in ("pcnt_read_raw", "pcnt_write_raw_zero",
                          "pcnt_drain_to_accum"):
            self.assertNotIn(
                forbidden, body,
                "dispatcher must not call ``%s`` — that helper goes "
                "through pcnt.value(), which the upstream PCNT runs in "
                "a synchronous-flush loop that re-invokes our handler. "
                "Hardware-confirmed reentrancy hang in 1.4.0/1.4.1."
                % forbidden
            )

    def test_dispatcher_uses_min_max_event_decoding(self):
        """The 1.4.2 dispatcher accounts for hardware auto-reset by
        adding the known limit value to ``accum`` based on which
        IRQ_MIN / IRQ_MAX event fired. Verify the source references
        the event-decoding statics."""
        m = re.search(
            r"_pcnt_encoder_irq_dispatch\([^)]*\)\s*\{(?P<body>.*?)\n\}",
            self.src, re.DOTALL,
        )
        body = m.group("body")
        self.assertIn("_pcnt_evt_h_lim", body,
                      "dispatcher must check the IRQ_MAX flag bit so "
                      "it can add +PCNT_LIMIT to accum on high-limit "
                      "auto-reset")
        self.assertIn("_pcnt_evt_l_lim", body,
                      "dispatcher must check the IRQ_MIN flag bit so "
                      "it can subtract PCNT_LIMIT from accum on "
                      "low-limit auto-reset")


if __name__ == "__main__":
    unittest.main()
