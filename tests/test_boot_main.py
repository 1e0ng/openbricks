# SPDX-License-Identifier: MIT
"""Boot behavior of the frozen ``_openbricks_main.py``.

Executes the actual frozen boot script (with hardware fakes installed
and ``launcher.run`` stubbed) and asserts what each NVS state wires:

* named + BLE persisted **on** → radio up, hub wired, launcher runs;
* named + BLE persisted **off** → radio down but hub + launcher still
  come up. This is the 1.10.2 fix: the old gate (``ble_repl
  .is_running()``) returned early on persisted-off, so one button
  toggle to "off" plus a reboot left the hub with a dark LED, both
  buttons dead, and no way to re-enable BLE short of USB or a
  full-erase reflash;
* no hub name (fresh chip) → drop to REPL so the ``openbricks flash``
  flow can write the name over mpremote; launcher must NOT run.
"""

import tests._fakes           # noqa: F401
import tests._fakes_ble       # noqa: F401
import tests._fakes_neopixel  # noqa: F401  (S3 hub LED)

import unittest

from machine import Timer
from tests._fakes_ble import _FakeBLE, _FakeNVS

import openbricks
from openbricks import ble_repl, bluetooth, launcher, pins


_here = __file__
_idx = _here.rfind("/")
_BOOT = (_here[:_idx] if _idx >= 0 else ".") + \
    "/../native/frozen/_openbricks_main.py"


def _run_boot():
    """Execute the frozen boot script the way ``boot.py`` does
    (module-level import runs ``_main()``)."""
    with open(_BOOT) as f:
        src = f.read()
    exec(compile(src, "_openbricks_main.py", "exec"),
         {"__name__": "_openbricks_main_under_test"})


class BootMainTests(unittest.TestCase):
    def setUp(self):
        _FakeNVS._reset_for_test()
        _FakeBLE._reset_for_test()
        Timer.reset_for_test()
        pins._claims_reset()
        del bluetooth._state_listeners[:]
        # The NUS bridge is module-level state; a bridge left over from
        # a previous case must not leak into the next boot.
        ble_repl._state["bridge"] = None
        ble_repl._state["stream"] = None
        launcher._singleton = None
        self._real_run = launcher.run
        self._run_calls = []
        calls = self._run_calls

        def _stub_run(*args, **kwargs):
            calls.append((args, kwargs))
        launcher.run = _stub_run

    def tearDown(self):
        launcher.run = self._real_run
        launcher._singleton = None
        pins._claims_reset()
        del bluetooth._state_listeners[:]
        Timer.reset_for_test()

    def _plant_name(self):
        nvs = _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE)
        nvs.set_blob(openbricks._HUB_NAME_NVS_KEY, b"TestHub")
        nvs.commit()

    def _persist_ble(self, enabled):
        nvs = _FakeNVS(bluetooth._NAMESPACE)
        nvs.set_i32(bluetooth._KEY, 1 if enabled else 0)
        nvs.commit()

    def test_named_ble_on_wires_everything(self):
        self._plant_name()
        self._persist_ble(True)
        _run_boot()
        self.assertTrue(_FakeBLE().active(), "radio must be up")
        self.assertTrue(38 in pins._claims,
                        "BLE button must be wired (S3 default 38 — "
                        "off ADC1, which sensors need)")
        self.assertFalse(5 in pins._claims,
                         "GPIO 5 must stay free for analog use")
        self.assertEqual(len(self._run_calls), 1,
                         "launcher.run must be entered")

    def test_named_ble_off_still_wires_hub_and_launcher(self):
        # THE trap: user pressed the BLE toggle off, then rebooted.
        self._plant_name()
        self._persist_ble(False)
        _run_boot()
        self.assertFalse(_FakeBLE().active(), "radio must stay down")
        self.assertTrue(
            38 in pins._claims,
            "BLE button must be wired even with BLE persisted off — "
            "it's the only way to turn BLE back on without a USB "
            "cable or a full-erase reflash")
        self.assertEqual(
            len(self._run_calls), 1,
            "launcher must run so the program button keeps working")

    def test_no_name_drops_to_repl_for_setup(self):
        # Fresh chip: mpremote needs the REPL to write the hub name;
        # the blocking launcher loop must not start.
        self._persist_ble(True)
        _run_boot()
        self.assertEqual(self._run_calls, [],
                         "nameless chip must drop to the REPL")
        self.assertFalse(_FakeBLE().active())


if __name__ == "__main__":
    unittest.main()
