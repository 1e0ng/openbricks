# SPDX-License-Identifier: MIT
"""``openbricks.parameters`` — the enum surface every string-typed
option moved to. Members are singletons that print like Pybricks'
(``Stop.COAST``), strings are rejected with a message naming the
members, and ``Stop`` carries the native stop codes as values."""

import unittest

from openbricks import parameters
from openbricks.parameters import Stop, DriveMode, LineMode


class EnumSurfaceTests(unittest.TestCase):

    def test_members_are_singletons_of_their_class(self):
        for cls in (Stop, DriveMode, LineMode):
            for m in cls.members():
                self.assertTrue(isinstance(m, cls))
                self.assertTrue(getattr(cls, m.name) is m)

    def test_repr_is_pybricks_style(self):
        self.assertEqual(repr(Stop.COAST), "Stop.COAST")
        self.assertEqual(str(LineMode.CENTER), "LineMode.CENTER")
        self.assertEqual("%s" % DriveMode.WHEEL, "DriveMode.WHEEL")

    def test_strings_never_compare_equal(self):
        self.assertFalse(Stop.COAST == "coast")
        self.assertFalse("coast" == Stop.COAST)
        self.assertFalse(Stop.COAST in ("coast", "brake"))
        self.assertTrue(Stop.COAST in (Stop.COAST, Stop.BRAKE))

    def test_members_hash_as_dict_keys(self):
        d = {Stop.COAST: 1, Stop.BRAKE: 2}
        self.assertEqual(d[Stop.BRAKE], 2)
        self.assertFalse("brake" in d)

    def test_stop_values_are_the_pybricks_and_native_codes(self):
        # native db_stop / stop() take 0 coast, 1 brake, 2 hold.
        self.assertEqual([m.value for m in Stop.members()], [0, 1, 2, 3])
        self.assertEqual([m.name for m in Stop.members()],
                         ["COAST", "BRAKE", "HOLD", "NONE"])

    def test_members_survive_a_module_reload(self):
        # The sim re-imports the firmware package between runs; a
        # member bound before the reload must still equal, hash and
        # check() like the one created after it.
        import sys
        before = Stop.COAST
        saved = sys.modules.pop("openbricks.parameters")
        try:
            # __import__ rather than importlib: MicroPython has none.
            fresh = __import__("openbricks.parameters").parameters
        finally:
            sys.modules["openbricks.parameters"] = saved
        self.assertFalse(fresh.Stop.COAST is before)
        self.assertEqual(fresh.Stop.COAST, before)
        self.assertEqual(hash(fresh.Stop.COAST), hash(before))
        self.assertTrue(before in (fresh.Stop.COAST, fresh.Stop.BRAKE))
        self.assertTrue(fresh.check(fresh.Stop, before, "then") == before)
        self.assertFalse(fresh.Stop.BRAKE == before)

    def test_member_lists(self):
        self.assertEqual([m.name for m in DriveMode.members()],
                         ["DUTY", "WHEEL"])
        self.assertEqual([m.name for m in LineMode.members()],
                         ["LEFT", "RIGHT", "CENTER"])


class CheckTests(unittest.TestCase):

    def test_member_passes_through(self):
        self.assertTrue(parameters.check(Stop, Stop.HOLD, "then") is Stop.HOLD)

    def test_string_is_called_out_with_the_remedy(self):
        try:
            parameters.check(Stop, "coast", "then")
            self.fail("expected TypeError")
        except TypeError as e:
            msg = str(e)
            self.assertTrue(msg.startswith("then must be one of "), msg)
            for m in Stop.members():
                self.assertTrue(repr(m) in msg, msg)
            self.assertTrue("the string 'coast'" in msg, msg)
            self.assertTrue("import Stop from openbricks.parameters" in msg,
                            msg)

    def test_wrong_enum_class_is_rejected(self):
        try:
            parameters.check(Stop, LineMode.LEFT, "then")
            self.fail("expected TypeError")
        except TypeError as e:
            self.assertTrue("LineMode.LEFT" in str(e), e)

    def test_subset_restricts_the_accepted_members(self):
        allowed = (Stop.COAST, Stop.BRAKE, Stop.HOLD)
        self.assertTrue(parameters.check(Stop, Stop.HOLD, "then",
                                         allowed=allowed) is Stop.HOLD)
        try:
            parameters.check(Stop, Stop.NONE, "then", allowed=allowed)
            self.fail("expected TypeError")
        except TypeError as e:
            self.assertFalse("Stop.NONE" in str(e).split("got")[0], e)
            self.assertTrue("Stop.HOLD" in str(e), e)


if __name__ == "__main__":
    unittest.main()
