Parameters
==========

Enumerated options — the Pybricks ``pybricks.parameters`` pattern.
Every API argument that selects one of a fixed set of behaviours
takes one of these members, never a string::

    from openbricks.parameters import Stop, DriveMode, LineMode

    db.straight(300, then=Stop.BRAKE)
    db = DriveBase(left, right, 88, 136, drive=DriveMode.WHEEL)
    qtr.set_mode(LineMode.CENTER)

A string is rejected at the call with a ``TypeError`` that names the
members, so a typo or a drifted spelling can never silently select a
different behaviour.

.. automodule:: openbricks.parameters
   :members:
   :undoc-members:
   :show-inheritance:
