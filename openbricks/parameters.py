# SPDX-License-Identifier: MIT
"""
Enumerated parameters — the Pybricks ``pybricks.parameters`` pattern.

Every API argument that selects one of a fixed set of behaviours takes
a member of one of these classes, never a string. A string compares
equal to nothing here, so a typo (``then="cost"``) or a drifted
spelling (``"continue"`` vs ``"none"``) fails at the call with a
message naming the members, instead of silently selecting a default
somewhere downstream. Members print as ``Stop.COAST`` like Pybricks'
``_PybricksEnum``, and ``Stop`` carries the Pybricks numbering
(``COAST=0, BRAKE=1, HOLD=2, NONE=3``), which the native drivebase
stop codes share.

Members compare by (enum name, member name) rather than by object
identity: the simulator drops the firmware modules from ``sys.modules``
between runs and re-imports them, so a ``Stop.COAST`` bound before the
reload must still equal the one created after it. A string compares
equal to no member either way.

MicroPython ships no ``enum`` module, so this is the smallest class
that gives the same surface: ``.name``, ``.value``, ``repr``, and a
class-level member list.

Example::

    from openbricks.parameters import Stop, LineMode
    db.straight(300, then=Stop.BRAKE)
    qtr.set_mode(LineMode.CENTER)
"""


class _Enum:
    """Base for the parameter enums. Members are class attributes
    created once at import; ``type(member)`` is the enum class."""

    _members = ()
    # Reload-stable type marker: after the sim re-imports this module
    # ``_Enum`` is a new class, so equality can't use isinstance.
    _is_parameter_enum = True

    def __init__(self, name, value):
        self.name = name
        self.value = value

    def __repr__(self):
        return "%s.%s" % (type(self).__name__, self.name)

    __str__ = __repr__

    def __eq__(self, other):
        return (getattr(other, "_is_parameter_enum", False) is True
                and type(other).__name__ == type(self).__name__
                and other.name == self.name)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((type(self).__name__, self.name))

    @classmethod
    def members(cls):
        """The members, in declaration order."""
        return cls._members


def _define(cls, *pairs):
    members = []
    for name, value in pairs:
        m = cls(name, value)
        setattr(cls, name, m)
        members.append(m)
    cls._members = tuple(members)
    return cls


class Stop(_Enum):
    """What a motor or drivebase does when a move ends — Pybricks
    ``Stop``.

    * ``COAST`` — cut torque; the wheel free-wheels.
    * ``BRAKE`` — actively hold zero speed (passively resist on
      open-loop motors).
    * ``HOLD`` — keep controlling the motor to hold the reached
      position.
    * ``NONE`` — do not decelerate; the move hands its end speed to
      the next command (concatenate maneuvers without stopping).
    """


_define(Stop, ("COAST", 0), ("BRAKE", 1), ("HOLD", 2), ("NONE", 3))


class DriveMode(_Enum):
    """How a serial-bus wheel servo is driven by the drivebase engine.

    * ``DUTY`` — the servo runs open-loop and the engine's FF+PI loop
      is the speed controller (the default since 1.89.0).
    * ``WHEEL`` — the servo's own wheel-mode velocity loop closes the
      speed; the engine commands speed setpoints.
    """


_define(DriveMode, ("DUTY", 0), ("WHEEL", 1))


class LineMode(_Enum):
    """Which feature of the line a QTR array follows.

    * ``LEFT`` — the line's left edge, under the left setpoint element.
    * ``RIGHT`` — the line's right edge, under the right setpoint element.
    * ``CENTER`` — the line's centre, the weighted centroid over every
      element.
    """


_define(LineMode, ("LEFT", 0), ("RIGHT", 1), ("CENTER", 2))


def check(enum_cls, value, param, allowed=None):
    """Raise ``TypeError`` unless ``value`` is a member of ``enum_cls``
    (and of ``allowed``, when the call accepts only a subset). The
    message names the accepted members; a string argument is called
    out explicitly, since that is the mistake this module exists to
    catch."""
    members = allowed if allowed is not None else enum_cls.members()
    for m in members:
        if value == m:
            return value
    names = ", ".join(repr(m) for m in members)
    if isinstance(value, str):
        raise TypeError(
            "%s must be one of %s — not the string %r (import %s from "
            "openbricks.parameters)" % (param, names, value,
                                        enum_cls.__name__))
    raise TypeError("%s must be one of %s, got %r" % (param, names, value))
