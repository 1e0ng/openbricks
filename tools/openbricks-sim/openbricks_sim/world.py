# SPDX-License-Identifier: MIT
"""
World loading + chassis injection.

MuJoCo supports ``<include file="..."/>`` but the includable file
must itself be a complete MJCF. Our chassis is a *fragment* (a
``<body>``, an ``<actuator>``, and a ``<sensor>`` section with no
outer ``<mujoco>`` envelope) so that one generator can target many
worlds. To splice the fragment into a world MJCF we do a small
amount of textual surgery:

* The chassis's ``<worldbody>`` body goes just before the world's
  closing ``</worldbody>``.
* The chassis's ``<actuator>`` section lands just before the
  world's closing ``</mujoco>`` (creating one if the world didn't
  have any actuators).
* Ditto for ``<sensor>``.

That's simpler than programmatic MJCF manipulation with a DOM, and
fragile only in the sense that a world MJCF with very creative
formatting might break the substitutions. All shipped worlds follow
the same shape, so this works.
"""

from pathlib import Path
import re

import mujoco

from openbricks_sim.chassis import ChassisSpec, chassis_mjcf


class WorldLoadError(Exception):
    pass


def _extract_fragment_sections(fragment: str):
    """Split the chassis fragment into (body, actuators, sensors)."""
    def _grab(tag):
        m = re.search(
            r"  <" + tag + r">.*?  </" + tag + r">\n",
            fragment, re.DOTALL)
        return m.group(0) if m else ""
    # The chassis body lives inside the fragment's ``<worldbody>``.
    body = ""
    m = re.search(r"  <worldbody>\n(.*?)  </worldbody>\n",
                  fragment, re.DOTALL)
    if m:
        body = m.group(1)
    return body, _grab("actuator"), _grab("sensor")


def _inject(world_xml: str, body: str, actuators: str, sensors: str) -> str:
    """Return a new MJCF string with the chassis fragment spliced in."""
    # World body insertion.
    if "</worldbody>" not in world_xml:
        raise WorldLoadError("world MJCF has no </worldbody> to splice into")
    merged = world_xml.replace("</worldbody>",
                               body + "\n  </worldbody>", 1)
    # Actuators — insert before </mujoco>.
    if "<actuator>" in merged:
        merged = merged.replace("<actuator>",
                                actuators.strip() + "\n  <actuator>", 1)
    else:
        merged = merged.replace("</mujoco>", actuators + "</mujoco>", 1)
    # Sensors.
    if "<sensor>" in merged:
        merged = merged.replace("<sensor>",
                                sensors.strip() + "\n  <sensor>", 1)
    else:
        merged = merged.replace("</mujoco>", sensors + "</mujoco>", 1)
    return merged


def load_world(world_path: str,
               chassis_spec: ChassisSpec = None,
               chassis_name: str = "chassis"):
    """Load a world MJCF from disk and splice in the default chassis.

    Returns a tuple ``(mujoco.MjModel, mujoco.MjData, merged_mjcf)``
    ready to step. The ``merged_mjcf`` string is returned mostly for
    test / debugging visibility — callers normally work with the
    MjModel + MjData only.
    """
    p = Path(world_path)
    if not p.is_file():
        raise WorldLoadError("world file not found: " + str(world_path))
    world_xml = p.read_text()

    fragment = chassis_mjcf(chassis_spec, name=chassis_name)
    body, actuators, sensors = _extract_fragment_sections(fragment)
    merged = _inject(world_xml, body, actuators, sensors)

    try:
        # ``from_xml_string`` resolves relative texture paths against
        # the CWD, not the world file. Chdir trick so ``mat.png``
        # refs inside the world resolve.
        import os
        cwd = os.getcwd()
        os.chdir(str(p.parent))
        try:
            model = mujoco.MjModel.from_xml_string(merged)
        finally:
            os.chdir(cwd)
    except Exception as e:
        raise WorldLoadError(
            "MuJoCo couldn't parse the merged MJCF: {}".format(e)) from e

    data = mujoco.MjData(model)
    return model, data, merged
