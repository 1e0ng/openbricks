# SPDX-License-Identifier: MIT
"""
World loading + chassis injection.

A map is JSON (``map.json``, :mod:`openbricks_sim.mapfile`); MuJoCo
reads MJCF, so the map is made into MJCF text in memory and the chassis
spliced into it. The chassis is a *fragment* (a ``<body>``, an
``<actuator>``, and a ``<sensor>`` section with no outer ``<mujoco>``
envelope) so that one generator can target many worlds. To splice the
fragment into the map's MJCF we do a small amount of textual surgery:

* The chassis's ``<worldbody>`` body goes just before the world's
  closing ``</worldbody>``.
* The chassis's ``<actuator>`` section lands just before the
  world's closing ``</mujoco>`` (creating one if the world didn't
  have any actuators).
* Ditto for ``<sensor>``.

That's simpler than programmatic MJCF manipulation with a DOM, and
safe because the map's MJCF is ours: :func:`mapfile.to_mjcf` writes it
in one shape.
"""

from pathlib import Path
import re

import mujoco

from openbricks_sim.chassis import ChassisSpec, chassis_mjcf
from openbricks_sim import mapfile


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
               chassis_name: str = "chassis",
               inertial=None, extra_geoms=None, world_map=None):
    """Load a map (``map.json``, see :mod:`openbricks_sim.mapfile`) and
    splice in the default chassis.

    Returns a tuple ``(mujoco.MjModel, mujoco.MjData, merged_mjcf)``
    ready to step. The ``merged_mjcf`` string — the MJCF MuJoCo is given,
    made in memory from the map — is returned for test / debugging
    visibility; callers normally work with the MjModel + MjData only.
    ``world_map`` replaces the file's map (the map editor's edited
    props); the file's directory still resolves the artwork and the
    props' models. Maps are JSON: an MJCF ``world.xml`` is refused.
    """
    p = Path(world_path)
    if p.suffix == ".xml":
        raise WorldLoadError(
            "maps are JSON since 4.32.0: %s is MJCF. Load its folder's map.json; a map of your own "
            "saved before then is converted the first time the sim lists it" % (world_path,))
    if not p.is_file():
        raise WorldLoadError("world file not found: " + str(world_path))
    try:
        m = mapfile.load(p) if world_map is None else mapfile.check(world_map)
        # the props are expanded into bodies as the model is made, so an edited .ldr or build
        # needs no build step
        world_xml = mapfile.to_mjcf(m, p.parent)
    except mapfile.MapError as e:
        raise WorldLoadError(str(e)) from e

    fragment = chassis_mjcf(chassis_spec, name=chassis_name,
                            inertial=inertial, extra_geoms=extra_geoms)
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
