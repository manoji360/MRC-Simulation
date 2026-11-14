"""Unity URDF Importer compatibility fixes that do not change the robot's meaning.

Each fix rewrites something the URDF spec allows to be implicit into its explicit form, so
the robot is identical to any URDF reader, but Unity's importer no longer trips over it.
Every change is recorded in the manifest and undone by `rebuild`.

Known importer problems (com.unity.robotics.urdf-importer v0.5.2):
  - A revolute/continuous/prismatic joint without <axis> crashes the import with a
    NullReferenceException (UrdfJointContinuous.AdjustMovement reads joint.axis.xyz).
    The URDF default axis is (1, 0, 0), so we write <axis xyz="1 0 0"/> explicitly.
  - A link without <inertial> keeps Unity's default ArticulationBody mass of 1 kg
    (UrdfInertial only sets mass when <inertial> exists). In URDF such a link is massless;
    on the OSU rover this added 29 kg to a 50.6 kg robot. We add a tiny placeholder
    inertial (default 1 g), the closest Unity equivalent of "massless".
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

from .model import Robot

AXIS_JOINT_TYPES = ("revolute", "continuous", "prismatic")
DEFAULT_AXIS = "1 0 0"


def joints_missing_axis(robot: Robot) -> list[str]:
    return [j.name for j in robot.joints.values() if j.type in AXIS_JOINT_TYPES and j.element.find("axis") is None]


def add_default_axes(robot: Robot) -> list[str]:
    """Write the URDF default axis explicitly where it was left out. Returns joint names."""
    names = joints_missing_axis(robot)
    for name in names:
        ET.SubElement(robot.joints[name].element, "axis", {"xyz": DEFAULT_AXIS})
    return names


PLACEHOLDER_MARK = "urdf_pipeline_placeholder"


def links_missing_inertial(robot: Robot) -> list[str]:
    return [l.name for l in robot.links.values() if l.element.find("inertial") is None]


def add_placeholder_inertials(robot: Robot, mass_kg: float) -> list[str]:
    """Give every link without <inertial> a tiny mass so Unity doesn't assume 1 kg."""
    names = links_missing_inertial(robot)
    inertia = f"{mass_kg * 1e-4:.3g}"  # roughly a 1 cm sphere of that mass
    for name in names:
        inertial = ET.SubElement(robot.links[name].element, "inertial", {PLACEHOLDER_MARK: "true"})
        ET.SubElement(inertial, "mass", {"value": f"{mass_kg:g}"})
        ET.SubElement(inertial, "inertia", {"ixx": inertia, "iyy": inertia, "izz": inertia,
                                             "ixy": "0", "ixz": "0", "iyz": "0"})
    return names


def remove_placeholder_inertials(robot: Robot, names: list[str]) -> None:
    for name in names:
        link = robot.links.get(name)
        if link is None:
            continue
        for inertial in link.element.findall("inertial"):
            if inertial.get(PLACEHOLDER_MARK) == "true":
                link.element.remove(inertial)


def remove_added_axes(robot: Robot, names: list[str]) -> None:
    for name in names:
        joint = robot.joints.get(name)
        if joint is None:
            continue
        axis = joint.element.find("axis")
        if axis is not None and axis.get("xyz") == DEFAULT_AXIS and len(axis.attrib) == 1:
            joint.element.remove(axis)
