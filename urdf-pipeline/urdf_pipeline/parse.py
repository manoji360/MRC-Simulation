"""Read, validate and write URDF files."""

from __future__ import annotations

import copy
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from .errors import PipelineError, ValidationError
from .model import JOINT_TYPES, Robot

_XACRO_MARKERS = (b"ros.org/wiki/xacro", b"<xacro:", b"$(find ")
XACRO_HINT = (
    "Expand it to plain URDF first. In the WSL Ubuntu terminal: "
    "xacro robot.urdf.xacro > robot.urdf  (see urdf-pipeline/env/wsl2-setup.md)."
)


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def load_urdf(path: str | Path) -> Robot:
    """Parse a URDF file. Raises PipelineError for unreadable or non-URDF files.

    Does not check the kinematic structure; call validate() (or load_valid_urdf()).
    """
    path = Path(path)
    if not path.is_file():
        raise PipelineError(
            f"File not found: {path}",
            hint="Check the path. Tip: drag the .urdf file onto prepare_urdf.bat.",
        )
    data = path.read_bytes()
    if path.suffix.lower() == ".xacro" or is_xacro(data):
        raise PipelineError(f"{path.name} is a xacro file, not plain URDF.", hint=XACRO_HINT)
    return parse_urdf_string(data, source=path.name)


_COMMENT = re.compile(rb"<!--.*?-->", re.DOTALL)


def is_xacro(data: bytes) -> bool:
    """True if the file still needs xacro. Commented-out xacro lines (which xacro itself
    leaves in its output) don't count."""
    active = _COMMENT.sub(b"", data)
    return any(marker in active for marker in _XACRO_MARKERS)


def load_valid_urdf(path: str | Path) -> Robot:
    robot = load_urdf(path)
    problems = validate(robot)
    if problems:
        raise ValidationError(problems, Path(path).name)
    return robot


def parse_urdf_string(data: str | bytes, source: str = "<string>") -> Robot:
    try:
        root = ET.fromstring(data)
    except ET.ParseError as exc:
        raise PipelineError(
            f"{source} is not valid XML: {exc}",
            hint="Open the file in a text editor and check the line/column mentioned above.",
        ) from None
    if _local(root.tag) != "robot":
        raise PipelineError(
            f"{source} is not a URDF: its top-level element is <{_local(root.tag)}>, expected <robot>."
        )

    robot = Robot(name=root.get("name", ""), attrib=dict(root.attrib))
    if not robot.name:
        robot.problems.append("The <robot> element has no name attribute.")
    for child in root:
        if not isinstance(child.tag, str):  # comments / processing instructions
            continue
        if child.tag in ("link", "joint"):
            table = robot.links if child.tag == "link" else robot.joints
            name = child.get("name")
            if not name:
                robot.problems.append(f"A <{child.tag}> element has no name attribute.")
            elif name in table:
                robot.problems.append(f"{child.tag.capitalize()} '{name}' is defined more than once.")
            elif child.tag == "link":
                robot.add_link(child)
            else:
                robot.add_joint(child)
        else:
            robot.extras.append(child)
    return robot


def validate(robot: Robot) -> list[str]:
    """Return every structural problem found (empty list = valid)."""
    problems = list(robot.problems)
    if not robot.links:
        problems.append("The robot has no <link> elements.")
        return problems

    parents_of: dict[str, list[str]] = {}
    for joint in robot.joints.values():
        name = joint.name
        if joint.type not in JOINT_TYPES:
            problems.append(
                f"Joint '{name}' has type '{joint.type}'; expected one of: {', '.join(sorted(JOINT_TYPES))}."
            )
        for role, link in (("parent", joint.parent), ("child", joint.child)):
            if not link:
                problems.append(f"Joint '{name}' has no <{role} link=\"...\"/>.")
            elif link not in robot.links:
                problems.append(f"Joint '{name}' refers to {role} link '{link}', which is not defined.")
        if joint.parent and joint.parent == joint.child:
            problems.append(f"Joint '{name}' connects link '{joint.parent}' to itself.")
        if joint.child:
            parents_of.setdefault(joint.child, []).append(name)
        if joint.mimic is not None and joint.mimic not in robot.joints:
            problems.append(f"Joint '{name}' mimics joint '{joint.mimic}', which is not defined.")

    for link, joints in parents_of.items():
        if len(joints) > 1 and link in robot.links:
            problems.append(
                f"Link '{link}' has {len(joints)} parent joints ({', '.join(joints)}); a link can have only one."
            )

    roots = robot.roots()
    if not roots:
        problems.append("No root link: every link is the child of a joint, so the joints form a loop.")
    elif len(roots) > 1:
        problems.append(
            f"Found {len(roots)} root links ({', '.join(roots)}); a URDF needs exactly one. "
            "Every link except the root must be attached by a joint."
        )
    else:
        reachable = set(robot.tree_order(roots[0]))
        unreached = [name for name in robot.links if name not in reachable]
        if unreached:
            problems.append(
                f"Links not connected to the root '{roots[0]}' (the joints form a loop): {', '.join(unreached)}."
            )
    return problems


# -- writing ---------------------------------------------------------------------------
def element_to_string(element: ET.Element) -> str:
    clone = copy.deepcopy(element)
    clone.tail = None
    ET.indent(clone, space="  ")
    return ET.tostring(clone, encoding="unicode")


def element_from_string(text: str) -> ET.Element:
    return ET.fromstring(text)


def to_xml_string(robot: Robot) -> str:
    attrib = dict(robot.attrib)
    attrib["name"] = robot.name
    root = ET.Element("robot", attrib)
    for element in (
        [l.element for l in robot.links.values()]
        + [j.element for j in robot.joints.values()]
        + list(robot.extras)
    ):
        root.append(copy.deepcopy(element))
    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def write_urdf(robot: Robot, path: str | Path) -> None:
    Path(path).write_text(to_xml_string(robot), encoding="utf-8", newline="\n")
