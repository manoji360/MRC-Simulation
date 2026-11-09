"""Collapse "frame-only" links out of the physics tree.

A frame-only link is a pure coordinate frame: no visual, no collision, no mass, attached
by a fixed joint, and every link below it is frame-only too (camera optical frames, IMU
frames, ...). Unity's URDF Importer still makes each one an ArticulationBody, which uses
up the 64-link budget. Collapsed frames are stored in the manifest (in both ROS and
Unity coordinates) so they can be re-created in Unity as plain Transforms.
"""

from __future__ import annotations

from .coordinates import unity_block_from_origin
from .kinematics import parse_origin
from .model import Robot
from .parse import element_from_string, element_to_string


def _bare(robot: Robot, name: str) -> bool:
    link = robot.links[name]
    if link.has_geometry:
        return False
    return link.element.find("inertial") is None or (link.mass or 0.0) == 0.0


def frame_only_links(robot: Robot) -> list[str]:
    """Frame-only link names, parents before children. Empty if the tree is invalid."""
    order = robot.tree_order()
    if not order:
        return []
    parents = robot.parent_joints()
    children = robot.child_joints()
    is_frame: dict[str, bool] = {}
    for name in reversed(order):
        joint = parents.get(name)
        is_frame[name] = (
            joint is not None
            and joint.type == "fixed"
            and _bare(robot, name)
            and all(is_frame.get(j.child, False) for j in children.get(name, []))
        )
    return [name for name in order if is_frame[name]]


def collapse_frames(robot: Robot) -> list[dict]:
    """Remove frame-only links (and their joints) from `robot`; return manifest entries."""
    names = frame_only_links(robot)
    parents = robot.parent_joints()
    entries = []
    for name in names:
        joint = parents[name]
        xyz, rpy = parse_origin(joint.element)
        entries.append(
            {
                "link": name,
                "parent_link": joint.parent,
                "joint": joint.name,
                "origin": {"xyz": list(xyz), "rpy": list(rpy)},
                "unity": unity_block_from_origin(xyz, rpy),
                "link_xml": element_to_string(robot.links[name].element),
                "joint_xml": element_to_string(joint.element),
            }
        )
    for name in names:
        robot.remove_joint(parents[name].name)
        robot.remove_link(name)
    return entries


def restore_frames(robot: Robot, entries: list[dict]) -> None:
    for entry in entries:
        robot.add_link(element_from_string(entry["link_xml"]))
        robot.add_joint(element_from_string(entry["joint_xml"]))
