"""In-memory URDF model.

The model wraps the original XML elements instead of copying every field into
dataclasses. Anything the pipeline does not explicitly change (custom tags, sensor
blocks, unusual attributes) is therefore written back exactly as it was read, which is
what makes the split/rebuild round trip lossless for *any* URDF.
"""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass, field

JOINT_TYPES = frozenset({"fixed", "revolute", "continuous", "prismatic", "floating", "planar"})


class Link:
    """A <link> element."""

    __slots__ = ("element",)

    def __init__(self, element: ET.Element) -> None:
        self.element = element

    @property
    def name(self) -> str:
        return self.element.get("name", "")

    @property
    def has_geometry(self) -> bool:
        return self.element.find("visual") is not None or self.element.find("collision") is not None

    @property
    def mass(self) -> float | None:
        mass = self.element.find("inertial/mass")
        if mass is None:
            return None
        try:
            return float(mass.get("value", "0"))
        except ValueError:
            return None


class Joint:
    """A <joint> element."""

    __slots__ = ("element",)

    def __init__(self, element: ET.Element) -> None:
        self.element = element

    @property
    def name(self) -> str:
        return self.element.get("name", "")

    @property
    def type(self) -> str:
        return self.element.get("type", "")

    def _link_ref(self, tag: str) -> str | None:
        ref = self.element.find(tag)
        return None if ref is None else ref.get("link")

    @property
    def parent(self) -> str | None:
        return self._link_ref("parent")

    @property
    def child(self) -> str | None:
        return self._link_ref("child")

    @property
    def mimic(self) -> str | None:
        mimic = self.element.find("mimic")
        return None if mimic is None else mimic.get("joint")


@dataclass
class Robot:
    """A parsed URDF: links, joints and every other top-level element ("extras")."""

    name: str
    attrib: dict[str, str]
    links: dict[str, Link] = field(default_factory=dict)
    joints: dict[str, Joint] = field(default_factory=dict)
    extras: list[ET.Element] = field(default_factory=list)
    # Problems found while parsing (e.g. duplicate names); reported by validate().
    problems: list[str] = field(default_factory=list)

    # -- editing -------------------------------------------------------------------
    def add_link(self, element: ET.Element) -> Link:
        link = Link(element)
        self.links[link.name] = link
        return link

    def add_joint(self, element: ET.Element) -> Joint:
        joint = Joint(element)
        self.joints[joint.name] = joint
        return joint

    def remove_link(self, name: str) -> None:
        del self.links[name]

    def remove_joint(self, name: str) -> None:
        del self.joints[name]

    def copy(self) -> Robot:
        return Robot(
            name=self.name,
            attrib=dict(self.attrib),
            links={n: Link(copy.deepcopy(l.element)) for n, l in self.links.items()},
            joints={n: Joint(copy.deepcopy(j.element)) for n, j in self.joints.items()},
            extras=[copy.deepcopy(e) for e in self.extras],
            problems=list(self.problems),
        )

    # -- tree queries --------------------------------------------------------------
    def parent_joints(self) -> dict[str, Joint]:
        """child link name -> the joint that attaches it."""
        result: dict[str, Joint] = {}
        for joint in self.joints.values():
            if joint.child and joint.child not in result:
                result[joint.child] = joint
        return result

    def child_joints(self) -> dict[str, list[Joint]]:
        """parent link name -> joints hanging off it, sorted by joint name (deterministic)."""
        result: dict[str, list[Joint]] = {}
        for joint in self.joints.values():
            if joint.parent:
                result.setdefault(joint.parent, []).append(joint)
        for joints in result.values():
            joints.sort(key=lambda j: j.name)
        return result

    def roots(self) -> list[str]:
        children = {j.child for j in self.joints.values() if j.child}
        return [name for name in self.links if name not in children]

    def tree_order(self, start: str | None = None) -> list[str]:
        """Link names reachable from `start` (default: the single root), parents first."""
        if start is None:
            roots = self.roots()
            if len(roots) != 1:
                return []
            start = roots[0]
        children = self.child_joints()
        order: list[str] = []
        seen = {start}
        queue = deque([start])
        while queue:
            name = queue.popleft()
            order.append(name)
            for joint in children.get(name, []):
                if joint.child and joint.child in self.links and joint.child not in seen:
                    seen.add(joint.child)
                    queue.append(joint.child)
        return order
