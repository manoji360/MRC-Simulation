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
"""Joint types the URDF specification allows."""


class Link:
    """A <link> element."""

    __slots__ = ("element",)
    """Link wraps one XML element; no per-instance dict, so copies stay cheap."""

    def __init__(self, element: ET.Element) -> None:
        """Wrap an existing <link> element (the element itself is kept and edited in place)."""
        self.element = element

    @property
    def name(self) -> str:
        """The link's name attribute."""
        return self.element.get("name", "")

    @property
    def has_geometry(self) -> bool:
        """True if the link has a <visual> or <collision> shape."""
        return self.element.find("visual") is not None or self.element.find("collision") is not None

    @property
    def mass(self) -> float | None:
        """Mass in kg from <inertial><mass value>, or None if missing or unreadable."""
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
    """Joint wraps one XML element; no per-instance dict, so copies stay cheap."""

    def __init__(self, element: ET.Element) -> None:
        """Wrap an existing <joint> element (the element itself is kept and edited in place)."""
        self.element = element

    @property
    def name(self) -> str:
        """The joint's name attribute."""
        return self.element.get("name", "")

    @property
    def type(self) -> str:
        """The joint type: fixed, revolute, continuous, prismatic, floating or planar."""
        return self.element.get("type", "")

    def _link_ref(self, tag: str) -> str | None:
        """The link name in this joint's <parent> or <child> element (tag), or None."""
        ref = self.element.find(tag)
        return None if ref is None else ref.get("link")

    @property
    def parent(self) -> str | None:
        """Name of the parent link."""
        return self._link_ref("parent")

    @property
    def child(self) -> str | None:
        """Name of the child link."""
        return self._link_ref("child")

    @property
    def mimic(self) -> str | None:
        """Name of the joint this one mimics (<mimic joint=...>), or None."""
        mimic = self.element.find("mimic")
        return None if mimic is None else mimic.get("joint")


@dataclass
class Robot:
    """A parsed URDF: links, joints and every other top-level element ("extras")."""

    name: str
    """Robot name (the <robot name> attribute)."""
    attrib: dict[str, str]
    """All attributes of the <robot> element, written back unchanged."""
    links: dict[str, Link] = field(default_factory=dict)
    """Links by name, in file order."""
    joints: dict[str, Joint] = field(default_factory=dict)
    """Joints by name, in file order."""
    extras: list[ET.Element] = field(default_factory=list)
    """Every other top-level element (materials, <gazebo>, <ros2_control>, ...), kept verbatim."""
    # Problems found while parsing (e.g. duplicate names); reported by validate().
    problems: list[str] = field(default_factory=list)
    """Problems found while parsing (e.g. duplicate names); reported by validate()."""

    # -- editing -------------------------------------------------------------------
    def add_link(self, element: ET.Element) -> Link:
        """Add a <link> element; returns its Link wrapper."""
        link = Link(element)
        self.links[link.name] = link
        return link

    def add_joint(self, element: ET.Element) -> Joint:
        """Add a <joint> element; returns its Joint wrapper."""
        joint = Joint(element)
        self.joints[joint.name] = joint
        return joint

    def remove_link(self, name: str) -> None:
        """Remove the link called name (its joints are not touched)."""
        del self.links[name]

    def remove_joint(self, name: str) -> None:
        """Remove the joint called name."""
        del self.joints[name]

    def copy(self) -> Robot:
        """Deep copy: every pipeline stage works on a copy, so the original stays unchanged."""
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
        """Links that are not the child of any joint (a valid robot has exactly one)."""
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
