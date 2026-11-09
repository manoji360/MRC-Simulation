"""Split a robot into several articulation "parts", each within the link limit.

Greedy bottom-up partition: walk the tree from the leaves up, tracking how many links
the still-open part at each link holds. Whenever a link's open part would exceed
`max_links`, cut the cheapest child joint (fixed joints first; joints inside a
keep-together group or involved in a mimic relation are expensive), largest subtree
first among equals, ties broken by name. Each cut joint becomes a "bridge joint" that a
Unity script reconnects.
"""

from __future__ import annotations

import copy
from collections import deque
from dataclasses import dataclass, field

from .errors import PipelineError
from .model import Robot

DEFAULT_CUT_COST = {
    "fixed": 1,
    "revolute": 5,
    "continuous": 5,
    "prismatic": 5,
    "planar": 5,
    "floating": 5,
    "mimic": 50,
    "keep_together": 1000,
}


@dataclass
class Part:
    id: str
    root_link: str
    links: list[str]


@dataclass
class SplitPlan:
    parts: list[Part]
    cut_joints: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def single_part_plan(robot: Robot) -> SplitPlan:
    order = robot.tree_order()
    return SplitPlan(parts=[Part("part_00", order[0], order)])


def plan_split(
    robot: Robot,
    max_links: int,
    keep_together: list[list[str]] | None = None,
    cut_cost: dict[str, int] | None = None,
) -> SplitPlan:
    if max_links < 1:
        raise PipelineError("max_links must be at least 1.")
    costs = {**DEFAULT_CUT_COST, **(cut_cost or {})}
    group: dict[str, int] = {}
    for index, names in enumerate(keep_together or []):
        for name in names:
            group.setdefault(name, index)
    mimic_related = {j.name for j in robot.joints.values() if j.mimic} | {
        j.mimic for j in robot.joints.values() if j.mimic
    }

    def cost(joint) -> int:
        value = costs.get(joint.type, 5)
        g = group.get(joint.parent)
        if g is not None and g == group.get(joint.child):
            value += costs["keep_together"]
        if joint.name in mimic_related:
            value += costs["mimic"]
        return value

    order = robot.tree_order()
    children = robot.child_joints()
    open_size: dict[str, int] = {}
    cuts: set[str] = set()
    warnings: list[str] = []
    for name in reversed(order):
        kids = children.get(name, [])
        size = 1 + sum(open_size[j.child] for j in kids)
        while size > max_links:
            candidates = [j for j in kids if j.name not in cuts]
            best = min(candidates, key=lambda j: (cost(j), -open_size[j.child], j.name))
            cuts.add(best.name)
            size -= open_size[best.child]
            g = group.get(best.parent)
            if g is not None and g == group.get(best.child):
                warnings.append(
                    f"Had to cut joint '{best.name}' inside keep-together group {g + 1}; "
                    "the group is larger than max_links allows."
                )
        open_size[name] = size

    parts: list[Part] = []
    pending = deque([order[0]])
    while pending:
        part_root = pending.popleft()
        links: list[str] = []
        queue = deque([part_root])
        while queue:
            name = queue.popleft()
            links.append(name)
            for joint in children.get(name, []):
                (pending if joint.name in cuts else queue).append(joint.child)
        parts.append(Part(f"part_{len(parts):02d}", part_root, links))
    ordered_cuts = [robot.parent_joints()[p.root_link].name for p in parts[1:]]
    return SplitPlan(parts=parts, cut_joints=ordered_cuts, warnings=warnings)


def build_part_robots(robot: Robot, plan: SplitPlan) -> list[Robot]:
    """One standalone Robot per part. Top-level extras go into part 0 unchanged."""
    part_of = {name: i for i, part in enumerate(plan.parts) for name in part.links}
    cut = set(plan.cut_joints)
    multi = len(plan.parts) > 1
    result = []
    for i, part in enumerate(plan.parts):
        name = f"{robot.name}_{part.id}" if multi else robot.name
        part_robot = Robot(name=name, attrib={**robot.attrib, "name": name})
        for link in part.links:
            part_robot.add_link(copy.deepcopy(robot.links[link].element))
        for joint in robot.joints.values():
            if joint.name not in cut and part_of.get(joint.child) == i:
                part_robot.add_joint(copy.deepcopy(joint.element))
        if i == 0:
            part_robot.extras = [copy.deepcopy(e) for e in robot.extras]
        result.append(part_robot)
    return result
