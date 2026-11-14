"""Physics sanity checks, and joint data converted for Unity physics.

Unity (PhysX articulations) is stricter than RViz: a link with shapes but no mass gets an
invented mass, bad inertia makes joints jitter, and an unlimited joint carrying other
parts swings freely. These checks warn before the robot ever reaches Unity.

Unit conversion (URDF -> Unity ArticulationDrive):
  revolute / continuous: radians -> degrees, rad/s -> deg/s; effort stays N*m
  prismatic:             metres stay metres, m/s stays m/s; effort stays N
"""

from __future__ import annotations

import math

from .model import Joint, Link, Robot

MASS_RATIO_WARN = 50.0
MOVABLE = ("revolute", "continuous", "prismatic")


def link_mass(link: Link) -> float | None:
    return link.mass


def total_mass(robot: Robot) -> float:
    return sum(m for m in (link_mass(l) for l in robot.links.values()) if m)


def _inertia(link: Link) -> dict[str, float] | None:
    inertia = link.element.find("inertial/inertia")
    if inertia is None:
        return None
    try:
        return {k: float(inertia.get(k, "0")) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    except ValueError:
        return {}


def _limit(joint: Joint) -> dict[str, float]:
    limit = joint.element.find("limit")
    if limit is None:
        return {}
    values = {}
    for key in ("lower", "upper", "effort", "velocity"):
        if limit.get(key) is not None:
            try:
                values[key] = float(limit.get(key))
            except ValueError:
                pass
    return values


def _dynamics(joint: Joint) -> dict[str, float]:
    dynamics = joint.element.find("dynamics")
    if dynamics is None:
        return {}
    values = {}
    for key in ("damping", "friction"):
        if dynamics.get(key) is not None:
            try:
                values[key] = float(dynamics.get(key))
            except ValueError:
                pass
    return values


def _movable_below(robot: Robot, link: str, children: dict[str, list[Joint]]) -> bool:
    stack = [link]
    while stack:
        for joint in children.get(stack.pop(), []):
            if joint.type in MOVABLE:
                return True
            stack.append(joint.child)
    return False


ROLES = ("wheel", "suspension", "arm", "passive")
_WHEEL_WORDS = ("wheel", "tire", "tyre")


def joint_roles(robot: Robot, overrides: dict[str, str] | None = None) -> dict[str, str]:
    """Guess what each movable joint does (override any guess with `overrides`).

    wheel:      continuous joint whose name (or child link name) says wheel/tire/tyre
    suspension: joint with a wheel somewhere below it
    arm:        other joint with an effort limit (meant to be driven)
    passive:    other joint with no effort limit (free / unactuated)
    """
    overrides = overrides or {}
    children = robot.child_joints()
    roles: dict[str, str] = {}
    movable = [j for j in robot.joints.values() if j.type in MOVABLE]
    for joint in movable:
        names = f"{joint.name} {joint.child}".lower()
        if overrides.get(joint.name) == "wheel" or (
            joint.name not in overrides and joint.type == "continuous" and any(w in names for w in _WHEEL_WORDS)
        ):
            roles[joint.name] = "wheel"
    wheels_below: dict[str, bool] = {}

    def has_wheel_below(link: str) -> bool:
        if link not in wheels_below:
            wheels_below[link] = any(
                roles.get(j.name) == "wheel" or has_wheel_below(j.child) for j in children.get(link, [])
            )
        return wheels_below[link]

    for joint in movable:
        if joint.name in roles:
            continue
        if joint.name in overrides:
            roles[joint.name] = overrides[joint.name]
        elif has_wheel_below(joint.child):
            roles[joint.name] = "suspension"
        else:
            roles[joint.name] = "arm" if _limit(joint).get("effort", 0) > 0 else "passive"
    return roles


def physics_issues(robot: Robot, role_overrides: dict[str, str] | None = None) -> list[tuple[str, str]]:
    """(level, message) pairs; level is 'warn' or 'info'. Assumes a valid robot."""
    issues: list[tuple[str, str]] = []
    parents = robot.parent_joints()
    children = robot.child_joints()
    roles = joint_roles(robot, role_overrides)

    no_mass = [l.name for l in robot.links.values() if l.has_geometry and not link_mass(l)]
    if no_mass:
        issues.append(("warn", f"{len(no_mass)} link(s) have shapes but no mass ({', '.join(no_mass)}). "
                               "Unity will invent a mass from the collision shape; add an <inertial> block."))

    for link in robot.links.values():
        inertia = _inertia(link)
        if inertia is None or not link_mass(link):
            continue
        if not inertia:
            issues.append(("warn", f"Link '{link.name}' has an unreadable <inertia> value."))
            continue
        a, b, c = inertia["ixx"], inertia["iyy"], inertia["izz"]
        if min(a, b, c) <= 0:
            issues.append(("warn", f"Link '{link.name}' has zero or negative inertia (ixx={a:g}, iyy={b:g}, izz={c:g}); "
                                   "the physics engine may become unstable."))
        elif a + b < c * (1 - 1e-6) or a + c < b * (1 - 1e-6) or b + c < a * (1 - 1e-6):
            issues.append(("warn", f"Link '{link.name}' has physically impossible inertia "
                                   f"(ixx={a:g}, iyy={b:g}, izz={c:g} break the triangle rule)."))

    for name, joint in parents.items():
        child_mass = link_mass(robot.links[name])
        parent_mass = link_mass(robot.links[joint.parent]) if joint.parent in robot.links else None
        if child_mass and parent_mass:
            ratio = max(child_mass / parent_mass, parent_mass / child_mass)
            if ratio > MASS_RATIO_WARN:
                issues.append(("warn", f"Links '{joint.parent}' ({parent_mass:.3g} kg) and '{name}' ({child_mass:.3g} kg) "
                                       f"differ in mass {ratio:.0f}x; large ratios can make Unity joints jitter."))

    for joint in robot.joints.values():
        if joint.type == "continuous" and _movable_below(robot, joint.child, children):
            issues.append(("warn", f"Joint '{joint.name}' is continuous (no limits) but carries other moving parts, "
                                   "so it will swing freely in Unity. If it is a suspension pivot, make it revolute "
                                   "with limits and <dynamics damping=...>."))
        elif roles.get(joint.name) == "passive" and not _dynamics(joint):
            issues.append(("warn", f"Joint '{joint.name}' ({joint.type}) is not driven and has no damping, so "
                                   f"'{joint.child}' will swing freely in Unity. If it is part of a closed linkage "
                                   "(which URDF cannot describe), consider making it fixed or adding damping."))
        if joint.type in MOVABLE:
            limit = _limit(joint)
            if joint.type != "continuous" and joint.element.find("limit") is None:
                issues.append(("warn", f"Joint '{joint.name}' ({joint.type}) has no <limit>; URDF requires one."))
            if limit.get("effort") == 0 or limit.get("velocity") == 0:
                issues.append(("warn", f"Joint '{joint.name}' has effort or velocity limit 0, so it cannot be driven."))

    undamped = [j.name for j in robot.joints.values() if j.type in MOVABLE and not _dynamics(j)]
    if undamped:
        issues.append(("info", f"{len(undamped)} moving joint(s) have no <dynamics> (damping/friction); "
                               "Unity will use its defaults."))
    issues.append(("info", f"Total mass: {total_mass(robot):.3f} kg."))
    return issues


def _clean(value: float | None) -> float | None:
    return None if value is None else round(value, 9)


def physics_block(robot: Robot, role_overrides: dict[str, str] | None = None) -> dict:
    """Manifest data for Unity scripts: mass plus every movable joint in Unity units."""
    roles = joint_roles(robot, role_overrides)
    joints = {}
    for joint in robot.joints.values():
        if joint.type not in MOVABLE:
            continue
        limit = _limit(joint)
        dynamics = _dynamics(joint)
        angular = joint.type in ("revolute", "continuous")
        scale = math.degrees(1.0) if angular else 1.0
        has_range = joint.type != "continuous" and "lower" in limit and "upper" in limit
        joints[joint.name] = {
            "type": joint.type,
            "role": roles[joint.name],
            "parent_link": joint.parent,
            "child_link": joint.child,
            "unity_drive": {
                "units": "degrees" if angular else "metres",
                "lower_limit": _clean(limit["lower"] * scale) if has_range else None,
                "upper_limit": _clean(limit["upper"] * scale) if has_range else None,
                "max_velocity": _clean(limit["velocity"] * scale) if "velocity" in limit else None,
                "force_limit": _clean(limit.get("effort")),
            },
            "urdf_dynamics": {"damping": dynamics.get("damping"), "friction": dynamics.get("friction")},
        }
    return {
        "total_mass_kg": round(total_mass(robot), 6),
        "links_without_mass": [l.name for l in robot.links.values() if l.has_geometry and not link_mass(l)],
        "joints": joints,
    }
