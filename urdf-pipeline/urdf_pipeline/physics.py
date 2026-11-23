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
"""Warn when two connected links differ in mass by more than this factor (PhysX articulations get unstable)."""
MOVABLE = ("revolute", "continuous", "prismatic")
"""Joint types that move (and get Unity drives)."""


def link_mass(link: Link) -> float | None:
    """Mass of one link in kg, or None."""
    return link.mass


def total_mass(robot: Robot) -> float:
    """Sum of every link's mass in kg."""
    return sum(m for m in (link_mass(l) for l in robot.links.values()) if m)


def _inertia(link: Link) -> dict[str, float] | None:
    """The link's <inertia> values (ixx ... iyz), {} if unreadable, None if absent."""
    inertia = link.element.find("inertial/inertia")
    if inertia is None:
        return None
    try:
        return {k: float(inertia.get(k, "0")) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    except ValueError:
        return {}


def _limit(joint: Joint) -> dict[str, float]:
    """The joint's <limit> attributes (lower, upper, effort, velocity) that are present and numeric."""
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
    """The joint's <dynamics> damping and friction values that are present and numeric."""
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
    """True if any moving joint exists anywhere below `link` in the tree."""
    stack = [link]
    while stack:
        for joint in children.get(stack.pop(), []):
            if joint.type in MOVABLE:
                return True
            stack.append(joint.child)
    return False


ROLES = ("wheel", "suspension", "arm", "passive")
"""Possible joint roles (see joint_roles)."""
_WHEEL_WORDS = ("wheel", "tire", "tyre")
"""Words in a joint or link name that mark a wheel."""


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
    """Round to 9 decimals for readable JSON (None stays None)."""
    return None if value is None else round(value, 9)


# ROS (x fwd, y left, z up) -> Unity (x right, y up, z fwd): (x, y, z) -> (-y, z, x).
_ROS_TO_UNITY = ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
"""ROS -> Unity axis change as a matrix: (x, y, z) -> (-y, z, x)."""


def _mat_mul(a, b):
    """Product of two 3x3 matrices."""
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _transpose(a):
    """Transpose of a 3x3 matrix."""
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _mat_vec(a, v):
    """3x3 matrix times a 3-vector."""
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rows(m):
    """A 3x3 matrix with every entry rounded for JSON."""
    return [[_clean(x) for x in row] for row in m]


def link_inertials(robot: Robot) -> dict:
    """Per link with <inertial>: mass, centre of mass and full inertia tensor, expressed in the
    link frame, in ROS and in Unity coordinates. The inertia tensor transforms as M I M^T for any
    orthogonal change of axes M (including Unity's handedness flip), so the Unity values are exact."""
    from .kinematics import parse_origin, rpy_matrix  # local import: kinematics imports model only

    result = {}
    for link in robot.links.values():
        inertial = link.element.find("inertial")
        if inertial is None or link_mass(link) is None:
            continue
        values = _inertia(link) or {}
        tensor = [[values.get("ixx", 0.0), values.get("ixy", 0.0), values.get("ixz", 0.0)],
                  [values.get("ixy", 0.0), values.get("iyy", 0.0), values.get("iyz", 0.0)],
                  [values.get("ixz", 0.0), values.get("iyz", 0.0), values.get("izz", 0.0)]]
        xyz, rpy = parse_origin(inertial)
        rot = rpy_matrix(rpy)                                  # inertial frame -> link frame
        tensor_link = _mat_mul(_mat_mul(rot, tensor), _transpose(rot))
        tensor_unity = _mat_mul(_mat_mul(_ROS_TO_UNITY, tensor_link), _transpose(_ROS_TO_UNITY))
        from .mass_properties import principal_axes

        moments, rotation = principal_axes(tensor_unity)
        result[link.name] = {
            "mass": link_mass(link),
            "com_ros": [_clean(v) for v in xyz],
            "com_unity": [_clean(v) for v in _mat_vec(_ROS_TO_UNITY, xyz)],
            "inertia_ros": _rows(tensor_link),
            "inertia_unity": _rows(tensor_unity),
            # Ready for ArticulationBody.inertiaTensor / inertiaTensorRotation (Unity link frame).
            "principal_moments_unity": [_clean(max(m, 1e-9)) for m in moments],
            "principal_rotation_unity_xyzw": [_clean(q) for q in rotation],
        }
    return result


def center_of_mass(robot: Robot) -> dict:
    """Whole-robot centre of mass relative to the root link, all joints at zero."""
    from .kinematics import parse_origin, world_transforms

    poses = world_transforms(robot)
    total, acc = 0.0, [0.0, 0.0, 0.0]
    for name, t in poses.items():
        link = robot.links[name]
        mass = link_mass(link)
        if not mass:
            continue
        com, _ = parse_origin(link.element.find("inertial"))
        world = [t[i][0] * com[0] + t[i][1] * com[1] + t[i][2] * com[2] + t[i][3] for i in range(3)]
        acc = [a + mass * w for a, w in zip(acc, world)]
        total += mass
    com = [a / total for a in acc] if total else [0.0, 0.0, 0.0]
    return {"ros": [_clean(v) for v in com], "unity": [_clean(v) for v in _mat_vec(_ROS_TO_UNITY, com)]}


def _joint_axis_in_root(robot: Robot, joint: Joint, poses: dict) -> list[float] | None:
    """A joint's axis direction relative to the root link (zero pose), in Unity coordinates."""
    from .kinematics import parse_axis

    axis = parse_axis(joint.element) or (1.0, 0.0, 0.0)  # URDF default axis
    pose = poses.get(joint.child)
    if pose is None:
        return None
    rot = [row[:3] for row in pose[:3]]                    # child link frame == joint frame
    world = _mat_vec(rot, axis)
    norm = math.sqrt(sum(v * v for v in world)) or 1.0
    return [_clean(v / norm) for v in _mat_vec(_ROS_TO_UNITY, world)]


def physics_block(robot: Robot, role_overrides: dict[str, str] | None = None) -> dict:
    """Manifest data for Unity scripts: masses, inertia, centre of mass and every movable joint,
    already converted to Unity units and coordinates."""
    from .kinematics import world_transforms

    roles = joint_roles(robot, role_overrides)
    poses = world_transforms(robot)
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
            # Joint axis direction relative to the root link (zero pose), Unity coordinates.
            "axis_unity_root": _joint_axis_in_root(robot, joint, poses),
        }
    return {
        "total_mass_kg": round(total_mass(robot), 6),
        "links_without_mass": [l.name for l in robot.links.values() if l.has_geometry and not link_mass(l)],
        "center_of_mass": center_of_mass(robot),
        "links": link_inertials(robot),
        "joints": joints,
    }


def joint_couplings(robot: Robot, configured: dict[str, dict] | None = None) -> list[dict]:
    """Couplings for Unity: follower angle = multiplier * leader angle + offset.

    Sources: URDF <mimic> tags (multiplier/offset as written; offset converted to Unity units) and
    the `joint_couplings` config. "same_rotation" / "opposite_rotation" multipliers are resolved
    from the joint axes (relative to the root link), so mirrored joints get the right sign."""
    from .errors import PipelineError
    from .kinematics import world_transforms

    poses = world_transforms(robot)
    couplings = []
    for joint in robot.joints.values():
        mimic = joint.element.find("mimic")
        if mimic is None or joint.type not in MOVABLE:
            continue
        scale = math.degrees(1.0) if joint.type in ("revolute", "continuous") else 1.0
        couplings.append({
            "follower": joint.name, "leader": mimic.get("joint"),
            "multiplier": float(mimic.get("multiplier", "1")),
            "offset": _clean(float(mimic.get("offset", "0")) * scale), "source": "urdf_mimic",
        })
    for follower, spec in (configured or {}).items():
        leader = spec["leader"]
        for name in (follower, leader):
            if name not in robot.joints or robot.joints[name].type not in MOVABLE:
                raise PipelineError(f"joint_couplings: '{name}' is not a moving joint of this robot.")
        multiplier = spec.get("multiplier", 1.0)
        if multiplier in ("same_rotation", "opposite_rotation"):
            a = _joint_axis_in_root(robot, robot.joints[follower], poses)
            b = _joint_axis_in_root(robot, robot.joints[leader], poses)
            dot = sum(x * y for x, y in zip(a, b))
            if abs(dot) < 0.9:
                raise PipelineError(f"joint_couplings: '{follower}' and '{leader}' do not turn about parallel axes, "
                                    "so 'same_rotation'/'opposite_rotation' is undefined; give a number instead.")
            multiplier = (1.0 if dot > 0 else -1.0) * (1.0 if spec["multiplier"] == "same_rotation" else -1.0)
        couplings = [c for c in couplings if c["follower"] != follower]
        couplings.append({
            "follower": follower, "leader": leader, "multiplier": float(multiplier),
            "offset": float(spec.get("offset_deg", 0.0)), "source": "config",
        })
    return couplings


GRAVITY = 9.81
"""Standard gravity in m/s^2 (used to turn masses into loads)."""


def joint_springs(robot: Robot, configured: dict[str, dict] | None = None,
                  role_overrides: dict[str, str] | None = None, couplings: list[dict] | None = None) -> list[dict]:
    """Springs/dampers URDF cannot describe (e.g. a suspension shock), for Unity position drives.

    "auto" stiffness holds the joint within `max_sag_deg` of `rest_deg` under the robot's static
    weight. The joint carries the weight resting on the wheels below it (static balance about the
    centre of mass, see _static_wheel_loads); the torque that weight makes is found by moving the joint (and every joint coupled to
    it, so a parallel four-bar's bogie translates instead of swinging) and measuring how far the
    wheels move vertically - virtual work, so it works for any swing arm or linkage. The torque is
    taken at the full design sag, which covers the lever growing as the joint swings.
    "auto" damping is critical damping for that load. Units: N*m/rad and N*m*s/rad."""
    from .errors import PipelineError
    from .kinematics import world_transforms

    if not configured:
        return []
    roles = joint_roles(robot, role_overrides)
    children: dict[str, list[Joint]] = {}
    for j in robot.joints.values():
        children.setdefault(j.parent, []).append(j)
    wheel_links = {robot.joints[j].child for j, r in roles.items() if r == "wheel"}
    weight = total_mass(robot) * GRAVITY
    followers: dict[str, list[tuple[str, float]]] = {}
    for c in couplings or []:
        followers.setdefault(c["leader"], []).append((c["follower"], c["multiplier"]))

    def wheels_below(link: str) -> list[str]:
        found = [link] if link in wheel_links else []
        for j in children.get(link, []):
            found += wheels_below(j.child)
        return found

    wheel_load = _static_wheel_loads(robot, sorted(wheel_links), weight)

    def wheel_height(name: str, angle: float, below: list[str]) -> float:
        positions = {name: angle, **{f: m * angle for f, m in followers.get(name, [])}}
        poses = world_transforms(robot, positions)
        return sum(poses[w][2][3] for w in below) / len(below)

    def generalized_force(name: str, angle: float, load: float, below: list[str]) -> float:
        h = 1e-4  # torque the weight puts on the joint = -load * d(height)/d(angle)
        return -load * (wheel_height(name, angle + h, below) - wheel_height(name, angle - h, below)) / (2 * h)

    springs = []
    for name, spec in configured.items():
        joint = robot.joints.get(name)
        if joint is None or joint.type not in ("revolute", "continuous"):
            raise PipelineError(f"joint_springs: '{name}' is not a revolute/continuous joint of this robot.")
        rest_deg = float(spec.get("rest_deg", 0.0))
        max_sag_deg = float(spec.get("max_sag_deg", 2.0))
        stiffness, damping = spec.get("stiffness_nm_per_rad", "auto"), spec.get("damping_nms_per_rad", "auto")
        basis, auto_k, auto_c = None, 0.0, 0.0
        if "auto" in (stiffness, damping):
            below = wheels_below(joint.child)
            if not below:
                raise PipelineError(f"joint_springs: '{name}' has no wheels below it, so 'auto' cannot work out its "
                                    "load; give stiffness_nm_per_rad and damping_nms_per_rad as numbers.")
            load = sum(wheel_load[w] for w in below)                  # N, straight down (-Z in ROS)
            rest = math.radians(rest_deg)
            at_rest = generalized_force(name, rest, load, below)
            sag = math.copysign(math.radians(max_sag_deg), at_rest)   # the way the weight pushes
            torque = abs(generalized_force(name, rest + sag, load, below))
            auto_k = torque / abs(sag)
            lever = abs(at_rest) / load if load else 0.0
            basis = {"load_kg": round(load / GRAVITY, 3), "lever_m": round(lever, 4),
                     "static_torque_nm": round(abs(at_rest), 3), "torque_at_design_sag_nm": round(torque, 3),
                     "sag_direction_deg": round(math.degrees(sag), 3), "wheels_below": len(below),
                     "coupled_joints_moved": [f for f, _ in followers.get(name, [])]}
        k = auto_k if stiffness == "auto" else float(stiffness)
        if damping == "auto":
            moment = basis["load_kg"] * basis["lever_m"] ** 2           # load mass on the lever arm
            auto_c = 2.0 * math.sqrt(k * moment)
        c = auto_c if damping == "auto" else float(damping)
        springs.append({"joint": name, "rest_deg": rest_deg, "max_sag_deg": max_sag_deg,
                        "stiffness_nm_per_rad": round(k, 3), "damping_nms_per_rad": round(c, 3),
                        "auto_basis": basis})
    return springs


def _static_wheel_loads(robot: Robot, wheels: list[str], weight: float) -> dict[str, float]:
    """Vertical load on each wheel (N) for the robot standing on flat ground.

    The loads must balance the weight and its moments about the centre of mass. With more than three
    wheels that is not unique, so take the smoothest answer (least squares; loads vary linearly across
    the ground plane), which is what a stiff chassis on equally stiff suspension gives. Falls back to
    equal shares if the wheels are in a line."""
    from .kinematics import world_transforms

    if not wheels:
        return {}
    poses = world_transforms(robot)
    com = center_of_mass(robot)["ros"]
    pts = [(poses[w][0][3] - com[0], poses[w][1][3] - com[1]) for w in wheels]
    # f_i = a + b*x_i + c*y_i with sum f = W, sum f*x = 0, sum f*y = 0 (moments about the COM).
    n = len(pts)
    sx, sy = sum(p[0] for p in pts), sum(p[1] for p in pts)
    sxx, syy, sxy = sum(p[0] ** 2 for p in pts), sum(p[1] ** 2 for p in pts), sum(p[0] * p[1] for p in pts)
    a_mat = [[n, sx, sy], [sx, sxx, sxy], [sy, sxy, syy]]
    det = (a_mat[0][0] * (a_mat[1][1] * a_mat[2][2] - a_mat[1][2] * a_mat[2][1])
           - a_mat[0][1] * (a_mat[1][0] * a_mat[2][2] - a_mat[1][2] * a_mat[2][0])
           + a_mat[0][2] * (a_mat[1][0] * a_mat[2][1] - a_mat[1][1] * a_mat[2][0]))
    if abs(det) < 1e-12:
        return {w: weight / n for w in wheels}

    def solve(rhs):  # Cramer's rule
        out = []
        for k in range(3):
            m = [row[:] for row in a_mat]
            for i in range(3):
                m[i][k] = rhs[i]
            out.append((m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det)
        return out

    a, b, c = solve([weight, 0.0, 0.0])
    return {w: max(0.0, a + b * x + c * y) for w, (x, y) in zip(wheels, pts)}


def locked_joints(robot: Robot, names: list[str] | None) -> list[str]:
    """Validated list of joints Unity should hold fixed at zero."""
    from .errors import PipelineError

    for name in names or []:
        if name not in robot.joints or robot.joints[name].type not in MOVABLE:
            raise PipelineError(f"locked_joints: '{name}' is not a moving joint of this robot.")
    return list(names or [])


def drive_limits_block(limits: dict | None) -> dict:
    """Drive limits for Unity (SI units, as configured). Deceleration defaults to acceleration,
    as in ros2 diff_drive_controller."""
    block = {k: v for k, v in (limits or {}).items() if v is not None}
    if "max_linear_accel_mps2" in block:
        block.setdefault("max_linear_decel_mps2", block["max_linear_accel_mps2"])
    return block


def unlimited_wheels(robot: Robot, role_overrides: dict[str, str] | None = None) -> list[str]:
    """Wheel joints with no effort or velocity limit in the URDF."""
    roles = joint_roles(robot, role_overrides)
    return [j for j, r in roles.items() if r == "wheel"
            and not {"effort", "velocity"} & set(_limit(robot.joints[j]))]


def joint_limits_block(robot: Robot, limits: dict | None) -> dict:
    """Real joint motion limits for Unity, in both radians and Unity degrees."""
    from .errors import PipelineError

    block = {}
    for name, spec in (limits or {}).items():
        joint = robot.joints.get(name)
        if joint is None or joint.type not in MOVABLE:
            raise PipelineError(f"joint_limits: '{name}' is not a moving joint of this robot.")
        scale = math.degrees(1.0) if joint.type in ("revolute", "continuous") else 1.0
        entry = {}
        if "max_velocity" in spec:
            entry["max_velocity"] = spec["max_velocity"]
            entry["max_velocity_unity"] = round(spec["max_velocity"] * scale, 6)
        if "max_acceleration" in spec:
            entry["max_acceleration"] = spec["max_acceleration"]
            entry["max_acceleration_unity"] = round(spec["max_acceleration"] * scale, 6)
        entry["units_unity"] = "degrees" if scale != 1.0 else "metres"
        block[name] = entry
    return block
