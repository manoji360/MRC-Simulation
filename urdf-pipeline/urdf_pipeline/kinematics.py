"""Zero-configuration forward kinematics (ROS frame).

Used to prove that the pipeline never moves a link, and to compute reference poses.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from .errors import PipelineError
from .model import Robot

Matrix = list[list[float]]


def _vec3(text: str | None, what: str) -> tuple[float, float, float]:
    if not text or not text.strip():
        return (0.0, 0.0, 0.0)
    try:
        values = [float(v) for v in text.split()]
    except ValueError:
        values = []
    if len(values) != 3:
        raise PipelineError(f"Invalid {what} value '{text}': expected three numbers.")
    return (values[0], values[1], values[2])


def parse_origin(element: ET.Element | None) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(xyz, rpy) of the <origin> child of `element` (zeros when absent)."""
    origin = None if element is None else element.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return _vec3(origin.get("xyz"), "origin xyz"), _vec3(origin.get("rpy"), "origin rpy")


def parse_axis(element: ET.Element | None) -> tuple[float, float, float] | None:
    axis = None if element is None else element.find("axis")
    return None if axis is None else _vec3(axis.get("xyz"), "axis xyz")


def rpy_matrix(rpy: tuple[float, float, float]) -> Matrix:
    """URDF rpy -> 3x3 rotation, R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def transform(xyz, rpy) -> Matrix:
    rot = rpy_matrix(rpy)
    return [rot[0] + [xyz[0]], rot[1] + [xyz[1]], rot[2] + [xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def matmul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


IDENTITY: Matrix = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def joint_motion(joint_type: str, axis, value: float) -> Matrix:
    """Transform of a joint moved by `value` (radians, or metres for prismatic) about/along its axis."""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    if joint_type == "prismatic":
        return [[1.0, 0.0, 0.0, x * value], [0.0, 1.0, 0.0, y * value], [0.0, 0.0, 1.0, z * value], [0.0, 0.0, 0.0, 1.0]]
    c, s, t = math.cos(value), math.sin(value), 1 - math.cos(value)  # Rodrigues
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def world_transforms(robot: Robot, positions: dict[str, float] | None = None) -> dict[str, Matrix]:
    """Pose of every link relative to the root link. Joints are at zero unless given in `positions`
    (joint name -> radians, or metres for prismatic joints)."""
    order = robot.tree_order()
    if not order:
        return {}
    parents = robot.parent_joints()
    result: dict[str, Matrix] = {order[0]: IDENTITY}
    for name in order[1:]:
        joint = parents[name]
        xyz, rpy = parse_origin(joint.element)
        pose = matmul(result[joint.parent], transform(xyz, rpy))
        value = (positions or {}).get(joint.name, 0.0)
        if value and joint.type in ("revolute", "continuous", "prismatic"):
            pose = matmul(pose, joint_motion(joint.type, parse_axis(joint.element) or (1.0, 0.0, 0.0), value))
        result[name] = pose
    return result


def max_difference(a: dict[str, Matrix], b: dict[str, Matrix]) -> float:
    """Largest element-wise difference between two pose sets (inf if link sets differ)."""
    if a.keys() != b.keys():
        return math.inf
    return max(
        (abs(a[n][i][j] - b[n][i][j]) for n in a for i in range(4) for j in range(4)),
        default=0.0,
    )
