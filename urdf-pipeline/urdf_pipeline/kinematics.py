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


def world_transforms(robot: Robot) -> dict[str, Matrix]:
    """Pose of every link relative to the root link, with all joints at zero."""
    order = robot.tree_order()
    if not order:
        return {}
    parents = robot.parent_joints()
    result: dict[str, Matrix] = {order[0]: IDENTITY}
    for name in order[1:]:
        joint = parents[name]
        xyz, rpy = parse_origin(joint.element)
        result[name] = matmul(result[joint.parent], transform(xyz, rpy))
    return result


def max_difference(a: dict[str, Matrix], b: dict[str, Matrix]) -> float:
    """Largest element-wise difference between two pose sets (inf if link sets differ)."""
    if a.keys() != b.keys():
        return math.inf
    return max(
        (abs(a[n][i][j] - b[n][i][j]) for n in a for i in range(4) for j in range(4)),
        default=0.0,
    )
