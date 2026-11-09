"""ROS (URDF) <-> Unity coordinate conversion.

ROS / URDF (REP-103): right-handed, X forward, Y left, Z up.
Unity:               left-handed,  X right,   Y up,   Z forward.

RULES FOR THIS TOOL
  1. Never convert the contents of URDF or STL files. The Unity URDF Importer converts
     them itself when it imports them; converting here too would double-convert.
  2. Convert every value that a Unity script reads from manifest.json (collapsed frames,
     bridge joints, reference poses), so C# code only reads numbers and does no math.

The formulas below match the Unity URDF Importer exactly
(com.unity.robotics.urdf-importer):
  - BuiltInExtensions.Ros2Unity(Vector3)     (x, y, z)    -> (-y, z, x)
  - BuiltInExtensions.Ros2Unity(Quaternion)  (x, y, z, w) -> (y, -z, -x, w)
  - BuiltInExtensions.Ros2UnityScale         (x, y, z)    -> (y, z, x)
  - UrdfOrigin.GetRotationFromUrdf           rpy -> Transform.Rotate(pitch, -yaw, -roll) [deg]
ROS-TCP-Connector's FLU.ConvertToRUF gives (-qy, qz, qx, -qw) for quaternions, which is the
same rotation (q and -q are equivalent).

Quaternions are (x, y, z, w) tuples throughout.
"""

from __future__ import annotations

import math

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


# -- ROS -> Unity ----------------------------------------------------------------------
def ros_to_unity_position(v: Vec3) -> Vec3:
    x, y, z = v
    return (-y, z, x)


def ros_to_unity_direction(v: Vec3) -> Vec3:
    """Directions (e.g. joint axes) convert like positions."""
    return ros_to_unity_position(v)


def ros_to_unity_scale(s: Vec3) -> Vec3:
    x, y, z = s
    return (y, z, x)


def ros_to_unity_quaternion(q: Quat) -> Quat:
    x, y, z, w = q
    return (y, -z, -x, w)


def ros_rpy_to_unity_euler_deg(rpy: Vec3) -> Vec3:
    """Euler angles (degrees) exactly as the URDF Importer passes them to Transform.Rotate."""
    roll, pitch, yaw = rpy
    return (math.degrees(pitch), -math.degrees(yaw), -math.degrees(roll))


# -- quaternion helpers ----------------------------------------------------------------
def rpy_to_quaternion(rpy: Vec3) -> Quat:
    """URDF rpy (fixed axes: roll about X, then pitch about Y, then yaw about Z)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q: Quat, v: Vec3) -> Vec3:
    x, y, z, w = q
    conj = (-x, -y, -z, w)
    rx, ry, rz, _ = quaternion_multiply(quaternion_multiply(q, (v[0], v[1], v[2], 0.0)), conj)
    return (rx, ry, rz)


def _axis_angle(axis: Vec3, degrees: float) -> Quat:
    half = math.radians(degrees) / 2
    s = math.sin(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))


def unity_euler_to_quaternion(euler_deg: Vec3) -> Quat:
    """Unity's Quaternion.Euler: rotates around Z, then X, then Y."""
    ex, ey, ez = euler_deg
    qx = _axis_angle((1.0, 0.0, 0.0), ex)
    qy = _axis_angle((0.0, 1.0, 0.0), ey)
    qz = _axis_angle((0.0, 0.0, 1.0), ez)
    return quaternion_multiply(qy, quaternion_multiply(qx, qz))


def matrix_to_quaternion(r: list[list[float]]) -> Quat:
    """3x3 (or upper-left of 4x4) rotation matrix -> quaternion, numerically stable."""
    m00, m01, m02 = r[0][0], r[0][1], r[0][2]
    m10, m11, m12 = r[1][0], r[1][1], r[1][2]
    m20, m21, m22 = r[2][0], r[2][1], r[2][2]
    trace = m00 + m11 + m22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    # Canonical sign (w >= 0) so output is deterministic.
    return q if q[3] >= 0 else (-q[0], -q[1], -q[2], -q[3])


def same_rotation(a: Quat, b: Quat, tol: float = 1e-9) -> bool:
    """True if a and b represent the same rotation (q and -q are equivalent)."""
    dot = sum(x * y for x, y in zip(a, b))
    return abs(abs(dot) - 1.0) <= tol


# -- manifest-friendly blocks ----------------------------------------------------------
def _clean(values) -> list[float]:
    """Round for readable JSON and turn -0.0 into 0.0."""
    return [0.0 if abs(v) < 1e-12 else round(v, 9) for v in values]


def unity_block_from_origin(xyz: Vec3, rpy: Vec3, axis: Vec3 | None = None) -> dict:
    """A joint/frame origin (relative to its parent link) in Unity terms."""
    block = {
        "position": _clean(ros_to_unity_position(xyz)),
        "rotation_quaternion_xyzw": _clean(ros_to_unity_quaternion(rpy_to_quaternion(rpy))),
        "rotation_euler_deg": _clean(ros_rpy_to_unity_euler_deg(rpy)),
    }
    if axis is not None:
        block["axis"] = _clean(ros_to_unity_direction(axis))
    return block


def pose_blocks_from_matrix(t: list[list[float]]) -> dict:
    """A 4x4 pose (relative to the robot root) in both ROS and Unity terms."""
    xyz = (t[0][3], t[1][3], t[2][3])
    q = matrix_to_quaternion(t)
    return {
        "ros": {"xyz": _clean(xyz), "quaternion_xyzw": _clean(q)},
        "unity": {
            "position": _clean(ros_to_unity_position(xyz)),
            "rotation_quaternion_xyzw": _clean(ros_to_unity_quaternion(q)),
        },
    }
