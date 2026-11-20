"""Tests of the ROS <-> Unity coordinate conversions."""

import math
import random

from urdf_pipeline.coordinates import (
    quaternion_multiply, ros_rpy_to_unity_euler_deg, ros_to_unity_position, ros_to_unity_quaternion,
    ros_to_unity_scale, rotate_vector, rpy_to_quaternion, same_rotation, unity_euler_to_quaternion,
    matrix_to_quaternion,
)
from urdf_pipeline.kinematics import rpy_matrix


def close(a, b, tol=1e-9):
    """True if two number sequences are equal within tol."""
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_matches_unity_urdf_importer_unit_tests():
    # Same vectors as BuiltInExtensionsTests in com.unity.robotics.urdf-importer.
    """Conversions give the same results as the URDF Importer's own unit tests."""
    assert ros_to_unity_position((1, 2, 3)) == (-2, 3, 1)
    assert ros_to_unity_scale((1, 2, 3)) == (2, 3, 1)
    assert ros_to_unity_quaternion((0.1, 0.2, 0.3, 0.4)) == (0.2, -0.3, -0.1, 0.4)


def test_ros_axes_map_to_unity_axes():
    """ROS forward/left/up map to Unity +Z / -X / +Y."""
    assert ros_to_unity_position((1, 0, 0)) == (0, 0, 1)    # forward -> Unity +Z
    assert ros_to_unity_position((0, 1, 0)) == (-1, 0, 0)   # left    -> Unity -X
    assert ros_to_unity_position((0, 0, 1)) == (0, 1, 0)    # up      -> Unity +Y


def test_rpy_quaternion_matches_rotation_matrix():
    """rpy_to_quaternion rotates vectors exactly like rpy_matrix."""
    rng = random.Random(1)
    for _ in range(200):
        rpy = tuple(rng.uniform(-math.pi, math.pi) for _ in range(3))
        q = rpy_to_quaternion(rpy)
        assert same_rotation(q, matrix_to_quaternion(rpy_matrix(rpy)))


def test_rotation_conversion_is_consistent_with_position_conversion():
    # ros_to_unity(R_ros * v) must equal q_unity * ros_to_unity(v)
    """Converting a rotation and converting positions agree (rotate-then-convert == convert-then-rotate)."""
    rng = random.Random(2)
    for _ in range(200):
        rpy = tuple(rng.uniform(-math.pi, math.pi) for _ in range(3))
        v = tuple(rng.uniform(-2, 2) for _ in range(3))
        q_ros = rpy_to_quaternion(rpy)
        left = ros_to_unity_position(rotate_vector(q_ros, v))
        right = rotate_vector(ros_to_unity_quaternion(q_ros), ros_to_unity_position(v))
        assert close(left, right, 1e-9)


def test_unity_euler_from_importer_equals_converted_quaternion():
    # The importer does Transform.Rotate(pitch, -yaw, -roll); our quaternion must match it.
    """The Euler angles the importer uses produce the same rotation as the converted quaternion."""
    rng = random.Random(3)
    for _ in range(200):
        rpy = tuple(rng.uniform(-math.pi, math.pi) for _ in range(3))
        via_euler = unity_euler_to_quaternion(ros_rpy_to_unity_euler_deg(rpy))
        via_quat = ros_to_unity_quaternion(rpy_to_quaternion(rpy))
        assert same_rotation(via_euler, via_quat, 1e-9)


def test_quaternion_multiply_identity():
    """Multiplying by the identity quaternion changes nothing."""
    q = rpy_to_quaternion((0.3, -0.2, 1.1))
    assert close(quaternion_multiply(q, (0, 0, 0, 1)), q)
