"""Controller tests without ROS: drive a simple unicycle model through the waypoints."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mrc_waypoint_demo.controller import (  # noqa: E402
    WaypointController, parse_waypoints, wrap_angle, yaw_from_quaternion,
)


def simulate(controller, x=0.0, y=0.0, yaw=0.0, dt=0.05, seconds=120.0, accel=2.0):
    """Unicycle with the rover's acceleration limit (2 m/s^2, 2 rad/s^2). Returns the path."""
    v = w = 0.0
    path = [(x, y)]
    for _ in range(int(seconds / dt)):
        cmd = controller.step(x, y, yaw)
        if cmd.done:
            break
        v += max(-accel * dt, min(accel * dt, cmd.linear - v))
        w += max(-accel * dt, min(accel * dt, cmd.angular - w))
        yaw = wrap_angle(yaw + w * dt)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        path.append((x, y))
    return path


def test_square_is_driven_and_ends_on_target():
    """The controller drives a 5 m square (with the rover's acceleration limits) and stops on the final waypoint."""
    c = WaypointController(parse_waypoints([5, 0, 5, 5, 0, 5, 0, 0]))
    path = simulate(c)
    assert c.finished and c.reached == [0, 1, 2, 3]
    assert math.dist(path[-1], (0.0, 0.0)) <= c.tolerance


def test_turns_in_place_when_target_is_behind():
    """With the target straight behind, the rover turns on the spot at full turn rate."""
    c = WaypointController([(-5.0, 0.0)])
    cmd = c.step(0.0, 0.0, 0.0)            # facing +X, target straight behind
    assert cmd.linear == 0.0 and abs(cmd.angular) == pytest.approx(c.max_turn)


def test_helpers():
    """yaw_from_quaternion, wrap_angle and parse_waypoints behave correctly."""
    assert yaw_from_quaternion(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)) == pytest.approx(math.pi / 2)
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    with pytest.raises(ValueError):
        parse_waypoints([1.0, 2.0, 3.0])
