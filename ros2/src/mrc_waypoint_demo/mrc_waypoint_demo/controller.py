"""Waypoint steering logic, free of ROS so it can be unit-tested anywhere.

The rover is skid-steer: it is commanded with a forward speed (m/s) and a turn rate (rad/s).
Each step:
  1. If the current waypoint is within `tolerance`, move on to the next one.
  2. Turn toward the waypoint: turn rate = k_heading * heading error (clamped).
  3. Drive forward only when roughly facing it: speed = max_speed * cos(error), and 0 when the error is
     bigger than `turn_in_place_deg` (turn on the spot first). Slow down within `slowdown_radius` of
     the last waypoint so the rover stops on it instead of overshooting.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


def wrap_angle(a: float) -> float:
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading (rotation about +Z, ROS convention: counter-clockwise from +X) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class Command:
    """One steering command for the rover."""
    linear: float = 0.0   # m/s, + = forward
    """Forward speed in m/s (+ = forward)."""
    angular: float = 0.0  # rad/s, + = turn left
    """Turn rate in rad/s (+ = turn left)."""
    done: bool = False
    """True once every waypoint has been reached (the command is then zero)."""


@dataclass
class WaypointController:
    """Steers a skid-steer rover through waypoints (see the module docstring for the rules)."""
    waypoints: list[tuple[float, float]]
    """Waypoints (x, y) in metres, odom frame."""
    max_speed: float = 1.0           # m/s
    """Top forward speed (m/s)."""
    max_turn: float = 1.0            # rad/s
    """Top turn rate (rad/s)."""
    k_heading: float = 1.5           # (rad/s) per rad of heading error
    """Turn rate per radian of heading error ((rad/s)/rad)."""
    tolerance: float = 0.3           # m: a waypoint counts as reached inside this radius
    """A waypoint counts as reached inside this radius (m)."""
    slowdown_radius: float = 1.5     # m: ramp speed down approaching the final waypoint
    """Slow down inside this distance (m) of the final waypoint."""
    turn_in_place_deg: float = 60.0  # turn on the spot when the heading error is larger than this
    """Turn on the spot (no forward speed) when the heading error is larger than this (degrees)."""
    index: int = 0
    """Index of the waypoint being driven to."""
    reached: list[int] = field(default_factory=list)
    """Indices of the waypoints reached so far."""

    @property
    def finished(self) -> bool:
        """True when every waypoint has been reached."""
        return self.index >= len(self.waypoints)

    def step(self, x: float, y: float, yaw: float) -> Command:
        """Command for the rover at (x, y) facing `yaw` (odom frame, metres / radians)."""
        while not self.finished:
            tx, ty = self.waypoints[self.index]
            if math.hypot(tx - x, ty - y) > self.tolerance:
                break
            self.reached.append(self.index)
            self.index += 1
        if self.finished:
            return Command(done=True)

        tx, ty = self.waypoints[self.index]
        distance = math.hypot(tx - x, ty - y)
        error = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        angular = max(-self.max_turn, min(self.max_turn, self.k_heading * error))
        if abs(error) > math.radians(self.turn_in_place_deg):
            linear = 0.0
        else:
            linear = self.max_speed * math.cos(error)
            if self.index == len(self.waypoints) - 1:  # last waypoint: arrive gently
                linear *= min(1.0, distance / self.slowdown_radius)
        return Command(linear=linear, angular=angular)


def parse_waypoints(flat: list[float]) -> list[tuple[float, float]]:
    """[x1, y1, x2, y2, ...] -> [(x1, y1), (x2, y2), ...] (ROS parameters cannot hold nested lists)."""
    if len(flat) % 2 != 0 or not flat:
        raise ValueError("waypoints must be a non-empty flat list of x, y pairs, e.g. [5.0, 0.0, 5.0, 5.0]")
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]
