"""ROS 2 node: drive the (simulated) rover through a list of waypoints.

Subscribes  /odom      nav_msgs/Odometry      rover pose, published by Unity (RosRoverBridge)
Publishes   /cmd_vel   geometry_msgs/Twist    forward speed (m/s) and turn rate (rad/s)
            /waypoint_demo/status  std_msgs/String  "waiting", "running 2/4", "done ..." (Unity's test watches it)

Parameters (ros2 run ... --ros-args -p name:=value):
  waypoints        [5.0, 0.0, 5.0, 5.0, 0.0, 5.0, 0.0, 0.0]   flat x, y list in the odom frame (start = 0, 0)
  max_speed 1.0    max_turn 1.0    k_heading 1.5    tolerance 0.3    slowdown_radius 1.5
  rate_hz 20.0     log_csv ""  (path; empty = no CSV)
"""

from __future__ import annotations

import csv
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .controller import WaypointController, parse_waypoints, yaw_from_quaternion

DEFAULT_SQUARE = [5.0, 0.0, 5.0, 5.0, 0.0, 5.0, 0.0, 0.0]
"""Default route: a 5 m square from the start point, as a flat [x1, y1, x2, y2, ...] list."""


class WaypointFollower(Node):
    """ROS 2 node: /odom in, /cmd_vel and /waypoint_demo/status out (see the module docstring)."""
    def __init__(self) -> None:
        """Read parameters, create the controller, publishers, the /odom subscription and the control timer."""
        super().__init__("waypoint_follower")
        p = {name: self.declare_parameter(name, default).value for name, default in [
            ("waypoints", DEFAULT_SQUARE), ("max_speed", 1.0), ("max_turn", 1.0), ("k_heading", 1.5),
            ("tolerance", 0.3), ("slowdown_radius", 1.5), ("rate_hz", 20.0), ("log_csv", ""),
        ]}
        self.controller = WaypointController(
            parse_waypoints(list(p["waypoints"])), max_speed=p["max_speed"], max_turn=p["max_turn"],
            k_heading=p["k_heading"], tolerance=p["tolerance"], slowdown_radius=p["slowdown_radius"])

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/waypoint_demo/status", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.timer = self.create_timer(1.0 / p["rate_hz"], self.on_timer)

        self.pose = None            # (x, y, yaw)
        self.odom_count = 0
        self.first_odom = None      # wall time of the first odom message
        self.start_time = None      # wall time the rover started moving
        self.finished_at = None
        self.waypoint_times: list[float] = []
        self.csv_file = open(p["log_csv"], "w", newline="") if p["log_csv"] else None
        self.csv = csv.writer(self.csv_file) if self.csv_file else None
        if self.csv:
            self.csv.writerow(["t", "x", "y", "yaw", "cmd_linear", "cmd_angular", "waypoint"])
        self.get_logger().info(f"Waypoints (odom frame): {self.controller.waypoints}. Waiting for /odom from Unity...")

    def on_odom(self, msg: Odometry) -> None:
        """Store the latest pose (x, y, yaw) from Unity's odometry."""
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.odom_count += 1
        if self.first_odom is None:
            self.first_odom = time.monotonic()
            self.get_logger().info("Receiving /odom - starting.")

    def on_timer(self) -> None:
        """Control loop (rate_hz): step the controller, publish /cmd_vel and status, log reached waypoints and the CSV row."""
        if self.pose is None:
            self.status_pub.publish(String(data="waiting"))
            return
        now = time.monotonic()
        self.start_time = self.start_time or now
        before = self.controller.index
        cmd = self.controller.step(*self.pose)
        for i in range(before, self.controller.index):
            self.waypoint_times.append(now - self.start_time)
            wx, wy = self.controller.waypoints[i]
            self.get_logger().info(f"Reached waypoint {i + 1}/{len(self.controller.waypoints)} ({wx:g}, {wy:g}) "
                                   f"at t={now - self.start_time:.1f} s")

        twist = Twist()
        twist.linear.x, twist.angular.z = float(cmd.linear), float(cmd.angular)
        self.cmd_pub.publish(twist)  # keep publishing (zeros when done): Unity stops the rover if /cmd_vel goes quiet
        if self.csv:
            self.csv.writerow([f"{now - self.start_time:.3f}", f"{self.pose[0]:.4f}", f"{self.pose[1]:.4f}",
                               f"{self.pose[2]:.4f}", f"{cmd.linear:.3f}", f"{cmd.angular:.3f}", self.controller.index])

        if cmd.done:
            if self.finished_at is None:
                self.finished_at = now
                x, y, _ = self.pose
                gx, gy = self.controller.waypoints[-1]
                rate = self.odom_count / max(now - self.first_odom, 1e-6)
                summary = (f"done time={now - self.start_time:.1f}s final_error={((x - gx) ** 2 + (y - gy) ** 2) ** 0.5:.3f}m "
                           f"odom_rate={rate:.1f}Hz waypoint_times={[round(t, 1) for t in self.waypoint_times]}")
                self.get_logger().info(summary)
                self.done_summary = summary
            self.status_pub.publish(String(data=self.done_summary))
        else:
            self.status_pub.publish(String(data=f"running {self.controller.index + 1}/{len(self.controller.waypoints)}"))

    def destroy_node(self) -> None:
        """Close the CSV log, then destroy the node."""
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main(args=None) -> None:
    """Run the node until Ctrl+C; publish a zero /cmd_vel on exit so the rover stops."""
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # stop the rover on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
