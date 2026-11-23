"""Start the Unity TCP endpoint and the waypoint follower.

  ros2 launch mrc_waypoint_demo waypoint_demo.launch.py
  ros2 launch mrc_waypoint_demo waypoint_demo.launch.py waypoints:="[3.0, 0.0, 3.0, 3.0]" max_speed:=0.8

Then press Play in Unity with "Enable Ros" ticked on the EnvironmentGenerator (or run the ROS PlayMode test).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Launch arguments plus the ros_tcp_endpoint and waypoint_follower nodes."""
    args = {
        "ros_ip": ("0.0.0.0", "address the Unity TCP endpoint listens on (0.0.0.0 = all)"),
        "ros_port": ("10000", "TCP port Unity connects to (ROS Settings in Unity)"),
        "waypoints": ("[5.0, 0.0, 5.0, 5.0, 0.0, 5.0, 0.0, 0.0]", "flat x, y list in the odom frame (rover start = 0, 0)"),
        "max_speed": ("1.0", "m/s"),
        "max_turn": ("1.0", "rad/s"),
        "tolerance": ("0.3", "m"),
        "log_csv": ("", "optional CSV path for the driven path"),
    }
    cfg = {name: LaunchConfiguration(name) for name in args}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=help_text)
          for name, (default, help_text) in args.items()],
        Node(
            package="ros_tcp_endpoint", executable="default_server_endpoint", name="unity_endpoint",
            emulate_tty=True,
            parameters=[{"ROS_IP": ParameterValue(cfg["ros_ip"], value_type=str), "ROS_TCP_PORT": cfg["ros_port"]}],
        ),
        Node(
            package="mrc_waypoint_demo", executable="waypoint_follower", name="waypoint_follower",
            output="screen", emulate_tty=True,
            # Launch values are parsed as YAML: "[5.0, 0.0]" becomes a float list, "1.0" a float.
            parameters=[{
                "waypoints": cfg["waypoints"],
                "max_speed": cfg["max_speed"],
                "max_turn": cfg["max_turn"],
                "tolerance": cfg["tolerance"],
                "log_csv": ParameterValue(cfg["log_csv"], value_type=str),  # "" must stay a string
            }],
        ),
    ])
