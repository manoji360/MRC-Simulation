#!/usr/bin/env bash
# Build the ROS 2 side of the Unity <-> ROS 2 test in a workspace (Ubuntu 24.04 / WSL2, ROS 2 Jazzy).
#   bash ros2/setup_ros2_bridge.sh            (workspace: ~/mrc_ros_ws)
#   WS=/path/to/ws bash ros2/setup_ros2_bridge.sh
# Afterwards:  source ~/mrc_ros_ws/install/setup.bash
#              ros2 launch mrc_waypoint_demo waypoint_demo.launch.py
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$HOME/mrc_ros_ws}"
ENDPOINT_URL="https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git"
ENDPOINT_REF="${ENDPOINT_REF:-main-ros2}"   # ROS 2 branch of Unity's TCP endpoint (pairs with ROS-TCP-Connector v0.7.0)

source /opt/ros/jazzy/setup.bash
mkdir -p "$WS/src"

# 1. Unity's TCP endpoint (bridges Unity's ROS-TCP-Connector to ROS 2 topics)
if [[ ! -d "$WS/src/ROS-TCP-Endpoint" ]]; then
  git clone --depth 1 --branch "$ENDPOINT_REF" "$ENDPOINT_URL" "$WS/src/ROS-TCP-Endpoint"
fi

# 2. Our package, linked (not copied) so edits in the repo are picked up by the next build
ln -sfn "$HERE/src/mrc_waypoint_demo" "$WS/src/mrc_waypoint_demo"

# 3. Build
cd "$WS"
colcon build --symlink-install --packages-select ros_tcp_endpoint mrc_waypoint_demo
echo
echo "Built. Next:"
echo "  source $WS/install/setup.bash"
echo "  ros2 launch mrc_waypoint_demo waypoint_demo.launch.py"
