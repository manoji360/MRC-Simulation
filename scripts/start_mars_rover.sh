#!/usr/bin/env bash

set -e

WORKSPACE="$HOME/rover_2026_ws"
UNITY_PROJECT="$HOME/UnityProjects/MarsRoverFrontend"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "=================================="
echo "  Starting Mars Rover Simulation"
echo "=================================="

# Check that GNOME Terminal is available
if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "Error: gnome-terminal is not installed."
    exit 1
fi

# Start Gazebo only if it is not already running
echo "[1/4] Checking Gazebo..."

if pgrep -f '^gz sim server$' >/dev/null; then
    echo "Gazebo server is already running. Skipping launch."
else
    echo "Starting Gazebo..."

    gnome-terminal --title="Mars Rover - Gazebo" -- \
        bash -c "
            source /opt/ros/jazzy/setup.bash
            source '$WORKSPACE/install/setup.bash'
            ros2 launch rover_gazebo gazebo_drive.launch.py
            exec bash
        "
fi

# Start ROS-TCP Endpoint if it is not already running
echo "[2/4] Checking ROS-TCP Endpoint..."

if ss -ltn | grep -q '127.0.0.1:10000'; then
    echo "ROS-TCP Endpoint port is already listening."
else
    gnome-terminal --title="Mars Rover - ROS TCP" -- \
        bash -c "
            source /opt/ros/jazzy/setup.bash
            source '$WORKSPACE/install/setup.bash'
            ros2 run ros_tcp_endpoint default_server_endpoint \
                --ros-args \
                -p ROS_IP:=127.0.0.1 \
                -p ROS_TCP_PORT:=10000
            exec bash
        "
fi

# Wait for ROS-TCP Endpoint to start
echo "Waiting for ROS-TCP Endpoint..."

TCP_READY=false

for i in {1..30}; do
    if ss -ltn | grep -q '127.0.0.1:10000'; then
        TCP_READY=true
        echo "ROS-TCP Endpoint is ready!"
        break
    fi

    sleep 1
done

if [ "$TCP_READY" = false ]; then
    echo "Error: ROS-TCP Endpoint did not start."
    exit 1
fi

# Wait for joy_to_drive and pause it
echo "[3/4] Waiting for joystick controller..."

JOY_PID=""

for i in {1..60}; do
    JOY_PID=$(pgrep -f '/rover2_control/joy_to_drive' | head -n 1 || true)

    if [ -n "$JOY_PID" ]; then
        kill -STOP "$JOY_PID"
        echo "Joystick controller paused (PID: $JOY_PID)"
        break
    fi

    sleep 1
done

if [ -z "$JOY_PID" ]; then
    echo "Warning: joy_to_drive was not found."
fi

# Open Unity project
echo "[4/4] Opening Unity..."

if command -v unityhub >/dev/null 2>&1; then
    unityhub --projectPath "$UNITY_PROJECT" >/dev/null 2>&1 &
else
    echo "Unity Hub was not found."
    echo "Please open Unity manually:"
    echo "$UNITY_PROJECT"
fi

echo ""
echo "=================================="
echo "  Mars Rover Startup Complete!"
echo "=================================="
echo ""
echo "1. Wait for Gazebo to load the rover."
echo "2. Open Unity and click Play."
echo "3. Click inside the Game window."
echo "4. Use WASD to control the rover."
