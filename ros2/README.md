# ROS 2 side of the Unity rover simulation

A minimal ROS 2 (Jazzy) test that drives the Unity rover from ROS, to check the ROS ↔ Unity loop.

```
 WSL2 / Ubuntu 24.04 (ROS 2 Jazzy)                      Windows (Unity 6.3)
 ┌──────────────────────────────┐   TCP 127.0.0.1:10000  ┌──────────────────────────────┐
 │ waypoint_follower  ── /cmd_vel ──► ros_tcp_endpoint ◄══►│ ROSConnection → RosRoverBridge│
 │   (mrc_waypoint_demo)  ◄── /odom ──                     │   → rover drive chain (real   │
 │   /waypoint_demo/status ──►                             │     accel limits) → physics   │
 └──────────────────────────────┘                         └──────────────────────────────┘
```
WSL2 must use **mirrored networking** (`urdf-pipeline/env/wslconfig.example`), so `127.0.0.1` reaches WSL from Windows.

## The algorithm (`mrc_waypoint_demo`)
`waypoint_follower` reads `/odom` and steers through waypoints (default: a 5 m square from the start point, `[5,0, 5,5, 0,5, 0,0]`):
- turn rate = 1.5 × heading error (max 1 rad/s);
- drive forward at 1 m/s × cos(heading error), turning on the spot if the error is over 60°;
- slow down within 1.5 m of the last waypoint; a waypoint counts as reached within 0.3 m;
- it publishes `/cmd_vel` at 20 Hz (zeros when done: Unity stops the rover if `/cmd_vel` goes quiet for 0.5 s);
- progress goes to `/waypoint_demo/status`; `log_csv:=/path.csv` records the driven path.

The steering logic is in `controller.py` (plain Python, unit-tested without ROS: `python -m pytest ros2/src/mrc_waypoint_demo/test`).

## Run it
```bash
# WSL, once: build Unity's TCP endpoint + this package into ~/mrc_ros_ws
bash ros2/setup_ros2_bridge.sh
# WSL, each time
source ~/mrc_ros_ws/install/setup.bash
ros2 launch mrc_waypoint_demo waypoint_demo.launch.py          # options: waypoints:="[3.0,0.0,3.0,3.0]" max_speed:=0.8 log_csv:=/tmp/path.csv
```
Then either:
- **Watch it:** open `unity/MRC-Rover`, select the `EnvironmentGenerator` in `RoverTestEnvironment`, tick **Enable Ros**, press Play. The HUD shows "ROS driving"; holding a drive key takes over from ROS.
- **Automated (headless):** close Unity, then `powershell -ExecutionPolicy Bypass -File unity\run_ros_demo.ps1`. Results: `unity/MRC-Rover/FeasibilityResults/tuned/ros.json`.

## Topics
| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Unity | `linear.x` m/s (+ forward), `angular.z` rad/s (+ left) |
| `/odom` | `nav_msgs/Odometry` | Unity → ROS | 30 Hz; frame `odom` (origin = rover start), child `rover_base_origin`, ROS axes |
| `/waypoint_demo/status` | `std_msgs/String` | ROS → Unity | `waiting`, `running 2/4`, `done time=… final_error=…` |

Note: on the real rover, `/cmd_vel` from the ground station is a **-1…1 fraction** that `joy_to_drive.py` scales to 3.7 m/s. Here it is in m/s (the ROS standard); the simulated drive chain applies the rover's real speed and acceleration limits.
