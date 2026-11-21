# Mars Rover Simulation — Setup Guide

## 1. Overview

The Mars Rover Simulation uses Unity as the frontend, ROS 2 as the communication layer, and Gazebo as the simulation backend.

The current implementation supports controlling a simulated rover in Gazebo using WASD keyboard inputs from Unity.

### System Architecture

```text
Unity Frontend
(WASD Keyboard Controller)
        |
        | Velocity Commands
        v
ROS–TCP Endpoint
        |
        v
ROS 2 Jazzy
        |
        v
Gazebo Harmonic
(Rover Physics and Movement)
```

The Unity frontend publishes velocity commands to the ROS 2 topic `/drive_controller/cmd_vel` using the `geometry_msgs/msg/TwistStamped` message type.

The Gazebo backend processes these commands through the active differential drive controller.

## 2. Development Environment

The current implementation has been tested with:

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- Unity 6
- Unity ROS–TCP Connector
- ROS–TCP Endpoint
- Python 3
- colcon

## 3. External Dependencies

The simulation currently relies on two external repositories:

**OSU Robotics Club — Rover_2026_2027**

Provides the rover model, URDF files, Gazebo environment, ROS 2 control packages, and launch configurations.

**Unity Technologies — ROS-TCP-Endpoint**

Provides the ROS 2 communication endpoint used by Unity.

These repositories are external dependencies and are not included directly in the capstone repository.

## 4. ROS 2 Workspace

The tested development workspace is located at:

```bash
~/rover_2026_ws
```

The workspace contains the rover ROS 2 packages and the ROS–TCP Endpoint.

The important packages include:

- `rover_gazebo`
- `rover_urdf`
- `rover2_control`
- `ros_tcp_endpoint`

The workspace must be built using colcon before launching the simulation.

```bash
cd ~/rover_2026_ws
colcon build --symlink-install
source install/setup.bash
```

### Gazebo Launch Configuration Fix

A local modification to `rover_gazebo/setup.py` is required for the tested configuration.

The change installs Python launch files into the package's `launch/` directory.

The modification is preserved in:

```text
patches/rover_gazebo_setup.patch
```

To apply it from the root of the external rover repository:

```bash
git apply /path/to/MRC-Simulation/patches/rover_gazebo_setup.patch
```

Apply the patch only if the external checkout does not already contain the fix, then rebuild the affected ROS 2 package.

## 5. Unity Frontend

The Unity project is stored in:

```text
unity/MarsRoverFrontend/
```

Open this directory as a Unity project.

The main scene is:

```text
Assets/Scenes/SampleScene.unity
```

The keyboard controller is implemented in:

```text
Assets/RoverKeyboardController.cs
```

Unity ROS Settings must use:

```text
Protocol: ROS2
ROS IP:   127.0.0.1
Port:     10000
```

These settings assume Unity and the ROS–TCP Endpoint run on the same computer.

## 6. Starting the Simulation

A startup script is provided:

```text
scripts/start_mars_rover.sh
```

The script performs the following actions:

1. Starts the Gazebo simulation if a Gazebo server is not already running.
2. Starts the ROS–TCP Endpoint if port 10000 is not already listening.
3. Pauses the joystick controller to avoid conflicting movement commands.
4. Opens the Unity project.

The script currently expects the ROS 2 workspace at `~/rover_2026_ws` and the Unity project at `~/UnityProjects/MarsRoverFrontend`.

To launch:

```bash
./scripts/start_mars_rover.sh
```

Wait for Gazebo to finish loading.

In Unity:

1. Open `SampleScene`.
2. Press Play.
3. Click inside the Game window.
4. Use WASD to control the rover.

## 7. Current Implementation Status

The following functionality has been verified:

- Gazebo launches with one rover.
- ROS 2 drive controllers activate successfully.
- Unity connects to ROS 2 through the ROS–TCP Endpoint.
- WASD inputs move the rover in Gazebo.
- Releasing the movement key stops the rover.
- The startup script successfully launches the complete simulation from a clean state.

## 8. Known Limitations

- Unity currently provides keyboard controls but does not display a synchronized rover model.
- Rover position and orientation are not yet synchronized from Gazebo to Unity.
- Camera visualization in Unity has not yet been implemented.
- The startup script uses local directory conventions and requires adjustment for other development environments.
- The external rover repository and ROS 2 dependencies must be installed separately.

## 9. Next Development Milestone

The next planned feature is bidirectional communication between Unity and Gazebo.

The goal is to subscribe to rover position and orientation data from ROS 2 and update a rover model in Unity to match the Gazebo simulation.
