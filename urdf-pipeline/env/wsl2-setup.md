# Dev environment setup: WSL2 + Ubuntu 24.04 + ROS 2 Jazzy

This guide sets up the Linux side of the URDF pipeline on a Windows 11 machine. When you're
done, you will have ROS 2 Jazzy running in WSL2, plus a validated copy of the rover model in
`urdf-pipeline/input/`. See [../design.md](../design.md) §11 for why we use WSL2 instead of a VM.

Unity and the Python pipeline stay on Windows. ROS 2 is only used to generate and check the
URDF (and later for the ROS–Unity bridge).

**Time:** about 30–45 minutes, mostly downloads. **Disk:** about 6 GB in WSL.

## 1. Install WSL2 and Ubuntu 24.04 (Windows, once)

Open **PowerShell as Administrator**:

```powershell
wsl --install -d Ubuntu-24.04
```

Reboot when it asks. After the reboot, an Ubuntu window opens and asks for a **Linux username
and password**. These are separate from your Windows login, and you'll need the password for
`sudo`.

Then, from a normal PowerShell:

```powershell
wsl --update
wsl -l -v          # Ubuntu-24.04 should show VERSION 2
```

## 2. Configure WSL (Windows, once)

Copy [wslconfig.example](wslconfig.example) to `%UserProfile%\.wslconfig`. Adjust `memory` and
`processors` to roughly half of what your machine has. Then restart WSL:

```powershell
Copy-Item "urdf-pipeline\env\wslconfig.example" "$env:UserProfile\.wslconfig"
wsl --shutdown
```

`networkingMode=mirrored` lets Unity on Windows and ROS 2 in WSL reach each other on
`localhost`.

## 3. Install ROS 2 Jazzy (Ubuntu, once)

Open the **Ubuntu 24.04** terminal (Start menu → "Ubuntu 24.04") and run the setup script from
the Windows checkout. Quote the path, since it may contain spaces:

```bash
cd "/mnt/c/Users/<you>/path/to/MRC-Simulation"
bash urdf-pipeline/env/setup_ros_jazzy.sh
```

It installs `ros-jazzy-desktop`, xacro, `realsense2_description`, `check_urdf`, colcon and
rosdep, and adds ROS to `~/.bashrc`. It's safe to run again if it gets interrupted.

Open a new Ubuntu terminal afterwards, or run `source ~/.bashrc`.

## 4. Generate the rover model (Ubuntu, whenever the rover repo changes)

```bash
bash urdf-pipeline/env/generate_input.sh
```

This script:

1. clones [OSURoboticsClub/Rover_2026_2027](https://github.com/OSURoboticsClub/Rover_2026_2027)
   into `~/rover_ws/src`;
2. builds only `rover_urdf` and `rover_arm_urdf` (no MoveIt or hardware packages);
3. expands `rover2_arm/config/rover.urdf.xacro` with `attachment:=arm`;
4. validates the result with `check_urdf`;
5. writes `urdf-pipeline/input/rover.urdf`, every mesh it references (in the same
   `<package>/urdf/...` layout as the `package://` paths), and `SOURCE.txt` with the upstream commit.

Options:

```bash
bash urdf-pipeline/env/generate_input.sh "$PWD" northrop_arm   # other attachment
ROVER_REF=<commit-sha> bash urdf-pipeline/env/generate_input.sh # pin an upstream commit
```

`urdf-pipeline/input/` is git-ignored on purpose. The meshes belong to the OSU Robotics Club,
so don't commit them unless the team confirms we have permission.

## 5. Verify

```bash
bash urdf-pipeline/env/verify_env.sh
```

Every line should say PASS. The `WARN` lines are advisory. To look at the model:

```bash
ros2 launch urdf_tutorial display.launch.py model:="$PWD/urdf-pipeline/input/rover.urdf"
```

RViz opens in a normal Windows window (through WSLg). Use the joint sliders to move the wheels and arm.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `wsl --install` says virtualization is disabled | Enable Intel VT-x / AMD-V in BIOS/UEFI. Turn on "Virtual Machine Platform" in *Windows Features* |
| `$'\r': command not found` when running a script | The script has Windows line endings. Run `git add --renormalize .`, or re-checkout; `.gitattributes` forces LF for `*.sh` |
| `rviz2` window is blank or crashes | Run `wsl --update` and update the GPU driver on Windows. As a last resort, `export LIBGL_ALWAYS_SOFTWARE=1` |
| `WARN WSL networking mode: nat` | `.wslconfig` was not picked up. Check the file name and location, then `wsl --shutdown` |
| `ros-apt-source` step fails | The GitHub API rate limit was hit. Wait a few minutes and re-run `setup_ros_jazzy.sh` |
| xacro error about a missing package or file | The rover repo layout changed. Check `SOURCE.txt` and the xacro args in `generate_input.sh` |
| Builds or git are slow under `/mnt/c` | Expected (cross-filesystem). The rover workspace lives in `~/rover_ws` for this reason |
