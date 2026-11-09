#!/usr/bin/env bash
# One-time setup of ROS 2 Jazzy + URDF tooling on Ubuntu 24.04 (WSL2).
# Safe to re-run. Asks for your sudo password.
#
# Usage (inside the Ubuntu-24.04 WSL terminal):
#   bash "/mnt/c/.../MRC-Simulation/urdf-pipeline/env/setup_ros_jazzy.sh"
set -euo pipefail

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

# 1. Must be Ubuntu 24.04 (Noble) - ROS 2 Jazzy's Tier-1 platform.
. /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "ERROR: expected Ubuntu 24.04, found ${PRETTY_NAME:-unknown}." >&2
  echo "Install it from Windows with: wsl --install -d Ubuntu-24.04" >&2
  exit 1
fi

# 2. UTF-8 locale (required by ROS 2).
log "Configuring UTF-8 locale"
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Universe repo + basics.
log "Enabling universe repository"
sudo apt-get install -y software-properties-common curl git
sudo add-apt-repository -y universe

# 4. ROS 2 apt source via the official ros2-apt-source package.
if dpkg -s ros2-apt-source >/dev/null 2>&1; then
  log "ros2-apt-source already installed"
else
  log "Adding ROS 2 apt source"
  ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  if [[ -z "$ROS_APT_SOURCE_VERSION" ]]; then
    echo "ERROR: could not determine latest ros-apt-source release (GitHub API rate limit?)." >&2
    exit 1
  fi
  tmp_deb="$(mktemp --suffix=.deb)"
  curl -fL -o "$tmp_deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb"
  sudo dpkg -i "$tmp_deb"
  rm -f "$tmp_deb"
fi

# 5. ROS 2 Jazzy + the tooling the rover model and pipeline need.
log "Installing ROS 2 Jazzy desktop and URDF tooling (this takes a while)"
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  ros-jazzy-xacro \
  ros-jazzy-realsense2-description \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-urdf-tutorial \
  liburdfdom-tools \
  python3-pytest

# 6. rosdep.
log "Initialising rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

# 7. Source ROS in every new shell.
if ! grep -qF 'source /opt/ros/jazzy/setup.bash' "$HOME/.bashrc"; then
  log "Adding ROS 2 Jazzy to ~/.bashrc"
  echo 'source /opt/ros/jazzy/setup.bash' >> "$HOME/.bashrc"
fi

log "Done. Open a new terminal (or run: source ~/.bashrc), then run generate_input.sh."
