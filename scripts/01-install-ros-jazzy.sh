#!/usr/bin/env bash
# 01-install-ros-jazzy.sh — Ubuntu 24.04 (Noble) ARM64/AMD64 -> ROS 2 Jazzy Desktop
# Follows the official Jazzy Debian install:
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# Safe to re-run: every step checks whether it is already done.
# Run INSIDE the VM as your normal user:  bash ~/vm-share/scripts/01-install-ros-jazzy.sh
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

[[ $EUID -ne 0 ]] || { echo "Run as your normal user, not with sudo." >&2; exit 1; }

step "Checking platform"
. /etc/os-release
ARCH=$(dpkg --print-architecture)
echo "    Ubuntu $VERSION_ID ($VERSION_CODENAME), architecture $ARCH"
if [[ "$VERSION_CODENAME" != "noble" ]]; then
  echo "ROS 2 Jazzy binaries target Ubuntu 24.04 'noble'. Stopping." >&2; exit 1
fi
[[ "$ARCH" == "arm64" || "$ARCH" == "amd64" ]] || { echo "Unsupported arch $ARCH" >&2; exit 1; }

step "Checking internet (ports/archive mirror reachable)"
if ! getent hosts packages.ros.org >/dev/null; then
  echo "DNS lookup failed. Fix the VM network first (UTM Network: Emulated VLAN)." >&2; exit 1
fi

step "Updating the base system"
sudo apt-get update
sudo apt-get -y upgrade

step "Installing base tools"
sudo apt-get install -y curl ca-certificates git nano python3 python3-venv python3-pip \
  python3-pytest locales software-properties-common mesa-utils spice-vdagent bindfs

step "UTF-8 locale"
if ! locale | grep -qi 'utf-8'; then
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
fi
locale | head -3

step "Enabling the Universe repository"
sudo add-apt-repository -y universe

step "Adding the official ROS 2 apt source"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
  [[ -n "$ROS_APT_SOURCE_VERSION" ]] || { echo "Could not read ros-apt-source release tag" >&2; exit 1; }
  echo "    ros-apt-source release: $ROS_APT_SOURCE_VERSION"
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
else
  echo "    already installed"
fi
sudo apt-get update
sudo apt-get -y upgrade   # ROS docs: upgrade before installing to avoid systemd/udev conflicts

step "Installing ROS 2 Jazzy Desktop + dev tools (large download, 10-30 min)"
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools

step "Initialising rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

step "Loading ROS in every new terminal (~/.bashrc)"
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

step "Verifying"
set +u; source /opt/ros/jazzy/setup.bash; set -u
echo "    ROS_DISTRO=$ROS_DISTRO"
ros2 pkg list | grep -qx demo_nodes_py && echo "    demo_nodes_py found"
cat <<MSG

ROS 2 Jazzy installed.
Quick test (two terminals):
  A: ros2 run demo_nodes_cpp talker
  B: ros2 run demo_nodes_py listener
Next: bash ~/vm-share/scripts/02-install-gazebo-turtlebot3.sh
MSG
