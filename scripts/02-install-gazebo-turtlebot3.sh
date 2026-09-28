#!/usr/bin/env bash
# 02-install-gazebo-turtlebot3.sh — Gazebo Harmonic (via ros_gz) + TurtleBot3 simulation for Jazzy
#   Gazebo pairing: https://gazebosim.org/docs/harmonic/ros_installation/
#   TurtleBot3 sim:  https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
# Prefers apt binaries. If a TurtleBot3 package has no binary for this CPU
# architecture, it builds the Jazzy branch from source in ~/tb3_ws instead.
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "Run 01-install-ros-jazzy.sh first." >&2; exit 1; }
source /opt/ros/jazzy/setup.bash

step "Installing Gazebo Harmonic through ros-jazzy-ros-gz"
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-gz

TB3_PKGS=(ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-teleop)
step "Checking TurtleBot3 binary availability for $(dpkg --print-architecture)"
MISSING=()
for p in "${TB3_PKGS[@]}"; do
  cand=$(apt-cache policy "$p" | awk '/Candidate:/ {print $2}')
  printf '    %-32s %s\n' "$p" "${cand:-<none>}"
  [[ -n "$cand" && "$cand" != "(none)" ]] || MISSING+=("$p")
done

if [[ ${#MISSING[@]} -eq 0 ]]; then
  step "Installing TurtleBot3 from apt"
  sudo apt-get install -y "${TB3_PKGS[@]}"
else
  step "Binaries missing (${MISSING[*]}) -> building TurtleBot3 Jazzy from source in ~/tb3_ws"
  # Install whatever binaries do exist, then build the rest.
  for p in "${TB3_PKGS[@]}"; do
    [[ " ${MISSING[*]} " == *" $p "* ]] || sudo apt-get install -y "$p"
  done
  mkdir -p ~/tb3_ws/src && cd ~/tb3_ws/src
  for repo in DynamixelSDK turtlebot3_msgs turtlebot3 turtlebot3_simulations; do
    [[ -d "$repo" ]] || git clone -b jazzy --depth 1 "https://github.com/ROBOTIS-GIT/${repo}.git"
  done
  cd ~/tb3_ws
  rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
  colcon build --symlink-install --packages-up-to turtlebot3_gazebo turtlebot3_teleop
  grep -qxF 'source ~/tb3_ws/install/setup.bash' ~/.bashrc || echo 'source ~/tb3_ws/install/setup.bash' >> ~/.bashrc
  source ~/tb3_ws/install/setup.bash
fi

step "Selecting the Burger model in every new terminal"
grep -qxF 'export TURTLEBOT3_MODEL=burger' ~/.bashrc || echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
export TURTLEBOT3_MODEL=burger

step "Verifying"
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix turtlebot3_gazebo
echo "    Gazebo: $(gz sim --versions 2>/dev/null | head -1 || echo 'gz CLI not on PATH (ok if ros_gz_sim works)')"
echo "    Renderer:"; glxinfo -B 2>/dev/null | grep -E 'OpenGL renderer|OpenGL version' | sed 's/^/      /' || true
cat <<MSG

Done. Test the simulator (in a NEW terminal):
  bash ~/vm-share/scripts/run-sim.sh            # TurtleBot3 world
  bash ~/vm-share/scripts/run-sim.sh --software # if the window is black or crashes
Next: bash ~/vm-share/scripts/03-build-workspace.sh
MSG
