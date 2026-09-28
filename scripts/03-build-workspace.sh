#!/usr/bin/env bash
# 03-build-workspace.sh — build the tb3_controller package.
# Source code stays on the Mac (~/vm-share/ros_ws/src); build/ install/ log/ live on the
# VM's own Linux disk (~/ros_ws) because colcon relies on symlinks and Linux permissions.
# With --symlink-install, edits to .py files on the Mac take effect without rebuilding;
# rebuild only after changing setup.py, package.xml or adding launch files.
set -eo pipefail
SHARE="$HOME/vm-share"
WS="$HOME/ros_ws"

[[ -d "$SHARE/ros_ws/src/tb3_controller" ]] || { echo "~/vm-share not mounted? Run 00-setup-share.sh" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/tb3_ws/install/setup.bash" ]] && source "$HOME/tb3_ws/install/setup.bash"

mkdir -p "$WS/src"
for pkg in "$SHARE"/ros_ws/src/*/; do
  name=$(basename "$pkg")
  [[ -L "$WS/src/$name" ]] || ln -s "$pkg" "$WS/src/$name"
  echo "linked $WS/src/$name -> $pkg"
done

cd "$WS"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install

grep -qxF 'source ~/ros_ws/install/local_setup.bash' ~/.bashrc || \
  echo 'source ~/ros_ws/install/local_setup.bash' >> ~/.bashrc

echo
echo "==> Unit tests (pure Python, no simulator)"
( cd "$SHARE/ros_ws/src/tb3_controller" && python3 -m pytest -q test ) || true

cat <<MSG

Built. Open a NEW terminal (or: source ~/ros_ws/install/local_setup.bash), then:
  bash ~/vm-share/scripts/run-sim.sh           # terminal A: Gazebo + TurtleBot3
  bash ~/vm-share/scripts/run-controller.sh    # terminal B: your Python controller
or both at once:
  ros2 launch tb3_controller sim_with_controller.launch.py
MSG
