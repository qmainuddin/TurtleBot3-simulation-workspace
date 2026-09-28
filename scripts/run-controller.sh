#!/usr/bin/env bash
# run-controller.sh [ros-args...] — run the obstacle_avoider node against the running simulation.
# Examples:
#   bash run-controller.sh
#   bash run-controller.sh -p max_linear:=0.12 -p stop_distance:=0.4
# Ctrl+C stops the node and sends a zero-velocity command.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/tb3_ws/install/setup.bash" ]] && source "$HOME/tb3_ws/install/setup.bash"
source "$HOME/ros_ws/install/local_setup.bash"
exec ros2 run tb3_controller obstacle_avoider --ros-args -p use_sim_time:=true "$@"
