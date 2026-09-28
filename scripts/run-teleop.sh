#!/usr/bin/env bash
# run-teleop.sh — drive TurtleBot3 by keyboard (w/x speed, a/d turn, s or space = stop).
# Do NOT run together with the controller: two nodes would fight over /cmd_vel.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/tb3_ws/install/setup.bash" ]] && source "$HOME/tb3_ws/install/setup.bash"
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
exec ros2 run turtlebot3_teleop teleop_keyboard
