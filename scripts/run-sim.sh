#!/usr/bin/env bash
# run-sim.sh [--software] [world]  — start Gazebo Harmonic with TurtleBot3 Burger.
#   world: world (default, obstacle arena) | house | empty
#   --software: force Mesa software OpenGL if the Gazebo window is black/crashes in the VM.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/tb3_ws/install/setup.bash" ]] && source "$HOME/tb3_ws/install/setup.bash"
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}

WORLD=world
for a in "$@"; do
  case "$a" in
    --software) export LIBGL_ALWAYS_SOFTWARE=1; echo "Using software rendering";;
    *) WORLD="$a";;
  esac
done
case "$WORLD" in
  empty) LAUNCH=empty_world.launch.py;;
  *)     LAUNCH=turtlebot3_${WORLD}.launch.py;;
esac
echo "Launching turtlebot3_gazebo $LAUNCH (model=$TURTLEBOT3_MODEL)"
exec ros2 launch turtlebot3_gazebo "$LAUNCH"
