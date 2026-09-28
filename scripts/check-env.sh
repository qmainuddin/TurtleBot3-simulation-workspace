#!/usr/bin/env bash
# check-env.sh — print a health report of the whole stack. Run anytime; changes nothing.
source /opt/ros/jazzy/setup.bash 2>/dev/null
[[ -f "$HOME/tb3_ws/install/setup.bash" ]] && source "$HOME/tb3_ws/install/setup.bash"
[[ -f "$HOME/ros_ws/install/local_setup.bash" ]] && source "$HOME/ros_ws/install/local_setup.bash"
ok()  { printf '  \033[32m[ OK ]\033[0m %s\n' "$*"; }
bad() { printf '  \033[31m[FAIL]\033[0m %s\n' "$*"; }
chk() { if eval "$2" >/dev/null 2>&1; then ok "$1"; else bad "$1"; fi; }

. /etc/os-release
echo "System: $PRETTY_NAME, $(dpkg --print-architecture), kernel $(uname -r)"
chk "Ubuntu 24.04 noble"                 '[[ $VERSION_CODENAME == noble ]]'
chk "Internet (packages.ros.org DNS)"    'getent hosts packages.ros.org'
chk "~/vm-share mounted + writable"      'mountpoint -q ~/vm-share && touch ~/vm-share/.t && rm ~/vm-share/.t'
chk "ROS_DISTRO=jazzy"                   '[[ $ROS_DISTRO == jazzy ]]'
chk "ros_gz_sim (Gazebo Harmonic bridge)" 'ros2 pkg prefix ros_gz_sim'
chk "turtlebot3_gazebo"                  'ros2 pkg prefix turtlebot3_gazebo'
chk "turtlebot3_teleop"                  'ros2 pkg prefix turtlebot3_teleop'
chk "tb3_controller built"               'ros2 pkg prefix tb3_controller'
chk "TURTLEBOT3_MODEL set (in new shells)" 'grep -q TURTLEBOT3_MODEL ~/.bashrc'
echo "Graphics:"; glxinfo -B 2>/dev/null | grep -E 'OpenGL (renderer|version)' | sed 's/^/  /' || echo "  glxinfo unavailable"
if ros2 topic list 2>/dev/null | grep -qx /scan; then
  echo "Simulation running. /cmd_vel type: $(ros2 topic type /cmd_vel 2>/dev/null)"
fi
