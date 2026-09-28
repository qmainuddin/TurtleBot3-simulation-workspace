#!/usr/bin/env bash
# probe.sh [world] — clean start, launch sim without controller, print one sample of each sensor.
source /opt/ros/jazzy/setup.bash
OUT=~/vm-share/logs/probe.txt; : > "$OUT"
kill_all() { pkill -f 'ros2 launch'; pkill -f 'gz sim'; pkill -f parameter_bridge; pkill -f robot_state_publisher;
             pkill -f tb3_nav_task; sleep 2; pkill -9 -f 'gz sim'; pkill -9 -f parameter_bridge; sleep 1; }
kill_all
echo "leftover gz: $(pgrep -fc 'gz sim')" | tee -a "$OUT"
cd ~/ros_ws && colcon build --symlink-install --packages-select tb3_nav_task 2>&1 | tail -1 | tee -a "$OUT"
source ~/ros_ws/install/local_setup.bash
INST=$(ros2 pkg prefix tb3_nav_task)/share/tb3_nav_task/launch/trial.launch.py
echo "installed launch: $(readlink -f "$INST")  prepare_model=$(grep -c _prepare_model "$INST")" | tee -a "$OUT"
ros2 launch tb3_nav_task trial.launch.py world:=${1:-open_field} gui:=false navigator:=false monitor:=false ${PROBE_ARGS} \
    > ~/vm-share/logs/probe_launch.log 2>&1 &
sleep 18
python3 ~/vm-share/ros_ws/src/tb3_nav_task/scripts/probe_sensors.py ${1:-open_field} 10 2>&1 | tee -a "$OUT"
kill_all
echo PROBE-DONE >> "$OUT"
