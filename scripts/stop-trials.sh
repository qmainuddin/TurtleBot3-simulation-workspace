#!/usr/bin/env bash
# stop-trials.sh — stop a running experiment batch and every Gazebo/ROS process it started.
pkill -f run_experiments.py
for p in 'ros2 launch tb3_nav_task' 'gz sim' parameter_bridge robot_state_publisher ros_gz_sim/create \
         tb3_nav_task/navigator tb3_nav_task/trial_monitor; do pkill -f "$p"; done
sleep 3
pkill -9 -f 'gz sim'; pkill -9 -f parameter_bridge
echo "stopped"
