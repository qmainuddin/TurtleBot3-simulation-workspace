#!/usr/bin/env bash
F=ros_ws/src/tb3_nav_task/launch/trial.launch.py
{ echo "bindfs view : $(grep -c _prepare_model ~/vm-share/$F) $(stat -c '%i %s %y' ~/vm-share/$F)";
  echo "9p raw view : $(sudo -n grep -c _prepare_model /mnt/utm/$F 2>&1) $(sudo -n stat -c '%i %s %y' /mnt/utm/$F 2>&1)";
  mount | grep -E 'utm|vm-share'; echo CC-DONE; } > ~/vm-share/logs/cache.txt 2>&1
