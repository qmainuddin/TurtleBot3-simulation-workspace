#!/usr/bin/env bash
# run-maze.sh — Trekker maze: rebuild, unit tests, plans F + G in Gazebo, analysis.
#   bash ~/vm-share/scripts/run-maze.sh            (headless batch, ~1.5-2 h, resumable)
#   bash ~/vm-share/scripts/run-maze.sh demo       (Gazebo GUI demo: Bug2 in the maze)
set -o pipefail
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/tb3_ws/install/setup.bash" ]] && source "$HOME/tb3_ws/install/setup.bash"
SHARE="$HOME/vm-share"
PKG="$SHARE/ros_ws/src/tb3_nav_task"
mkdir -p "$SHARE/logs"

if [[ "$1" == "demo" ]]; then
  source "$HOME/ros_ws/install/local_setup.bash"
  NAV='{}'; [[ "${2:-bug2}" == bug2 ]] && NAV='{"k_align": 2.0, "turn_first": true}'
  ros2 launch tb3_nav_task trial.launch.py world:=maze strategy:="${2:-bug2}" gui:=true \
    nav_params:="$NAV" timeout:=600.0 trial_id:="demo_maze_${2:-bug2}" plan:=demo results_dir:="$SHARE/task02/demo" \
    2>&1 | tee "$SHARE/logs/maze_demo_${2:-bug2}.txt"
  exit
fi

LOG="$SHARE/logs/maze_run.txt"
{
  echo "==> $(date) build"
  cd "$HOME/ros_ws" && colcon build --symlink-install --packages-select tb3_nav_task || exit 1
  source "$HOME/ros_ws/install/local_setup.bash"
  ls "$HOME/ros_ws/install/tb3_nav_task/share/tb3_nav_task/worlds/"
  echo "==> unit tests"
  ( cd "$PKG" && python3 -m pytest -q test )
  cp -n "$SHARE/task02/results/results.csv" "$SHARE/task02/results/results_before_maze.csv"
  echo "==> $(date) experiments H F G I"
  cd "$SHARE" && python3 "$PKG/scripts/run_experiments.py" --plan H F G I
  echo "==> analysis"
  python3 "$PKG/scripts/analyze_results.py" > /dev/null && echo "analysis written"
  echo "MAZE_RUN_DONE $(date)"
} 2>&1 | tee "$LOG"
