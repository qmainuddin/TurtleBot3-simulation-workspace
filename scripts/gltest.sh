#!/usr/bin/env bash
# gltest.sh — find a Gazebo rendering configuration where the Burger's gpu_lidar produces /scan.
source /opt/ros/jazzy/setup.bash
TB3=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo
export GZ_SIM_RESOURCE_PATH=$TB3/models:$GZ_SIM_RESOURCE_PATH
SRC=~/ros_ws/install/tb3_nav_task/share/tb3_nav_task/worlds/open_field.sdf
OUT=~/vm-share/logs/gltest.txt; : > "$OUT"
mk() {  # mk <engine> -> world with the burger included
  sed -e "s#<render_engine>ogre2</render_engine>#<render_engine>$1</render_engine>#" \
      -e "s#</world>#<include><uri>model://turtlebot3_burger</uri><name>burger</name><pose>-2.8 -1.8 0.01 0 0 0</pose></include></world>#" \
      "$SRC" > /tmp/gltest_$1.sdf
}
mk ogre2; mk ogre
try() {  # try <label> <engine> <env...>
  local label=$1 eng=$2; shift 2
  local log=/tmp/gltest_$label.log
  pkill -f 'gz sim' 2>/dev/null; sleep 1
  env "$@" gz sim -s -r -v 3 /tmp/gltest_$eng.sdf > "$log" 2>&1 &
  local pid=$!
  sleep 12
  local scan
  scan=$(env "$@" timeout 15 gz topic -e -t /scan -n 1 2>/dev/null | grep -m1 -E 'count|ranges' )
  local alive=dead; kill -0 $pid 2>/dev/null && alive=alive
  kill $pid 2>/dev/null; sleep 2; pkill -9 -f 'gz sim' 2>/dev/null
  printf '%-22s server=%-5s scan=%-3s %s\n' "$label" "$alive" "$([ -n "$scan" ] && echo YES || echo no)" \
     "$(grep -m1 -oE 'Segmentation fault|Unable to [^.]{0,60}|OpenGL [0-9.]+[^,]{0,30}' "$log")" | tee -a "$OUT"
}
try ogre2_default ogre2
try ogre2_sw      ogre2 LIBGL_ALWAYS_SOFTWARE=1
try ogre2_sw_surf ogre2 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe EGL_PLATFORM=surfaceless
try ogre2_sw_x11  ogre2 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe EGL_PLATFORM=x11
try ogre1_default ogre
try ogre1_sw      ogre  LIBGL_ALWAYS_SOFTWARE=1
echo DONE >> "$OUT"
