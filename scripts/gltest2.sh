#!/usr/bin/env bash
# gltest2.sh — ogre2 GPU-LiDAR configurations; reports whether /scan has real (varied) ranges.
source /opt/ros/jazzy/setup.bash
TB3=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo
export GZ_SIM_RESOURCE_PATH=$TB3/models:$GZ_SIM_RESOURCE_PATH
SRC=~/ros_ws/install/tb3_nav_task/share/tb3_nav_task/worlds/open_field.sdf
OUT=~/vm-share/logs/gltest2.txt; : > "$OUT"
sed -e "s#<render_engine>ogre</render_engine>#<render_engine>ogre2</render_engine>#" \
    -e "s#</world>#<include><uri>model://turtlebot3_burger</uri><name>burger</name><pose>-2.8 -1.8 0.01 0 0 0</pose></include></world>#" \
    "$SRC" > /tmp/gl2.sdf
ls -la /dev/dri/ >> "$OUT" 2>&1; id >> "$OUT"
try() {
  local label=$1; shift
  local log=/tmp/gl2_$label.log
  pkill -9 -f 'gz sim' 2>/dev/null; sleep 1
  env "$@" gz sim -s -r -v 4 /tmp/gl2.sdf > "$log" 2>&1 &
  local pid=$!
  sleep 14
  local vals
  vals=$(timeout 12 gz topic -e -t /scan -n 1 2>/dev/null | grep -E '^ranges:' | awk '{print $2}' | sort -u | wc -l)
  local alive=dead; kill -0 $pid 2>/dev/null && alive=alive
  kill $pid 2>/dev/null; sleep 2; pkill -9 -f 'gz sim' 2>/dev/null
  printf '%-16s server=%-5s distinct_ranges=%-4s %s\n' "$label" "$alive" "$vals" \
    "$(grep -m1 -oE 'Segmentation fault|OpenGL [^,]{0,60}|EGL[^,]{0,80}' "$log")" | tee -a "$OUT"
  grep -iE 'egl|dri|libgl|renderer|opengl' "$log" | head -6 | cut -c1-200 | sed 's/^/     /' >> "$OUT"
}
DBG="EGL_LOG_LEVEL=debug LIBGL_DEBUG=verbose"
try sw_softpipe   $DBG LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=softpipe
try sw_surf_swr   $DBG LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless MESA_LOADER_DRIVER_OVERRIDE=swrast
try sw_kms        $DBG LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=kms_swrast
try virgl_ovr     $DBG MESA_GL_VERSION_OVERRIDE=4.3 MESA_GLSL_VERSION_OVERRIDE=430
try sw_llvm_ovr   $DBG LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_GL_VERSION_OVERRIDE=4.5
try sw_nodev      $DBG LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
echo DONE >> "$OUT"
