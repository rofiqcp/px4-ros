#!/usr/bin/env bash
# Launch PX4 v1.17 Differential Rover on the sirobo desktop using Gazebo GUI.
# SIM ONLY; does not connect to the Cube 2 USB serial device.
set -euo pipefail
PX4_ROOT=/media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0
cd "$PX4_ROOT"
if ! test -x build/px4_sitl_default/bin/px4; then
  echo 'SITL binary missing; run make px4_sitl gz_rover_differential after installing PX4 dependencies' >&2
  exit 2
fi
if ps -eo args= | grep -Fx "$PX4_ROOT/build/px4_sitl_default/bin/px4" >/dev/null; then
  echo 'Another PX4 SITL is running. Close it before starting a GUI session.' >&2
  exit 3
fi
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
if ! DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" timeout 5 xdpyinfo >/dev/null 2>&1; then
  echo "Cannot access the sirobo graphical display. Use run_gazebo_regression.py headless." >&2
  exit 4
fi
unset HEADLESS
export ROS_DOMAIN_ID=105
export GZ_SIM_RESOURCE_PATH="$PX4_ROOT/Tools/simulation/gz/models:$PX4_ROOT/Tools/simulation/gz/worlds"
echo 'Opening Gazebo Differential Rover. This is SITL, NOT physical Cube HITL.'
echo 'Stop with Ctrl+C; mission commands only use ROS_DOMAIN_ID=105 + /dev/pts/* in regression suite.'
exec make px4_sitl gz_rover_differential
