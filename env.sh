#!/usr/bin/env bash
# Portable PX4-ROS workspace environment. Does not overwrite caller's ROOT/CACHE.
PX4_ROS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[[ -f "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; return 2; }
_source_ros_safe() {
  local had_nounset=0 rc
  [[ $- == *u* ]] && had_nounset=1
  set +u
  source "$1"
  rc=$?
  if (( had_nounset )); then set -u; fi
  return "$rc"
}
_source_ros_safe "$ROS_SETUP" || return
PX4_ROS_KEY="$(printf '%s' "$PX4_ROS_ROOT" | cksum | cut -d' ' -f1)"
PX4_ROS_COLCON_CACHE="${ROS_COLCON_CACHE_BASE:-${XDG_CACHE_HOME:-$HOME/.cache}/otomasi-colcon}/$(basename "$PX4_ROS_ROOT")-$PX4_ROS_KEY"
[[ -f "$PX4_ROS_COLCON_CACHE/install/setup.bash" ]] || { echo "Build required: $PX4_ROS_ROOT/build.sh" >&2; return 1; }
_source_ros_safe "$PX4_ROS_COLCON_CACHE/install/setup.bash"
unset -f _source_ros_safe
