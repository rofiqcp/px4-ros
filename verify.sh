#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$ROOT/env.sh"
for pkg in pixhawk_interfaces pixhawk_avoidance_adapter pixhawk_bridge stock_px4_supervisor; do
  ros2 pkg prefix "$pkg" >/dev/null
  echo "OK: $pkg"
done
echo "ROS-PX4 package and environment verification passed (hardware not tested)."
