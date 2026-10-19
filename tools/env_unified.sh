#!/usr/bin/env bash
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
LIDAR_ROOT="${LIDAR_WORKSPACE:-$(cd -- "$ROOT/../slam3d-livox-mid360s" 2>/dev/null && pwd -P)}"
[[ -f "$LIDAR_ROOT/env.sh" ]] || { echo "Missing LiDAR env.sh; set LIDAR_WORKSPACE" >&2; return 2; }
source "$LIDAR_ROOT/env.sh" || return
source "$ROOT/env.sh" || return
