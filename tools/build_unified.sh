#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
LIDAR_ROOT="${LIDAR_WORKSPACE:-$(cd -- "$ROOT/../slam3d-livox-mid360s" 2>/dev/null && pwd -P)}"
[[ -x "$LIDAR_ROOT/build.sh" ]] || { echo "Missing LiDAR workspace/build.sh; set LIDAR_WORKSPACE" >&2; exit 2; }
"$LIDAR_ROOT/build.sh"
exec "$ROOT/build.sh"
