#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$ROOT/env.sh"
exec ros2 launch pixhawk_bridge pixhawk.launch.py "$@"
