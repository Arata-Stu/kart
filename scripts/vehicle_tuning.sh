#!/usr/bin/env bash
set -euo pipefail
KART_TUNING_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == "collect" ]]; then
  shift
  # Requires the caller's ROS/workspace environment. No automatic recording/AUTO.
  exec ros2 launch kart_bringup tuning_collect.launch.py "$@"
fi
exec "${KART_TUNING_PYTHON:-python3}" "$KART_TUNING_ROOT/tools/vehicle_tuning/cli.py" "$@"
