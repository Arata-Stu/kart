#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$root/ros2_ws/src/kart_e2e${PYTHONPATH:+:$PYTHONPATH}"
exec /opt/inference/bin/python "$root/scripts/build_racing_engine.py" "$@"
