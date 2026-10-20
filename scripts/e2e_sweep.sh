#!/usr/bin/env bash
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$kart_root/ros2_ws/src/kart_e2e${PYTHONPATH:+:$PYTHONPATH}"
kart_python=${KART_E2E_PYTHON:-/usr/bin/python3}
exec "$kart_python" "$kart_root/scripts/lib/e2e_sweep.py" "$@"
