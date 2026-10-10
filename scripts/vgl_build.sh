#!/usr/bin/env bash
# One command on the target GPU; verified existing engines are reused.
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    echo 'Usage: bash scripts/vgl_build.sh [model-name (default: 424x240)] [--retry]'
    exit 0
fi
kart_model=424x240
if (( $# > 0 )) && [[ "$1" != --* ]]; then kart_model=$1; shift; fi
set +u
if [[ -f /opt/ros/lyrical/setup.bash ]]; then source /opt/ros/lyrical/setup.bash; fi
if [[ -f "$kart_root/ros2_ws/install/setup.bash" ]]; then source "$kart_root/ros2_ws/install/setup.bash"; fi
set -u
exec bash "$kart_root/scripts/vgl_model.sh" build --name "$kart_model" "$@"
