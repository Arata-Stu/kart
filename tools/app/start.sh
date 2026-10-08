#!/usr/bin/env bash
set -e
APP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
KART_ROOT="$(cd "$APP_ROOT/../.." && pwd)"
if [[ -f /opt/ros/lyrical/setup.bash ]]; then source /opt/ros/lyrical/setup.bash; fi
if [[ -f "$KART_ROOT/ros2_ws/install/setup.bash" ]]; then source "$KART_ROOT/ros2_ws/install/setup.bash"; fi
export PYTHONPATH="$APP_ROOT:$KART_ROOT/ros2_ws/src/kart_mapping${PYTHONPATH:+:$PYTHONPATH}"
exec "${KART_STUDIO_PYTHON:-python3}" -m kart_studio.server "$@"
