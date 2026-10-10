#!/usr/bin/env bash
# TUI and CLI entrypoint. Run inside the ROS development container.
set -eo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -f "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
fi
if [[ -f "$kart_root/ros2_ws/install/setup.bash" ]]; then
    source "$kart_root/ros2_ws/install/setup.bash"
fi
exec "${KART_BRINGUP_PYTHON:-python3}" "$kart_root/scripts/lib/bringup.py" "$@"
