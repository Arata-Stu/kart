#!/usr/bin/env bash
# Run inside the kart ROS development container.
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)

if ! command -v colcon >/dev/null 2>&1; then
    echo 'colconが見つかりません。./scripts/dev.shでROS開発コンテナへ入ってから実行してください。' >&2
    exit 1
fi

cd "$kart_root/ros2_ws"
exec colcon build --symlink-install "$@"
