#!/usr/bin/env bash
# Reuse the baked Docker environment, or explicitly set up a local environment.
set -euo pipefail
if (( $# )); then
    echo 'Usage: bash scripts/perception_setup.sh (reuse Docker runtime or set up project-local environment)' >&2
    exit 2
fi
perception_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
perception_definition="$perception_root/docker/python/perception"
if [[ -x /opt/perception/bin/python && -f /opt/kart/models/yolov8s.pt ]]; then
    /opt/perception/bin/python "$perception_definition/prepare.py" /opt/kart/models/yolov8s.pt
    echo 'Docker内の学習環境を使用します: /opt/perception/bin/python'
    exit 0
fi
perception_env="$perception_root/.venv/perception"
mkdir -p "$perception_root/.venv"
if command -v uv >/dev/null 2>&1; then
    perception_uv=$(command -v uv)
else
    perception_bootstrap="$perception_root/.venv/perception-bootstrap"
    python3 -m pip install --disable-pip-version-check --target "$perception_bootstrap" 'uv==0.12.0'
    perception_uv="$perception_bootstrap/bin/uv"
fi
UV_PROJECT_ENVIRONMENT="$perception_env" "$perception_uv" sync \
    --project "$perception_definition" --python 3.12 --locked --no-dev
"$perception_env/bin/python" "$perception_definition/prepare.py" \
    "$perception_root/weights/racing/yolov8s/yolov8s.pt"
