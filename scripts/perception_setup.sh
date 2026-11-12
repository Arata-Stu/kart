#!/usr/bin/env bash
# Explicit UI/CLI setup only. Keep training dependencies separate from ROS/inference.
set -euo pipefail
if (( $# )); then
    echo 'Usage: bash scripts/perception_setup.sh (fixed project-local environment)' >&2
    exit 2
fi
perception_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
perception_env="$perception_root/.venv/perception"
mkdir -p "$perception_root/.venv"
if command -v uv >/dev/null 2>&1; then
    perception_uv=$(command -v uv)
else
    perception_bootstrap="$perception_root/.venv/perception-bootstrap"
    python3 -m pip install --disable-pip-version-check --target "$perception_bootstrap" 'uv==0.9.11'
    perception_uv="$perception_bootstrap/bin/uv"
fi
if [[ ! -x "$perception_env/bin/python" ]]; then
    "$perception_uv" venv --python 3.11 "$perception_env"
fi
"$perception_env/bin/python" -c 'import sys; assert sys.version_info[:2] == (3, 11), "perception environment must use Python 3.11"'
"$perception_uv" pip install --python "$perception_env/bin/python" \
    'ultralytics==8.4.209' 'torch==2.15.0' 'torchvision==0.29.2' \
    'rosbags==0.11.9' 'Pillow==13.0.0' 'onnx==1.21.0' 'onnxruntime==1.27.0' \
    'numpy==2.4.6' 'PyYAML==6.0.3'
"$perception_env/bin/python" - "$perception_root" <<'PY'
import hashlib
import os
from pathlib import Path
import sys
import tempfile
import urllib.request

root = Path(sys.argv[1])
weights = root / 'weights/racing/yolov8s/yolov8s.pt'
expected = '1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a'
def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()
weights.parent.mkdir(parents=True, exist_ok=True)
if weights.exists() or weights.is_symlink():
    if not weights.is_file() or digest(weights) != expected:
        raise SystemExit(f'既存の重みは変更しません。公式YOLOv8sとのhash不一致を確認してください: {weights}')
    print(f'公式重みを保持: {weights}', flush=True)
else:
    url = 'https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8s.pt'
    with tempfile.TemporaryDirectory(prefix='.download-', dir=weights.parent) as folder:
        target = Path(folder) / 'yolov8s.pt'
        total = 0
        with urllib.request.urlopen(url, timeout=60) as response, target.open('wb') as stream:
            while block := response.read(1024 * 1024):
                total += len(block)
                if total > 64 * 1024 * 1024:
                    raise ValueError('公式重みが64 MiB上限を超えています')
                stream.write(block)
                print(f'YOLOv8s download: {total:,} bytes', flush=True)
        if digest(target) != expected:
            raise ValueError('公式YOLOv8sのSHA-256が一致しません。重みは公開しません')
        if weights.exists() or weights.is_symlink():
            raise ValueError('取得中に重みが配置されました。既存ファイルは上書きしません')
        os.link(target, weights)  # Atomic publication that cannot overwrite an existing file/link.
os.environ['YOLO_OFFLINE'] = 'true'
os.environ['YOLO_AUTOINSTALL'] = 'false'
import importlib.metadata
versions = {name: importlib.metadata.version(name) for name in ('ultralytics', 'torch', 'rosbags', 'Pillow', 'onnx')}
print('学習環境を準備しました:', root / '.venv/perception/bin/python', versions, flush=True)
PY
