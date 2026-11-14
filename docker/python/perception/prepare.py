#!/usr/bin/env python3
"""Validate the perception runtime and provision the fixed official YOLO weights."""
import argparse
import hashlib
import importlib.metadata
import os
from pathlib import Path
import tempfile
import urllib.request


WEIGHTS_URL = 'https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8s.pt'
WEIGHTS_SHA256 = '1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a'


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def ensure_weights(weights):
    weights = Path(weights)
    weights.parent.mkdir(parents=True, exist_ok=True)
    if weights.exists() or weights.is_symlink():
        if not weights.is_file() or digest(weights) != WEIGHTS_SHA256:
            raise ValueError(f'既存の重みは変更しません。公式YOLOv8sとのhash不一致を確認してください: {weights}')
        print(f'公式重みを保持: {weights}', flush=True)
        return
    with tempfile.TemporaryDirectory(prefix='.download-', dir=weights.parent) as folder:
        target = Path(folder) / 'yolov8s.pt'
        total = 0
        with urllib.request.urlopen(WEIGHTS_URL, timeout=60) as response, target.open('wb') as stream:
            while block := response.read(1024 * 1024):
                total += len(block)
                if total > 64 * 1024 * 1024:
                    raise ValueError('公式重みが64 MiB上限を超えています')
                stream.write(block)
                print(f'YOLOv8s download: {total:,} bytes', flush=True)
        if digest(target) != WEIGHTS_SHA256:
            raise ValueError('公式YOLOv8sのSHA-256が一致しません。重みは公開しません')
        # Atomic publication cannot overwrite an existing file or symlink.
        os.link(target, weights)


def validate_runtime():
    os.environ['YOLO_OFFLINE'] = 'true'
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    with tempfile.TemporaryDirectory(prefix='perception-check-') as cache:
        os.environ['YOLO_CONFIG_DIR'] = cache
        os.environ['MPLCONFIGDIR'] = cache
        import torch
        import torchvision
        import ultralytics
        import onnx
        import onnxruntime
        from rosbags.highlevel import AnyReader
        from PIL import Image
        # CPU operators can be checked at image build time without a GPU.
        assert torchvision.extension._has_ops(), 'torchvision operators are unavailable'
        torchvision.ops.nms(torch.tensor([[0., 0., 1., 1.]]), torch.tensor([1.]), .5)
    versions = {name: importlib.metadata.version(name)
                for name in ('ultralytics', 'torch', 'torchvision', 'rosbags', 'Pillow', 'onnx', 'onnxruntime')}
    print('学習環境を確認しました:', versions, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('weights', type=Path)
    args = parser.parse_args()
    if not args.weights.is_absolute():
        parser.error('weights must be an absolute path')
    validate_runtime()
    ensure_weights(args.weights)


if __name__ == '__main__':
    main()
