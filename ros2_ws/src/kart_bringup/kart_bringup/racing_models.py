"""ROS-free model contracts shared by engine build, launch and bag reports."""
import hashlib
import json
from pathlib import Path


COCO_CLASSES = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
    'toothbrush',
)


def get_profile(name):
    if name == 'coco':
        classes, tracked, stem = COCO_CLASSES, '2', 'yolov8s'
    elif name == 'opponent':
        classes, tracked, stem = ('opponent',), '0', 'opponent'
    else:
        raise ValueError(f'Unknown racing model profile: {name!r}; choose coco or opponent')
    return dict(name=name, num_classes=len(classes), class_names=list(classes),
                track_class_id=tracked, source_name=stem + '.onnx', engine_name=stem + '.plan')


def engine_profile(engine, requested=''):
    """Resolve the engine's classes and reject a conflicting/stale build manifest.

    External engines without a manifest default to COCO; legacy one-class
    engines can be selected explicitly with model_profile:=opponent.
    """
    path = Path(engine)
    manifest = path.parent / 'build.json'
    data = json.loads(manifest.read_text()) if manifest.is_file() else {}
    if not isinstance(data, dict):
        raise ValueError(f'Invalid racing engine manifest: {manifest}')
    recorded = data.get('model_profile', '')
    if recorded and requested and recorded != requested:
        raise ValueError('Engine and requested model profiles differ; rebuild or select the matching engine')
    if data and not recorded and not requested:
        raise ValueError('Legacy engine metadata: rebuild with racing_trt.sh --model opponent, '
                         'or explicitly set model_profile:=opponent')
    profile = get_profile(recorded or requested or 'coco')
    if recorded and data.get('num_classes') != profile['num_classes']:
        raise ValueError('Engine metadata class count does not match its model profile')
    if data:
        expected = data.get('engine_sha256')
        with path.open('rb') as stream:
            actual = hashlib.file_digest(stream, 'sha256').hexdigest()
        if expected != actual:
            raise ValueError('Engine hash does not match build.json; rebuild the engine')
    return profile
