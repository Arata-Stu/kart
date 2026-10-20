"""ROS-free tuning protocol and stop gate; shared with the Notebook adapter."""
import hashlib
import json
import math
import time
from pathlib import Path

MAX_BYTES = 3 * 1024 * 1024


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(encoded(value)).hexdigest()


def map_identity(folder):
    root = Path(folder)
    if root.is_symlink():
        raise ValueError('Localization root must not be a symlink')
    source = root / 'cuvslam_map'
    if source.is_symlink():
        raise ValueError('Localization assets must not be symlinks')
    files = sorted(source.rglob('*')) if source.is_dir() else [root / 'snapshot.json']
    result = hashlib.sha256()
    count = 0
    for path in files:
        if path.is_symlink():
            raise ValueError('Localization map symlinks are unsupported')
        if not path.is_file():
            continue
        count += 1
        result.update(str(path.relative_to(root)).encode() + b'\0')
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                result.update(chunk)
        result.update(b'\0')
    if not count:
        raise ValueError('cuvslam_map or snapshot.json is required for map identity')
    return result.hexdigest()


def validate(snapshot, identity):
    if len(encoded(snapshot)) > MAX_BYTES:
        raise ValueError('Tuning snapshot exceeds 3 MiB')
    if snapshot.get('map_id') != identity:
        raise ValueError('NotebookとJetsonの地図が一致しません')
    if snapshot.get('revision') != digest({k: v for k, v in snapshot.items() if k != 'revision'}):
        raise ValueError('Snapshot revision mismatch')
    from .document import normalize, select, KINDS
    document = snapshot['document']
    # Network requests contain embedded Studio geometry, never remote CSV paths.
    if document.get('schema') != 'kart.hdmap.v2' or 'frame_id' in document:
        raise ValueError('Embedded Studio v2 document required')
    normalized = normalize(document)
    lane = select(normalized, snapshot['lane_id'])
    kind = snapshot['line_type']
    if lane is None or kind not in KINDS or kind not in lane['lines']:
        raise ValueError('選択したlane・生成ラインがありません')
    speed = snapshot['centerline_speed_mps']
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed < 0:
        raise ValueError('Centerline speed must be finite and nonnegative')
    if sum(len(line['points']) for lane in normalized['lanes'] for line in lane['lines'].values()) > 20000:
        raise ValueError('Tuning snapshot exceeds 20000 points')
    return normalized


class StopGate:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.mode = None
        self.mode_at = self.speed_at = float('-inf')
        self.speed = None
        self.stopped_at = None

    def update_mode(self, mode):
        self.mode, self.mode_at = mode, self.clock()

    def update_speed(self, speed):
        now = self.clock()
        if now - self.speed_at > .5:
            self.stopped_at = None
        self.speed_at = now
        self.speed = speed if speed is not None and math.isfinite(speed) else None
        if self.speed is not None and abs(self.speed) < .05:
            if self.stopped_at is None:
                self.stopped_at = now
        else:
            self.stopped_at = None

    def issue(self):
        now = self.clock()
        if self.mode != 3 or now - self.mode_at > .3:
            return '最新のSTOP状態が必要です'
        if now - self.speed_at > .5 or self.stopped_at is None or now - self.stopped_at < 1:
            return '最新の速度情報で1秒以上の停車確認が必要です'
        return ''
