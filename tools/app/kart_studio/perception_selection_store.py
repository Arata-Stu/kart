"""Store review priorities separately from labels and detect changed input images."""
import hashlib
import json
import math
import time

from .storage import atomic_json, name, read_json, within


def image_signature(project):
    values = dict(frames=[{key: frame.get(key) for key in
                         ('id', 'source_id', 'width', 'height', 'stamp_ns', 'record_stamp_ns', 'split')}
                          for frame in project['frames']],
                  sources=[{key: source.get(key) for key in ('id', 'content_fingerprint', 'topic', 'split')}
                           for source in project['sources']])
    return hashlib.sha256(json.dumps(values, sort_keys=True, allow_nan=False).encode()).hexdigest()


class SelectionStore:
    def __init__(self, datasets):
        self.datasets = datasets

    def get(self, project_id):
        project = self.datasets.load(project_id)
        path = within(self.datasets.folder(project_id), 'selection.json', exists=False)
        if not path.is_file():
            return dict(ranked=[], stale=False, annotations_changed=False)
        result = read_json(path)
        result['stale'] = result.get('images_signature') != image_signature(project)
        result['annotations_changed'] = result.get('project_revision') != project['revision']
        return result

    def publish(self, project_id, expected_revision, expected_images, result):
        if result.get('method') not in ('yolo', 'dinov3') or not isinstance(result.get('ranked'), list):
            raise ValueError('画像選定の結果形式が不正です')
        if len(result['ranked']) > 500:
            raise ValueError('選定画像は500枚までです')
        with self.datasets._lock(project_id):
            project = self.datasets.load(project_id)
            if project['revision'] != expected_revision or image_signature(project) != expected_images:
                raise ValueError('選定中に画像やannotationが更新されました。選定を実行し直してください')
            frames = {frame['id']: frame for frame in project['frames']}
            seen = set()
            for rank, row in enumerate(result['ranked'], 1):
                frame_id = name(row.get('frame_id'))
                if frame_id not in frames or frame_id in seen or frames[frame_id]['reviewed'] or frames[frame_id]['split'] != 'train':
                    raise ValueError('選定対象は未確認のtrain画像だけです')
                seen.add(frame_id)
                if row.get('rank') != rank or type(row.get('score')) not in (int, float) or not math.isfinite(row['score']):
                    raise ValueError('選定順位またはscoreが不正です')
                if not isinstance(row.get('reason'), str) or len(row['reason']) > 1024:
                    raise ValueError('選定理由が不正です')
            published = dict(result, project_id=project_id, project_revision=expected_revision,
                             images_signature=expected_images, candidate_split='train', created=time.time(),
                             automatic_ground_truth=False)
            atomic_json(within(self.datasets.folder(project_id), 'selection.json', exists=False), published)
        return dict(published, stale=False, annotations_changed=False)
