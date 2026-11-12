"""Confined RC-car image annotation projects and immutable YOLO datasets.

ROS, Pillow and rosbags are not imported until image extraction is requested.
Sampling uses bag record time; source header and record stamps remain separate.
"""
from contextlib import contextmanager
import copy
import fcntl
import hashlib
import io
import json
import math
from pathlib import Path
import shutil
import tempfile
import time
import uuid

from .storage import atomic_json, name, read_json, within


CLASS_NAMES = ['rc_car']
MAX_FRAMES = 500
MAX_PROJECT_FRAMES = 5000
MAX_BOXES = 100
MAX_PIXELS = 16_777_216
MAX_IMAGE_BYTES = 64 * 1024 * 1024
IMAGE_TYPES = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')


def _check(job):
    if job is not None:
        job.check()


def _log(job, message):
    if job is not None:
        job.log(message)


def _revision(value):
    if type(value) is not int or value < 1:
        raise ValueError('revisionは正の整数で指定してください')
    return value


def _boxes(rows, width, height):
    if not isinstance(rows, list) or len(rows) > MAX_BOXES:
        raise ValueError(f'bboxは画像ごとに{MAX_BOXES}件以下で指定してください')
    result = []
    for row in rows:
        if not isinstance(row, (list, tuple)) or len(row) != 4:
            raise ValueError('bboxは[x1,y1,x2,y2]の4座標です')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in row):
            raise ValueError('bbox座標は有限の数値で指定してください')
        x1, y1, x2, y2 = map(float, row)
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError('bboxは画像内で幅・高さが正になるよう指定してください')
        result.append([x1, y1, x2, y2])
    return result


def _dimensions(width, height):
    if (type(width) is not int or type(height) is not int or width < 1 or height < 1 or
            width > 8192 or height > 8192 or width * height > MAX_PIXELS):
        raise ValueError('画像サイズが不正か上限を超えています')


def decode_image(message, message_type):
    """Decode only supported 8-bit raw or JPEG/PNG RGB training images."""
    try:
        from PIL import Image, UnidentifiedImageError
    except ImportError as exc:
        raise ValueError('画像抽出には実行PythonのPillowが必要です') from exc

    if len(message.data) > MAX_IMAGE_BYTES:
        raise ValueError('画像データが64 MiB上限を超えています')
    if message_type == 'sensor_msgs/msg/Image':
        width, height = int(message.width), int(message.height)
        _dimensions(width, height)
        encodings = {'rgb8': ('RGB', 'RGB', 3), 'bgr8': ('RGB', 'BGR', 3), 'mono8': ('L', 'L', 1)}
        if message.encoding not in encodings:
            raise ValueError(f'未対応の画像encodingです: {message.encoding}')
        mode, raw_mode, channels = encodings[message.encoding]
        step = int(message.step)
        if step < width * channels or step * height > MAX_IMAGE_BYTES or len(message.data) < step * height:
            raise ValueError('Imageのstepまたはdata長が不正です')
        return Image.frombytes(mode, (width, height), bytes(message.data), 'raw', raw_mode, step, 1).convert('RGB')
    if message_type != 'sensor_msgs/msg/CompressedImage':
        raise ValueError('RGB ImageまたはJPEG/PNG CompressedImageを指定してください')
    try:
        with Image.open(io.BytesIO(bytes(message.data))) as image:
            if image.format not in ('JPEG', 'PNG'):
                raise ValueError('CompressedImageはJPEGまたはPNGのみ対応しています')
            _dimensions(*image.size)
            return image.convert('RGB')
    except (UnidentifiedImageError, OSError, Image.DecompressionBombError) as exc:
        raise ValueError('CompressedImageを画像として読み込めません') from exc


def _bag_messages(bag, topic, period_ns=0, job=None):
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise ValueError('bag画像抽出には実行Pythonのrosbagsが必要です') from exc

    with AnyReader([bag], default_typestore=get_typestore(Stores.LATEST)) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            raise ValueError(f'bagに指定topicがありません: {topic}')
        if any(connection.msgtype not in IMAGE_TYPES for connection in connections):
            raise ValueError('指定topicは対応するROS2 RGB画像ではありません')
        next_stamp = None
        for connection, timestamp, raw in reader.messages(connections=connections):
            _check(job)
            if next_stamp is not None and timestamp < next_stamp:
                continue
            next_stamp = timestamp + period_ns
            yield int(timestamp), connection.msgtype, reader.deserialize(raw, connection.msgtype)


class DatasetStore:
    def __init__(self, base, records=None):
        self.base = Path(base).resolve()
        self.root = within(self.base, 'perception/projects', exists=False)
        self.root.mkdir(parents=True, exist_ok=True)
        self.records_input = None if records is None else Path(records).absolute()
        self.records = None if records is None else Path(records).resolve()

    def folder(self, project_id):
        return within(self.root, name(project_id))

    @contextmanager
    def _lock(self, project_id):
        path = within(self.folder(project_id), '.lock', exists=False)
        with path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def _document(self, project_id):
        document = read_json(within(self.folder(project_id), 'project.json'))
        if (document.get('id') != project_id or document.get('class_names') != CLASS_NAMES or
                not isinstance(document.get('frames'), list) or not isinstance(document.get('sources'), list)):
            raise ValueError('annotation projectの形式が不正です')
        _revision(document['revision'])
        return document

    @staticmethod
    def _public(document):
        result = copy.deepcopy(document)
        for frame in result['frames']:
            frame.pop('image', None)
        return result

    @staticmethod
    def _summary(document):
        reviewed = [frame for frame in document['frames'] if frame['reviewed']]
        return dict(id=document['id'], name=document['name'], revision=document['revision'],
                    frames_count=len(document['frames']), reviewed_count=len(reviewed),
                    positive_count=sum(bool(frame['boxes']) for frame in reviewed),
                    negative_count=sum(not frame['boxes'] for frame in reviewed))

    def catalog(self):
        result = []
        for path in sorted(self.root.iterdir()):
            if path.name.startswith('.') or path.is_symlink() or not path.is_dir():
                continue
            try:
                result.append(self._summary(self._document(name(path.name))))
            except (ValueError, KeyError, OSError):
                continue
        return result

    def create(self, project_id, display_name=None):
        project_id = name(project_id)
        title = project_id if display_name is None else display_name
        if not isinstance(title, str) or not title.strip() or len(title) > 120:
            raise ValueError('project名は1〜120文字で指定してください')
        destination = within(self.root, project_id, exists=False)
        if destination.exists():
            raise ValueError('同じIDのprojectが存在します')
        with tempfile.TemporaryDirectory(prefix='.create-', dir=self.root) as temp:
            staging = Path(temp) / project_id
            staging.mkdir()
            document = dict(id=project_id, name=title.strip(), class_names=list(CLASS_NAMES),
                            revision=1, created=time.time(), frames=[], sources=[])
            atomic_json(staging / 'project.json', document)
            try:
                staging.rename(destination)
            except FileExistsError as exc:
                raise ValueError('同じIDのprojectが存在します') from exc
        return self._public(document)

    def load(self, project_id):
        return self._public(self._document(project_id))

    def save_frame(self, project_id, frame_id, project_revision, frame_revision, boxes, reviewed=True):
        name(frame_id)
        if type(reviewed) is not bool:
            raise ValueError('reviewedはtrue/falseで指定してください')
        with self._lock(project_id):
            document = self._document(project_id)
            if _revision(project_revision) != document['revision']:
                raise ValueError('projectが更新されています。再読み込みしてください')
            frame = self._frame(document, frame_id)
            if _revision(frame_revision) != frame['revision']:
                raise ValueError('画像のannotationが更新されています。再読み込みしてください')
            frame.update(boxes=_boxes(boxes, frame['width'], frame['height']), reviewed=reviewed,
                         revision=frame['revision'] + 1)
            document['revision'] += 1
            atomic_json(self.folder(project_id) / 'project.json', document)
        return self._public(document)

    def apply_suggestions(self, project_id, expected_revision, suggestions):
        if not isinstance(suggestions, dict) or len(suggestions) > MAX_PROJECT_FRAMES:
            raise ValueError('推定bboxの画像一覧が不正です')
        with self._lock(project_id):
            document = self._document(project_id)
            if _revision(expected_revision) != document['revision']:
                raise ValueError('推定中にprojectが更新されました。実行し直してください')
            changed = False
            for frame_id, boxes in suggestions.items():
                frame = self._frame(document, name(frame_id))
                boxes = _boxes(boxes, frame['width'], frame['height'])
                if frame['suggestions'] != boxes:
                    frame['suggestions'] = boxes
                    frame['revision'] += 1
                    changed = True
            if changed:
                document['revision'] += 1
                atomic_json(self.folder(project_id) / 'project.json', document)
        return self._public(document)

    @staticmethod
    def _frame(document, frame_id):
        for frame in document['frames']:
            if frame['id'] == frame_id:
                return frame
        raise ValueError('指定画像がprojectにありません')

    def image_path(self, project_id, frame_id):
        frame = self._frame(self._document(project_id), name(frame_id))
        image = within(self.folder(project_id), frame['image'])
        if not image.is_file():
            raise ValueError('画像ファイルがありません')
        return image

    def _bag_path(self, bag):
        path = Path(bag).expanduser()
        if self.records is not None:
            if path.is_absolute():
                for root in (self.records, self.records_input):
                    try:
                        path = path.relative_to(root)
                        break
                    except ValueError:
                        pass
                else:
                    raise ValueError('bagはrecord rootの配下から選択してください')
            path = within(self.records, path)
        elif path.is_symlink():
            raise ValueError('bagのsymlinkは使えません')
        path = path.resolve(strict=True)
        if not path.is_dir():
            raise ValueError('ROS2 bagのdirectoryを指定してください')
        within(path, 'metadata.yaml')
        return path

    def _identity(self, bag, job=None):
        metadata_path = within(bag, 'metadata.yaml')
        if metadata_path.stat().st_size > 4 * 1024 * 1024:
            raise ValueError('bag metadataが大きすぎます')
        metadata = metadata_path.read_bytes()
        parts = []
        for path in bag.rglob('*'):
            _check(job)
            if path.name.endswith(('.db3', '.mcap', '.db3.zstd', '.mcap.zstd')):
                parts.append(path)
                if len(parts) > 512:
                    raise ValueError('bagの512分割上限を超えています')
        if not parts:
            raise ValueError('bagの記録ファイルがないか、512分割上限を超えています')
        samples = []
        for path in sorted(parts):
            _check(job)
            path = within(bag, path.relative_to(bag))
            size = path.stat().st_size
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                digest.update(stream.read(131072))
                if size > 131072:
                    stream.seek(max(131072, size - 131072))
                    digest.update(stream.read(131072))
            samples.append(dict(size=size, sampled_sha256=digest.hexdigest()))
        content = hashlib.sha256(json.dumps(samples, sort_keys=True).encode()).hexdigest()
        return dict(resolved_path=str(bag), metadata_sha256=hashlib.sha256(metadata).hexdigest(),
                    content_fingerprint=content, identity_method='size+head-tail-128KiB-v1', parts=samples)

    @staticmethod
    def _reject_duplicate(document, source):
        for existing in document['sources']:
            if (existing['resolved_path'] == source['resolved_path'] or
                    existing['content_fingerprint'] == source['content_fingerprint']):
                raise ValueError('同じbagまたは同一内容のbagは重複登録できません。splitもbag単位で固定です')

    def extract(self, project_id, bag, topic, split='train', rate_hz=2., max_frames=500,
                expected_revision=None, job=None):
        if split not in ('train', 'val'):
            raise ValueError('splitはtrainまたはvalです')
        if not isinstance(topic, str) or not topic.startswith('/') or len(topic) > 256:
            raise ValueError('RGB topicを指定してください')
        if (isinstance(rate_hz, bool) or not isinstance(rate_hz, (int, float)) or
                not math.isfinite(rate_hz) or not .1 <= rate_hz <= 30):
            raise ValueError('間引きHzは0.1〜30で指定してください')
        if type(max_frames) is not int or not 1 <= max_frames <= MAX_FRAMES:
            raise ValueError(f'抽出枚数は1〜{MAX_FRAMES}で指定してください')
        bag = self._bag_path(bag)
        identity = self._identity(bag, job)
        with self._lock(project_id):
            initial = self._document(project_id)
            revision = initial['revision'] if expected_revision is None else _revision(expected_revision)
            if revision != initial['revision']:
                raise ValueError('projectが更新されています。再読み込みしてください')
            self._reject_duplicate(initial, identity)
        folder = self.folder(project_id)
        source_id = 'bag_' + uuid.uuid4().hex[:16]
        period = max(1, round(1_000_000_000 / rate_hz))
        _log(job, f'RGB抽出: {bag.name} · {topic} · {rate_hz:g} Hz · 最大{max_frames}枚 · {split}')
        with tempfile.TemporaryDirectory(prefix='.extract-', dir=folder) as temp:
            batch = Path(temp) / source_id
            batch.mkdir()
            frames, next_stamp = [], None
            for record_stamp, message_type, message in _bag_messages(bag, topic, period, job):
                _check(job)
                if next_stamp is not None and record_stamp < next_stamp:
                    continue
                next_stamp = record_stamp + period
                image = decode_image(message, message_type)
                frame_id = f'{source_id}_{len(frames):05d}'
                filename = frame_id + '.jpg'
                image.save(batch / filename, format='JPEG', quality=95, subsampling=0)
                stamp = message.header.stamp
                header_stamp = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                frames.append(dict(id=frame_id, revision=1, width=image.width, height=image.height,
                                   bag=str(bag), source_id=source_id, stamp_ns=str(header_stamp),
                                   record_stamp_ns=str(record_stamp), frame_id=str(message.header.frame_id)[:256],
                                   split=split, reviewed=False, boxes=[], suggestions=[],
                                   image=f'images/{source_id}/{filename}'))
                if len(frames) % 10 == 0:
                    _log(job, f'RGB抽出: {len(frames)}枚')
                if len(frames) >= max_frames:
                    break
            if not frames:
                raise ValueError('指定topicから画像を1枚も抽出できませんでした')
            if self._identity(bag, job) != identity:
                raise ValueError('抽出中にbagが変更されました。実行し直してください')
            source = dict(identity, id=source_id, bag=str(bag), topic=topic, split=split,
                          sampling_clock='bag_record', rate_hz=float(rate_hz), frames_count=len(frames))
            with self._lock(project_id):
                document = self._document(project_id)
                if document['revision'] != revision:
                    raise ValueError('抽出中にprojectが更新されました。実行し直してください')
                self._reject_duplicate(document, source)
                if len(document['frames']) + len(frames) > MAX_PROJECT_FRAMES:
                    raise ValueError(f'project全体の{MAX_PROJECT_FRAMES}枚上限を超えます')
                target = within(folder, 'images/' + source_id, exists=False)
                target.parent.mkdir(exist_ok=True)
                _check(job)
                batch.rename(target)
                try:
                    document['frames'].extend(frames)
                    document['sources'].append(source)
                    document['revision'] += 1
                    atomic_json(folder / 'project.json', document)
                except BaseException:
                    # SIGINT can interrupt the worker too. Remove staged image
                    # publication only if the atomic metadata commit did not
                    # finish; never delete images referenced by committed JSON.
                    committed = read_json(folder / 'project.json')
                    if not any(item['id'] == source_id for item in committed['sources']):
                        shutil.rmtree(target)
                    raise
        _log(job, f'RGB抽出完了: {len(frames)}枚。bboxを確認して保存してください')
        return self._public(document)

    def snapshot(self, project_id, expected_revision, destination, job=None):
        with self._lock(project_id):
            document = self._document(project_id)
            if _revision(expected_revision) != document['revision']:
                raise ValueError('annotationが更新されています。再読み込みしてください')
        selected = [frame for frame in document['frames'] if frame['reviewed']]
        sources = {source['id']: source for source in document['sources']}
        assignments, counts = {}, {split: dict(frames=0, positive=0, negative=0) for split in ('train', 'val')}
        for frame in selected:
            _dimensions(frame['width'], frame['height'])
            frame['boxes'] = _boxes(frame['boxes'], frame['width'], frame['height'])
            split = frame['split']
            source = sources[frame['source_id']]
            if split not in counts or source['split'] != split:
                raise ValueError('bag単位のtrain/val割当てが不正です')
            for identity in (source['resolved_path'], source['content_fingerprint']):
                if identity in assignments and assignments[identity] != split:
                    raise ValueError('同じbagをtrainとvalの両方には使えません')
                assignments[identity] = split
            counts[split]['frames'] += 1
            counts[split]['positive' if frame['boxes'] else 'negative'] += 1
        if any(counts[split]['positive'] < 1 for split in counts):
            raise ValueError('独立したbagのtrain・valに確認済みpositive画像がそれぞれ1枚以上必要です')
        destination = Path(destination).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination = within(destination.parent, destination.name, exists=False)
        if destination.exists():
            raise ValueError('学習snapshotの出力先が既に存在します')
        used_sources = {frame['source_id'] for frame in selected}
        metadata = dict(schema='kart.perception.dataset.v1', project_id=project_id,
                        annotation_revision=document['revision'], class_names=list(CLASS_NAMES),
                        created=time.time(), counts=counts, frames=copy.deepcopy(selected),
                        sources=[source for source in document['sources'] if source['id'] in used_sources])
        metadata['split_bag_ids'] = {
            split: sorted({frame['source_id'] for frame in selected if frame['split'] == split})
            for split in counts}
        for frame in metadata['frames']:
            frame['source_image'] = frame['image']
            frame['image'] = f"images/{frame['split']}/{frame['id']}.jpg"
        annotation = dict(revision=document['revision'], frames=selected, class_names=CLASS_NAMES)
        metadata['annotation_sha256'] = hashlib.sha256(
            json.dumps(annotation, sort_keys=True, allow_nan=False).encode()).hexdigest()
        with tempfile.TemporaryDirectory(prefix='.snapshot-', dir=destination.parent) as temp:
            staging = Path(temp) / 'dataset'
            for split in counts:
                (staging / 'images' / split).mkdir(parents=True)
                (staging / 'labels' / split).mkdir(parents=True)
            for frame in selected:
                _check(job)
                source_image = within(self.folder(project_id), frame['image'])
                split, frame_id = frame['split'], name(frame['id'])
                shutil.copyfile(source_image, staging / 'images' / split / (frame_id + '.jpg'))
                width, height = frame['width'], frame['height']
                labels = []
                for x1, y1, x2, y2 in frame['boxes']:
                    values = ((x1+x2)/(2*width), (y1+y2)/(2*height), (x2-x1)/width, (y2-y1)/height)
                    labels.append('0 ' + ' '.join(f'{value:.10g}' for value in values))
                (staging / 'labels' / split / (frame_id + '.txt')).write_text('\n'.join(labels) + ('\n' if labels else ''))
            yaml = ('path: ' + json.dumps(str(destination)) + '\ntrain: images/train\nval: images/val\nnc: 1\n'
                    'names:\n  0: rc_car\n')
            (staging / 'data.yaml').write_text(yaml)
            atomic_json(staging / 'snapshot.json', metadata)
            with self._lock(project_id):
                if self._document(project_id)['revision'] != expected_revision:
                    raise ValueError('snapshot生成中にannotationが更新されました。実行し直してください')
                _check(job)
                staging.rename(destination)
        return dict(dataset_yaml=destination / 'data.yaml', snapshot_json=destination / 'snapshot.json',
                    metadata=metadata, frames_count=len(selected), annotation_revision=document['revision'])
