"""Annotation transactions, bag separation and real ROS2 image extraction."""
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from kart_studio.perception_dataset import DatasetStore, decode_image
from kart_studio.storage import atomic_json


HAS_PIL = importlib.util.find_spec('PIL') is not None
HAS_ROSBAGS = importlib.util.find_spec('rosbags') is not None
HEADER_STAMP = 1_234_567_890_123_456_789


def raw_image(encoding='rgb8', width=100, height=80, stamp=HEADER_STAMP):
    channels = 1 if encoding == 'mono8' else 3
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return NS(width=width, height=height, encoding=encoding, step=width*channels,
              data=bytes([42] * width*height*channels),
              header=NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id='rgb_optical'))


class Job:
    def __init__(self, fail_after=None):
        self.checks, self.fail_after, self.messages = 0, fail_after, []

    def check(self):
        self.checks += 1
        if self.fail_after is not None and self.checks > self.fail_after:
            raise RuntimeError('cancelled test job')

    def log(self, message):
        self.messages.append(message)


@unittest.skipUnless(HAS_PIL, 'Pillow is required for image dataset tests')
class DatasetStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.records = self.base / 'record'
        self.records.mkdir()
        self.store = DatasetStore(self.base, self.records)
        self.store.create('course', 'RCカー教師')

    def bag(self, key, payload=None):
        path = self.records / key
        path.mkdir()
        (path / 'metadata.yaml').write_text('same metadata is not proof of the same recording')
        (path / 'data.db3').write_bytes(key.encode() if payload is None else payload)
        return path

    def extract(self, bag, split='train', rows=None, **kwargs):
        if rows is None:
            rows = [(1_000_000_000+t, 'sensor_msgs/msg/Image', raw_image(stamp=HEADER_STAMP+i))
                    for i, t in enumerate((0, 100_000_000, 500_000_000, 1_000_000_000))]
        with patch('kart_studio.perception_dataset._bag_messages', return_value=iter(rows)):
            return self.store.extract('course', bag, '/camera/rgb', split=split, **kwargs)

    def save(self, project, frame, boxes, reviewed=True, class_ids=None):
        return self.store.save_frame('course', frame['id'], project['revision'], frame['revision'], boxes, reviewed,
                                     class_ids=class_ids)

    def test_extract_records_exact_clocks_samples_by_record_time_and_starts_unreviewed(self):
        job = Job()
        project = self.extract(self.bag('train'), job=job)
        self.assertEqual(project['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(project['revision'], 2)
        self.assertEqual(len(project['frames']), 3)
        self.assertEqual([f['record_stamp_ns'] for f in project['frames']],
                         ['1000000000', '1500000000', '2000000000'])
        self.assertEqual(project['frames'][1]['stamp_ns'], str(HEADER_STAMP+2))
        self.assertTrue(all(not f['reviewed'] and f['boxes'] == [] and f['suggestions'] == []
                            for f in project['frames']))
        self.assertNotIn('image', project['frames'][0])
        self.assertTrue(self.store.image_path('course', project['frames'][0]['id']).is_file())
        self.assertGreater(job.checks, 0)
        self.assertTrue(job.messages)
        summary = self.store.catalog()[0]
        self.assertEqual((summary['frames_count'], summary['reviewed_count'], summary['negative_count']), (3, 0, 0))

    def test_frame_revision_and_project_revision_reject_lost_updates(self):
        project = self.extract(self.bag('train'))
        original, frame = copy.deepcopy(project), project['frames'][0]
        project = self.save(project, frame, [[0, 0, 100, 80]])
        self.assertEqual(project['frames'][0]['revision'], 2)
        for project_revision, frame_revision in ((original['revision'], 1), (project['revision'], 1)):
            with self.subTest(project=project_revision, frame=frame_revision), self.assertRaises(ValueError):
                self.store.save_frame('course', frame['id'], project_revision, frame_revision, [])
        self.assertEqual(self.store.load('course'), project)

    def test_invalid_boxes_and_flags_never_modify_saved_annotations(self):
        project = self.extract(self.bag('train'))
        frame = project['frames'][0]
        bad = [[[math.nan, 0, 10, 10]], [[0, 0, math.inf, 10]], [[-1, 0, 10, 10]],
               [[0, 0, 101, 10]], [[0, 0, 10, 81]], [[10, 0, 10, 10]],
               [[10, 0, 5, 10]], [[True, 0, 10, 10]], [[0, 0, 1]], [[0, 0, 1, 1]]*101]
        for boxes in bad:
            with self.subTest(boxes=repr(boxes)[:80]), self.assertRaises(ValueError):
                self.save(project, frame, boxes)
        with self.assertRaises(ValueError):
            self.save(project, frame, [], reviewed='true')
        for ids in ([], [2], [-1], [True], [1.], [0, 1], '1'):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                self.save(project, frame, [[1, 1, 20, 20]], class_ids=ids)
        self.assertEqual(self.store.load('course'), project)

    def test_legacy_rc_geometry_survives_but_both_old_positive_and_negative_need_review(self):
        project = self.extract(self.bag('train'))
        project = self.save(project, project['frames'][0], [[10, 20, 30, 60]])
        project = self.save(project, project['frames'][1], [])
        path = self.store.folder('course') / 'project.json'
        legacy = json.loads(path.read_text())
        legacy['class_names'] = ['rc_car']
        for frame in legacy['frames']:
            frame.pop('class_ids', None)
        atomic_json(path, legacy)
        before = path.read_bytes()
        upgraded = self.store.load('course')
        self.assertEqual(path.read_bytes(), before, 'read-only migration must not overwrite old data')
        self.assertTrue(upgraded['class_schema_upgraded'])
        self.assertEqual(upgraded['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(upgraded['frames'][0]['boxes'], [[10, 20, 30, 60]])
        self.assertEqual(upgraded['frames'][0]['class_ids'], [0])
        self.assertTrue(all(not f['reviewed'] and f['class_review_required'] for f in upgraded['frames']))
        updated = self.save(upgraded, upgraded['frames'][0], [[10, 20, 30, 60], [50, 20, 70, 60]],
                            class_ids=[0, 1])
        self.assertEqual(updated, self.store.load('course'))
        self.assertEqual(updated['frames'][0]['class_ids'], [0, 1])
        self.assertTrue(updated['frames'][0]['reviewed'])
        self.assertFalse(updated['frames'][1]['reviewed'])
        self.assertFalse(updated['frames'][0]['class_review_required'])

    def test_suggestions_never_mark_images_reviewed_or_replace_confirmed_boxes(self):
        project = self.extract(self.bag('train'))
        project = self.save(project, project['frames'][0], [[10, 10, 30, 40]])
        revision = project['revision']
        proposed = {project['frames'][0]['id']: [[20, 20, 50, 50]],
                    project['frames'][1]['id']: [[10, 10, 20, 20]]}
        updated = self.store.apply_suggestions('course', revision, proposed)
        self.assertEqual(updated['frames'][0]['boxes'], [[10., 10., 30., 40.]])
        self.assertTrue(updated['frames'][0]['reviewed'])
        self.assertFalse(updated['frames'][1]['reviewed'])
        self.assertEqual(updated['frames'][1]['boxes'], [])
        with self.assertRaises(ValueError):
            self.store.apply_suggestions('course', revision, proposed)
        with self.assertRaises(ValueError):
            self.store.apply_suggestions('course', updated['revision'], {**proposed, 'missing': []})
        self.assertEqual(self.store.load('course'), updated)

    def test_duplicate_bags_including_copies_cannot_cross_train_validation(self):
        bag = self.bag('train')
        project = self.extract(bag)
        with self.assertRaisesRegex(ValueError, '重複'):
            self.extract(bag, 'val')
        copied = self.records / 'renamed-copy'
        shutil.copytree(bag, copied)
        with self.assertRaisesRegex(ValueError, '重複'):
            self.extract(copied, 'val')
        # Independent payloads with identical metadata remain usable.
        result = self.extract(self.bag('independent'), 'val')
        self.assertEqual(len(result['frames']), 2*len(project['frames']))

    def test_snapshot_uses_only_reviewed_images_and_keeps_confirmed_negatives(self):
        self.extract(self.bag('train'))
        project = self.extract(self.bag('validation'), 'val')
        train = next(frame for frame in project['frames'] if frame['split'] == 'train')
        project = self.save(project, train, [[10, 20, 30, 60], [50, 20, 70, 60]], class_ids=[0, 1])
        val = next(frame for frame in project['frames'] if frame['split'] == 'val')
        project = self.save(project, val, [[0, 0, 40, 80], [50, 0, 100, 80]], class_ids=[0, 1])
        negative = next(frame for frame in project['frames'] if not frame['reviewed'] and frame['split'] == 'train')
        project = self.save(project, negative, [])
        draft = next(frame for frame in project['frames'] if not frame['reviewed'])
        project = self.save(project, draft, [[1, 1, 20, 20]], reviewed=False)
        result = self.store.snapshot('course', project['revision'], self.base / 'snapshot')
        metadata = result['metadata']
        self.assertEqual(result['frames_count'], 3)
        self.assertEqual(metadata['annotation_revision'], project['revision'])
        self.assertEqual(metadata['schema'], 'kart.perception.dataset.v2')
        self.assertEqual(metadata['class_names'], ['rc_car', 'obstacle'])
        self.assertEqual(metadata['counts']['train'], dict(frames=2, positive=1, negative=1,
                                                         class_boxes=dict(rc_car=1, obstacle=1)))
        self.assertEqual(metadata['counts']['val'], dict(frames=1, positive=1, negative=0,
                                                       class_boxes=dict(rc_car=1, obstacle=1)))
        self.assertTrue(set(metadata['split_bag_ids']['train']).isdisjoint(metadata['split_bag_ids']['val']))
        self.assertEqual((self.base / f"snapshot/labels/train/{train['id']}.txt").read_text(),
                         '0 0.2 0.5 0.2 0.5\n1 0.6 0.5 0.2 0.5\n')
        self.assertIn('nc: 2\nnames:\n  0: rc_car\n  1: obstacle\n', result['dataset_yaml'].read_text())
        self.assertEqual((self.base / f"snapshot/labels/train/{negative['id']}.txt").read_text(), '')
        self.assertEqual(len(list((self.base / 'snapshot/images').rglob('*.jpg'))), 3)
        self.assertTrue(all((self.base / 'snapshot' / frame['image']).is_file() for frame in metadata['frames']))
        before = result['snapshot_json'].read_bytes()
        project = self.save(project, project['frames'][0], [])
        self.assertEqual(result['snapshot_json'].read_bytes(), before)
        with self.assertRaises(ValueError):
            self.store.snapshot('course', project['revision'], self.base / 'snapshot')
        summary = self.store.catalog()[0]
        self.assertEqual(summary['reviewed_count'], 3)

    def test_snapshot_requires_positive_reviewed_images_on_both_independent_sides(self):
        project = self.extract(self.bag('train'))
        project = self.save(project, project['frames'][0], [[1, 1, 20, 20]])
        with self.assertRaisesRegex(ValueError, 'positive'):
            self.store.snapshot('course', project['revision'], self.base / 'bad-snapshot')
        project = self.extract(self.bag('validation'), 'val')
        val = next(frame for frame in project['frames'] if frame['split'] == 'val')
        project = self.save(project, val, [])
        with self.assertRaisesRegex(ValueError, 'positive'):
            self.store.snapshot('course', project['revision'], self.base / 'bad-snapshot')
        self.assertFalse((self.base / 'bad-snapshot').exists())

    def test_failed_cancelled_empty_or_concurrent_extraction_does_not_publish_partial_frames(self):
        bag = self.bag('train')
        initial = self.store.load('course')
        broken = raw_image(encoding='mono16')
        rows = [(1_000_000_000, 'sensor_msgs/msg/Image', raw_image()),
                (2_000_000_000, 'sensor_msgs/msg/Image', broken)]
        with self.assertRaises(ValueError):
            self.extract(bag, rows=rows)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            self.extract(bag, job=Job(fail_after=3))
        with self.assertRaises(ValueError):
            self.extract(bag, rows=[])
        self.assertEqual(self.store.load('course'), initial)
        self.assertFalse(list(self.store.folder('course').glob('.extract-*')))
        project = self.extract(bag)
        other = self.bag('validation')

        def changing(*_args):
            self.save(project, project['frames'][0], [])
            yield 3_000_000_000, 'sensor_msgs/msg/Image', raw_image()

        with patch('kart_studio.perception_dataset._bag_messages', side_effect=changing):
            with self.assertRaisesRegex(ValueError, '更新'):
                self.store.extract('course', other, '/camera/rgb', split='val', expected_revision=project['revision'])
        self.assertEqual(len(self.store.load('course')['frames']), len(project['frames']))
        self.assertEqual(len(list((self.store.folder('course') / 'images').iterdir())), 1)

    def test_bag_change_or_failed_metadata_commit_rolls_back_staged_images(self):
        bag = self.bag('train')
        before = self.store.load('course')

        def changed(*_args):
            yield 1_000_000_000, 'sensor_msgs/msg/Image', raw_image()
            (bag / 'data.db3').write_bytes(b'changed while extracting')

        with patch('kart_studio.perception_dataset._bag_messages', side_effect=changed):
            with self.assertRaisesRegex(ValueError, 'bagが変更'):
                self.store.extract('course', bag, '/camera/rgb')
        with patch('kart_studio.perception_dataset.atomic_json', side_effect=OSError('write failed')):
            with self.assertRaises(OSError):
                self.extract(bag)
        with patch('kart_studio.perception_dataset.atomic_json', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                self.extract(bag)
        self.assertEqual(self.store.load('course'), before)
        self.assertFalse(list((self.store.folder('course') / 'images').iterdir()))

    def test_annotation_change_during_snapshot_does_not_publish_an_outdated_dataset(self):
        self.extract(self.bag('train'))
        project = self.extract(self.bag('validation'), 'val')
        for split in ('train', 'val'):
            frame = next(frame for frame in project['frames'] if frame['split'] == split)
            project = self.save(project, frame, [[1, 1, 20, 20], [30, 30, 60, 60]], class_ids=[0, 1])
        copy_image = shutil.copyfile
        changed = False

        def edit_during_copy(*args):
            nonlocal changed
            result = copy_image(*args)
            if not changed:
                changed = True
                self.save(project, project['frames'][0], [])
            return result

        destination = self.base / 'outdated-snapshot'
        with patch('kart_studio.perception_dataset.shutil.copyfile', side_effect=edit_during_copy):
            with self.assertRaisesRegex(ValueError, 'snapshot生成中'):
                self.store.snapshot('course', project['revision'], destination)
        self.assertFalse(destination.exists())
        self.assertFalse(list(self.base.glob('.snapshot-*')))

    def test_paths_and_image_files_are_confined(self):
        project = self.extract(self.bag('train'))
        for value in ('../outside', '/tmp/outside'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.store.create(value)
        with self.assertRaises(ValueError):
            self.store.extract('course', self.base, '/camera/rgb')
        frame = project['frames'][0]
        image = self.store.image_path('course', frame['id'])
        original = image.read_bytes()
        image.unlink()
        outside = self.base / 'outside.jpg'
        outside.write_bytes(original)
        image.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, 'シンボリック'):
            self.store.image_path('course', frame['id'])


@unittest.skipUnless(HAS_PIL, 'Pillow is required for image decoding tests')
class ImageDecodeTests(unittest.TestCase):
    def test_rgb_bgr_padding_and_mono_are_decoded_without_resize(self):
        for encoding, data, expected in (('rgb8', bytes([1, 2, 3, 0]), (1, 2, 3)),
                                         ('bgr8', bytes([1, 2, 3, 0]), (3, 2, 1)),
                                         ('mono8', bytes([42, 0, 0, 0]), (42, 42, 42))):
            with self.subTest(encoding=encoding):
                msg = raw_image(encoding, width=1, height=1)
                msg.step, msg.data = 4, data
                image = decode_image(msg, 'sensor_msgs/msg/Image')
                self.assertEqual(image.size, (1, 1))
                self.assertEqual(image.getpixel((0, 0)), expected)

    def test_jpeg_png_and_corruption_or_invalid_raw_data(self):
        from PIL import Image
        for kind in ('JPEG', 'PNG'):
            data = io.BytesIO()
            Image.new('RGB', (10, 20), 'red').save(data, format=kind)
            self.assertEqual(decode_image(NS(data=data.getvalue()), 'sensor_msgs/msg/CompressedImage').size, (10, 20))
        msg = raw_image()
        msg.step = 1
        with self.assertRaises(ValueError):
            decode_image(msg, 'sensor_msgs/msg/Image')
        with self.assertRaises(ValueError):
            decode_image(NS(data=b'not an image'), 'sensor_msgs/msg/CompressedImage')


@unittest.skipUnless(HAS_PIL and HAS_ROSBAGS, 'Pillow and rosbags are required for real ROS2 bag tests')
class RealBagTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.records = self.base / 'record'
        self.records.mkdir()
        self.store = DatasetStore(self.base, self.records)
        self.store.create('real-bag')

    def write_bag(self, key, compressed=False, count=4):
        import numpy as np
        from PIL import Image
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
        store = get_typestore(Stores.LATEST)
        types = store.types
        kind = 'sensor_msgs/msg/CompressedImage' if compressed else 'sensor_msgs/msg/Image'
        bag = self.records / key
        with Writer(bag, version=9) as writer:
            connection = writer.add_connection('/rgb', kind, typestore=store)
            for index in range(count):
                sec, nanosec = divmod(HEADER_STAMP+index, 1_000_000_000)
                header = types['std_msgs/msg/Header'](
                    stamp=types['builtin_interfaces/msg/Time'](sec=sec, nanosec=nanosec), frame_id='rgb_optical')
                if compressed:
                    data = io.BytesIO()
                    Image.new('RGB', (10, 20), 'red').save(data, format='PNG')
                    message = types[kind](header=header, format='png', data=np.frombuffer(data.getvalue(), dtype=np.uint8))
                else:
                    message = types[kind](header=header, width=10, height=20, encoding='rgb8', is_bigendian=0,
                                          step=30, data=np.full(600, index, dtype=np.uint8))
                writer.write(connection, 2_000_000_000+index*250_000_000, store.serialize_cdr(message, kind))
        return bag

    def test_real_cdr_image_and_compressed_image_extract_without_ros(self):
        for key, compressed, split in (('raw', False, 'train'), ('png', True, 'val')):
            with self.subTest(kind=key):
                bag = self.write_bag(key, compressed)
                project = self.store.extract('real-bag', bag, '/rgb', split=split)
                frames = [frame for frame in project['frames'] if frame['split'] == split]
                self.assertEqual(len(frames), 2)
                self.assertEqual(frames[1]['stamp_ns'], str(HEADER_STAMP+2))
                self.assertEqual(frames[1]['record_stamp_ns'], '2500000000')
                self.assertEqual((frames[0]['width'], frames[0]['height']), (10, 20))

    def test_missing_topic_and_zero_selected_frames_leave_project_unchanged(self):
        bag = self.write_bag('raw')
        before = self.store.load('real-bag')
        with self.assertRaisesRegex(ValueError, 'topic'):
            self.store.extract('real-bag', bag, '/absent')
        empty = self.write_bag('empty', count=0)
        with self.assertRaisesRegex(ValueError, 'topic|1枚'):
            self.store.extract('real-bag', empty, '/rgb')
        self.assertEqual(self.store.load('real-bag'), before)


if __name__ == '__main__':
    unittest.main()
