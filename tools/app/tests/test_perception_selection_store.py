"""Priorities are advisory: never change labels or expire after every review."""
import copy
import tempfile
from pathlib import Path
import unittest

from kart_studio.perception_dataset import DatasetStore
from kart_studio.perception_selection_store import SelectionStore, image_signature
from kart_studio.storage import atomic_json


class SelectionStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.datasets = DatasetStore(Path(self.temp.name))
        self.project = self.datasets.create('course')
        self.project['sources'] = [dict(id='bag-a', content_fingerprint='hash-a', topic='/rgb', split='train'),
                                   dict(id='bag-b', content_fingerprint='hash-b', topic='/rgb', split='val')]
        self.project['frames'] = [dict(id='frame-a', source_id='bag-a', width=100, height=80,
            stamp_ns='1000', record_stamp_ns='2000', split='train', reviewed=False,
            boxes=[], suggestions=[], revision=1, image='images/a.jpg'),
            dict(id='frame-val', source_id='bag-b', width=100, height=80, stamp_ns='3000',
                 split='val', reviewed=False, boxes=[], suggestions=[], revision=1, image='images/b.jpg')]
        atomic_json(self.datasets.folder('course') / 'project.json', self.project)
        self.selections = SelectionStore(self.datasets)
        self.result = dict(method='yolo', ranked=[dict(frame_id='frame-a', rank=1, score=.4, reason='候補＋多様性')])

    def test_uncreated_selection_is_empty_and_is_not_a_missing_data_error(self):
        self.assertEqual(self.selections.get('course'), dict(ranked=[], stale=False, annotations_changed=False))

    def test_publishing_priority_never_creates_labels_or_marks_images_reviewed(self):
        before = self.datasets.load('course')
        saved = self.selections.publish('course', 1, image_signature(before), self.result)
        self.assertEqual(self.datasets.load('course'), before)
        self.assertFalse(saved['automatic_ground_truth'])
        self.assertFalse(self.selections.get('course')['stale'])

    def test_advisory_detection_counts_area_and_review_flags_survive_storage_reload(self):
        result = copy.deepcopy(self.result)
        result['ranked'][0].update(detections=8, review_count=5, max_bbox_area_ratio=.72,
                                  review_flags=['many_detections', 'large_bbox'], hard_review=True)
        before = self.datasets.load('course')
        self.selections.publish('course', 1, image_signature(before), result)
        restored = SelectionStore(DatasetStore(Path(self.temp.name))).get('course')
        self.assertEqual(restored['ranked'], result['ranked'])
        row = restored['ranked'][0]
        self.assertIs(type(row['detections']), int)
        self.assertIs(type(row['review_count']), int)
        self.assertEqual(row['max_bbox_area_ratio'], .72)
        self.assertIs(row['hard_review'], True)
        self.assertFalse(restored['automatic_ground_truth'])
        self.assertEqual(self.datasets.load('course'), before,
                         'hard-review flags remain advice and never become human labels')

    def test_human_review_keeps_priority_usable_but_new_images_invalidate_it(self):
        self.selections.publish('course', 1, image_signature(self.project), self.result)
        project = self.datasets.save_frame('course', 'frame-a', 1, 1, [[10, 10, 50, 50]], True)
        current = self.selections.get('course')
        self.assertFalse(current['stale'])
        self.assertTrue(current['annotations_changed'])
        project['frames'].append(dict(project['frames'][0], id='frame-new', reviewed=False, boxes=[], class_ids=[]))
        atomic_json(self.datasets.folder('course') / 'project.json', project)
        self.assertTrue(self.selections.get('course')['stale'])

    def test_class_contract_change_expires_old_selection_even_when_images_are_unchanged(self):
        legacy = dict(self.project, class_names=['rc_car'])
        self.assertNotEqual(image_signature(legacy), image_signature(self.project))

    def test_val_reviewed_duplicates_nan_and_conflicts_cannot_publish(self):
        for change in ('val', 'duplicate', 'nan', 'conflict'):
            result = copy.deepcopy(self.result)
            if change == 'val': result['ranked'][0]['frame_id'] = 'frame-val'
            if change == 'duplicate': result['ranked'].append(dict(result['ranked'][0], rank=2))
            if change == 'nan': result['ranked'][0]['score'] = float('nan')
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.selections.publish('course', 0 if change == 'conflict' else 1, image_signature(self.project), result)
        self.assertFalse((self.datasets.folder('course') / 'selection.json').exists())


if __name__ == '__main__':
    unittest.main()
