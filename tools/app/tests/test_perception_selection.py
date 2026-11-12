"""Annotation-selection contracts with injected YOLO/DINO inference boundaries."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from kart_studio import perception_selection as selection


def tensor(values):
    return SimpleNamespace(cpu=lambda: SimpleNamespace(tolist=lambda: values))


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.weights = self.root / 'local.pt'
        self.weights.write_bytes(b'local checkpoint; encoder/model boundary injected')
        self.addCleanup(patch.stopall)
        patch.object(selection.runtime, '_device', return_value='cpu').start()

    def frame(self, identity, *, source='bag-a', stamp=0, split='train', reviewed=False,
              boxes=None, colour=(20, 30, 40), red_box=False, size=(64, 48)):
        from PIL import Image, ImageDraw
        path = self.root / (identity + '.png')
        with Image.new('RGB', size, colour) as image:
            if red_box:
                ImageDraw.Draw(image).rectangle((0, 0, 19, 15), fill=(255, 0, 0))
            image.save(path)
        return dict(id=identity, path=str(path), width=size[0], height=size[1], source_id=source,
                    stamp_ns=str(stamp), split=split, reviewed=reviewed, boxes=boxes or [])

    def plan(self, frames, **kwargs):
        return dict(action='select', output=str(self.root / 'output'), weights=str(self.weights),
                    encoder_weights=str(self.weights), encoder_repo=str(self.root / 'local-dinov3'),
                    device='cpu', frames=frames, **kwargs)

    def model(self, confidence_by_name, classes=80, boxes_by_name=None, shapes_by_name=None):
        model = SimpleNamespace(names={i: str(i) for i in range(classes)}, task='detect')
        def predict(**kwargs):
            identity = Path(kwargs['source']).stem
            scores = confidence_by_name.get(identity, [])
            boxes = (boxes_by_name or {}).get(identity, [[1, 2, 20, 30] for _ in scores])
            return [SimpleNamespace(orig_shape=(shapes_by_name or {}).get(identity, (48, 64)),
                boxes=SimpleNamespace(xyxy=tensor(boxes), conf=tensor(scores)))]
        model.predict = Mock(side_effect=predict)
        return model

    def test_import_stays_lightweight_for_studio_server(self):
        subprocess.run([sys.executable, '-c',
            'import sys; import kart_studio.perception_selection; '
            'assert not any(m in sys.modules for m in ("torch", "ultralytics", "PIL", "numpy"))'], check=True)

    def test_balanced_budget_spans_time_and_sources_before_inference(self):
        rows = [dict(id=f'a{i}', source_id='a', stamp_ns=i * 1_000_000_000) for i in range(100)]
        rows += [dict(id=f'b{i}', source_id='b', stamp_ns=i * 1_000_000_000) for i in range(3)]
        sampled = selection.balanced_frames(rows, 8)
        self.assertEqual(len(sampled), 8)
        self.assertEqual(len({row['id'] for row in sampled}), 8)
        self.assertEqual(sum(row['source_id'] == 'b' for row in sampled), 3)
        a_times = [row['stamp_ns'] for row in sampled if row['source_id'] == 'a']
        self.assertLess(min(a_times), 20_000_000_000)
        self.assertGreater(max(a_times), 80_000_000_000)
        self.assertEqual(sampled, selection.balanced_frames(list(reversed(rows)), 8))

    def test_duplicate_timestamps_do_not_repeat_or_hang(self):
        rows = [dict(id=f'a{i:02}', source_id='a', stamp_ns=0) for i in range(15)]
        sampled = selection.balanced_frames(rows, 10)
        self.assertEqual(len({row['id'] for row in sampled}), 10)
        rows[-1]['stamp_ns'] = 100
        sampled = selection.balanced_frames(rows, 13)
        self.assertEqual(len({row['id'] for row in sampled}), 13)

    def test_record_time_bins_work_with_frozen_image_header(self):
        rows = [dict(id=f'a{i:02}', source_id='a', stamp_ns=0, record_stamp_ns=i * 1_000_000_000)
                for i in range(100)]
        sampled = selection.balanced_frames(rows, 3)
        self.assertEqual([row['id'] for row in sampled], ['a16', 'a49', 'a82'])

    def test_yolo_uses_all_classes_includes_misses_and_never_changes_labels(self):
        frames = [self.frame('positive'), self.frame('miss'),
                  self.frame('val_unreviewed', split='val'),
                  self.frame('train_done', reviewed=True, boxes=[[1, 2, 20, 30]]),
                  self.frame('val_done', split='val', reviewed=True, boxes=[[1, 2, 20, 30]])]
        original = copy.deepcopy(frames)
        model = self.model({'positive': [.8, .45]})
        with patch.object(selection.runtime, '_load_yolo', return_value=model):
            result = selection.run(self.plan(frames, limit=10))
        self.assertEqual({row['frame_id'] for row in result['ranked']}, {'positive', 'miss'})
        self.assertEqual(model.predict.call_count, 2)
        for call in model.predict.call_args_list:
            self.assertIsNone(call.kwargs['classes'])
            self.assertEqual(call.kwargs['max_det'], 100)
        self.assertEqual(result['provenance']['candidate_split'], 'train')
        self.assertFalse(result['provenance']['annotations_modified'])
        self.assertFalse(result['provenance']['score_is_probability'])
        self.assertEqual(frames, original)
        stored = json.loads((self.root / 'output/selection.json').read_text())
        self.assertEqual(stored, result)
        self.assertFalse(any(path.name in ('project.json', 'suggestions.json') for path in self.root.rglob('*.json')))

    def test_inference_decodes_only_budget_and_accepts_trained_one_class(self):
        frames = [self.frame(f'f{i}', stamp=i * 2_000_000_000) for i in range(20)]
        model = self.model({}, classes=1)
        with patch.object(selection.runtime, '_load_yolo', return_value=model):
            result = selection.run(self.plan(frames, max_images=3, limit=100))
        self.assertEqual(model.predict.call_count, 3)
        self.assertEqual(len(result['ranked']), 3)
        self.assertEqual(result['provenance']['candidates'], 20)
        self.assertEqual(result['provenance']['evaluated'], 3)

    def test_many_and_large_boxes_are_advisory_and_low_scores_do_not_trigger_flags(self):
        frames = [self.frame(name) for name in ('normal', 'many', 'large', 'below', 'weak', 'miss')]
        original = copy.deepcopy(frames)
        model = self.model({'normal': [.1] * 3, 'many': [.1] * 4, 'large': [.5],
                            'below': [.5], 'weak': [.099] * 100}, boxes_by_name={
                                'large': [[0, 0, 64 * .40, 48]],
                                'below': [[0, 0, 64 * .399, 48]],
                                'weak': [[0, 0, 64, 48]] * 100})
        with patch.object(selection.runtime, '_load_yolo', return_value=model):
            result = selection.run(self.plan(frames))
        ranked = {row['frame_id']: row for row in result['ranked']}
        self.assertEqual(ranked['normal']['review_flags'], [])
        self.assertEqual(ranked['many']['review_flags'], ['many_detections'])
        self.assertEqual(ranked['many']['review_count'], 4)
        self.assertEqual(ranked['large']['review_flags'], ['large_bbox'])
        self.assertAlmostEqual(ranked['large']['max_bbox_area_ratio'], .40)
        self.assertFalse(ranked['below']['hard_review'])
        self.assertEqual(ranked['weak']['detections'], 100)
        self.assertEqual(ranked['weak']['review_count'], 0)
        self.assertEqual(ranked['weak']['max_bbox_area_ratio'], 0)
        self.assertFalse(ranked['weak']['hard_review'])
        self.assertIn('全候補100件', ranked['weak']['reason'])
        self.assertNotIn('YOLO無検出', ranked['weak']['reason'])
        self.assertIn('誤検出の断定ではありません', ranked['many']['reason'])
        self.assertEqual(result['provenance']['hard_review_candidates'], 2)
        self.assertEqual(result['provenance']['review_confidence'], .10)
        self.assertEqual(frames, original)
        self.assertEqual(json.loads((self.root / 'output/selection.json').read_text()), result)

    def test_box_area_uses_clipped_pixels_and_is_resolution_independent(self):
        frames = [self.frame('small'), self.frame('double', size=(128, 96)),
                  self.frame('clipped'), self.frame('invalid')]
        boxes = {'small': [[0, 0, 25.6, 48]], 'double': [[0, 0, 51.2, 96]],
                 'clipped': [[-100, -100, 25.6, 1000]],
                 'invalid': [[float('nan'), 0, 20, 30], [0, 0, float('inf'), 30],
                             [0, 0, 0, 30], [30, 0, 20, 30], [-100, 0, -1, 30],
                             [1, 2, 20, 30], [1, 2, 20, 30], [1, 2, 20, 30]]}
        model = self.model({'small': [.5], 'double': [.5], 'clipped': [.5],
                            'invalid': [.9] * 5 + [float('nan'), 0., 1.1]},
                           boxes_by_name=boxes, shapes_by_name={'double': (96, 128)})
        with patch.object(selection.runtime, '_load_yolo', return_value=model):
            result = selection.run(self.plan(frames))
        ranked = {row['frame_id']: row for row in result['ranked']}
        for identity in ('small', 'double', 'clipped'):
            self.assertAlmostEqual(ranked[identity]['max_bbox_area_ratio'], .4)
            self.assertEqual(ranked[identity]['review_flags'], ['large_bbox'])
        self.assertEqual(ranked['invalid']['detections'], 0)
        self.assertEqual(ranked['invalid']['review_flags'], [])

    def test_review_thresholds_are_validated_and_custom_settings_apply(self):
        frame = self.frame('candidate')
        for change in ({'review_confidence': .049}, {'review_confidence': float('nan')},
                       {'review_confidence': True}, {'expected_max_detections': -1},
                       {'expected_max_detections': 3.5}, {'expected_max_detections': 101},
                       {'large_bbox_area_ratio': 0}, {'large_bbox_area_ratio': 1.01}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                selection.run(self.plan([frame], **change))
        self.assertFalse((self.root / 'output').exists())
        model = self.model({'candidate': [.05]})
        with patch.object(selection.runtime, '_load_yolo', return_value=model):
            result = selection.run(self.plan([frame], review_confidence=.05,
                                            expected_max_detections=0, large_bbox_area_ratio=1))
        self.assertEqual(result['ranked'][0]['review_flags'], ['many_detections'])
        self.assertEqual(result['provenance']['expected_max_detections'], 0)

    def test_many_boxes_do_not_receive_an_additive_score_bonus(self):
        frames = [self.frame('single'), self.frame('many')]
        model = self.model({'single': [.6], 'many': [.6] * 10})
        with patch.object(selection.runtime, '_load_yolo', return_value=model):
            result = selection.run(self.plan(frames))
        self.assertEqual(result['ranked'][0]['group'], 'hard_review')
        self.assertEqual(result['ranked'][0]['frame_id'], 'many')
        self.assertEqual(result['ranked'][0]['score'], result['ranked'][1]['score'])

    def test_no_train_candidates_does_not_load_model_and_cli_is_usable(self):
        frames = [self.frame('val', split='val'), self.frame('done', reviewed=True)]
        plan = self.plan(frames)
        with patch.object(selection.runtime, '_load_yolo') as loader:
            result = selection.run(plan)
        loader.assert_not_called()
        self.assertEqual(result['ranked'], [])
        plan['output'] = str(self.root / 'cli-output')
        source = self.root / 'plan.json'
        source.write_text(json.dumps(plan))
        subprocess.run([sys.executable, '-m', 'kart_studio.perception_selection', '--plan', str(source)], check=True)
        self.assertEqual(json.loads((self.root / 'cli-output/selection.json').read_text())['ranked'], [])

    def test_diversity_pushes_duplicate_below_distinct_candidate(self):
        def row(identity, score, feature, stamp):
            return dict(frame=dict(id=identity, source_id='bag', stamp_ns=stamp),
                        score=score, feature=feature, background=False, reason='candidate')
        rows = [row('best', 1, [1, 0], 0), row('duplicate', .99, [1, 0], 10),
                row('distinct', .8, [0, 1], 2_000_000_000)]
        ordered = selection.diverse_order(rows, 3)
        self.assertEqual([row['frame_id'] for row in ordered], ['best', 'distinct', 'duplicate'])
        self.assertNotIn('_redundancy', rows[0])

    def test_background_and_reproducible_exploration_slots(self):
        rows = [dict(frame=dict(id=f'f{i}', source_id='bag', stamp_ns=i * 2_000_000_000),
                     score=1 - i / 100, feature=[1, 0], background=i >= 10, reason='candidate')
                for i in range(20)]
        ranked = selection.diverse_order(rows, 12)
        self.assertEqual(ranked[4]['group'], 'background')
        self.assertGreaterEqual(int(ranked[4]['frame_id'][1:]), 10)
        self.assertEqual(ranked[9]['group'], 'exploration')
        self.assertEqual(ranked, selection.diverse_order(list(reversed(rows)), 12))
        self.assertEqual(len({row['frame_id'] for row in ranked}), 12)

    def test_hard_review_slots_keep_normal_background_and_exploration_in_the_mix(self):
        rows = [dict(frame=dict(id=f'f{i}', source_id='bag', stamp_ns=i * 2_000_000_000),
                     score=1 if i < 20 else .5, feature=[1, 0], background=i >= 40,
                     hard_review=i < 20, reason='advisory') for i in range(45)]
        ranked = selection.diverse_order(rows, 20)
        for index in (0, 2, 10, 12):
            self.assertEqual(ranked[index]['group'], 'hard_review')
            self.assertTrue(ranked[index]['hard_review'])
        for index in (4, 14):
            self.assertEqual(ranked[index]['group'], 'background')
            self.assertGreaterEqual(int(ranked[index]['frame_id'][1:]), 40)
        for index in (9, 19):
            self.assertEqual(ranked[index]['group'], 'exploration')
        for row in ranked:
            if row['group'] == 'priority':
                self.assertFalse(row['hard_review'])
        self.assertEqual(ranked, selection.diverse_order(list(reversed(rows)), 20))
        self.assertEqual(len({row['frame_id'] for row in ranked}), 20)
        # Scarce pools fall back without repeating or dropping available frames.
        only_hard = selection.diverse_order(rows[:4], 20)
        self.assertEqual(len({row['frame_id'] for row in only_hard}), 4)

    def test_similarity_and_temporal_penalties_also_apply_to_hard_review(self):
        rows = [dict(frame=dict(id=name, source_id='bag', stamp_ns=stamp), score=score,
                     feature=feature, background=False, hard_review=True, reason='advisory')
                for name, stamp, score, feature in (
                    ('best', 0, 1., [1, 0]), ('duplicate', 10, .99, [1, 0]),
                    ('distinct', 2_000_000_000, .8, [0, 1]))]
        ranked = selection.diverse_order(rows, 3)
        self.assertEqual([row['frame_id'] for row in ranked], ['best', 'distinct', 'duplicate'])

    def test_dinov3_uses_local_crops_and_only_reviewed_train_seeds(self):
        frames = [self.frame('seed_pos', reviewed=True, boxes=[[0, 0, 20, 16]], colour=(0, 0, 255), red_box=True),
                  self.frame('seed_neg', reviewed=True, colour=(0, 0, 255)),
                  self.frame('candidate_rc', colour=(0, 0, 255), red_box=True),
                  self.frame('candidate_bg', colour=(0, 0, 255)),
                  self.frame('val_seed', split='val', reviewed=True, boxes=[[0, 0, 64, 48]], colour=(255, 0, 0)),
                  self.frame('val_candidate', split='val', colour=(255, 0, 0))]
        opened = []
        actual_image = selection._image
        def image(frame):
            opened.append(frame['id'])
            return actual_image(frame)
        view_sizes = []
        def encoder(images):
            result = []
            for image in images:
                view_sizes.append(image.size)
                pixels = getattr(image, 'get_flattened_data', image.getdata)()
                red = sum(r > 200 and b < 50 for r, _, b in pixels) / (image.width * image.height)
                result.append([1, 0] if red > .25 else [0, 1])
            return result
        with patch.object(selection, '_load_encoder', return_value=(encoder, {'test_encoder': True})), \
                patch.object(selection, '_image', side_effect=image):
            result = selection.run(self.plan(frames, method='dinov3'))
        self.assertEqual([row['frame_id'] for row in result['ranked']], ['candidate_rc', 'candidate_bg'])
        self.assertGreater(result['ranked'][0]['score'], result['ranked'][1]['score'])
        self.assertNotIn('val_seed', opened)
        self.assertNotIn('val_candidate', opened)
        self.assertEqual(result['provenance']['positive_seed_frames'], ['seed_pos'])
        self.assertEqual(result['provenance']['negative_seed_frames'], ['seed_neg'])
        self.assertEqual(result['provenance']['seed_split'], 'train')
        self.assertEqual(result['provenance']['positive_seed_crops'], 1)
        self.assertEqual(result['provenance']['negative_seed_crops'], 14)
        self.assertIn((21, 16), view_sizes)
        self.assertIn('確率ではありません', result['ranked'][0]['reason'])

    def test_val_positive_cannot_satisfy_dino_seed_requirement(self):
        frames = [self.frame('negative', reviewed=True), self.frame('candidate'),
                  self.frame('val_positive', split='val', reviewed=True, boxes=[[1, 2, 20, 30]])]
        with patch.object(selection, '_load_encoder') as loader:
            with self.assertRaisesRegex(ValueError, 'trainの確認済み正例・負例'):
                selection.run(self.plan(frames, method='dinov3'))
        loader.assert_not_called()
        self.assertFalse((self.root / 'output/selection.json').exists())

    def test_dino_seed_crops_are_bounded_and_inferred_per_image(self):
        frames = [self.frame(f'pos{i}', reviewed=True, boxes=[[1, 2, 20, 30]],
                             stamp=i * 2_000_000_000) for i in range(40)]
        frames += [self.frame(f'neg{i}', reviewed=True, source='bag-b',
                              stamp=i * 2_000_000_000) for i in range(20)]
        frames.append(self.frame('candidate'))
        batch_sizes = []
        def encoder(images):
            batch_sizes.append(len(images))
            return [[1, 1] for image in images]
        with patch.object(selection, '_load_encoder', return_value=(encoder, {})):
            result = selection.run(self.plan(frames, method='dinov3'))
        self.assertEqual(result['provenance']['positive_seed_crops'], 32)
        self.assertEqual(result['provenance']['negative_seed_crops'], 32)
        self.assertEqual(len(result['provenance']['positive_seed_frames']), 32)
        self.assertEqual(len(result['provenance']['negative_seed_frames']), 16)
        self.assertLessEqual(max(batch_sizes), 14)
        self.assertEqual(sum(batch_sizes), 32 + 32 + 14)

    def test_missing_local_dino_repository_fails_without_fallback(self):
        frames = [self.frame('positive', reviewed=True, boxes=[[1, 2, 20, 30]]),
                  self.frame('negative', reviewed=True), self.frame('candidate')]
        with self.assertRaises(FileNotFoundError):
            selection.run(self.plan(frames, method='dinov3'))
        self.assertFalse((self.root / 'output/selection.json').exists())

    def test_invalid_limits_duplicate_ids_and_image_geometry_fail(self):
        frame = self.frame('candidate')
        for change in ({'limit': 501}, {'max_images': 501}, {'max_images': True}, {'method': 'remote'}):
            with self.assertRaises(ValueError):
                selection.run(self.plan([frame], **change))
        with self.assertRaisesRegex(ValueError, '画像ID'):
            selection.run(self.plan([frame, frame]))
        frame['width'] = 63
        model = self.model({})
        with patch.object(selection.runtime, '_load_yolo', return_value=model):
            with self.assertRaisesRegex(ValueError, 'サイズmetadata'):
                selection.run(self.plan([frame]))
        self.assertFalse((self.root / 'output/selection.json').exists())

    def test_output_is_not_overwritten_and_changed_weights_fail(self):
        frame = self.frame('candidate')
        model = self.model({})
        with patch.object(selection.runtime, '_load_yolo', return_value=model):
            selection.run(self.plan([frame]))
            with self.assertRaisesRegex(ValueError, '上書き'):
                selection.run(self.plan([frame]))
        plan = self.plan([frame])
        plan['output'] = str(self.root / 'changed')
        def change_weights(*args):
            self.weights.write_bytes(b'changed during inference')
            return [], {}
        with patch.object(selection, '_yolo_rows', side_effect=change_weights):
            with self.assertRaisesRegex(ValueError, '重みファイルが変更'):
                selection.run(plan)
        self.assertFalse((self.root / 'changed/selection.json').exists())

    def test_bad_feature_vectors_fail_instead_of_producing_nan_ranks(self):
        for value in ([], [0, 0], [float('nan'), 1]):
            with self.assertRaises(ValueError):
                selection._unit(value)
        with self.assertRaises(ValueError):
            selection._dot([1, 0], [1])


if __name__ == '__main__':
    unittest.main()
