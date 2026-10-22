"""The optional region centerline must not change default generation or boundaries."""

import copy
import math
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_studio.geometry import centerline, edges
from kart_studio.lines import generate
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json


def ring():
    circle = lambda r, count: [[r * math.cos(i * math.tau / count), r * math.sin(i * math.tau / count)] for i in range(count)]
    return dict(left=circle(3, 30), right=circle(5, 50), closed=True)


def traced_course():
    # Approximate image coordinates, scaled for this synthetic fixture only.
    # This is not the user's map, calibrated scale, or physical vehicle test.
    inner = [[596,391],[633,357],[793,357],[824,369],[802,378],[670,378],[609,420],
             [597,512],[656,570],[820,584],[849,605],[835,622],[627,617],[583,570]]
    outer = [[518,318],[617,262],[896,272],[969,334],[966,402],[878,471],[735,468],
             [720,476],[731,486],[881,494],[962,561],[959,653],[875,709],[610,706],[519,631]]
    return dict(left=[[x/100, -y/100] for x,y in inner],
                right=[[x/100, -y/100] for x,y in outer], closed=True)


@unittest.skipUnless(importlib.util.find_spec("numpy"), "region method requires NumPy")
class RegionCenterlineTests(unittest.TestCase):
    def test_default_is_exactly_the_existing_method(self):
        doc = ring()
        expected, _, _ = centerline(doc['left'], doc['right'], True, .1)
        with patch('kart_studio.region_centerline.extract', side_effect=AssertionError('unexpected')):
            result = generate(doc, 'centerline', {})
        self.assertEqual(result['method'], 'paired_boundary_midpoints')
        self.assertEqual(result['points'], expected)

    def test_ring_unequal_counts_direction_and_seam(self):
        doc = ring()
        for swap, reverse in ((False, False), (False, True), (True, False)):
            with self.subTest(swap=swap, reverse=reverse):
                other = copy.deepcopy(doc)
                if reverse:
                    other['right'] = list(reversed(other['right'][7:] + other['right'][:7]))
                if swap:
                    other['left'], other['right'] = other['right'], other['left']
                original = copy.deepcopy(other)
                out = generate(other, 'centerline', dict(centerline_method='region'))
                self.assertEqual(other, original)
                self.assertEqual(out['method'], 'region_equal_distance_v1')
                self.assertLess(out['region_info']['max_distance_imbalance_m'], .01)
                self.assertTrue(all(3.95 < math.hypot(*p) < 4.02 for p in out['points']))
                area = lambda ps: sum(a[0]*b[1]-a[1]*b[0] for a,b in edges(ps, True))
                self.assertGreater(area(out['points']) * area(other['left']), 0)

    def test_concave_traced_course_improves_equidistance(self):
        doc = traced_course()
        _, old_widths, _ = centerline(doc['left'], doc['right'], True, .1)
        old_bias = max(abs(a-b)/(a+b) for a,b in old_widths)
        out = generate(doc, 'centerline', dict(centerline_method='region'))
        new_bias = max(abs(a-b)/(a+b) for a,b in out['widths'])
        self.assertGreater(old_bias, .5)
        self.assertLess(new_bias, .02)
        self.assertLess(new_bias, old_bias / 10)
        self.assertTrue(all(min(widths) > .145 for widths in out['widths']))

    def test_open_memory_budget_and_unknown_method_rejected_without_fallback(self):
        doc = ring()
        with self.assertRaisesRegex(ValueError, '閉路専用'):
            generate(dict(doc, closed=False), 'centerline', dict(centerline_method='region'))
        with patch('kart_studio.region_centerline.MAX_CELLS', 10):
            with self.assertRaisesRegex(ValueError, '大きすぎ'):
                generate(doc, 'centerline', dict(centerline_method='region'))
        with self.assertRaisesRegex(ValueError, '未知'):
            generate(doc, 'centerline', dict(centerline_method='unknown'))
        with patch.dict('sys.modules', {'numpy': None}):
            with self.assertRaisesRegex(ValueError, 'NumPy'):
                generate(doc, 'centerline', dict(centerline_method='region'))

    def test_region_choice_does_not_change_raceline_generation(self):
        doc = ring()
        expected = generate(doc, 'raceline', dict(spacing=.5))
        with patch('kart_studio.region_centerline.extract', side_effect=AssertionError('unexpected')):
            result = generate(doc, 'raceline', dict(spacing=.5, centerline_method='region'))
        self.assertEqual(result['points'], expected['points'])
        self.assertEqual(result['method'], expected['method'])

    def test_raceline_uses_published_region_center_instead_of_invalid_midpoints(self):
        doc = traced_course()
        for side in ('left', 'right'):
            doc[side] = [[x * .8, y * .8] for x, y in doc[side]]
        # The old midpoint route is too close to a sharp inner corner. A green
        # region Center was displayed, but Raceline silently started elsewhere.
        with self.assertRaisesRegex(ValueError, 'Raceline初期経路.*境界中点'):
            generate(doc, 'raceline', {})
        doc['lines'] = {'centerline': generate(doc, 'centerline', dict(centerline_method='region'))}
        original = copy.deepcopy(doc)
        with patch('kart_studio.raceline_reference.centerline', side_effect=AssertionError('unexpected pairing')):
            result = generate(doc, 'raceline', {})
        self.assertEqual(result['reference_info']['method'], 'region_equal_distance_v1')
        self.assertEqual(result['settings']['margin'], .05)
        self.assertEqual(doc, original)

    def test_ambiguous_and_multiple_grid_contours_are_rejected(self):
        import numpy as np
        from kart_studio.region_centerline import _trace
        with self.assertRaisesRegex(ValueError, '曖昧'):
            _trace(np, np.array([[1., -1.], [-1., 1.]]), np.ones((2,2), dtype=bool), [0,0], .02, lambda: None)
        field = np.ones((7, 15))
        field[2:5, 2:5] = -1
        field[2:5, 10:13] = -1
        with self.assertRaisesRegex(ValueError, '複数'):
            _trace(np, field, np.ones(field.shape, dtype=bool), [0,0], .02, lambda: None)

    def test_cancel_is_honored(self):
        def cancel(): raise InterruptedError('cancel')
        with self.assertRaises(InterruptedError):
            generate(ring(), 'centerline', dict(centerline_method='region'), cancel)

    def test_region_publication_and_export_preserve_boundaries_and_method(self):
        with tempfile.TemporaryDirectory() as tmp:
            maps = Maps(Path(tmp) / 'maps')
            folder = maps.root / 'course'
            folder.mkdir()
            atomic_json(folder / 'snapshot.json', dict(schema='kart.snapshot.v1', frame='map',
                points=[[0,0,0]], trajectory=[], provenance={}))
            maps.initialize(folder, 'course')
            doc = maps.load('course')
            lane = doc['lanes'][0]
            lane.update(ring())
            doc = maps.save('course', doc)
            maps.generate('course', 'centerline', dict(centerline_method='region'), doc['revision'], lambda: None)
            saved = maps.load('course')
            line = saved['lanes'][0]['lines']['centerline']
            self.assertEqual(line['settings']['centerline_method'], 'region')
            self.assertEqual(line['method'], 'region_equal_distance_v1')
            self.assertIn('region_info', line)
            self.assertEqual(saved['lanes'][0]['left'], lane['left'])
            self.assertEqual(saved['lanes'][0]['right'], lane['right'])
            maps.generate('course', 'raceline', {}, saved['revision'], lambda: None)
            saved = maps.load('course')
            self.assertEqual(saved['lanes'][0]['lines']['centerline'], line)
            exported = maps.export('course')
            self.assertTrue((exported / 'centerline.csv').is_file())
            metadata = json.loads((exported / 'metadata.json').read_text())
            reference = metadata['lanes'][saved['lanes'][0]['id']]['lines']['raceline']['reference_info']
            self.assertEqual(reference['source'], 'saved_centerline')
            self.assertEqual(reference['method'], 'region_equal_distance_v1')
            maps.generate('course', 'centerline', dict(centerline_method='paired'), saved['revision'], lambda: None)
            self.assertEqual(maps.load('course')['lanes'][0]['lines']['centerline']['method'], 'paired_boundary_midpoints')

    def test_failed_region_generation_keeps_previous_saved_center(self):
        with tempfile.TemporaryDirectory() as tmp:
            maps = Maps(Path(tmp) / 'maps')
            folder = maps.root / 'course'
            folder.mkdir()
            atomic_json(folder / 'snapshot.json', dict(schema='kart.snapshot.v1', frame='map',
                points=[[0,0,0]], trajectory=[], provenance={}))
            maps.initialize(folder, 'course')
            doc = maps.load('course')
            doc['lanes'][0].update(ring())
            doc = maps.save('course', doc)
            maps.generate('course', 'centerline', {}, doc['revision'], lambda: None)
            before = maps.load('course')
            with patch('kart_studio.region_centerline.MAX_CELLS', 10):
                with self.assertRaises(ValueError):
                    maps.generate('course', 'centerline', dict(centerline_method='region'), before['revision'], lambda: None)
            self.assertEqual(maps.load('course'), before)
