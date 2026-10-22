"""Reject crossing routes and backtrack helper steps without relaxing checks."""

import copy
import json
import math
import os
import unittest
from pathlib import Path
from unittest.mock import Mock

from kart_studio.geometry import PathValidationError, simple
from kart_studio.lines import DEFAULTS, generate, validate_raceline_curvature
from kart_studio.optimizer_worker import _noncrossing_output
from kart_studio.raceline_reference import validate
from test_region_centerline import traced_course


BOWTIE = [[-1, -1], [1, 1], [-1, 1], [1, -1]]


class TopologyTests(unittest.TestCase):
    def test_intersection_reports_both_segments(self):
        with self.assertRaisesRegex(PathValidationError, "区間1と区間3") as caught:
            simple(BOWTIE, True)
        diagnostic = caught.exception.diagnostic
        self.assertEqual(diagnostic["reason"], "self_intersection")
        self.assertEqual(diagnostic["points"], BOWTIE[:2])
        self.assertEqual(diagnostic["other_points"], BOWTIE[2:])

    def test_lateral_backtracking_preserves_route_order(self):
        def candidate(scale):
            return [[-1, -1], [1, -1 + 2 * scale],
                    [1 - 2 * scale, 1], [-1 + 2 * scale, 1 - 2 * scale]]

        create = Mock(side_effect=candidate)
        line, scale, failures = _noncrossing_output(create)
        self.assertEqual(scale, .5)
        self.assertEqual(line, candidate(.5))
        self.assertEqual([call.args[0] for call in create.call_args_list], [1, .5])
        self.assertEqual(failures[0]["diagnostic"]["reason"], "self_intersection")
        simple(line, True)

    def test_backtracking_is_bounded_and_does_not_return_crossing_route(self):
        create = Mock(return_value=BOWTIE)
        with self.assertRaisesRegex(PathValidationError, "Raceline最適化後") as caught:
            _noncrossing_output(create)
        self.assertEqual(create.call_count, 9)
        self.assertEqual(caught.exception.diagnostic["stage"], "raceline_optimized_output")

    def test_non_crossing_candidate_must_also_meet_curvature_limit(self):
        def create(scale):
            radius = .1 / scale
            return [[radius * math.cos(i * math.tau / 20), radius * math.sin(i * math.tau / 20)]
                    for i in range(20)]

        p = dict(DEFAULTS, curvature_limit=6)
        line, scale, failures = _noncrossing_output(create, lambda line: validate_raceline_curvature(line, p))
        self.assertEqual(scale, .5)
        self.assertEqual(failures[0]["diagnostic"]["reason"], "curvature")
        validate_raceline_curvature(line, p)

    def test_smoothed_reference_is_identified_separately_from_output(self):
        corridor = Mock()
        try:
            simple(BOWTIE, True)
        except PathValidationError as failure:
            corridor.validate.side_effect = failure
        with self.assertRaisesRegex(PathValidationError, "平滑化後の初期経路") as caught:
            validate(BOWTIE, corridor, 0, dict(source="saved_centerline"),
                     stage="raceline_smoothed_reference")
        self.assertEqual(caught.exception.diagnostic["stage"], "raceline_smoothed_reference")


@unittest.skipUnless(os.environ.get("KART_TEST_HELPER"), "Requires installed helper and NumPy")
class RealHelperTopologyTests(unittest.TestCase):
    def test_user_e522_boundaries_meet_limit_six_without_changing_vehicle(self):
        doc = json.loads((Path(__file__).parent / "fixtures/e522_boundaries.json").read_text())
        doc["lines"] = {"centerline": generate(doc, "centerline", dict(spacing=.1))}
        self.assertEqual(len(doc["lines"]["centerline"]["points"]), 220)
        original = copy.deepcopy(doc)
        result = generate(doc, "raceline", dict(optimizer="mincurv", curvature_limit=6))
        self.assertLessEqual(max(abs(row[4]) for row in result["profile"]), 6 * 1.01)
        self.assertEqual(result["optimizer_info"]["lateral_step_scale"], .5)
        self.assertEqual(result["settings"]["vehicle_width"], .19)
        self.assertEqual(result["settings"]["vehicle_length"], .35)
        self.assertEqual(result["settings"]["margin"], .05)
        self.assertEqual(doc, original)

    def course(self):
        # Synthetic image-derived geometry, not the user's map or a vehicle test.
        doc = traced_course()
        for side in ("left", "right"):
            doc[side] = [[x * .8, y * .8] for x, y in doc[side]]
        doc["lines"] = {"centerline": generate(doc, "centerline", dict(centerline_method="region"))}
        return doc

    def test_real_min_curv_backtracks_a_crossing_spline(self):
        doc = self.course()
        original = copy.deepcopy(doc)
        # Loose curvature here isolates the topology behavior. This value is
        # only for the synthetic test and is not a vehicle recommendation.
        result = generate(doc, "raceline", dict(optimizer="mincurv", curvature_limit=20))
        info = result["optimizer_info"]
        self.assertLess(info["lateral_step_scale"], 1)
        self.assertTrue(info["topology_refinements"])
        self.assertEqual(info["topology_refinements"][0]["diagnostic"]["reason"], "self_intersection")
        self.assertLessEqual(max(abs(row[4]) for row in result["profile"]), 20 * 1.01)
        self.assertEqual(result["settings"]["margin"], .05)
        self.assertEqual(doc, original)
        simple(result["points"], True)

    def test_real_backtracking_does_not_bypass_curvature_limit(self):
        with self.assertRaisesRegex(ValueError, "曲率.*上限") as caught:
            generate(self.course(), "raceline", dict(optimizer="mincurv", curvature_limit=1))
        diagnostic = caught.exception.diagnostic
        self.assertEqual(diagnostic["reason"], "curvature")
        self.assertEqual(diagnostic["curvature_limit"], 1)
        self.assertGreater(diagnostic["curvature_abs"], 1.01)
