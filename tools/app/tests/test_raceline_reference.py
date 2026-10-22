"""Raceline must start from the displayed Centerline, without changing it."""

import copy
import math
import os
import unittest
from unittest.mock import patch

from kart_studio.geometry import PathValidationError, centerline
from kart_studio.lines import generate
from kart_studio.raceline_reference import build
from test_studio import track


def near_boundary_reference():
    doc = track()
    doc["lines"] = {"centerline": dict(
        points=[[3.1 * math.cos(i * math.tau / 80), 3.1 * math.sin(i * math.tau / 80)]
                for i in range(80)], method="near_boundary_fixture")}
    return doc


class RacelineReferenceTests(unittest.TestCase):
    def test_missing_saved_center_keeps_existing_initial_geometry(self):
        doc = track()
        expected, _, _ = centerline(doc["left"], doc["right"], True, .5)
        line, _, info = build(doc, .5)
        self.assertEqual(line, expected)
        self.assertEqual(info["source"], "paired_boundaries")

    def test_saved_center_does_not_recompute_paired_geometry_or_mutate_document(self):
        doc = track()
        doc["lines"] = {"centerline": generate(doc, "centerline", dict(spacing=.2))}
        original = copy.deepcopy(doc)
        with patch("kart_studio.raceline_reference.centerline", side_effect=AssertionError("unexpected pairing")):
            result = generate(doc, "raceline", dict(spacing=.5))
        self.assertEqual(result["reference_info"]["source"], "saved_centerline")
        self.assertEqual(result["reference_info"]["method"], "paired_boundary_midpoints")
        self.assertEqual(doc, original)

    def test_invalid_saved_center_is_rejected_without_paired_fallback(self):
        doc = track()
        doc["lines"] = {"centerline": dict(points=[[9, 9], [10, 9], [9, 10]])}
        with patch("kart_studio.raceline_reference.centerline", side_effect=AssertionError("unexpected fallback")):
            with self.assertRaisesRegex(PathValidationError, "Raceline初期経路.*保存済みCenterline") as caught:
                build(doc, .5)
        self.assertEqual(caught.exception.diagnostic["stage"], "raceline_reference")

    def test_local_initial_clearance_error_names_the_stage(self):
        with self.assertRaisesRegex(PathValidationError, "Raceline初期経路") as caught:
            generate(near_boundary_reference(), "raceline", dict(spacing=.5))
        self.assertEqual(caught.exception.diagnostic["reference_source"], "saved_centerline")

    @unittest.skipUnless(os.environ.get("KART_TEST_HELPER"), "Requires installed helper")
    def test_both_helpers_can_move_an_initial_reference_away_from_boundary(self):
        # The reference is inside the corridor but lacks the final clearance.
        # Previously it was rejected before either QP was ever called.
        doc = near_boundary_reference()
        original = copy.deepcopy(doc)
        for method in ("mincurv", "mincurv_iqp"):
            with self.subTest(method=method):
                result = generate(doc, "raceline", dict(optimizer=method, spacing=.5))
                info = result["reference_info"]
                self.assertEqual(info["source"], "saved_centerline")
                self.assertEqual(result["optimizer_info"]["reference_info"], info)
                self.assertEqual(result["settings"]["margin"], .05)
                self.assertGreater(len(result["points"]), 30)
        self.assertEqual(doc, original)
