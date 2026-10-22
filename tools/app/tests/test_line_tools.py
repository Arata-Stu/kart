"""Behavioral checks for speed limits, map registration and optimizer selection."""

import math
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from kart_studio import registration
from kart_studio.lines import DEFAULTS, generate
from kart_studio.maps import Maps
from kart_studio.speed_sections import sections
from kart_studio.storage import atomic_json, read_json
from test_studio import track


class SpeedTests(unittest.TestCase):
    def test_short_section_inserts_boundaries_and_brakes_before_entry(self):
        doc = dict(
            left=[[0, 2], [20, 2]],
            right=[[0, -2], [20, -2]],
            custom=[[0, 0], [20, 0]],
            closed=False,
            custom_speeds=[dict(start=8.01, end=8.02, speed=0.5)],
        )
        result = generate(doc, "customline", dict(spacing=0.5))
        rows = result["profile"]
        for s in (8.01, 8.02):
            row = min(rows, key=lambda r: abs(r[0] - s))
            self.assertAlmostEqual(row[0], s)
            self.assertLessEqual(row[5], 0.5)
        before = min(rows, key=lambda r: abs(r[0] - 8))
        self.assertLess(before[5], 0.6)
        self.assertTrue(
            all(
                -DEFAULTS["decel"] - 1e-7 <= r[6] <= DEFAULTS["accel"] + 1e-7
                for r in rows
            )
        )

    def test_wrap_and_overlap(self):
        doc = track()
        doc["custom_speeds"] = [
            dict(start=20, end=2, speed=0.8),
            dict(start=0, end=1, speed=0.4),
        ]
        r = generate(doc, "customline", {})
        self.assertLessEqual(r["profile"][0][5], 0.4)
        self.assertTrue(
            all(
                row[5] <= 0.8 + 1e-8
                for row in r["profile"]
                if row[0] >= 20 or row[0] <= 2
            )
        )
        self.assertTrue(
            all(
                -DEFAULTS["decel"] - 1e-6 <= row[6] <= DEFAULTS["accel"] + 1e-6
                for row in r["profile"]
            )
        )

    def test_invalid_intervals(self):
        for value in (
            [dict(start=0, end=0, speed=1)],
            [dict(start=4, end=1, speed=1)],
            [dict(start=0, end=6, speed=1)],
            [dict(start=0, end=2, speed=float("nan"))],
        ):
            with self.assertRaises(ValueError):
                sections(value, [[0, 0], [5, 0]], False)


class RegistrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.maps = Maps(Path(self.temp.name))
        for key in ("source", "target"):
            d = dict(track(), revision=1, lines={}, title=key, point_count=1)
            d["custom_speeds"] = [dict(start=1, end=3, speed=0.6)]
            d["lines"] = {
                kind: generate(d, kind, {"spacing": 0.5})
                for kind in ("centerline", "customline")
            }
            folder = self.maps.root / key
            atomic_json(folder / "map.json", d)
            atomic_json(folder / "snapshot.json", {"marker": key})
            atomic_json(folder / "cloud.json", {"marker": key})
            (folder / "cuvslam_map").mkdir()
            (folder / "cuvslam_map" / "data").write_bytes(b"vslam-data")
        self.body = dict(source="source", revision=1, x=10, y=-2, yaw=90)

    def test_rigid_transform_preserves_profiles_and_vslam(self):
        original = self.maps.load("target")
        source = self.maps.load("source")
        p = registration.preview(self.maps, "target", self.body)
        self.assertEqual(self.maps.load("target"), original)
        result = registration.apply(
            self.maps, "target", dict(self.body, token=p["token"])
        )
        x, y = source["lanes"][0]["left"][0]
        self.assertAlmostEqual(result["lanes"][0]["left"][0][0], 10 - y)
        self.assertAlmostEqual(result["lanes"][0]["left"][0][1], -2 + x)
        for before, after in zip(
            source["lanes"][0]["lines"]["customline"]["profile"],
            result["lanes"][0]["lines"]["customline"]["profile"],
        ):
            self.assertEqual(before[0], after[0])
            self.assertEqual(before[4:], after[4:])
            self.assertAlmostEqual(
                math.cos(after[3]), math.cos(before[3] + math.pi / 2)
            )
        self.assertEqual(
            source["lanes"][0]["custom_speeds"], result["lanes"][0]["custom_speeds"]
        )
        self.assertEqual(self.maps.load("source"), source)
        folder = self.maps.folder("target")
        self.assertEqual(read_json(folder / result["registration"]["backup"]), original)
        self.assertEqual(read_json(folder / "snapshot.json"), {"marker": "target"})
        self.assertEqual(read_json(folder / "cloud.json"), {"marker": "target"})
        self.assertEqual((folder / "cuvslam_map" / "data").read_bytes(), b"vslam-data")
        self.assertTrue((self.maps.export("target") / "customline.csv").is_file())

    def test_preview_rejects_changed_source_transform_and_target(self):
        proposal = registration.preview(self.maps, "target", self.body)
        body = dict(self.body, token=proposal["token"])
        with self.assertRaises(ValueError):
            registration.apply(self.maps, "target", dict(body, x=11))
        source = self.maps.load("source")
        source["revision"] += 1
        atomic_json(self.maps.folder("source") / "map.json", source)
        with self.assertRaises(ValueError):
            registration.apply(self.maps, "target", body)
        body["revision"] = 0
        with self.assertRaises(ValueError):
            registration.preview(self.maps, "target", body)

    def test_speed_edit_invalidates_only_custom_and_persists(self):
        doc = self.maps.load("source")
        doc["lanes"][0]["custom_speeds"][0]["speed"] = 0.3
        result = self.maps.save("source", doc)
        self.assertIn("centerline", result["lanes"][0]["lines"])
        self.assertNotIn("customline", result["lanes"][0]["lines"])
        self.assertEqual(
            self.maps.load("source")["lanes"][0]["custom_speeds"][0]["speed"], 0.3
        )


class OptimizerTests(unittest.TestCase):
    def test_clearance_refinement_stops_without_returning_invalid_candidate(self):
        from kart_studio.geometry import PathValidationError
        from kart_studio.optimizer_worker import _validated_solution

        failure = PathValidationError(
            "距離が不足", [0, 0], [1, 0], 0,
            reason="clearance", distance_m=0.144, required_clearance_m=0.145,
        )
        for exhausted in (True, False):
            with self.subTest(exhausted=exhausted):
                corridor = Mock()
                corridor.validate.side_effect = failure
                solve = Mock(return_value=[[0, 0], [1, 0], [0, 1]])
                if not exhausted:
                    solve.side_effect = [solve.return_value, ValueError("constraints are inconsistent, no solution")]
                with self.assertRaises(ValueError) as caught:
                    _validated_solution(solve, corridor, DEFAULTS)
                self.assertIs(caught.exception, failure)
                self.assertEqual(solve.call_count, 8 if exhausted else 2)
                self.assertEqual(solve.call_args_list[0].args, (0.0,))
                self.assertGreater(solve.call_args_list[1].args[0], 0)

    def test_worker_failure_preserves_ui_diagnostic(self):
        import json
        from kart_studio.optimizer import optimize

        for diagnostic in (dict(segment=153, reason="clearance", distance_m=0.144), None):
            with self.subTest(diagnostic=diagnostic):
                def launch(command, **kwargs):
                    Path(command[-1]).write_text(json.dumps(dict(error="距離が不足", diagnostic=diagnostic)))
                    process = Mock(returncode=1)
                    process.poll.return_value = 1
                    return process

                with patch("kart_studio.optimizer.subprocess.Popen", side_effect=launch):
                    with self.assertRaisesRegex(ValueError, "距離が不足") as caught:
                        optimize(track(), {}, lambda: None)
                if diagnostic is None:
                    # Maps.generate enriches diagnostics when the attribute
                    # exists; a None attribute would mask the original error.
                    self.assertFalse(hasattr(caught.exception, "diagnostic"))
                else:
                    self.assertEqual(caught.exception.diagnostic, diagnostic)

    def test_constraint_error_reports_settings_without_dependency_advice(self):
        from kart_studio.optimizer_worker import run

        for message in (
            "constraints are inconsistent, no solution",
            "Problem not solvable, track might be too small",
        ):
            with self.subTest(message=message), patch(
                "kart_studio.optimizer_worker._run", side_effect=ValueError(message)
            ):
                with self.assertRaisesRegex(ValueError, "曲率上限=1.*最小旋回半径=1") as caught:
                    run({}, dict(DEFAULTS, optimizer="mincurv_iqp"))
                self.assertNotIn("KART_RACELINE_PYTHON", str(caught.exception))

    def test_unrelated_solver_error_is_preserved(self):
        from kart_studio.optimizer_worker import run

        with patch("kart_studio.optimizer_worker._run", side_effect=ValueError("matrix not positive definite")):
            with self.assertRaisesRegex(ValueError, "^matrix not positive definite$"):
                run({}, {})

    def test_unknown_and_open_methods_rejected(self):
        with self.assertRaises(ValueError):
            generate(track(), "raceline", dict(optimizer="unknown"))
        doc = dict(
            left=[[0, 1], [10, 1]], right=[[0, -1], [10, -1]], custom=[], closed=False
        )
        for method in ("mincurv", "mincurv_iqp"):
            with self.assertRaisesRegex(ValueError, "閉路"):
                generate(doc, "raceline", dict(optimizer=method))

    def test_missing_runtime_does_not_fallback(self):
        with patch.dict(os.environ, KART_RACELINE_PYTHON="/nonexistent/kart-python"):
            with self.assertRaises(OSError):
                generate(track(), "raceline", dict(optimizer="mincurv", spacing=0.5))

    def test_cancel_terminates_helper_process(self):
        from kart_studio.optimizer import optimize

        processes = []
        original = subprocess.Popen

        def launch(*args, **kwargs):
            process = original(*args, **kwargs)
            processes.append(process)
            return process

        def cancel():
            raise InterruptedError("cancelled")

        with patch("kart_studio.optimizer.subprocess.Popen", side_effect=launch):
            with self.assertRaises(InterruptedError):
                optimize(track(), {}, cancel)
        self.assertEqual(len(processes), 1)
        self.assertIsNotNone(processes[0].poll())

    @unittest.skipUnless(
        os.environ.get("KART_TEST_HELPER"),
        "Set KART_TEST_HELPER=1 and KART_RACELINE_PYTHON to test installed helper",
    )
    def test_real_helper_both_methods(self):
        doc = track()
        for method in ("mincurv", "mincurv_iqp"):
            with self.subTest(method=method):
                result = generate(doc, "raceline", dict(optimizer=method, spacing=0.5))
                self.assertEqual(result["method"], "tph_" + method)
                self.assertGreater(len(result["points"]), 30)
                self.assertEqual(result["settings"]["margin"], 0.05)
                rows = result["profile"]
                self.assertLessEqual(max(abs(r[4]) for r in rows), 2.02)
                self.assertTrue(
                    all(
                        r[5] ** 2 * abs(r[4]) <= DEFAULTS["lateral_accel"] + 1e-6
                        for r in rows
                    )
                )

    @unittest.skipUnless(os.environ.get("KART_TEST_HELPER"), "Requires installed helper")
    def test_real_helper_refines_interpolated_clearance_violation(self):
        # Both methods used to fail at 0.144 m against a 0.145 m requirement.
        from kart_studio.geometry import Corridor

        for method in ("mincurv", "mincurv_iqp"):
            with self.subTest(method=method):
                doc = track()
                result = generate(doc, "raceline", dict(optimizer=method, spacing=0.5))
                info = result["optimizer_info"]
                self.assertGreater(info["interpolation_reserve_m"], 0)
                self.assertLess(info["interpolation_reserve_m"], 0.05)
                self.assertEqual(info["clearance_refinements"][0]["reserve_m"], 0)
                self.assertLess(info["clearance_refinements"][0]["diagnostic"]["distance_m"], 0.145)
                Corridor(doc["left"], doc["right"], True).validate(result["points"], 0.145)

    @unittest.skipUnless(os.environ.get("KART_TEST_HELPER"), "Requires installed helper")
    def test_real_helper_infeasible_curvature_is_reported_through_subprocess(self):
        for method in ("mincurv", "mincurv_iqp"):
            with self.subTest(method=method):
                with self.assertRaisesRegex(ValueError, "最適化制約.*曲率上限=0.01"):
                    generate(track(), "raceline", dict(optimizer=method, spacing=0.5, curvature_limit=0.01))

    @unittest.skipUnless(os.environ.get("KART_TEST_HELPER"), "Requires installed helper")
    def test_real_helper_reports_insufficient_width(self):
        from kart_studio.optimizer import optimize
        from kart_studio.lines import settings

        doc = track()
        for side, radius in (("left", 3.87), ("right", 4.13)):
            doc[side] = [[radius * math.cos(i * math.tau / 80), radius * math.sin(i * math.tau / 80)] for i in range(80)]
        with self.assertRaisesRegex(ValueError, "法線方向幅=.*必要幅=0.290"):
            optimize(doc, settings(dict(optimizer="mincurv_iqp", spacing=0.2)), lambda: None)
