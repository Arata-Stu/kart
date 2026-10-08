"""Behavioral checks for speed limits, map registration and optimizer selection."""

import math
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

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
                self.assertLessEqual(max(abs(r[4]) for r in result["profile"]), 2.02)
                self.assertTrue(
                    all(
                        r[5] ** 2 * abs(r[4]) <= DEFAULTS["lateral_accel"] + 1e-6
                        for r in result["profile"]
                    )
                )
