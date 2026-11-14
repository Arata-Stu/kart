"""Lap startup selection and integration, without ROS or host cache writes."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from kart_bringup.lap_maintenance import maintenance_parameters
from kart_bringup.mission import composition

PACKAGE = Path(__file__).resolve().parents[1]


class LapMaintenanceTests(unittest.TestCase):
    def test_flags_default_off_and_independent_drive_overrides(self):
        path = PACKAGE / "config/mission.yaml"
        for mode in ("drive", "collect", "environment", "e2e"):
            config = composition(path, mode)
            self.assertFalse(config["lap_vgl"])
            self.assertFalse(config["lap_cache_clear"])
            for key in ("lap_vgl", "lap_cache_clear"):
                if mode == "drive":
                    self.assertTrue(composition(path, mode, {key: "true"})[key])
                else:
                    with self.assertRaisesRegex(ValueError, "drive-only"):
                        composition(path, mode, {key: "true"})
                with self.assertRaises(ValueError):
                    composition(path, mode, {key: "yes"})

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / "localization").mkdir()
        (self.root / "system").mkdir()
        self.params = dict(vgl_enabled=False, cache_clear_enabled=False, map_frame="map",
                           vgl_trigger_service="/trigger", diagnostics_topic="/diag",
                           pose_hint_topic="/hint", cache_clear_service="/kart/cache/clear")
        self.write_params()
        (self.root / "localization/workflow.json").write_text(json.dumps(dict(
            request_hint="/trigger", vslam_diagnostics="/diag", pose_hint="/hint")))
        (self.root / "system/jetson_cache.yaml").write_text(yaml.safe_dump({
            "/**/jetson_cache_service": {"ros__parameters": {
                "service_name": "/kart/cache/clear", "socket_path": str(self.root / "cache.sock")}}}))
        self.cfg = dict(localization=True, tracking=True, lap_vgl=True, lap_cache_clear=False)
        self.doc = dict(frame_id="map", lanes=[dict(id="main", closed=True,
                                                   lines={"centerline": {"points": [[0,0], [1,0], [0,1]]}})])

    def write_params(self):
        (self.root / "localization/lap_manager.yaml").write_text(yaml.safe_dump({
            "/**/lap_manager": {"ros__parameters": self.params}}))

    def resolve(self, **kwargs):
        with patch("kart_hdmap.document.load", return_value=self.doc):
            return maintenance_parameters(self.root, self.cfg, "/map", "main", "centerline", **kwargs)

    def test_runtime_flags_override_yaml_without_mutating_file(self):
        resolved = self.resolve()
        self.assertTrue(resolved["vgl_enabled"])
        self.assertFalse(resolved["cache_clear_enabled"])
        stored = yaml.safe_load((self.root / "localization/lap_manager.yaml").read_text())
        self.assertFalse(stored["/**/lap_manager"]["ros__parameters"]["vgl_enabled"])

    def test_invalid_routes_frames_and_parking_rejected(self):
        with self.assertRaisesRegex(ValueError, "parking"):
            self.resolve(parking=True)
        self.doc["lanes"][0]["closed"] = False
        with self.assertRaisesRegex(ValueError, "closed"):
            self.resolve()
        self.doc["lanes"][0]["closed"] = True
        self.params["vgl_trigger_service"] = "/wrong"
        self.write_params()
        with self.assertRaisesRegex(ValueError, "workflow"):
            self.resolve()
        self.params["vgl_trigger_service"] = "/trigger"
        self.params["map_frame"] = "wrong"
        self.write_params()
        with self.assertRaisesRegex(ValueError, "frame"):
            self.resolve()

    def test_cache_on_requires_installed_host_socket(self):
        self.cfg.update(lap_vgl=False, lap_cache_clear=True)
        with self.assertRaisesRegex(ValueError, "host/install.sh"):
            self.resolve()
        with patch.object(Path, "is_socket", return_value=True):
            resolved = self.resolve()
        self.assertFalse(resolved["vgl_enabled"])
        self.assertTrue(resolved["cache_clear_enabled"])

    def test_standard_bag_profiles_record_lap_events_and_state(self):
        for suffix in ("", "_openeb", "_environment", "_environment_openeb"):
            path = PACKAGE / f"config/recording/bag_manager{suffix}.yaml"
            topics = yaml.safe_load(path.read_text())["/**/kart_bag_manager"]["ros__parameters"]["topics"]
            for topic in ("/localization/lap_events", "/localization/lap_status"):
                self.assertEqual(topics.count(topic), 1)


if __name__ == "__main__":
    unittest.main()
