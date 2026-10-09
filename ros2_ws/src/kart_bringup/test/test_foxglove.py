"""Verify that visualization does not expose control writes or heavy streams."""

import re
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((ROOT / "config/visualization/foxglove.yaml").read_text())[
    "/**/foxglove_bridge"
]["ros__parameters"]


def allowed(key, name):
    # Upstream uses ECMAScript regex_match with icase; these patterns use the common subset.
    return any(re.fullmatch(pattern, name, re.IGNORECASE) for pattern in PARAMS[key])


class FoxgloveTests(unittest.TestCase):
    def test_telemetry(self):
        for name in (
            "/tf_static",
            "/hdmap/markers",
            "/visual_slam/tracking/slam_path",
            "/system/jetson/diagnostics",
            "/e2e/status",
            "/bag/status",
        ):
            self.assertTrue(allowed("topic_whitelist", name), name)
        for name in (
            "/realsense/color/image_raw",
            "/visual_slam/vis/landmarks_cloud",
            "/e2e/tensor_output",
            "/hdmap/document",
            "/rosout",
            "/parameter_events",
            "/visual_slam/tracking/slam_path_extra",
        ):
            self.assertFalse(allowed("topic_whitelist", name), name)

    def test_write_boundaries(self):
        for name in (
            "/localization/pose_hint",
            "/visual_localization/trigger_localization",
        ):
            self.assertTrue(allowed("client_topic_whitelist", name))
        for name in (
            "/initialpose",
            "/auto/control_cmd",
            "/vehicle/control_cmd",
            "/tf",
            "/operation_mode/request",
            "/bag/request",
            "/localization/pose_hint/extra",
        ):
            self.assertFalse(allowed("client_topic_whitelist", name), name)
        self.assertTrue(
            allowed("service_whitelist", "/visual_localization/trigger_localization")
        )
        for name in (
            "/visual_slam/set_slam_pose",
            "/visual_slam/reset",
            "/node/set_parameters",
        ):
            self.assertFalse(allowed("service_whitelist", name), name)
        self.assertEqual(set(PARAMS["capabilities"]), {"clientPublish", "services"})
        self.assertFalse(allowed("param_whitelist", "/node:param"))
        self.assertFalse(allowed("asset_uri_allowlist", "file:///etc/passwd"))
        self.assertFalse(PARAMS["remote_access"])
        self.assertFalse(PARAMS["sysinfo"])
