"""Final snapshot checks without ROS; landmark updates need not match image rate."""

import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from kart_mapping.capture_state import LocalizationGate
from kart_mapping.frames import FinalFrames


def message(seconds):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=0))
    )


class CaptureTailTests(unittest.TestCase):
    def collector(self):
        path = (
            Path(__file__).resolve().parents[3]
            / "ros2_ws/src/kart_mapping/kart_mapping/capture_collector.py"
        )
        spec = importlib.util.spec_from_file_location("kart_mapping._tail_test", path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(
            "sys.modules",
            {
                "diagnostic_msgs.msg": SimpleNamespace(DiagnosticArray=object),
                "rclpy.qos": SimpleNamespace(qos_profile_sensor_data=None),
                "kart_mapping.collector": SimpleNamespace(Collector=object),
            },
        ):
            spec.loader.exec_module(module)
        node = module.CaptureCollector.__new__(module.CaptureCollector)
        node.gate = LocalizationGate()
        values = {"localized_in_exist_map": "Yes", "vo_status": "OK"}
        node.gate.update(2_000_000_000, values, 1)
        node.gate.update(24_000_000_000, values, 2)
        node.cloud, node.path = message(10), message(24)
        node.frames = FinalFrames()
        node.frames.transforms["odom"] = {"stamp_ns": "24000000000"}
        node.get_logger = Mock(return_value=Mock())
        node.snapshot = lambda provenance: provenance
        return node

    def test_old_landmark_with_current_tracking_is_retained(self):
        node = self.collector()
        result = node.final_snapshot(24_000_000_000, 500_000_000, {})
        self.assertEqual(result["cloud_tail_age_s"], 14)
        self.assertFalse(result["cloud_within_tail_tolerance"])
        node.get_logger().warning.assert_called_once()

    def test_stale_tracking_tf_and_pre_localization_cloud_are_rejected(self):
        for field in ("path", "tf", "cloud", "diagnostic"):
            with self.subTest(field=field):
                node = self.collector()
                if field == "path":
                    node.path = message(10)
                elif field == "tf":
                    node.frames.transforms["odom"]["stamp_ns"] = "10000000000"
                elif field == "cloud":
                    node.cloud = message(1)
                else:
                    node.gate.stamp = 10_000_000_000
                with self.assertRaises(ValueError):
                    node.final_snapshot(24_000_000_000, 500_000_000, {})
