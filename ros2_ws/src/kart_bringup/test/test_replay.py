"""Offline playback validation without ROS, GPU, or user recordings."""

import json
import tempfile
import unittest
from pathlib import Path

import yaml
from kart_bringup.mission import discover
from kart_bringup.replay import playback
from kart_e2e.contract import runtime_settings, validate_runtime

CONFIG = Path(__file__).resolve().parents[1] / "config"


class ReplayTests(unittest.TestCase):
    def test_filter_readiness_clock_and_missing_inputs(self):
        workflow = json.loads((CONFIG / "localization/workflow.json").read_text())
        required = [
            workflow[k]
            for k in ("left_image", "right_image", "left_info", "right_info")
        ]
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory) / "date/time/run"
            bag.mkdir(parents=True)
            rows = [
                {"topic_metadata": {"name": t}, "message_count": 10}
                for t in required + ["/tf_static", "/tf", "/vehicle/control_cmd"]
            ]
            meta = bag / "metadata.yaml"
            meta.write_text(
                yaml.safe_dump(
                    {"rosbag2_bagfile_information": {"topics_with_message_count": rows}}
                )
            )
            command = playback(CONFIG, bag)
            self.assertEqual(command[command.index("--rate") + 1], "1.0")
            self.assertIn("--start-paused", command)
            self.assertIn("--clock", command)
            topics = command[command.index("--topics") + 1 :]
            self.assertEqual(set(topics), set(required + ["/tf_static"]))
            self.assertEqual(discover(directory, "bag"), [bag.resolve()])
            for rate in ("0", "-1", "nan", "inf", "5"):
                with self.assertRaises(ValueError):
                    playback(CONFIG, bag, rate)
            rows[0]["message_count"] = 0
            meta.write_text(
                yaml.safe_dump(
                    {"rosbag2_bagfile_information": {"topics_with_message_count": rows}}
                )
            )
            with self.assertRaisesRegex(ValueError, "lacks required"):
                playback(CONFIG, bag)

    def test_rviz_overlay_and_clock(self):
        config = yaml.safe_load((CONFIG / "evaluation/localization.rviz").read_text())
        manager = config["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        raw = (CONFIG / "evaluation/localization.rviz").read_text()
        for topic in (
            "/hdmap/markers",
            "/hdmap/centerline",
            "/hdmap/raceline",
            "/hdmap/customline",
            "/visual_slam/vis/localizer_map_cloud",
        ):
            self.assertIn(topic, raw)
        self.assertNotIn("rviz_default_plugins/Image", raw)

    def test_evaluation_routes_only_localization_map_rviz_replay(self):
        import importlib.util
        import sys
        from types import ModuleType, SimpleNamespace
        from unittest.mock import patch

        lu = ModuleType("isaac_ros_launch_utils")
        lu.__path__ = []
        lu.get_path = lambda package, suffix: CONFIG
        lu.include = lambda package, path, **kwargs: (path, kwargs["launch_arguments"])
        lut = ModuleType("isaac_ros_launch_utils.all_types")
        with patch.dict(
            sys.modules,
            {"isaac_ros_launch_utils": lu, "isaac_ros_launch_utils.all_types": lut},
        ):
            spec = importlib.util.spec_from_file_location(
                "evaluation_test", CONFIG.parent / "launch/evaluation.launch.py"
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            args = SimpleNamespace(
                enable_vgl="false",
                base_frame="",
                bag="/bag",
                rate="",
                map_dir="/bundle",
                model_dir="/models",
                map_file="/hdmap.json",
                lane_id="lane_001",
                line_type="raceline",
                rviz="",
            )
            with (
                patch.object(module, "playback"),
                patch.object(module, "resolve"),
                patch.object(
                    module, "hdmap_choices", return_value=[("lane_001", "raceline")]
                ),
            ):
                actions = module.add_evaluation(args)
            self.assertEqual(
                [a[0] for a in actions],
                [
                    "launch/localization.launch.py",
                    "launch/hdmap.launch.py",
                    "launch/modules/evaluation/rviz.launch.py",
                    "launch/modules/evaluation/replay.launch.py",
                ],
            )
            self.assertEqual(actions[-1][1]["enable_vgl"], "false")
            self.assertEqual(actions[0][1]["enable_vgl"], "false")
            self.assertEqual(actions[0][1]["base_frame"], "camera_link")
            self.assertEqual(actions[0][1]["visualize"], "true")
            self.assertEqual(actions[0][1]["use_sim_time"], "true")
            self.assertEqual(actions[1][1]["use_sim_time"], "true")

    def test_readiness_waits_for_both_nodes_on_both_images(self):
        import importlib.util
        import sys
        from types import ModuleType, SimpleNamespace
        from unittest.mock import MagicMock, patch

        rclpy = ModuleType("rclpy")
        node = ModuleType("rclpy.node")
        node.Node = object
        services = ModuleType("rosbag2_interfaces.srv")
        services.Resume = SimpleNamespace(Request=lambda: object())
        with patch.dict(
            sys.modules,
            {"rclpy": rclpy, "rclpy.node": node, "rosbag2_interfaces.srv": services},
        ):
            spec = importlib.util.spec_from_file_location(
                "readiness_test", CONFIG.parent / "kart_bringup/replay_ready.py"
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            ready = module.ReplayReady.__new__(module.ReplayReady)
            ready.topics = ["left", "right"]
            ready.nodes = {"visual_slam", "visual_global_localization"}
            ready.future = None
            ready.deadline = float("inf")
            ready.client = MagicMock()
            ready.client.service_is_ready.return_value = True
            ready.get_subscriptions_info_by_topic = lambda topic: [
                SimpleNamespace(node_name="visual_slam")
            ]
            ready.check()
            ready.client.call_async.assert_not_called()
            ready.get_subscriptions_info_by_topic = lambda topic: [
                SimpleNamespace(node_name=name) for name in ready.nodes
            ]
            ready.check()
            ready.client.call_async.assert_called_once()

    def test_runtime_metadata_exact_and_legacy_rejected(self):
        settings = {"fixed_throttle": 0.12, "max_throttle": 0.2}
        with tempfile.TemporaryDirectory() as directory:
            metadata = Path(directory) / "metadata.json"
            metadata.write_text(json.dumps({"runtime": settings}))
            self.assertEqual(runtime_settings(directory), settings)
            metadata.write_text("{}")
            with self.assertRaisesRegex(ValueError, "re-export"):
                runtime_settings(directory)
        for bad in (
            None,
            {},
            {"fixed_throttle": 0.3, "max_throttle": 0.2},
            {"fixed_throttle": float("nan"), "max_throttle": 0.2},
            {"fixed_throttle": True, "max_throttle": 0.2},
        ):
            with self.assertRaises(ValueError):
                validate_runtime(bad)


if __name__ == "__main__":
    unittest.main()
