"""Offline playback validation without ROS, GPU, or user recordings."""

import json
import tempfile
import unittest
from pathlib import Path

import yaml
from kart_bringup.mission import discover
from kart_bringup.replay import playback, imu_topics
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

    def test_vio_imu_is_required_and_filtered(self):
        topics=["/realsense/infra1/image_rect_raw","/realsense/infra2/image_rect_raw",
                "/realsense/infra1/camera_info","/realsense/infra2/camera_info","/tf_static"]
        with tempfile.TemporaryDirectory() as directory:
            metadata=Path(directory)/'metadata.yaml'
            def write():metadata.write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'topics_with_message_count':[{'topic_metadata':{'name':t},'message_count':10} for t in topics]}}))
            write()
            with self.assertRaisesRegex(ValueError,'/realsense/imu'):playback(CONFIG,directory,extra_topics=imu_topics(CONFIG,1))
            topics.append('/realsense/imu');write()
            command=playback(CONFIG,directory,extra_topics=imu_topics(CONFIG,1))
            self.assertIn('/realsense/imu',command[command.index('--topics')+1:])
            self.assertEqual(imu_topics(CONFIG,0),[])

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
                base_frame="",tracking_mode="",imu_frame="",multicam_mode="",
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
                patch.object(module, "resolve", return_value=({'cuvslam':{'tracking_mode':0}},None,None,None)),
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
            args.tracking_mode='1';args.imu_frame='camera_imu_optical_frame';args.multicam_mode='2'
            with patch.object(module,'playback') as player,patch.object(module,'resolve',return_value=({'cuvslam':{'tracking_mode':1}},None,None,None)) as resolver,patch.object(module,'hdmap_choices',return_value=[('lane_001','raceline')]):
                actions=module.add_evaluation(args)
            self.assertEqual(player.call_args.kwargs['extra_topics'],['/realsense/imu'])
            self.assertEqual(resolver.call_args.args[3]['tracking_mode'],'1')
            self.assertEqual(actions[0][1]['tracking_mode'],'1')
            self.assertEqual(actions[0][1]['multicam_mode'],'2')
            self.assertEqual(resolver.call_args.args[3]['multicam_mode'],'2')
            self.assertEqual(actions[0][1]['imu_frame'],'camera_imu_optical_frame')
            self.assertEqual(actions[-1][1]['tracking_mode'],'1')


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
            ready.imu_topic=''
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
            ready.future=None;ready.client.reset_mock();ready.imu_topic='/imu'
            ready.get_subscriptions_info_by_topic=lambda topic:[] if topic=='/imu' else [SimpleNamespace(node_name=n) for n in ready.nodes]
            ready.check();ready.client.call_async.assert_not_called()
            ready.get_subscriptions_info_by_topic=lambda topic:[SimpleNamespace(node_name='visual_slam')] if topic=='/imu' else [SimpleNamespace(node_name=n) for n in ready.nodes]
            ready.check();ready.client.call_async.assert_called_once()


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


class BagShapeTests(unittest.TestCase):
    def validate(self, changes=None, missing=None, wrong_type=None):
        from types import SimpleNamespace
        from unittest.mock import patch
        from kart_bringup.replay import validate_bag_shape
        workflow = json.loads((CONFIG / 'localization/workflow.json').read_text())
        topics = [workflow[k] for k in ('left_image', 'right_image', 'left_info', 'right_info')]
        rows = [(t, SimpleNamespace(width=640, height=360), 0) for t in topics if t != missing]
        for topic, msg, _ in rows:
            if changes and topic in changes:
                msg.width, msg.height = changes[topic]
        class Reader:
            def __init__(self): self.index = 0
            def open(self, *_): pass
            def get_all_topics_and_types(self):
                return [SimpleNamespace(name=t, type='wrong' if t == wrong_type else
                    ('sensor_msgs/msg/Image' if i < 2 else 'sensor_msgs/msg/CameraInfo')) for i, t in enumerate(topics)]
            def set_filter(self, *_): pass
            def has_next(self): return self.index < len(rows)
            def read_next(self):
                row = rows[self.index]; self.index += 1; return row
        with tempfile.TemporaryDirectory() as bag, patch.dict('sys.modules', {
            'rosbag2_py': SimpleNamespace(SequentialReader=Reader, StorageOptions=lambda **kw: kw,
                ConverterOptions=lambda *args: args, StorageFilter=lambda **kw: kw),
            'rclpy.serialization': SimpleNamespace(deserialize_message=lambda raw, _: raw),
            'sensor_msgs.msg': SimpleNamespace(Image=object, CameraInfo=object),
        }):
            return validate_bag_shape(CONFIG, bag, [1, 3, 360, 640])

    def test_recorded_size_matches_model_despite_live_yaml_424x240(self):
        sizes = self.validate()
        self.assertEqual(len(sizes), 4)
        self.assertTrue(all(size == [640, 360] for size in sizes.values()))

    def test_each_image_and_camera_info_mismatch_is_reported_with_sizes(self):
        workflow = json.loads((CONFIG / 'localization/workflow.json').read_text())
        for key in ('left_image', 'right_image', 'left_info', 'right_info'):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'bag=424x240, model=640x360'):
                self.validate(changes={workflow[key]: [424, 240]})

    def test_missing_message_and_wrong_type_rejected(self):
        workflow = json.loads((CONFIG / 'localization/workflow.json').read_text())
        with self.assertRaisesRegex(ValueError, 'メッセージ'):
            self.validate(missing=workflow['right_info'])
        with self.assertRaisesRegex(ValueError, 'wrong type'):
            self.validate(wrong_type=workflow['left_image'])


if __name__ == "__main__":
    unittest.main()
