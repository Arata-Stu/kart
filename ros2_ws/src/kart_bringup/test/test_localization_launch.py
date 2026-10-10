"""Exercise the actual ROS launch graph when ROS/Isaac launch libraries exist."""

import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

try:
    import isaac_ros_launch_utils as lu
    from launch import LaunchContext
    from launch.actions import (
        DeclareLaunchArgument,
        GroupAction,
        IncludeLaunchDescription,
        OpaqueFunction,
    )
    from launch_ros.actions import Node
except ImportError as exc:
    raise unittest.SkipTest(f"ROS/Isaac launch libraries unavailable: {exc}") from exc

PATH = Path(__file__).resolve().parents[1] / "launch/localization.launch.py"
spec = importlib.util.spec_from_file_location("kart_localization_launch", PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LocalizationLaunchTest(unittest.TestCase):
    def setUp(self):
        patcher = patch.object(
            module,
            "plan",
            return_value=(
                {"vslam": "/kart_vslam_container", "vgl": "/kart_vgl_container"},
                [
                    {"name": "kart_vslam_container", "type": "multithreaded"},
                    {"name": "kart_vgl_container", "type": "multithreaded"},
                ],
            ),
        )
        self.route = patcher.start()
        self.addCleanup(patcher.stop)

    def test_arguments_discoverable_without_gpu_assets(self):
        entities = module.generate_launch_description().entities
        arguments = [a for a in entities if isinstance(a, DeclareLaunchArgument)]
        self.assertEqual(
            [a.name for a in arguments],
            [
                "map_dir",
                "model_dir",
                "base_frame",
                "use_sim_time",
                "visualize",
                "vslam_container",
                "create_vslam_container",
                "vgl_container",
                "create_vgl_container",
            ],
        )
        self.assertIsNone(arguments[0].default_value)
        self.assertIsNone(arguments[1].default_value)
        self.assertEqual(sum(isinstance(a, OpaqueFunction) for a in entities), 1)

    def fixture(self):
        return (
            {"cuvslam": {"use_sim_time": False}, "vgl": {"use_sim_time": False}},
            {"cuvslam": [], "vgl": []},
            {
                "vslam_container": "kart_vslam_container",
                "vgl_container": "kart_vgl_container",
            },
            {"input_shape": [1, 3, 240, 424]},
        )

    def test_separate_container_and_load_actions(self):
        args = SimpleNamespace(
            map_dir="/map",
            model_dir="/model",
            base_frame="",
            visualize="",
            use_sim_time=False,
            vslam_container="",
            vgl_container="",
            create_vslam_container="",
            create_vgl_container="",
        )
        with (
            patch.object(module, "resolve", return_value=self.fixture()) as resolve,
            patch.object(lu, "get_path", return_value=Path("/config")),
        ):
            actions = module.add_localization(args)
        self.assertEqual(resolve.call_args.args[3], {"use_sim_time": False})
        self.assertEqual(sum(isinstance(a, Node) for a in actions), 2)
        groups = [a for a in actions if isinstance(a, GroupAction)]
        self.assertEqual(len(groups), 2)
        self.assertTrue(
            all(
                any(
                    isinstance(a, IncludeLaunchDescription)
                    for a in group.get_sub_entities()
                )
                for group in groups
            )
        )

    def test_invalid_assets_start_no_containers(self):
        args = SimpleNamespace(
            map_dir="/map", model_dir="/model", base_frame="", use_sim_time=""
        )
        with (
            patch.object(module, "resolve", side_effect=ValueError("bad assets")),
            patch.object(lu, "get_path", return_value=Path("/config")),
            patch.object(lu, "component_container") as container,
        ):
            with self.assertRaisesRegex(ValueError, "bad assets"):
                module.add_localization(args)
            container.assert_not_called()

    def test_real_argument_evaluation_keeps_false_override(self):
        description = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations.update(
            map_dir="/map", model_dir="/model", base_frame="", use_sim_time="False"
        )
        callback = next(
            a for a in description.entities if isinstance(a, OpaqueFunction)
        )
        with (
            patch.object(module, "resolve", return_value=self.fixture()) as resolve,
            patch.object(lu, "get_path", return_value=Path("/config")),
        ):
            callback.execute(context)
        self.assertEqual(resolve.call_args.args[3], {"use_sim_time": False})

    def test_external_containers_create_no_processes(self):
        self.route.return_value = ({"vslam": "/external", "vgl": "/external"}, [])
        args = SimpleNamespace(
            map_dir="/map",
            model_dir="/model",
            base_frame="",
            visualize="",
            use_sim_time="",
            vslam_container="external",
            vgl_container="external",
            create_vslam_container=False,
            create_vgl_container=False,
        )
        with (
            patch.object(module, "resolve", return_value=self.fixture()),
            patch.object(lu, "get_path", return_value=Path("/config")),
        ):
            actions = module.add_localization(args)
        self.assertFalse(any(isinstance(a, Node) for a in actions))
        self.assertEqual(sum(isinstance(a, GroupAction) for a in actions), 2)
