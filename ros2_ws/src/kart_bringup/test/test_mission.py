"""Portable composition, discovery and CLI tests; does not access devices."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from kart_bringup.mission import composition, discover, sensor_parameters

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
CONFIG = PACKAGE / "config"


class MissionTests(unittest.TestCase):
    def test_collect_is_light_and_external_sensor_container(self):
        cfg = composition(
            CONFIG / "mission.yaml",
            "collect",
            {
                "enable_bridge": "false",
                "create_sensor_container": "false",
                "sensor_container": "/external/sensors",
            },
        )
        self.assertFalse(cfg["localization"])
        self.assertFalse(cfg["tracking"])
        self.assertFalse(cfg["create_sensor_container"])
        self.assertFalse(cfg["enable_bridge"])
        self.assertTrue(composition(CONFIG / "mission.yaml", "drive")["localization"])
        with self.assertRaises(ValueError):
            composition(
                CONFIG / "mission.yaml", "collect", {"create_sensor_container": "yes"}
            )
        with self.assertRaises(ValueError):
            composition(
                CONFIG / "mission.yaml", "collect", {"sensor_container": "/a/b"}
            )

    def test_fps_preserves_resolution_and_other_parameters(self):
        path = CONFIG / "sensors/realsense.yaml"
        original = sensor_parameters(path)
        changed = sensor_parameters(path, 60, 90)
        self.assertEqual(changed["rgb_camera.color_profile"], "424x240x60")
        self.assertEqual(changed["depth_module.infra_profile"], "424x240x90")
        self.assertEqual(changed["camera_name"], "camera")
        self.assertEqual(changed["base_frame_id"], "link")
        for key in original.keys() - {
            "rgb_camera.color_profile",
            "depth_module.infra_profile",
        }:
            self.assertEqual(changed[key], original[key])
        off = sensor_parameters(path, "0", "0")
        self.assertFalse(off["enable_color"])
        self.assertFalse(off["enable_infra1"])
        self.assertFalse(off["enable_infra2"])
        self.assertEqual(
            off["depth_module.infra_profile"], original["depth_module.infra_profile"]
        )
        with self.assertRaises(ValueError):
            sensor_parameters(path, "nan", "")

    def test_discovery_excludes_trash_pending_symlinks_and_malformed(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder).resolve()
            for name in ("course v2", ".trash/old", "pending", "broken"):
                d = root / name
                d.mkdir(parents=True)
                (d / "map.json").write_text(
                    json.dumps(
                        {
                            "schema": "kart.hdmap.v2",
                            "frame": "map",
                            "lanes": [
                                {
                                    "id": "lane_001",
                                    "closed": False,
                                    "left": [[0, 1], [2, 1]],
                                    "right": [[0, -1], [2, -1]],
                                    "lines": {
                                        "centerline": {"points": [[0, 0], [2, 0]]}
                                    },
                                }
                            ],
                            "snapshot_status": "pending"
                            if name == "pending"
                            else "ready",
                        }
                    )
                )
            (root / "broken/map.json").write_text("not-json[")
            (root / "link").symlink_to(root / "course v2", target_is_directory=True)
            self.assertEqual(discover(root, "hdmap"), [root / "course v2/map.json"])
            b = root / "bundle"
            b.mkdir()
            (b / "vgl_profile.json").write_text(
                json.dumps({"schema": "kart.vgl.v1", "status": "complete"})
            )
            self.assertEqual(discover(root, "bundle"), [b])
            self.assertEqual(discover(root / "missing", "hdmap"), [])

    def test_cli_dry_run_never_calls_ros_or_requires_maps_for_collect(self):
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/bringup.py"),
                "--mode",
                "collect",
                "--no-bridge",
                "--rgb-fps",
                "0",
                "--infra-fps",
                "60",
                "--record-dir",
                "/tmp/record with spaces",
                "--dry-run",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("mission.launch.py", result.stdout)
        self.assertIn("enable_bridge:=false", result.stdout)
        self.assertNotIn("map_dir:=", result.stdout)
        self.assertIn("RGB: OFF", result.stdout)

    def test_bad_drive_inputs_fail_before_launch(self):
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/bringup.py"),
                "--mode",
                "drive",
                "--no-bridge",
                "--infra-fps",
                "0",
                "--dry-run",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("Infra", result.stderr)

    def test_invalid_mode_and_unknown_override(self):
        with self.assertRaises(ValueError):
            composition(CONFIG / "mission.yaml", "other")
        with self.assertRaises(ValueError):
            composition(CONFIG / "mission.yaml", "collect", {"typo": True})


class MissionCompositionTests(unittest.TestCase):
    """Test callback routing with launch actions mocked, not ROS execution."""

    def test_modes_route_and_external_container_is_not_created(self):
        import importlib.util
        from types import ModuleType, SimpleNamespace
        from unittest.mock import MagicMock, patch

        lu = ModuleType("isaac_ros_launch_utils")
        lu.__path__ = []
        lu.get_path = lambda pkg, suffix: PACKAGE / suffix
        lu.log_info = lambda message: ("log", message)
        lu.component_container = MagicMock(return_value=("container",))
        lu.include = MagicMock(side_effect=lambda pkg, path, **kwargs: (path, kwargs))
        lut = ModuleType("isaac_ros_launch_utils.all_types")
        lu.all_types = lut
        index = ModuleType("ament_index_python")
        index.__path__ = []
        packages = ModuleType("ament_index_python.packages")
        packages.get_package_share_directory = lambda name: "/mock/" + name
        with patch.dict(
            sys.modules,
            {
                "isaac_ros_launch_utils": lu,
                "isaac_ros_launch_utils.all_types": lut,
                "ament_index_python": index,
                "ament_index_python.packages": packages,
            },
        ):
            spec = importlib.util.spec_from_file_location(
                "mission_launch_test", PACKAGE / "launch/mission.launch.py"
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            values = {
                key: ""
                for key in (
                    *module.OVERRIDES,
                    "device",
                    "record_dir",
                    "run_name",
                    "e2e_model_dir",
                    "rgb_fps",
                    "infra_fps",
                    "map_dir",
                    "model_dir",
                    "map_file",
                    "lane_id",
                    "line_type",
                )
            }
            values.update(
                mode="collect",
                enable_bridge="false",
                create_sensor_container="false",
                sensor_container="/external/sensors",
            )
            args = SimpleNamespace(**values)
            module.add_mission(args)
            self.assertEqual(
                [c.args[1] for c in lu.include.call_args_list],
                [
                    "launch/modules/sensors/realsense.launch.py",
                    "launch/vehicle.launch.py",
                ],
            )
            lu.component_container.assert_not_called()
            lu.include.reset_mock()
            args.mode = "drive"
            args.lane_id = "lane_001"
            args.line_type = "centerline"
            with (
                patch.object(
                    module,
                    "resolve",
                    return_value=({}, {}, {}, {"input_shape": [1, 3, 240, 424]}),
                ),
                patch.object(
                    module, "hdmap_choices", return_value=[("lane_001", "centerline")]
                ),
            ):
                module.add_mission(args)
            self.assertEqual(
                [c.args[1] for c in lu.include.call_args_list],
                [
                    "launch/modules/sensors/realsense.launch.py",
                    "launch/vehicle.launch.py",
                    "launch/localization.launch.py",
                    "launch/tracking.launch.py",
                ],
            )
            localization = lu.include.call_args_list[2].kwargs["launch_arguments"]
            self.assertEqual(localization["vslam_container"], "/external/sensors")
            self.assertEqual(localization["create_vslam_container"], "false")
            lu.component_container.assert_not_called()
            lu.include.reset_mock()
            args.mode = "e2e"
            with (
                patch("kart_e2e.contract.model_contract"),
                patch("kart_e2e.contract.runtime_settings"),
            ):
                module.add_mission(args)
            paths = [c.args[1] for c in lu.include.call_args_list]
            self.assertNotIn("launch/localization.launch.py", paths)
            self.assertNotIn("launch/tracking.launch.py", paths)
            inference = lu.include.call_args_list[-1].kwargs["launch_arguments"]
            self.assertEqual(inference["container_name"], "/external/sensors")
            self.assertEqual(inference["create_container"], "false")
            self.assertEqual(inference["drive_enabled"], "true")


if __name__ == "__main__":
    unittest.main()
