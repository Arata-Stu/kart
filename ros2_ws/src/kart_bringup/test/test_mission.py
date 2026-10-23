"""Portable composition, discovery and CLI tests; does not access devices."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from kart_bringup.mission import composition, discover, sensor_parameters, camera_config, recording_config

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
CONFIG = PACKAGE / "config"


class MissionTests(unittest.TestCase):
    def test_imu_profiles_and_vio_requirement(self):
        from kart_bringup.mission import validate_localization_imu
        for mode in ("collect", "environment"):
            path = camera_config(CONFIG, mode)
            original = sensor_parameters(path)
            for profile, rates in (("low", (100, 200)), ("high", (200, 400))):
                params = sensor_parameters(path, imu_profile=profile)
                self.assertEqual((params["accel_fps"], params["gyro_fps"]), rates)
                self.assertTrue(params["enable_accel"] and params["enable_gyro"])
                self.assertEqual(params["rgb_camera.color_profile"], original["rgb_camera.color_profile"])
                validate_localization_imu({"cuvslam": {"tracking_mode": 1}}, params)
            off = sensor_parameters(path, imu_profile="off")
            self.assertFalse(off["enable_accel"] or off["enable_gyro"])
            validate_localization_imu({"cuvslam": {"tracking_mode": 0}}, off)
            with self.assertRaisesRegex(ValueError, "VIO requires IMU"):
                validate_localization_imu({"cuvslam": {"tracking_mode": 1}}, off)
            self.assertEqual(sensor_parameters(path, imu_profile=""), original)
            with self.assertRaises(ValueError):
                sensor_parameters(path, imu_profile="invalid")

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

    def test_resolution_profiles_and_conflicts(self):
        path = CONFIG / "sensors/realsense.yaml"
        params = sensor_parameters(path, rgb_profile="1280x720x30", infra_profile="0")
        self.assertEqual(params["rgb_camera.color_profile"], "1280x720x30")
        self.assertTrue(params["enable_color"])
        self.assertFalse(params["enable_infra1"])
        self.assertFalse(params["enable_infra2"])
        self.assertEqual(params["depth_module.infra_profile"], "424x240x60")
        for kwargs in ({"rgb_profile": "1280x720x90"},
                       {"rgb_profile": "1280x720x30", "rgb_fps": "30"},
                       {"infra_profile": "0", "infra_fps": "0"}):
            with self.assertRaises(ValueError):
                sensor_parameters(path, **kwargs)

    def test_d455_additional_widescreen_profiles(self):
        from kart_bringup.mission import CAMERA_PROFILES

        path = CONFIG / "sensors/realsense.yaml"
        for resolution in ("480x270", "640x360"):
            for fps in (30, 60, 90):
                profile = f"{resolution}x{fps}"
                with self.subTest(profile=profile):
                    self.assertIn(profile, CAMERA_PROFILES["RGB"])
                    self.assertIn(profile, CAMERA_PROFILES["Infra"])
                    params = sensor_parameters(path, rgb_profile=profile, infra_profile=profile)
                    self.assertEqual(params["rgb_camera.color_profile"], profile)
                    self.assertEqual(params["depth_module.infra_profile"], profile)

    def test_stream_specific_enumerated_profiles(self):
        from kart_bringup.mission import CAMERA_PROFILES, validate_camera_profile
        path = CONFIG / "sensors/realsense.yaml"
        self.assertNotIn("848x480x90", CAMERA_PROFILES["RGB"])
        self.assertIn("848x480x90", CAMERA_PROFILES["Infra"])
        for stream, profiles in CAMERA_PROFILES.items():
            for profile in profiles:
                validate_camera_profile(stream, profile)
        params = sensor_parameters(path, rgb_profile="848x480x60", infra_profile="848x480x90")
        self.assertEqual(params["rgb_camera.color_profile"], "848x480x60")
        self.assertEqual(params["depth_module.infra_profile"], "848x480x90")
        with self.assertRaisesRegex(ValueError, "RGB"):
            sensor_parameters(path, rgb_profile="848x480x90")
        with self.assertRaisesRegex(ValueError, "RGB"):
            sensor_parameters(camera_config(CONFIG, "environment"), rgb_fps="90")
        with tempfile.TemporaryDirectory() as folder:
            import yaml
            config = yaml.safe_load(path.read_text())
            config["/**/realsense"]["ros__parameters"]["rgb_camera.color_profile"] = "848x480x90"
            invalid = Path(folder) / "camera.yaml"
            invalid.write_text(yaml.safe_dump(config))
            with self.assertRaisesRegex(ValueError, "RGB"):
                sensor_parameters(invalid)

    def test_environment_depth_imu_and_recording_contract(self):
        import yaml
        cfg = composition(CONFIG / "mission.yaml", "environment")
        self.assertFalse(any(cfg[k] for k in ("localization", "tracking", "e2e")))
        camera = camera_config(CONFIG, "environment")
        params = sensor_parameters(camera)
        self.assertEqual(params["rgb_camera.color_profile"], "1280x720x30")
        self.assertTrue(params["enable_depth"])
        self.assertEqual(params["depth_module.depth_format"], "Z16")
        self.assertTrue(params["enable_accel"])
        self.assertTrue(params["enable_gyro"])
        self.assertEqual(params["unite_imu_method"], 2)
        changed = sensor_parameters(camera, infra_profile="640x480x60")
        self.assertEqual(changed["depth_module.depth_profile"], "640x480x60")
        self.assertEqual(sensor_parameters(camera, infra_profile="0")["depth_module.depth_profile"], "848x480x30")
        for evs in (False, True):
            bag = yaml.safe_load(recording_config(CONFIG, "environment", evs).read_text())["/**/kart_bag_manager"]["ros__parameters"]
            for topic in ("realsense/depth/image_rect_raw", "realsense/depth/camera_info",
                          "realsense/color/image_raw", "realsense/infra1/image_rect_raw",
                          "realsense/infra2/image_rect_raw", "realsense/imu",
                          "realsense/accel/sample", "realsense/gyro/sample", "/tf_static"):
                self.assertIn(topic, bag["topics"])
            self.assertEqual(bool(bag["raw_recording_driver_node"]), evs)
        normal = sensor_parameters(camera_config(CONFIG, "collect"))
        self.assertFalse(normal["enable_depth"])

    def test_environment_cli_dry_run(self):
        result = subprocess.run([sys.executable, str(ROOT / "scripts/lib/bringup.py"),
                                 "--mode", "environment", "--dry-run"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("mode:=environment", result.stdout)
        self.assertIn("Depth Z16: 848x480x30", result.stdout)
        self.assertIn("bag_manager_environment.yaml", result.stdout)

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
                str(ROOT / "scripts/lib/bringup.py"),
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
                str(ROOT / "scripts/lib/bringup.py"),
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
                    "evs_tensor_config",
                    "rgb_fps",
                    "infra_fps",
                    "map_dir",
                    "model_dir",
                    "map_file",
                    "lane_id",
                    "line_type",
                    "multicam_mode",
                )
            }
            values.update(
                mode="collect",
                rgb_profile="1280x720x30",
                infra_profile="",
                auto_fixed_throttle="0.10",
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
            vehicle_args = lu.include.call_args_list[-1].kwargs['launch_arguments']
            self.assertEqual(vehicle_args['auto_fixed_throttle'], '0.10')
            lu.component_container.assert_not_called()
            lu.include.reset_mock()
            lu.include.reset_mock()
            args.mode = "environment"
            args.imu_profile = "low"
            module.add_mission(args)
            camera_args = lu.include.call_args_list[0].kwargs["launch_arguments"]
            self.assertTrue(camera_args["camera_config"].endswith("realsense_environment.yaml"))
            self.assertEqual(camera_args["imu_profile"], "low")
            vehicle_args = lu.include.call_args_list[1].kwargs["launch_arguments"]
            self.assertTrue(vehicle_args["bag_config"].endswith("bag_manager_environment.yaml"))
            self.assertEqual(len(lu.include.call_args_list), 2)
            lu.include.reset_mock()
            args.mode = "drive"
            args.lane_id = "lane_001"
            args.line_type = "centerline"
            args.multicam_mode = "2"
            with (
                patch.object(
                    module,
                    "resolve",
                    return_value=({"cuvslam": {"tracking_mode": 0}}, {}, {}, {"input_shape": [1, 3, 240, 424]}),
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
            self.assertEqual(localization['multicam_mode'],'2')
            camera_args = lu.include.call_args_list[0].kwargs["launch_arguments"]
            self.assertEqual(camera_args["rgb_profile"], "1280x720x30")
            lu.component_container.assert_not_called()
            lu.include.reset_mock()
            args.mode = "e2e"
            with (
                patch("kart_e2e.contract.routing_settings", return_value=None),
                patch("kart_e2e.contract.model_contract"),
                patch("kart_e2e.contract.input_contract", return_value=("rgb", "image", [1, 3, 120, 212])),
                patch("kart_e2e.contract.runtime_settings"),
                patch("kart_e2e.engine_cache.require_engine"),
            ):
                module.add_mission(args)
            paths = [c.args[1] for c in lu.include.call_args_list]
            self.assertNotIn("launch/localization.launch.py", paths)
            self.assertNotIn("launch/tracking.launch.py", paths)
            inference = lu.include.call_args_list[-1].kwargs["launch_arguments"]
            self.assertEqual(inference["container_name"], "/external/sensors")
            self.assertEqual(inference["create_container"], "false")
            self.assertEqual(inference["drive_enabled"], "true")
            self.assertEqual(inference['use_fixed_throttle'], 'true')
            vehicle_call = next(call for call in lu.include.call_args_list if call.args[1]=='launch/vehicle.launch.py')
            self.assertEqual(vehicle_call.kwargs['launch_arguments']['auto_fixed_throttle'], '-1')

            lu.include.reset_mock()
            args.enable_evs = 'true'
            args.evs_tensor_config = '/model/openeb.yaml'
            args.rgb_profile = ''
            args.rgb_fps = '0'
            with (
                patch('kart_e2e.contract.routing_settings', return_value=None),
                patch('kart_e2e.contract.model_contract'),
                patch('kart_e2e.contract.runtime_settings'),
                patch('kart_e2e.contract.input_contract', return_value=('evs', 'input_tensor', [1,20,120,160])),
                patch('kart_e2e.engine_cache.require_engine'),
                patch('kart_bringup.evs.pipeline_configuration'),
                patch('kart_bringup.e2e.validate_evs_tensor'),
            ):
                module.add_mission(args)
            paths = [c.args[1] for c in lu.include.call_args_list]
            self.assertEqual(paths.count('launch/modules/sensors/openeb.launch.py'), 1)
            self.assertIn('launch/evs_e2e.launch.py', paths)
            self.assertNotIn('launch/e2e.launch.py', paths)
            evs = lu.include.call_args_list[-1].kwargs['launch_arguments']
            self.assertEqual(evs['evs_tensor_config'], '/model/openeb.yaml')
            self.assertNotIn('container_name', evs)
            self.assertNotIn('launch/localization.launch.py', paths)
            self.assertNotIn('launch/tracking.launch.py', paths)
            lu.include.reset_mock()
            args.infra_fps = '0'
            args.create_sensor_container = 'true'
            args.sensor_container = 'kart_sensor_container'
            with (
                patch('kart_e2e.contract.routing_settings', return_value=None),
                patch('kart_e2e.contract.model_contract'),
                patch('kart_e2e.contract.runtime_settings'),
                patch('kart_e2e.contract.input_contract', return_value=('evs', 'input_tensor', [1,20,120,160])),
                patch('kart_e2e.engine_cache.require_engine'),
                patch('kart_bringup.evs.pipeline_configuration'),
                patch('kart_bringup.e2e.validate_evs_tensor'),
            ):
                module.add_mission(args)
            paths = [c.args[1] for c in lu.include.call_args_list]
            self.assertNotIn('launch/modules/sensors/realsense.launch.py', paths)
            lu.component_container.assert_not_called()


if __name__ == "__main__":
    unittest.main()
