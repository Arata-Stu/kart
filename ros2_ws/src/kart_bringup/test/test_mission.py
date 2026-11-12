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
    def test_debug_image_default_off_and_explicit_boolean_override(self):
        self.assertFalse(composition(CONFIG / 'mission.yaml', 'drive')['perception_debug_image'])
        self.assertTrue(composition(CONFIG / 'mission.yaml', 'drive', {'perception_debug_image': 'true'})['perception_debug_image'])
        self.assertFalse(composition(CONFIG / 'mission.yaml', 'drive', {'perception_debug_image': 'false'})['perception_debug_image'])
        with self.assertRaises(ValueError):
            composition(CONFIG / 'mission.yaml', 'drive', {'perception_debug_image': 'yes'})

    def test_perception_mode_preserves_line_tracking_and_checks_values(self):
        for selection in ('detect', 'track'):
            config = composition(CONFIG / 'mission.yaml', 'drive', {'perception_mode': selection})
            self.assertEqual(config['perception_mode'], selection)
            self.assertTrue(config['tracking'], 'object tracking must not disable reference-line following')
        self.assertEqual(composition(CONFIG / 'mission.yaml', 'drive')['perception_mode'], 'track')
        self.assertEqual(composition(CONFIG / 'mission.yaml', 'drive', {'perception_mode': ''})['perception_mode'], 'track')
        with self.assertRaisesRegex(ValueError, 'perception_mode'):
            composition(CONFIG / 'mission.yaml', 'drive', {'perception_mode': 'invalid'})

    def test_depth_profiles_and_shared_stereo_selection(self):
        from kart_bringup.mission import DEPTH_PROFILES
        path = camera_config(CONFIG, 'collect')
        for profile in DEPTH_PROFILES:
            params = sensor_parameters(path, infra_profile='0', depth_profile=profile)
            self.assertTrue(params['enable_depth'])
            self.assertEqual(params['depth_module.depth_profile'], profile)
        self.assertFalse(sensor_parameters(path)['enable_depth'])
        for zero in ('0', 0):
            self.assertFalse(sensor_parameters(camera_config(CONFIG, 'environment'), depth_profile=zero)['enable_depth'])
        params = sensor_parameters(path, depth_profile='640x480x60', infra_profile='640x480x60')
        self.assertEqual(params['depth_module.depth_profile'], params['depth_module.infra_profile'])
        for profile in ('1280x800x30', '1280x720x90', '256x144x30'):
            with self.assertRaises(ValueError):
                sensor_parameters(path, infra_profile='0', depth_profile=profile)
        with self.assertRaisesRegex(ValueError, '不一致'):
            sensor_parameters(path, depth_profile='848x480x30', infra_profile='424x240x90')

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
        self.assertEqual(changed["rgb_camera.color_profile"], "640x480x60")
        self.assertEqual(changed["depth_module.infra_profile"], "640x480x90")
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
        self.assertEqual(params["depth_module.infra_profile"], "640x480x90")
        for kwargs in ({"rgb_profile": "1280x720x90"},
                       {"rgb_profile": "1280x720x30", "rgb_fps": "30"},
                       {"infra_profile": "0", "infra_fps": "0"}):
            with self.assertRaises(ValueError):
                sensor_parameters(path, **kwargs)

    def test_numeric_launch_zero_disables_profiles(self):
        # Isaac ArgumentContainer converts CLI "0" into int 0 before callbacks.
        for filename in ("realsense.yaml", "realsense_environment.yaml"):
            path = CONFIG / "sensors" / filename
            original = sensor_parameters(path)
            for zero in ("0", 0):
                for stream, flags in (("rgb", ("enable_color",)),
                                      ("infra", ("enable_infra1", "enable_infra2"))):
                    with self.subTest(config=filename, zero=zero, stream=stream):
                        params = sensor_parameters(path, **{stream + "_profile": zero})
                        expected = dict(original, **{flag: False for flag in flags})
                        self.assertEqual(params, expected)
                params = sensor_parameters(path, rgb_profile=zero, infra_profile=zero)
                self.assertFalse(params["enable_color"])
                self.assertFalse(params["enable_infra1"])
                self.assertFalse(params["enable_infra2"])
                self.assertEqual(params["enable_depth"], original["enable_depth"])
                self.assertEqual(params["enable_gyro"], original["enable_gyro"])
                self.assertEqual(params, sensor_parameters(path, zero, zero))
            with self.assertRaises(ValueError):
                sensor_parameters(path, rgb_profile=0, rgb_fps=0)

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
        self.assertIn("848x100x100", CAMERA_PROFILES["Infra"])
        self.assertNotIn("848x100x100", CAMERA_PROFILES["RGB"])
        narrow = sensor_parameters(path, rgb_profile="0", infra_profile="848x100x100")
        self.assertEqual(narrow["depth_module.infra_profile"], "848x100x100")
        self.assertTrue(narrow["enable_infra1"] and narrow["enable_infra2"])
        self.assertEqual(narrow["depth_module.infra1_format"], "Y8")
        self.assertEqual(narrow["depth_module.infra2_format"], "Y8")
        with self.assertRaisesRegex(ValueError, "RGB"):
            sensor_parameters(path, rgb_profile="848x100x100")
        self.assertNotIn("848x480x90", CAMERA_PROFILES["RGB"])
        self.assertIn("848x480x90", CAMERA_PROFILES["Infra"])
        self.assertNotIn("640x480x90", CAMERA_PROFILES["RGB"])
        self.assertIn("640x480x90", CAMERA_PROFILES["Infra"])
        for stream, profiles in CAMERA_PROFILES.items():
            for profile in profiles:
                validate_camera_profile(stream, profile)
        params = sensor_parameters(path, rgb_profile="848x480x60", infra_profile="848x480x90")
        self.assertEqual(params["rgb_camera.color_profile"], "848x480x60")
        self.assertEqual(params["depth_module.infra_profile"], "848x480x90")
        params = sensor_parameters(path, rgb_profile="640x480x60", infra_profile="640x480x90")
        self.assertEqual(params["depth_module.infra_profile"], "640x480x90")
        with self.assertRaises(ValueError):
            sensor_parameters(path, rgb_profile="640x480x90")
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
        with self.assertRaisesRegex(ValueError, "不一致"):
            sensor_parameters(camera, infra_profile="640x480x60")
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
                "--no-tuning",
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
    def test_vehicle_model_owns_one_camera_container_and_mapless_odometry(self):
        import importlib.util
        from types import ModuleType, SimpleNamespace
        from unittest.mock import MagicMock, patch
        import yaml

        lu=ModuleType('isaac_ros_launch_utils');lu.__path__=[]
        lu.get_path=lambda pkg,suffix:PACKAGE/suffix
        lu.log_info=lambda text:('log',text)
        lu.component_container=MagicMock(return_value=('container',))
        lu.include=MagicMock(side_effect=lambda pkg,path,**kwargs:(path,kwargs))
        lut=ModuleType('isaac_ros_launch_utils.all_types')
        index=ModuleType('ament_index_python');index.__path__=[]
        packages=ModuleType('ament_index_python.packages')
        packages.get_package_share_directory=lambda name:'/mock/'+name
        with patch.dict(sys.modules,{'isaac_ros_launch_utils':lu,
                'isaac_ros_launch_utils.all_types':lut,'ament_index_python':index,
                'ament_index_python.packages':packages}):
            spec=importlib.util.spec_from_file_location('vehicle_model_mission',PACKAGE/'launch/mission.launch.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            values={key:'' for key in (*module.OVERRIDES,'device','record_dir','run_name',
                'rgb_fps','infra_fps','map_dir','model_dir','map_file','lane_id','line_type',
                'e2e_model_dir','multicam_mode')}
            values.update(mode='collect',record_profile='vehicle-model',enable_bridge='false',
                          infra_profile='424x240x60',imu_profile='low')
            actions=module.add_mission(SimpleNamespace(**values))
            paths=[call.args[1] for call in lu.include.call_args_list]
            self.assertEqual(paths,['launch/modules/sensors/realsense.launch.py',
                'launch/vehicle.launch.py','launch/modules/localization/vslam.launch.py'])
            lu.component_container.assert_called_once_with('kart_sensor_container',container_type='multithreaded')
            camera,vehicle,vslam=[call.kwargs['launch_arguments'] for call in lu.include.call_args_list]
            self.assertEqual(camera['container_name'],vslam['container_name'])
            self.assertEqual(vslam['container_name'],'/kart_sensor_container')
            self.assertEqual(camera['infra_profile'],'424x240x60')
            self.assertTrue(camera['camera_config'].endswith('realsense_vehicle_model.yaml'))
            self.assertTrue(vehicle['bag_config'].endswith('bag_manager_vehicle_model.yaml'))
            self.assertEqual(float(vehicle['auto_fixed_throttle']),.1)
            self.assertEqual(vehicle['auto_fixed_throttle_r2_gate'],'true')
            params=json.loads(vslam['parameters_json'])
            expected=yaml.safe_load((CONFIG/'tuning/cuvslam.yaml').read_text())['/**/visual_slam']['ros__parameters']
            self.assertEqual(params,expected)
            self.assertFalse(params['enable_localization_n_mapping'])
            self.assertFalse(params['publish_map_to_odom_tf'] or params['publish_odom_to_base_tf'])
            remaps=json.loads(vslam['remappings_json'])
            self.assertIn(['/diagnostics','/localization/vslam/diagnostics'],remaps)
            self.assertFalse(any('AUTO request' in str(action) for action in actions))
            self.assertTrue(any('effective VSLAM=' in str(action) for action in actions))
            self.assertLess(next(i for i,action in enumerate(actions) if 'effective VSLAM=' in str(action)),
                            actions.index(('container',)))
            with self.assertRaisesRegex(ValueError,'own its sensor container'):
                module.add_mission(SimpleNamespace(**dict(values,create_sensor_container='false')))
            with self.assertRaisesRegex(ValueError,'saved maps'):
                module.add_mission(SimpleNamespace(**dict(values,map_dir='/unrelated/map')))

    def test_realsense_launch_passes_numeric_off_to_driver(self):
        import importlib.util
        from types import ModuleType, SimpleNamespace
        from unittest.mock import MagicMock, patch

        lu = ModuleType("isaac_ros_launch_utils")
        lu.__path__ = []
        lu.get_path = lambda pkg, suffix: PACKAGE / suffix
        lu.log_info = MagicMock()
        lu.load_composable_nodes = MagicMock()
        lut = ModuleType("isaac_ros_launch_utils.all_types")
        lut.ComposableNode = MagicMock()
        with patch.dict(sys.modules, {"isaac_ros_launch_utils": lu,
                                      "isaac_ros_launch_utils.all_types": lut}):
            spec = importlib.util.spec_from_file_location(
                "realsense_launch_test", PACKAGE / "launch/modules/sensors/realsense.launch.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            args = SimpleNamespace(camera_config="", rgb_fps="", infra_fps="",
                                   rgb_profile=0, infra_profile=0, imu_profile="",
                                   container_name="/sensors")
            module.add_camera(args)
            params = lut.ComposableNode.call_args.kwargs["parameters"][0]
            for flag in ("enable_color", "enable_infra1", "enable_infra2"):
                self.assertIs(params[flag], False)
            self.assertTrue(params["enable_gyro"])

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
                infra_profile="424x240x60",
                depth_profile="424x240x60",
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
            self.assertEqual(lu.include.call_args_list[0].kwargs["launch_arguments"]["depth_profile"], "424x240x60")
            identified = SimpleNamespace(**{**vars(args), 'auto_fixed_throttle_r2_gate': 'true'})
            module.add_mission(identified)
            self.assertEqual(lu.include.call_args_list[-1].kwargs['launch_arguments']['auto_fixed_throttle_r2_gate'], 'true')
            no_stereo = SimpleNamespace(**{**vars(identified), 'infra_profile': '0'})
            with self.assertRaisesRegex(ValueError, 'stereo Infra'):
                module.add_mission(no_stereo)
            no_stereo.auto_fixed_throttle_r2_gate = ''
            with patch.object(module, 'load', return_value=({}, {
                    'joy_manager': {'effective': {'auto_fixed_throttle_r2_gate': True}}})), \
                    self.assertRaisesRegex(ValueError, 'stereo Infra'):
                module.add_mission(no_stereo)
            minimal = SimpleNamespace(**{**vars(identified), 'record_profile': 'vslam-map',
                                        'rgb_profile': '0', 'infra_profile': '640x480x90', 'depth_profile': '0'})
            with self.assertRaisesRegex(ValueError, 'standard record profile'):
                module.add_mission(minimal)
            # The effective YAML setting must be checked even without an explicit gate argument.
            minimal.auto_fixed_throttle_r2_gate = ''
            with patch.object(module, 'load', return_value=({}, {
                    'joy_manager': {'effective': {'auto_fixed_throttle_r2_gate': True}}})), \
                    self.assertRaisesRegex(ValueError, 'standard record profile'):
                module.add_mission(minimal)
            saved = vars(args).copy()
            import tempfile
            with tempfile.NamedTemporaryFile() as engine:
                engine.write(b"fixture")
                engine.flush()
                args.perception_engine = engine.name
                with self.assertRaisesRegex(ValueError, "640x480"):
                    module.add_mission(args)
                args.rgb_profile = "640x480x30"
                lu.include.reset_mock()
                module.add_mission(args)
                camera, perception, _ = lu.include.call_args_list
                self.assertEqual(perception.args[1], "launch/racing_perception.launch.py")
                self.assertEqual(perception.kwargs["launch_arguments"]["target_container"],
                                 camera.kwargs["launch_arguments"]["container_name"])
                self.assertEqual(perception.kwargs['launch_arguments']['tracking_enabled'], 'true')
                self.assertEqual(perception.kwargs['launch_arguments']['debug_image'], 'false')
                args.perception_mode = 'detect'
                args.perception_debug_image = 'true'
                lu.include.reset_mock()
                module.add_mission(args)
                self.assertEqual(lu.include.call_args_list[1].kwargs['launch_arguments']['tracking_enabled'], 'false')
                self.assertEqual(lu.include.call_args_list[1].kwargs['launch_arguments']['debug_image'], 'true')
                lu.component_container.assert_not_called()
            args = SimpleNamespace(**saved)
            args.record_profile = "vslam-map"
            args.rgb_profile = args.depth_profile = ""
            lu.include.reset_mock()
            module.add_mission(args)
            camera_args = lu.include.call_args_list[0].kwargs["launch_arguments"]
            vehicle_args = lu.include.call_args_list[1].kwargs["launch_arguments"]
            self.assertTrue(camera_args["camera_config"].endswith("realsense_vslam_map.yaml"))
            self.assertTrue(vehicle_args["bag_config"].endswith("bag_manager_vslam_map.yaml"))
            self.assertEqual(camera_args["infra_profile"], "424x240x60")
            import yaml
            from kart_bringup.mission import validate_record_profile
            sensors = sensor_parameters(camera_args["camera_config"], infra_profile=camera_args["infra_profile"])
            validate_record_profile("vslam-map", "collect", sensors, False)
            bag = yaml.safe_load(Path(vehicle_args["bag_config"]).read_text())["/**/kart_bag_manager"]["ros__parameters"]
            workflow = json.loads((CONFIG / "mapping/workflow.json").read_text())
            self.assertEqual(set(bag["topics"]), {workflow[k] for k in
                ("left_image", "right_image", "left_info", "right_info", "tf_static")} | {"/tf"})
            self.assertFalse(bag["record_all"])
            self.assertFalse(bag["raw_recording_driver_node"])
            for key in ("enable_color", "enable_depth", "enable_accel", "enable_gyro"):
                self.assertFalse(sensors[key])
                with self.assertRaises(ValueError):
                    validate_record_profile("vslam-map", "collect", dict(sensors, **{key: True}), False)
            with self.assertRaises(ValueError):
                validate_record_profile("vslam-map", "collect", sensors, True)
            with self.assertRaises(ValueError):
                validate_record_profile("vslam-map", "collect", dict(sensors, enable_infra2=False), False)
            args = SimpleNamespace(**saved)
            args.depth_profile = ""
            lu.component_container.assert_not_called()
            lu.include.reset_mock()
            lu.include.reset_mock()
            args.mode = "environment"
            args.infra_profile = ""
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
            args.infra_profile = "424x240x60"
            args.enable_foxglove = "false"
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
            self.assertEqual(localization["vgl_container"], "/external/sensors")
            self.assertEqual(localization["create_vgl_container"], "false")
            self.assertEqual(localization['multicam_mode'],'2')
            camera_args = lu.include.call_args_list[0].kwargs["launch_arguments"]
            self.assertEqual(camera_args["rgb_profile"], "1280x720x30")
            lu.component_container.assert_not_called()
            lu.include.reset_mock()
            args.mode = "e2e"
            with (
                patch("kart_e2e.contract.routing_settings", return_value=None),
                patch("kart_e2e.contract.model_contract", return_value=(None, "steer_throttle")),
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
            self.assertEqual(vehicle_call.kwargs['launch_arguments']['auto_fixed_throttle_r2_gate'], 'false')

            lu.include.reset_mock()
            args.enable_evs = 'true'
            args.evs_tensor_config = '/model/openeb.yaml'
            args.evs_aggregation = 'evt3_runs_grouped_gpu'
            args.evs_image = 'false'
            args.rgb_profile = ''
            args.rgb_fps = '0'
            with (
                patch('kart_e2e.contract.routing_settings', return_value=None),
                patch('kart_e2e.contract.model_contract', return_value=(None, 'steer_throttle')),
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
            sensors = [c.kwargs['launch_arguments'] for c in lu.include.call_args_list
                       if c.args[1] == 'launch/modules/sensors/openeb.launch.py']
            self.assertEqual(sensors[0]['evs_aggregation'], 'evt3_runs_grouped_gpu')
            self.assertEqual(sensors[0]['evs_image'], 'false')
            self.assertNotIn('container_name', evs)
            self.assertNotIn('launch/localization.launch.py', paths)
            self.assertNotIn('launch/tracking.launch.py', paths)
            lu.include.reset_mock()
            args.infra_profile = ''
            args.infra_fps = '0'
            args.create_sensor_container = 'true'
            args.sensor_container = 'kart_sensor_container'
            with (
                patch('kart_e2e.contract.routing_settings', return_value=None),
                patch('kart_e2e.contract.model_contract', return_value=(None, 'steer_throttle')),
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
