"""Vehicle-model profile configuration, without ROS processes or vehicle access."""
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kart_bringup.mission import camera_config, recording_config, sensor_parameters, validate_record_profile
from kart_bringup.recording_profiles import profile_vehicle_overrides, vehicle_model_defaults


CONFIG = ROOT / "config"


def params(path, node):
    return yaml.safe_load(path.read_text())[f"/**/{node}"]["ros__parameters"]


class VehicleModelProfileTests(unittest.TestCase):
    def test_profile_has_complete_sensor_and_recorder_parameters(self):
        camera = camera_config(CONFIG, "collect", "vehicle-model")
        sensors = sensor_parameters(camera)
        baseline = params(CONFIG / "sensors/realsense.yaml", "realsense")
        self.assertEqual(set(sensors), set(baseline))
        self.assertEqual({k for k in sensors if sensors[k] != baseline[k]}, {"enable_color"})
        self.assertFalse(sensors["enable_color"])
        self.assertFalse(sensors["enable_depth"])
        self.assertEqual(sensors["depth_module.infra_profile"], "640x480x90")
        self.assertEqual((sensors["accel_fps"], sensors["gyro_fps"]), (200, 400))
        self.assertEqual(sensors["unite_imu_method"], 2)
        validate_record_profile("vehicle-model", "collect", sensors, False)

        recording = recording_config(CONFIG, "collect", False, "vehicle-model")
        bag = params(recording, "kart_bag_manager")
        self.assertEqual(set(bag), set(params(CONFIG / "recording/bag_manager.yaml", "kart_bag_manager")))
        self.assertFalse(bag["record_all"])
        self.assertEqual(bag["raw_recording_request_topic"], "")
        topics = {"/" + name.lstrip("/") for name in bag["topics"]}
        required = {
            "/teleop/fixed_config", "/teleop/control_cmd", "/vehicle/control_cmd",
            "/operation_mode/state", "/vehicle/active_path", "/diagnostics",
            "/vehicle/trim/state", "/vehicle/output_channels", "/joy",
            "/realsense/infra1/image_rect_raw", "/realsense/infra2/image_rect_raw",
            "/realsense/infra1/camera_info", "/realsense/infra2/camera_info",
            "/realsense/imu", "/realsense/accel/sample", "/realsense/gyro/sample",
            "/realsense/accel/imu_info", "/realsense/gyro/imu_info",
            "/visual_slam/tracking/odometry", "/localization/vslam/diagnostics",
            "/tf", "/tf_static", "/bag/status",
        }
        self.assertTrue(required <= topics, required - topics)
        self.assertEqual(len(topics), len(bag["topics"]))
        self.assertFalse(any(any(word in topic for word in ("/color/", "/depth/", "event_camera", "/e2e/", "/planning/"))
                             for topic in topics))

    def test_profile_defaults_only_fill_empty_values_without_mutation(self):
        self.assertEqual(vehicle_model_defaults(CONFIG), {
            "auto_fixed_throttle": 0.1, "auto_fixed_throttle_r2_gate": True})
        supplied = {"device": "/dev/ttyACM9", "auto_fixed_throttle": "", "auto_fixed_throttle_r2_gate": ""}
        original = dict(supplied)
        result = profile_vehicle_overrides(CONFIG, "vehicle-model", "collect", supplied)
        self.assertEqual(supplied, original)
        self.assertEqual(result, {"device": "/dev/ttyACM9", "auto_fixed_throttle": "0.1",
                                  "auto_fixed_throttle_r2_gate": "true"})
        for throttle in ("0", "0.125", "1"):
            explicit = {"auto_fixed_throttle": throttle, "auto_fixed_throttle_r2_gate": "true"}
            self.assertEqual(profile_vehicle_overrides(CONFIG, "vehicle-model", "collect", explicit), explicit)
        for profile in ("", "standard", "vslam-map"):
            self.assertEqual(profile_vehicle_overrides(CONFIG, profile, "collect", supplied), supplied)

    def test_conflicting_operator_controls_and_unknown_profiles_rejected(self):
        for throttle in ("-1", "-0.1", "1.1", "nan", "inf", "abc", None, True, 0.1):
            with self.subTest(throttle=throttle), self.assertRaises(ValueError):
                profile_vehicle_overrides(CONFIG, "vehicle-model", "collect", {"auto_fixed_throttle": throttle})
        for gate in ("false", "True", "0", None, True):
            with self.subTest(gate=gate), self.assertRaises(ValueError):
                profile_vehicle_overrides(CONFIG, "vehicle-model", "collect", {"auto_fixed_throttle_r2_gate": gate})
        for profile in ("unknown", "vehicle_model"):
            with self.subTest(profile=profile), self.assertRaisesRegex(ValueError, "Unknown record profile"):
                profile_vehicle_overrides(CONFIG, profile, "collect", {})
            with self.assertRaises(ValueError):
                camera_config(CONFIG, "collect", profile)
            with self.assertRaises(ValueError):
                recording_config(CONFIG, "collect", False, profile)
        for mode in ("environment", "drive", "e2e", "eval"):
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, "collect専用"):
                profile_vehicle_overrides(CONFIG, "vehicle-model", mode, {})

    def test_explicit_sensor_overrides_are_validated_after_resolution(self):
        camera = camera_config(CONFIG, "collect", "vehicle-model")
        lower = sensor_parameters(camera, infra_profile="640x480x30", imu_profile="low")
        validate_record_profile("vehicle-model", "collect", lower, False)
        self.assertEqual((lower["accel_fps"], lower["gyro_fps"]), (100, 200))
        for override in ({"rgb_profile": "640x480x30"}, {"depth_profile": "640x480x90"},
                         {"infra_profile": "0"}, {"imu_profile": "off"}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                validate_record_profile("vehicle-model", "collect", sensor_parameters(camera, **override), False)
        sensors = sensor_parameters(camera)
        for changed in ({"enable_infra2": False}, {"enable_accel": False},
                        {"enable_gyro": False}, {"unite_imu_method": 0}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                validate_record_profile("vehicle-model", "collect", dict(sensors, **changed), False)
        with self.assertRaises(ValueError):
            validate_record_profile("vehicle-model", "collect", sensors, True)

    def test_malformed_profile_defaults_rejected(self):
        valid = vehicle_model_defaults(CONFIG)
        variants = [None, [], {}, dict(valid, unexpected=1),
                    dict(valid, auto_fixed_throttle=True), dict(valid, auto_fixed_throttle=float("nan")),
                    dict(valid, auto_fixed_throttle=-1), dict(valid, auto_fixed_throttle=2),
                    dict(valid, auto_fixed_throttle_r2_gate="true"), dict(valid, auto_fixed_throttle_r2_gate=False)]
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "recording").mkdir()
            path = root / "recording/vehicle_model.yaml"
            for document in variants:
                path.write_text(yaml.safe_dump(document))
                with self.subTest(document=document), self.assertRaises(ValueError):
                    vehicle_model_defaults(root)
            path.write_text(yaml.safe_dump(valid))
            self.assertEqual(vehicle_model_defaults(root), valid)

    def test_existing_profile_paths_and_validation_unchanged(self):
        for profile in ("", "standard"):
            self.assertEqual(camera_config(CONFIG, "collect", profile), CONFIG / "sensors/realsense.yaml")
            self.assertEqual(recording_config(CONFIG, "collect", False, profile), CONFIG / "recording/bag_manager.yaml")
        self.assertEqual(camera_config(CONFIG, "environment"), CONFIG / "sensors/realsense_environment.yaml")
        minimal = sensor_parameters(camera_config(CONFIG, "collect", "vslam-map"))
        validate_record_profile("vslam-map", "collect", minimal, False)
        with self.assertRaises(ValueError):
            validate_record_profile("vslam-map", "collect", dict(minimal, enable_accel=True), False)


if __name__ == "__main__":
    unittest.main()
