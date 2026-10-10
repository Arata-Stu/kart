import copy
from pathlib import Path
import tempfile
import unittest

import yaml

from kart_bringup.vehicle_tf import load_transforms, publisher_arguments

CONFIG = Path(__file__).resolve().parents[1] / "config/vehicle/transforms.yaml"


class VehicleTfTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "transforms.yaml"
        self.data = yaml.safe_load(CONFIG.read_text())

    def load(self, required=False):
        self.path.write_text(yaml.safe_dump(self.data))
        return load_transforms(self.path, require_camera=required)

    def test_unmeasured_mount_is_not_identity(self):
        self.data["transforms"]["camera_mount"].update(xyz=None, rpy=None)
        self.data["transforms"]["evs_mount"].update(xyz=None, rpy=None)
        ready, missing = self.load()
        self.assertEqual(set(ready), {"rear_axle"})
        self.assertEqual(missing, ["camera_mount", "evs_mount"])
        with self.assertRaisesRegex(ValueError, "camera_link.*未設定"):
            self.load(True)

    def test_configured_mount_and_rpy_order(self):
        mount = self.data["transforms"]["camera_mount"]
        mount.update(xyz=[0.2, 0.01, 0.1], rpy=[0.01, -0.2, 0.3])
        ready, _ = self.load(True)
        args = publisher_arguments(ready["camera_mount"])
        values = dict(zip(args[::2], args[1::2]))
        self.assertEqual(values["--frame-id"], "base_link")
        self.assertEqual(values["--child-frame-id"], "camera_link")
        self.assertEqual(values["--pitch"], "-0.2")
        self.assertEqual(values["--yaw"], "0.3")

    def test_invalid_mount_values_and_optical_ownership(self):
        original = copy.deepcopy(self.data)
        for update in (
            dict(xyz=[float("nan"), 0, 0], rpy=[0, 0, 0]),
            dict(xyz=[True, 0, 0], rpy=[0, 0, 0]),
            dict(xyz=[0, 0], rpy=[0, 0, 0]),
            dict(child="camera_infra1_optical_frame"),
        ):
            self.data = copy.deepcopy(original)
            self.data["transforms"]["camera_mount"].update(update)
            with self.assertRaises(ValueError):
                self.load(True)

    def test_rear_axle_is_definition_not_adjustable_mount(self):
        self.data["transforms"]["rear_axle"]["xyz"] = [0.1, 0, 0]
        with self.assertRaises(ValueError):
            self.load()


if __name__ == "__main__":
    unittest.main()
