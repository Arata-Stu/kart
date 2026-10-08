import tempfile
import unittest
from pathlib import Path

from kart_bringup.monitoring import is_jetson, should_start


class MonitoringTest(unittest.TestCase):
    def test_jetson_detection_and_arm_notebook(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.assertFalse(is_jetson("arm64", root))
            (root / "etc").mkdir()
            (root / "etc/nv_tegra_release").write_text("L4T")
            self.assertTrue(is_jetson("aarch64", root))
            self.assertFalse(is_jetson("x86_64", root))

    def test_modes(self):
        self.assertTrue(should_start("auto", True))
        self.assertFalse(should_start("auto", False))
        self.assertTrue(should_start("on", False))
        self.assertFalse(should_start("off", True))
        with self.assertRaises(ValueError):
            should_start("typo", True)

    def test_bag_defaults_record_hardware_metrics(self):
        import yaml

        root = Path(__file__).resolve().parents[2]
        for path in (
            root / "kart_bringup/config/recording/bag_manager.yaml",
            root / "kart_bag_manager/config/bag_manager.yaml",
        ):
            topics = yaml.safe_load(path.read_text())["/**/kart_bag_manager"][
                "ros__parameters"
            ]["topics"]
            self.assertEqual(topics.count("/system/jetson/diagnostics"), 1)
