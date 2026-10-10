"""Portable EVS selection tests. No camera, ROS runtime or CUDA required."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from kart_bringup.evs import bias_files, bias_selection, pipeline_configuration

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
CONFIG = PACKAGE / "config/sensors"


class EvsTests(unittest.TestCase):
    def test_backend_profiles_and_packet_off(self):
        import yaml
        for backend in ("cpu", "cuda", "cuda_async"):
            paths, overrides = pipeline_configuration(CONFIG, backend)
            self.assertEqual(paths[1].name, f"openeb_tensor_{backend}.yaml")
            self.assertEqual(overrides, {"tensor_backend": backend})
        driver = yaml.safe_load((CONFIG / "openeb.yaml").read_text())["/**/event_camera_driver"]["ros__parameters"]
        self.assertFalse(driver["packet_publish_enabled"])
        self.assertFalse(driver["raw_recording_auto_start"])
        with self.assertRaises(ValueError):
            pipeline_configuration(CONFIG, "unknown")

    def test_bias_default_preserve_and_validation(self):
        self.assertIsNone(bias_selection(""))
        self.assertEqual(bias_selection("@default"), "")
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder).resolve()
            good = root / "day light.bias"
            good.write_text("178 % bias_diff_off\n")
            (root / "settings.json").write_text("{}")
            (root / ".hidden").mkdir()
            (root / ".hidden/old.bias").write_text("1")
            self.assertEqual(bias_files(root), [good])
            _, overrides = pipeline_configuration(CONFIG, "cpu", "E522", str(good))
            self.assertEqual(overrides["bias_file"], str(good))
            self.assertEqual(overrides["serial"], "E522")
            for path in (root / "missing.bias", root / "settings.json"):
                with self.assertRaises(ValueError):
                    bias_selection(str(path))
            good.write_text("")
            with self.assertRaises(ValueError):
                bias_selection(str(good))

    def run_cli(self, *args):
        return subprocess.run([sys.executable, str(ROOT / "scripts/lib/bringup.py"),
                               "--mode", "collect", "--no-bridge", "--dry-run", *args],
                              text=True, capture_output=True, env=os.environ.copy())

    def test_cli_bias_with_spaces_and_backend(self):
        with tempfile.TemporaryDirectory() as folder:
            bias = Path(folder).resolve() / "day light.bias"
            bias.write_text("178 % bias_diff_off\n")
            for backend in ("cpu", "cuda", "cuda_async"):
                result = self.run_cli("--evs", "--evs-backend", backend,
                                      "--evs-bias-file", str(bias), "--evs-serial", "E522")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("enable_evs:=true", result.stdout)
                self.assertIn(f"evs_backend:={backend}", result.stdout)
                self.assertIn(f"evs_bias_file:={bias}", result.stdout)
                self.assertIn("packet topic OFF", result.stdout)

    def test_tui_selects_backend_and_bias(self):
        import contextlib
        import importlib.util
        import io
        from unittest.mock import patch
        spec = importlib.util.spec_from_file_location("bringup_tui", ROOT / "scripts/lib/bringup.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder:
            bias = Path(folder).resolve() / "indoor.bias"
            bias.write_text("178 % bias_diff_off\n")
            answers = iter(("collect", "30", "60", True, "cpu", bias, False))
            def choose(title, values, label=str, default=0):
                print(title)
                return next(answers)
            output = io.StringIO()
            with patch.object(sys, "argv", ["bringup.py", "--no-bridge", "--dry-run",
                                            "--run-name", "trial", "--bias-root", folder]), \
                 patch.object(sys.stdin, "isatty", return_value=True), \
                 patch.object(module, "choose", side_effect=choose), \
                 contextlib.redirect_stdout(output):
                module.main()
            self.assertIn("EVS decoder", output.getvalue())
            self.assertIn("evs_backend:=cpu", output.getvalue())
            self.assertIn(f"evs_bias_file:={bias}", output.getvalue())

    def test_launch_applies_explicit_bias_after_all_yaml(self):
        import importlib.util
        import types
        from unittest.mock import Mock, patch
        class Configuration:
            def __init__(self, key):
                self.key = key
            def perform(self, context):
                return context[self.key]
        modules = {name: types.ModuleType(name) for name in (
            "ament_index_python", "ament_index_python.packages", "launch",
            "launch.actions", "launch.substitutions", "launch_ros", "launch_ros.actions")}
        modules["ament_index_python.packages"].get_package_share_directory = lambda _: str(PACKAGE)
        modules["launch"].LaunchDescription = list
        for name in ("DeclareLaunchArgument", "LogInfo", "OpaqueFunction"):
            setattr(modules["launch.actions"], name, Mock())
        modules["launch.substitutions"].LaunchConfiguration = Configuration
        node = Mock()
        modules["launch_ros.actions"].Node = node
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location("evs_launch_test", PACKAGE / "launch/modules/sensors/openeb.launch.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.setup({"evs_backend": "cuda", "evs_serial": "E522", "evs_bias_file": "@default"})
        params = node.call_args.kwargs["parameters"]
        self.assertEqual(len(params), 5)
        self.assertEqual(params[-1], {"tensor_backend": "cuda", "serial": "E522", "bias_file": ""})
        self.assertTrue(params[1].endswith("openeb_tensor_cuda.yaml"))

    def test_cli_conflict_and_missing_bias_rejected(self):
        result = self.run_cli("--evs", "--no-evs")
        self.assertNotEqual(result.returncode, 0)
        result = self.run_cli("--evs-bias-file", "/missing.bias")
        self.assertNotEqual(result.returncode, 0)
        result = self.run_cli("--evs", "--evs-bias-file", "@default")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("evs_bias_file:=@default", result.stdout)


if __name__ == "__main__":
    unittest.main()
