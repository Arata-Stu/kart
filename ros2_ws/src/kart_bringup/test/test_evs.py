"""Portable EVS selection tests. No camera, ROS runtime or CUDA required."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from kart_bringup.evs import bias_files, bias_selection, pipeline_configuration, AGGREGATIONS

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

    def test_aggregation_profiles_preserve_custom_tensor_contract(self):
        import yaml
        profile=CONFIG/'openeb_tensor_cuda_async_runs_gpu_direct.yaml';original=profile.read_bytes()
        for aggregation in AGGREGATIONS:
            paths,overrides=pipeline_configuration(CONFIG,tensor_config=str(profile),aggregation=aggregation)
            self.assertEqual(paths[1],profile.resolve())
            self.assertEqual(overrides,{'tensor_backend':'cuda_async','event_aggregation':aggregation})
            values=yaml.safe_load(profile.read_text())["/**/event_tensor"]["ros__parameters"]
            self.assertEqual((values['bins'],values['width'],values['height']),(10,160,120))
            self.assertEqual(values['channel_mean'],[0.0])
        self.assertEqual(profile.read_bytes(),original)
        for backend in ['cpu','cuda']:
            with self.assertRaises(ValueError):pipeline_configuration(CONFIG,backend,aggregation='evt3_runs_grouped_gpu')
        with self.assertRaises(ValueError):pipeline_configuration(CONFIG,aggregation='typo')
        for filename in ['openeb_tensor_cuda_async_grouped_gpu.yaml','openeb_tensor_cuda_async_runs_gpu_direct.yaml']:
            self.assertTrue((CONFIG/filename).is_file())

    def test_cli_selects_each_aggregation_and_rejects_conflicts(self):
        for aggregation in AGGREGATIONS:
            result=self.run_cli('--evs-aggregation',aggregation)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('evs_backend:=cuda_async',result.stdout)
            self.assertIn('evs_aggregation:='+aggregation,result.stdout)
        for extra in [('--evs-backend','cpu'),('--evs-backend','cuda'),('--no-evs',)]:
            self.assertNotEqual(self.run_cli('--evs-aggregation','evt3_runs_grouped_gpu',*extra).returncode,0)

    def test_tui_selects_grouped_aggregation(self):
        import contextlib,importlib.util,io
        from unittest.mock import patch
        spec=importlib.util.spec_from_file_location('bringup_tui_grouped',ROOT/'scripts/lib/bringup.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        def choose(title,values,label=str,default=0):
            if title=='EVS decoder':return 'cuda_async'
            if title=='EVS 集計方式':
                self.assertIn('evt3_runs_grouped_gpu',values);return 'evt3_runs_grouped_gpu'
            if title=='EVS bias file':return '@default'
            if title.startswith('EVS（'):return True
            if title=='AUTOスロットル':return 'normal'
            if title.startswith('IMU'):return 'low'
            if title.startswith('RGB'):return '424x240x30'
            if title.upper().startswith('INFRA'):return '424x240x60'
            if title.startswith('Foxglove'):return False
            return 'collect'
        output=io.StringIO()
        with patch.object(sys,'argv',['bringup.py','--no-bridge','--dry-run','--run-name','trial']),patch.object(sys.stdin,'isatty',return_value=True),patch.object(module,'choose',side_effect=choose),contextlib.redirect_stdout(output):module.main()
        self.assertIn('evs_aggregation:=evt3_runs_grouped_gpu',output.getvalue())

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
            answers = iter(("collect", "424x240x30", "424x240x60", "low", True, "cpu", bias, False, "normal"))
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
            module.setup({"evs_backend": "cuda", "evs_serial": "E522", "evs_bias_file": "@default", "evs_tensor_config": "", "evs_aggregation": ""})
        params = node.call_args.kwargs["parameters"]
        self.assertEqual(len(params), 5)
        self.assertEqual(params[-1], {"tensor_backend": "cuda", "serial": "E522", "bias_file": ""})
        self.assertTrue(params[1].endswith("openeb_tensor_cuda.yaml"))
        node.reset_mock()
        custom=str(CONFIG/'openeb_tensor_cuda_async_runs_gpu_direct.yaml')
        module.setup({'evs_backend':'cuda_async','evs_serial':'','evs_bias_file':'',
                      'evs_tensor_config':custom,'evs_aggregation':'evt3_runs_grouped_gpu'})
        params=node.call_args.kwargs['parameters']
        self.assertEqual(params[1],custom)
        self.assertEqual(params[-1],{'tensor_backend':'cuda_async','event_aggregation':'evt3_runs_grouped_gpu'})

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
