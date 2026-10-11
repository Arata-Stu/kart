"""fzf selection contract; no ROS, Docker or vehicle commands."""
import importlib.util
import os
import io
import sys
import json
import tempfile
from contextlib import redirect_stdout
import yaml
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "bringup_tui", Path(__file__).resolve().parents[1] / "lib/bringup.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FzfTests(unittest.TestCase):
    def test_default_duplicate_labels_and_original_path(self):
        values = [Path("/maps/a b"), Path("/maps/a\nb")]
        def run(command, **kwargs):
            self.assertIn("--read0", command)
            self.assertIn("--with-nth=2..", command)
            self.assertNotIn("FZF_DEFAULT_OPTS", kwargs["env"])
            self.assertNotIn("FZF_DEFAULT_OPTS_FILE", kwargs["env"])
            rows = kwargs["input"].split("\0")
            self.assertTrue(rows[0].startswith("1\t"))
            return subprocess.CompletedProcess(command, 0, rows[0] + "\0")
        with patch.object(module.shutil, "which", return_value="/bin/fzf"),              patch.object(module.subprocess, "run", side_effect=run),              patch.dict(os.environ, {"FZF_DEFAULT_OPTS": "--filter=other", "FZF_DEFAULT_OPTS_FILE": "/tmp/options"}):
            self.assertIs(module.choose("Map", values, label=lambda _: "同名", default=1), values[1])

    def test_cancel_and_invalid_output_never_select(self):
        with patch.object(module.shutil, "which", return_value="/bin/fzf"):
            for code in (1, 130, -2):
                with patch.object(module.subprocess, "run", return_value=subprocess.CompletedProcess([], code, "")):
                    with self.assertRaises(KeyboardInterrupt):
                        module.choose("Mode", ["collect", "drive"])
            for code, output in ((2, ""), (0, "not a candidate\0")):
                with patch.object(module.subprocess, "run", return_value=subprocess.CompletedProcess([], code, output)):
                    with self.assertRaises(ValueError):
                        module.choose("Mode", ["collect", "drive"])

    def test_missing_dependency_or_candidates(self):
        with patch.object(module.shutil, "which", return_value=None):
            with self.assertRaisesRegex(ValueError, "apt-get install"):
                module.choose("Mode", ["collect"])
            with self.assertRaisesRegex(ValueError, "候補がありません"):
                module.choose("Map", [])


class FreshDiagnosticTests(unittest.TestCase):
    def test_interactive_fourth_mode_skips_maps_and_models(self):
        config = module.ROOT / "ros2_ws/src/kart_bringup/config"
        flow = json.loads((config / "localization/workflow.json").read_text())
        topics = [flow[k] for k in ("left_image", "right_image", "left_info", "right_info")] + ["/tf_static"]
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / "run"
            bag.mkdir()
            rows = [{"topic_metadata": {"name": t}, "message_count": 10} for t in topics]
            (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {"topics_with_message_count": rows}}))
            titles = []
            def choose(title, values, *args, **kwargs):
                titles.append(title)
                if title == "用途": return "eval"
                if title == "評価するlocalization": return "fresh"
                if title == "追跡方式": return "0"
                if title == "再生rosbag": return values[0]
                self.fail(f"Unexpected selection: {title}")
            output = io.StringIO()
            with patch.object(sys, "argv", ["bringup.py", "--dry-run", "--record-dir", folder]), patch.object(sys.stdin, "isatty", return_value=True), patch.object(module, "choose", side_effect=choose), redirect_stdout(output):
                module.main()
            text = output.getvalue()
            self.assertIn("sim_vslam.launch.py", text)
            self.assertIn("visualize:=true", text)
            self.assertIn("rviz:=true", text)
            self.assertIn("tracking_mode:=0", text)
            self.assertNotIn("map_dir:=", text)
            self.assertEqual(titles, ["用途", "評価するlocalization", "追跡方式", "再生rosbag"])
            with patch.object(sys, "argv", ["bringup.py", "--mode", "eval", "--eval-localization", "fresh", "--bag", str(bag), "--tracking-mode", "1", "--dry-run"]):
                with self.assertRaisesRegex(ValueError, "/realsense/imu"):
                    module.main()


if __name__ == "__main__":
    unittest.main()
