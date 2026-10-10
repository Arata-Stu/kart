"""fzf selection contract; no ROS, Docker or vehicle commands."""
import importlib.util
import os
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


if __name__ == "__main__":
    unittest.main()
