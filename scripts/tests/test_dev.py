"""Test remembered Docker selection with a fake CLI; never starts Docker."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "dev.sh"


class DevProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "scripts").mkdir()
        shutil.copyfile(SCRIPT, self.root / "scripts/dev.sh")
        tools = self.root / "bin"
        tools.mkdir()
        for name, content in {
            "uname": '#!/bin/sh\necho Darwin\n',
            "isaac-ros": '#!/bin/sh\nprintf "%s\\n" "$@"\n',
        }.items():
            p = tools / name
            p.write_text(content)
            p.chmod(0o755)
        self.env = dict(os.environ, PATH=str(tools) + os.pathsep + os.environ["PATH"])
        self.state = self.root / ".kart-dev-profile"

    def run_dev(self, *args):
        return subprocess.run(["bash", str(self.root / "scripts/dev.sh"), *args],
                              env=self.env, text=True, capture_output=True)

    def plugin(self):
        for part in ("hal", "hal_psee_plugins", "licensing"):
            p = self.root / "docker/silky_evcam_plugin_source" / part
            p.mkdir(parents=True)
            (p / "source.txt").write_text("test fixture")

    def test_first_build_then_plain_dev_remembers_evs(self):
        self.plugin()
        first = self.run_dev("--evs", "--build-local")
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertIn("--build-local", first.stdout)
        self.assertEqual(self.state.read_text().strip(), "evs")
        shutil.rmtree(self.root / "docker")
        second = self.run_dev()
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertIn("docker.run.container_name=kart_evs_dev", second.stdout)
        self.assertIn("[realsense,openeb,kart]", second.stdout)
        self.assertNotIn("--build-local", second.stdout)

    def test_standard_selection_is_remembered(self):
        self.state.write_text("evs\n")
        explicit = self.run_dev("--no-evs")
        self.assertEqual(explicit.returncode, 0, explicit.stderr)
        self.assertEqual(self.state.read_text().strip(), "standard")
        self.assertNotIn("kart_evs_dev", self.run_dev().stdout)

    def test_missing_build_inputs_do_not_save_selection(self):
        result = self.run_dev("--evs", "--build-local")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.state.exists())

    def test_help_and_default_do_not_save(self):
        for args in ((), ("--evs", "--help")):
            result = self.run_dev(*args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(self.state.exists())


if __name__ == "__main__":
    unittest.main()
