"""Host GID discovery without hardware, Docker, or permission changes."""
import importlib.util
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "device_args", Path(__file__).resolve().parents[1] / "input-dockerargs.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class DeviceGroupsTests(unittest.TestCase):
    def setUp(self):
        env = patch.dict(module.os.environ, {"SSH_AUTH_SOCK": ""})
        env.start()
        self.addCleanup(env.stop)

    def test_serial_input_and_standard_groups_are_deduplicated(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            inputs = root / "input"
            inputs.mkdir()
            base, output = root / "base", root / "output"
            base.write_text("-e EXISTING=yes\n")
            owners = {"event0": 996, "ttyACM0": 20, "ttyUSB0": 42}
            for name in (*owners, "ttyUSB_regular"):
                ((inputs if name.startswith("event") else root) / name).touch()
            original_stat = Path.stat

            def device_stat(path, *args, **kwargs):
                if path.name in owners:
                    return SimpleNamespace(st_mode=stat.S_IFCHR | 0o660, st_gid=owners[path.name])
                return original_stat(path, *args, **kwargs)

            with patch.object(module.grp, "getgrnam", side_effect=lambda name: SimpleNamespace(
                    gr_gid={"input": 996, "dialout": 20, "plugdev": 46}[name])), patch.object(
                    Path, "stat", device_stat), patch.object(module.platform, "machine", return_value="aarch64"):
                gids = module.write_args(base, output, inputs, root, root / 'no-ssh')
            self.assertEqual(gids, {20, 42, 46, 996})
            self.assertEqual(output.read_text().splitlines(),
                             ["-e EXISTING=yes", "--group-add 20", "--group-add 42", "--group-add 46", "--group-add 996"])

    def test_ssh_directory_and_notebook_agent(self):
        import shlex
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ssh = root / "host home/.ssh"
            ssh.mkdir(parents=True)
            (ssh / "known_hosts").write_text("fixture")
            base, output = root / "base", root / "output"
            base.write_text("")
            with patch.object(module.grp, "getgrnam", side_effect=KeyError), patch.object(
                module.platform, "machine", return_value="x86_64"
            ), patch.dict(module.os.environ, {"SSH_AUTH_SOCK": "/tmp/test agent"}), patch.object(
                Path, "is_socket", return_value=True
            ):
                module.write_args(base, output, root / "missing", root, ssh)
            lines = output.read_text().splitlines()
            args = [shlex.split(line) for line in lines if line]
            self.assertIn(["--mount", f"type=bind,source={ssh.resolve()},target=/home/admin/.ssh,readonly"], args)
            self.assertIn(["--mount", "type=bind,source=/tmp/test agent,target=/kart-ssh-agent"], args)
            self.assertIn(["-e", "SSH_AUTH_SOCK=/kart-ssh-agent"], args)
            self.assertEqual((ssh / "known_hosts").read_text(), "fixture")

    def test_missing_groups_and_devices(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            base, output = root / "base", root / "output"
            base.write_text("-e EXISTING=yes\n")
            with patch.object(module.grp, "getgrnam", side_effect=KeyError):
                self.assertEqual(module.write_args(base, output, root / "missing", root, root / "no-ssh"), set())
            self.assertNotIn("--group-add", output.read_text())


if __name__ == "__main__":
    unittest.main()
