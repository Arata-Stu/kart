"""Verify entrypoint group registration without modifying host users/groups."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / "docker/scripts/kart-device-groups.sh"


class EntrypointGroupsTests(unittest.TestCase):
    def run_extension(self, groups, fail=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / "calls"
            commands = {
                "id": '#!/bin/bash\nif [[ "$1" == -u ]]; then echo 0; else echo "$TEST_GROUPS"; fi\n',
                "getent": '#!/bin/bash\n[[ "$2" != 996 ]]\n',
                "groupadd": '#!/bin/bash\nprintf "groupadd %s\\n" "$*" >> "$TEST_LOG"\n',
                "usermod": '#!/bin/bash\nprintf "usermod %s\\n" "$*" >> "$TEST_LOG"\nexit "$TEST_FAIL"\n',
            }
            for name, text in commands.items():
                path = root / name
                path.write_text(text)
                path.chmod(0o755)
            env = {**os.environ, "PATH": str(root) + os.pathsep + os.environ["PATH"],
                   "USERNAME": "admin", "TEST_GROUPS": groups, "TEST_LOG": str(log),
                   "TEST_FAIL": "1" if fail else "0"}
            result = subprocess.run(["bash", "-c", 'source "$1"; echo CONTINUED', "_", str(SCRIPT)],
                                    env=env, capture_output=True, text=True)
            return result, log.read_text() if log.exists() else ""

    def test_register_numeric_groups_and_create_missing_without_root(self):
        result, calls = self.run_extension("0 20 46 996")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(calls.splitlines(), [
            "usermod --append --groups 20 admin",
            "usermod --append --groups 46 admin",
            "groupadd --gid 996 kart_device_996",
            "usermod --append --groups 996 admin",
        ])
        self.assertIn("CONTINUED", result.stdout)

    def test_registration_error_stops_entrypoint(self):
        result, _ = self.run_extension("0 20", fail=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("CONTINUED", result.stdout)

    def test_root_only_adds_no_groups(self):
        result, calls = self.run_extension("0")
        self.assertEqual(result.returncode, 0)
        self.assertEqual(calls, "")


if __name__ == "__main__":
    unittest.main()
