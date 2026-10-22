"""Portable metadata collection tests; no live ROS or DDS claim."""
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.pipeline_audit import ros_snapshot as audit


class SnapshotTests(unittest.TestCase):
    def test_udp_missing_and_reset_are_unknown(self):
        self.assertIsNone(audit.udp_counters(None))
        self.assertIsNone(audit.udp_counters("Udp: A B\nUdp: 1\n"))
        self.assertIsNone(audit.counter_delta(None, {"InErrors": 1}))
        self.assertIsNone(audit.counter_delta({"InErrors": 5}, {"InErrors": 1}))
        before = audit.udp_counters("Ip: Other\nUdp: InErrors RcvbufErrors\nUdp: 4 2\n")
        self.assertEqual(audit.counter_delta(before, {"InErrors": 7, "RcvbufErrors": 3}),
                         {"InErrors": 3, "RcvbufErrors": 1})

    def test_capture_only_uses_metadata_commands_and_preserves_failures(self):
        def run(argv):
            return dict(argv=argv, returncode=1 if "param" in argv else 0, stdout="", stderr="")

        with patch.object(audit, "command", side_effect=run) as calls, \
             patch.object(audit, "read", return_value=None), \
             patch.object(audit.time, "sleep"):
            result = audit.capture(["/image"], ["/camera"], 0)
        self.assertFalse(result["metadata_complete"])
        self.assertIsNone(result["udp"]["delta"])
        for call in calls.call_args_list:
            argv = call.args[0]
            if argv[0] == "ros2":
                self.assertIn(argv[1:3], (["node", "list"], ["topic", "list"],
                                        ["topic", "info"], ["param", "dump"]))
        self.assertEqual(result["parameters"]["/camera"]["returncode"], 1)

    def test_missing_tools_and_timeout_are_not_success(self):
        for error in (FileNotFoundError("ros2"), subprocess.TimeoutExpired(["ros2"], 10)):
            with patch.object(audit.subprocess, "run", side_effect=error):
                self.assertIsNone(audit.command(["ros2", "node", "list"])["returncode"])

    def test_cli_overrides_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "capture.json"
            argv = ["ros_snapshot", "--output", str(output), "--topic", "/camera/image",
                    "--sample-seconds", "0"]
            with patch("sys.argv", argv), patch.object(audit, "capture", return_value={"metadata_complete": False}) as capture:
                self.assertEqual(audit.main(), 2)
                self.assertEqual(capture.call_args.args[0], ["/camera/image"])
                with self.assertRaises(FileExistsError):
                    audit.main()
                self.assertEqual(capture.call_count, 1)
