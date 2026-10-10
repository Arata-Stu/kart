import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from kart_e2e.build_engine import build


class EngineTests(unittest.TestCase):
    def test_build_command_report_and_nonempty_engine(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binary = root / "trtexec"
            binary.touch()
            onnx = root / "model.onnx"
            onnx.write_bytes(b"fixture")

            def execute(args, **kwargs):
                self.assertIn("--duration=3", args)
                self.assertFalse(any("fp16" in arg for arg in args))
                engine = Path(
                    next(
                        a.split("=", 1)[1]
                        for a in args
                        if a.startswith("--saveEngine=")
                    )
                )
                engine.write_bytes(b"engine")
                timings = Path(
                    next(
                        a.split("=", 1)[1]
                        for a in args
                        if a.startswith("--exportTimes=")
                    )
                )
                timings.write_text(
                    json.dumps(
                        [
                            {"computeMs": 2.0, "latencyMs": 3.0},
                            {"computeMs": 4.0, "latencyMs": 5.0},
                        ]
                    )
                )

            with (
                patch("kart_e2e.build_engine.platform.machine", return_value="x86_64"),
                patch(
                    "kart_e2e.build_engine.model_contract",
                    return_value=({"model_file_path": str(onnx)}, "steer_only"),
                ),
                patch(
                    "kart_e2e.build_engine.shutil.which",
                    side_effect=lambda name: str(binary) if name == "trtexec" else None,
                ),
                patch("kart_e2e.build_engine.subprocess.run", side_effect=execute),
            ):
                report = build(root, root / "output")
            self.assertEqual(report["engine_size_bytes"], 6)
            self.assertEqual(report["timing_ms"]["computeMs"]["median"], 3)
            self.assertEqual(onnx.read_bytes(), b"fixture")

    def test_arm_is_rejected(self):
        with (
            patch("kart_e2e.build_engine.platform.machine", return_value="aarch64"),
            self.assertRaises(ValueError),
        ):
            build(Path("/unused"), Path("/unused"))
