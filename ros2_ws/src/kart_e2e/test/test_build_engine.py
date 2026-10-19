import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from kart_e2e.build_engine import build


class EngineTests(unittest.TestCase):
    def test_build_command_report_and_nonempty_engine(self):
        for precision in ("fp32", "fp16", "int8"):
            with self.subTest(precision=precision):
                self.check_build(precision)

    def check_build(self, precision):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binary = root / "trtexec"
            binary.touch()
            (root/"metadata.json").write_text(json.dumps({"quantization":{"format":"QDQ","precision":"int8"}} if precision=="int8" else {}))
            onnx = root / "model.onnx"
            onnx.write_bytes(b"fixture")

            def execute(args, **kwargs):
                self.assertIn("--duration=3", args)
                self.assertEqual("--fp16" in args, precision in ("fp16", "int8"))
                self.assertEqual("--noTF32" in args, precision == "fp32")
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
                patch("kart_e2e.build_engine.input_contract", return_value=("rgb", "image", [1,3,120,212])),
                patch("kart_e2e.build_engine.validate_engine") as validate,
            ):
                report = build(root, root / "output", precision)
                validate.assert_called_once_with(root/"output/model.plan", "steer_only", "image", [1,3,120,212])
            self.assertEqual(report["precision"], precision)
            self.assertEqual(report["io_dtype"], "fp32")
            self.assertEqual(report["engine_size_bytes"], 6)
            self.assertEqual(report["timing_ms"]["computeMs"]["median"], 3)
            self.assertEqual(onnx.read_bytes(), b"fixture")

    def test_arm_is_rejected(self):
        with (
            patch("kart_e2e.build_engine.platform.machine", return_value="aarch64"),
            self.assertRaises(ValueError),
        ):
            build(Path("/unused"), Path("/unused"))

    def test_unknown_precision_is_rejected(self):
        with self.assertRaises(ValueError):
            build(Path('/unused'), Path('/unused'), 'bf16')
