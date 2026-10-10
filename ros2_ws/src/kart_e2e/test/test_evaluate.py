import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(
    all(importlib.util.find_spec(x) for x in ("torch", "onnx", "onnxruntime", "PIL")),
    "Offline inference dependencies required",
)
class EvaluationTests(unittest.TestCase):
    def test_real_onnx_both_modes_and_throttle_metadata(self):
        import numpy as np
        import onnx
        from onnx import helper, TensorProto, numpy_helper
        from PIL import Image
        from kart_e2e.contract import SPEC
        from kart_e2e.evaluate import evaluate

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            data = root / "dataset"
            data.mkdir()
            Image.new("RGB", (424, 240), (100, 120, 150)).save(data / "image.png")
            (data / "samples.jsonl").write_text(
                "\n".join(
                    json.dumps(
                        dict(
                            image="image.png",
                            stamp_ns=1_000_000_000 + i * 100_000_000,
                            steering=0.1,
                            throttle=0.15,
                        )
                    )
                    for i in range(3)
                )
            )
            (data / "metadata.json").write_text('{"samples":3,"skipped":2}')
            for mode in ("steer_only", "steer_throttle"):
                model = root / mode
                model.mkdir()
                values = np.array(
                    [[0.3]] if mode == "steer_only" else [[0.3, 0.8]], dtype=np.float32
                )
                graph = helper.make_graph(
                    [
                        helper.make_node(
                            "ReduceMean",
                            ["image"],
                            ["mean"],
                            axes=[1, 2, 3],
                            keepdims=0,
                        ),
                        helper.make_node("Mul", ["mean", "zero"], ["neutral"]),
                        helper.make_node("Add", ["neutral", "values"], ["control"]),
                    ],
                    "fixture",
                    [
                        helper.make_tensor_value_info(
                            "image", TensorProto.FLOAT, [1, 3, 120, 212]
                        )
                    ],
                    [
                        helper.make_tensor_value_info(
                            "control", TensorProto.FLOAT, list(values.shape)
                        )
                    ],
                    [
                        numpy_helper.from_array(
                            np.array([0], dtype=np.float32), "zero"
                        ),
                        numpy_helper.from_array(values, "values"),
                    ],
                )
                artifact = helper.make_model(
                    graph, opset_imports=[helper.make_opsetid("", 17)]
                )
                artifact.ir_version = 8
                onnx.save(artifact, str(model / "model.onnx"))
                (model / "metadata.json").write_text(
                    json.dumps(
                        dict(
                            schema=1,
                            model_spec=SPEC,
                            output_mode=mode,
                            input_name="image",
                            input_shape=[1, 3, 120, 212],
                            output_name="control",
                            output_shape=list(values.shape),
                            onnx_sha256=hashlib.sha256(
                                (model / "model.onnx").read_bytes()
                            ).hexdigest(),
                            runtime=dict(fixed_throttle=0.12, max_throttle=0.2),
                        )
                    )
                )
                output = root / (mode + "-result")
                report = evaluate(model, data, output)
                self.assertEqual(report["samples"], 3)
                self.assertAlmostEqual(
                    report["metrics"]["steering_command"]["mae"], 0.2, places=6
                )
                self.assertAlmostEqual(
                    report["preview"][0]["throttle_command"],
                    0.12 if mode == "steer_only" else 0.2,
                )
                self.assertEqual(
                    len((output / "predictions.csv").read_text().splitlines()), 4
                )
                json.loads((output / "report.json").read_text())
