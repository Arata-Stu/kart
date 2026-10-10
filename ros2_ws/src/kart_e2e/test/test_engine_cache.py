import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_e2e.engine_cache import digest, reusable


class EngineCacheTests(unittest.TestCase):
    def test_cache_requires_matching_hashes_hardware_and_loadability(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            model, engine = root / "model.onnx", root / "model.plan"
            model.write_bytes(b"onnx")
            engine.write_bytes(b"plan")
            hardware = {"gpu_uuid": "test", "tensorrt": "10.0"}
            manifest = {
                "schema": 1,
                "hardware": hardware,
                "onnx_sha256": digest(model),
                "engine_sha256": digest(engine),
            }
            metadata = engine.with_suffix(".json")
            with (
                patch(
                    "kart_e2e.engine_cache.model_contract",
                    return_value=(
                        {
                            "model_file_path": str(model),
                            "engine_file_path": str(engine),
                        },
                        "steer_only",
                    ),
                ),
                patch("kart_e2e.engine_cache.fingerprint", return_value=hardware),
                patch("kart_e2e.engine_cache.validate_engine") as validate,
            ):
                self.assertFalse(reusable(root))
                metadata.write_text(json.dumps(manifest))
                self.assertTrue(reusable(root))
                validate.assert_called_once_with(engine, "steer_only")
                for field, value in [
                    ("hardware", {"gpu_uuid": "other"}),
                    ("onnx_sha256", "other"),
                    ("engine_sha256", "other"),
                ]:
                    metadata.write_text(json.dumps(dict(manifest, **{field: value})))
                    self.assertFalse(reusable(root), field)
                metadata.write_text(json.dumps(manifest))
                validate.side_effect = ValueError("wrong binding")
                self.assertFalse(reusable(root))


class EnginePreflightTests(unittest.TestCase):
    def test_missing_engine_stops_before_launch(self):
        from unittest.mock import patch
        from kart_e2e.engine_cache import require_engine

        with patch("kart_e2e.engine_cache.reusable", return_value=False):
            with self.assertRaisesRegex(ValueError, "e2e_trt.sh"):
                require_engine("model")
        with patch("kart_e2e.engine_cache.reusable", return_value=True):
            require_engine("model")
