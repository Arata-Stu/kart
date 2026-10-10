import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import lab


class LabTest(unittest.TestCase):
    def setUp(self):
        preflight = patch.object(lab, "gpu_preflight", return_value={})
        preflight.start()
        self.addCleanup(preflight.stop)

    def test_stage_reference_preserves_bytes_and_records_build_manifest(self):
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(lab, "ARTIFACTS", Path(temporary)),
        ):
            reference = Path(temporary) / "reference.onnx"
            reference.write_bytes(b"verified reference")
            with patch.object(lab, "REFERENCE_SHA", lab.digest(reference)):
                args = types.SimpleNamespace(name="official", reference=reference)
                lab.stage_reference(args)
                directory, data = lab.manifest("official")
                self.assertEqual(
                    (directory / "aliked.onnx").read_bytes(), reference.read_bytes()
                )
                self.assertEqual((data["width"], data["height"]), (1920, 1200))
                with self.assertRaisesRegex(ValueError, "already exists"):
                    lab.stage_reference(args)

    def test_stage_reference_rejects_unknown_model_before_creating_run(self):
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(lab, "ARTIFACTS", Path(temporary)),
        ):
            reference = Path(temporary) / "reference.onnx"
            reference.write_bytes(b"wrong model")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                lab.stage_reference(
                    types.SimpleNamespace(name="official", reference=reference)
                )
            self.assertFalse(lab.run_dir("official").exists())

    def test_exporter_uses_relocated_scalar_api(self):
        class ScalarType:
            from_value = staticmethod(lambda value: value)
            from_dtype = staticmethod(lambda dtype: dtype)
            onnx_type = lambda self: None
            dtype = lambda self: None

        exporter = types.SimpleNamespace(JitScalarType=None)
        module = types.SimpleNamespace(JitScalarType=ScalarType)
        with patch.object(
            lab.importlib, "import_module", return_value=module
        ) as imported:
            result = lab.configure_exporter_types(exporter)
        imported.assert_called_once_with(
            "torch.onnx._internal.torchscript_exporter._type_utils"
        )
        self.assertIs(exporter.JitScalarType, ScalarType)
        self.assertEqual(result, imported.call_args.args[0])

    def test_exporter_preserves_existing_scalar_api(self):
        original = object()
        exporter = types.SimpleNamespace(JitScalarType=original)
        with patch.object(lab.importlib, "import_module") as imported:
            self.assertEqual(lab.configure_exporter_types(exporter), "vendor")
        imported.assert_not_called()
        self.assertIs(exporter.JitScalarType, original)

    def test_exporter_rejects_incomplete_scalar_api(self):
        exporter = types.SimpleNamespace(JitScalarType=None)
        with patch.object(
            lab.importlib,
            "import_module",
            return_value=types.SimpleNamespace(JitScalarType=object),
        ):
            with self.assertRaisesRegex(ValueError, "missing from_value"):
                lab.configure_exporter_types(exporter)
        self.assertIsNone(exporter.JitScalarType)

    def test_paths_and_invalid_sizes(self):
        for name in ("../production", "/opt/ros", ".", "a/b", ""):
            with self.assertRaises(ValueError):
                lab.run_dir(name)
        self.assertEqual(lab.run_dir("424x240-v2").name, "424x240-v2")
        with self.assertRaises(argparse.ArgumentTypeError):
            lab.positive("0")

    def test_manifest_rejects_replaced_model(self):
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(lab, "ARTIFACTS", Path(temporary)),
        ):
            directory = lab.run_dir("test")
            directory.mkdir(parents=True)
            model = directory / "aliked.onnx"
            model.write_bytes(b"original model")
            lab.save(directory / "manifest.json", {"onnx_sha256": lab.digest(model)})
            lab.manifest("test")
            model.write_bytes(b"old model accidentally copied here")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                lab.manifest("test")

    def test_existing_export_is_not_overwritten_before_importing_torch(self):
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(lab, "ARTIFACTS", Path(temporary)),
        ):
            directory = lab.run_dir("424x240")
            directory.mkdir(parents=True)
            model = directory / "aliked.onnx"
            model.write_bytes(b"keep")
            args = types.SimpleNamespace(name=None, width=424, height=240)
            with self.assertRaisesRegex(ValueError, "already exists"):
                lab.export(args)
            self.assertEqual(model.read_bytes(), b"keep")

    def test_engine_inspection_rejects_old_shape(self):
        # Exercise the guard that was missing in the earlier profile-only exporter.
        shape = [1, 3, 1200, 1920]

        class Logger:
            WARNING = 1

            def __init__(self, level):
                pass

        class Engine:
            num_io_tensors = 1
            device_memory_size_v2 = 100

            def get_tensor_name(self, index):
                return "image"

            def get_tensor_shape(self, name):
                return shape

            def get_tensor_mode(self, name):
                return "INPUT"

        class Runtime:
            def __init__(self, logger):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

            def deserialize_cuda_engine(self, content):
                return Engine()

        fake = types.SimpleNamespace(
            Logger=Logger,
            Runtime=Runtime,
            __version__="test",
            init_libnvinfer_plugins=lambda *args: None,
        )
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.dict(sys.modules, {"tensorrt": fake}),
        ):
            path = Path(temporary) / "aliked.engine"
            path.write_bytes(b"fake engine")
            with self.assertRaisesRegex(ValueError, "Engine input size mismatch"):
                lab.engine_info(path, 424, 240)
            shape[:] = [1, 3, 240, 424]
            result = lab.engine_info(path, 424, 240)
            self.assertEqual(result["context_memory_bytes"], 100)
            self.assertEqual(result["tensors"][0]["shape"], shape)

    def test_prepare_rejects_changed_source_without_network(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary)
            for relative in lab.FILES:
                path = source / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"original")
            hashes = {p: lab.digest(source / p) for p in lab.FILES}
            lab.save(
                source / "source-lock.json", {"commit": lab.COMMIT, "files": hashes}
            )
            (source / "nets/aliked.py").write_bytes(b"modified")
            with (
                patch.object(lab, "SOURCE", source),
                patch("urllib.request.urlopen") as network,
            ):
                with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                    lab.prepare(types.SimpleNamespace(source_only=True))
                network.assert_not_called()

    def test_build_reuses_verified_runtime_without_exporters(self):
        with (
            tempfile.TemporaryDirectory() as tmp,
            patch.object(lab, "ARTIFACTS", Path(tmp)),
        ):
            directory = lab.run_dir("small")
            (directory / "runtime_models").mkdir(parents=True)
            (directory / "aliked.onnx").write_bytes(b"onnx")
            lab.save(
                directory / "manifest.json",
                dict(
                    width=424,
                    height=240,
                    onnx_sha256=lab.digest(directory / "aliked.onnx"),
                ),
            )
            with (
                patch.object(lab, "verify_runtime") as verify,
                patch.object(lab.subprocess, "run") as run,
            ):
                lab.build(types.SimpleNamespace(name="small"))
                verify.assert_called_once()
                run.assert_not_called()
            with (
                patch.object(
                    lab, "verify_runtime", side_effect=ValueError("GPU changed")
                ),
                patch.object(lab.subprocess, "run") as run,
            ):
                with self.assertRaisesRegex(ValueError, "GPU changed"):
                    lab.build(types.SimpleNamespace(name="small"))
                run.assert_not_called()

    def test_runtime_cache_rejects_hardware_and_engine_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            target = directory / "runtime_models/aliked_lightglue"
            target.mkdir(parents=True)
            (target / "aliked.onnx").write_bytes(b"onnx")
            (target / "aliked_test.engine").write_bytes(b"extractor")
            (target / "lightglue_aliked_test.engine").write_bytes(b"matcher")
            data = dict(
                width=424, height=240, onnx_sha256=lab.digest(target / "aliked.onnx")
            )
            with (
                patch("kart_e2e.engine_cache.fingerprint", return_value={"gpu": "A"}),
                patch.object(lab, "inspect_engines"),
            ):
                lab.verify_runtime(directory, data)
                lab.verify_runtime(directory, data)
                (target / "aliked_test.engine").write_bytes(b"changed")
                with self.assertRaisesRegex(ValueError, "GPU/TensorRT"):
                    lab.verify_runtime(directory, data)
                (target / "aliked_test.engine").write_bytes(b"extractor")
            with (
                patch("kart_e2e.engine_cache.fingerprint", return_value={"gpu": "B"}),
                patch.object(lab, "inspect_engines"),
            ):
                with self.assertRaisesRegex(ValueError, "GPU/TensorRT"):
                    lab.verify_runtime(directory, data)

    def test_help_is_available_without_runtime_imports(self):
        for stage in (
            "export",
            "compare",
            "build",
            "inspect",
            "prepare",
            "doctor",
            "stage-reference",
        ):
            result = subprocess.run(
                [sys.executable, str(Path(lab.__file__)), stage, "--help"],
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_build_uses_reexported_onnx_and_isolated_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            share = root / "installed"
            model_dir = share / "models/aliked_lightglue"
            model_dir.mkdir(parents=True)
            (model_dir / "aliked.onnx").write_bytes(b"old large model")
            (model_dir / "lightglue_aliked.onnx").write_bytes(b"unchanged matcher")
            config_dir = share / "configs/isaac"
            config_dir.mkdir(parents=True)
            (config_dir / "keypoint_creation_config.pb.txt").write_text(
                "original config"
            )
            for name in ("export_extractor_engine", "export_lightglue_engine"):
                binary = share / "bin/visual_mapping" / name
                binary.parent.mkdir(parents=True, exist_ok=True)
                binary.touch()
            with patch.object(lab, "ARTIFACTS", root):
                directory = lab.run_dir("small")
                directory.mkdir(parents=True)
                (directory / "aliked.onnx").write_bytes(b"new small model")
                lab.save(
                    directory / "manifest.json",
                    dict(
                        width=424,
                        height=240,
                        onnx_sha256=lab.digest(directory / "aliked.onnx"),
                    ),
                )

                def run(command, **kwargs):
                    source = Path(command[command.index("--model_dir") + 1])
                    output = Path(command[command.index("--output_model_dir") + 1])
                    self.assertEqual(
                        (source / "aliked_lightglue/aliked.onnx").read_bytes(),
                        b"new small model",
                    )
                    self.assertEqual(output, directory / "runtime_models")
                    if "export_lightglue_engine" in command[0]:
                        target = (
                            output / "aliked_lightglue/lightglue_aliked_test.engine"
                        )
                        target.parent.mkdir(exist_ok=True)
                        target.touch()

                fake_config = types.SimpleNamespace(
                    configure=lambda text, width, height: f"{width}x{height}"
                )
                with (
                    patch.dict(
                        sys.modules,
                        {
                            "tensorrt": types.SimpleNamespace(),
                            "configure_vgl_extractor": fake_config,
                        },
                    ),
                    patch.object(
                        lab.subprocess, "check_output", return_value=str(share)
                    ),
                    patch.object(lab.subprocess, "run", side_effect=run) as executed,
                    patch.object(lab, "inspect_engine") as inspected,
                    patch.object(lab, "extractor_shape", return_value="424x240"),
                    patch.object(lab, "model_files", return_value=(None, None, {})),
                    patch.object(lab, "inspect_engines"),
                    patch.object(lab, "verify_runtime"),
                ):
                    lab.build(types.SimpleNamespace(name="small"))
                    self.assertEqual(executed.call_count, 2)
                    inspected.assert_called_once()
                self.assertEqual(
                    (model_dir / "aliked.onnx").read_bytes(), b"old large model"
                )


if __name__ == "__main__":
    unittest.main()
