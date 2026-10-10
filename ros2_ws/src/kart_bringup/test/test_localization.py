"""Portable asset/TF contract tests; no claim of GPU or ROS integration."""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml
from kart_bringup.localization import resolve
from kart_bringup.prepare_vgl_map import build
from kart_bringup.vgl_assets import CONFIG_FILES, validate_bundle

CONFIG = Path(__file__).resolve().parents[1] / "config/localization"


class LocalizationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root / "source"
        self.models = self.root / "models"
        self.templates = self.root / "templates"
        self.output = self.root / "bundle"
        self.write(self.source / "cuvslam_map/map.mdb", "map")
        self.write(self.source / "map_frames/rectified/frames_meta.json", "{}")
        for name in (
            "aliked.onnx",
            "aliked_test.engine",
            "lightglue_aliked_test.engine",
        ):
            self.write(self.models / "aliked_lightglue" / name, name)
        for name in CONFIG_FILES:
            self.write(self.templates / name, "setting: true")
        self.write(
            self.templates / CONFIG_FILES[0],
            "aliked_detector { tensorrt_config { input_dimension {"
            + " ".join(
                f"{k}: {v}" for k in ("min", "opt", "max") for v in (1, 3, 480, 640)
            )
            + "} } }",
        )
        self.write(
            self.templates / "localizer_config.pb.txt", "save_debug_images: true"
        )
        self.inspected = []

    def write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value)

    def run_official(self, command, **kwargs):
        if "--help" in command:
            return SimpleNamespace(
                stdout=" ".join(
                    "--" + k
                    for k in (
                        "map_folder",
                        "raw_image_folder",
                        "config_folder_path",
                        "model_dir",
                        "binary_folder_path",
                        "feature_type",
                        "extract_feature",
                        "build_bow_index",
                    )
                )
            )
        target = Path(
            next(s.split("=", 1)[1] for s in command if s.startswith("--map_folder="))
        )
        for name in (
            "keyframes/frames_meta.json",
            "bow_index.pb",
            "vocabulary/vocab.pb",
        ):
            self.write(target / name, "data")
        return SimpleNamespace(returncode=0)

    def prepare(self, run=None):
        return build(
            self.source,
            self.models,
            self.output,
            {"width": 424, "height": 240, "feature_type": "aliked"},
            self.templates,
            self.root / "bin",
            inspect=lambda engines, shape: self.inspected.append(shape),
            run=run or self.run_official,
        )

    def test_prepare_resolve_and_pose_only_contract(self):
        self.prepare()
        params, remaps, _, profile = resolve(
            CONFIG,
            self.output,
            self.models,
            {"use_sim_time": "true", "base_frame": "base_link"},
        )
        self.assertEqual(self.inspected, [[1, 3, 240, 424]])
        self.assertEqual(profile["input_shape"], [1, 3, 240, 424])
        for settings in params.values():
            self.assertTrue(settings["use_sim_time"])
            self.assertEqual(settings["base_frame"], "base_link")
        self.assertFalse(params["cuvslam"]["enable_slam_visualization"])
        self.assertFalse(params["vgl"]["vgl_enable_debug"])
        self.assertFalse(params["vgl"]["publish_rectified_images"])
        self.assertFalse(params["vgl"]["publish_map_to_odom_tf"])
        self.assertIn(
            ("visual_slam/initial_pose", "/localization/pose_hint"), remaps["cuvslam"]
        )
        self.assertEqual((self.source / "cuvslam_map/map.mdb").read_text(), "map")
        self.assertIn(
            "save_debug_images: false",
            (self.output / "vgl_runtime_config/localizer_config.pb.txt").read_text(),
        )

    def test_mismatched_model_rejected(self):
        self.prepare()
        self.write(self.models / "aliked_lightglue/aliked.onnx", "different")
        with self.assertRaisesRegex(ValueError, "ONNX"):
            validate_bundle(self.output, self.models)

    def test_mismatched_slam_map_rejected(self):
        self.prepare()
        self.write(self.output / "cuvslam_map/map.mdb", "different")
        with self.assertRaisesRegex(ValueError, "paired"):
            validate_bundle(self.output, self.models)

    def test_failed_preparation_not_published(self):
        def fail(command, **kwargs):
            if "--help" in command:
                return self.run_official(command, **kwargs)
            raise subprocess.CalledProcessError(1, command)

        with self.assertRaises(subprocess.CalledProcessError):
            self.prepare(fail)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob(".vgl-*")), [])
        self.assertTrue((self.source / "cuvslam_map/map.mdb").is_file())

    def test_existing_output_preserved(self):
        self.output.mkdir()
        self.write(self.output / "keep", "original")
        with self.assertRaisesRegex(ValueError, "new output"):
            self.prepare()
        self.assertEqual((self.output / "keep").read_text(), "original")

    def test_tf_conflict_rejected(self):
        self.prepare()
        custom = self.root / "config"
        shutil.copytree(CONFIG, custom)
        settings = yaml.safe_load((custom / "vgl.yaml").read_text())
        settings["/**/visual_global_localization"]["ros__parameters"][
            "publish_map_to_odom_tf"
        ] = True
        (custom / "vgl.yaml").write_text(yaml.safe_dump(settings))
        with self.assertRaisesRegex(ValueError, "alone"):
            resolve(custom, self.output, self.models)

    def test_config_tampering_rejected(self):
        self.prepare()
        self.write(
            self.output / "vgl_runtime_config/localizer_config.pb.txt",
            "save_debug_images: true",
        )
        with self.assertRaisesRegex(ValueError, "config changed"):
            validate_bundle(self.output, self.models)

    def test_missing_vocabulary_rejected(self):
        self.prepare()
        shutil.rmtree(self.output / "cuvgl_map/vocabulary")
        with self.assertRaisesRegex(ValueError, "vocabulary"):
            validate_bundle(self.output, self.models)

    def test_unknown_boolean_rejected(self):
        with self.assertRaisesRegex(ValueError, "true or false"):
            resolve(CONFIG, self.output, self.models, {"use_sim_time": "perhaps"})


if __name__ == "__main__":
    unittest.main()


class VslamOnlyTest(unittest.TestCase):
    def test_studio_map_without_vgl_assets(self):
        from kart_bringup.mission import discover

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            folder = root / "course" / "cuvslam_map"
            folder.mkdir(parents=True)
            self.assertEqual(discover(root, "vslam"), [])
            (folder / "map.mdb").write_bytes(b"test map")
            self.assertEqual(discover(root, "vslam"), [folder.parent])
            self.assertEqual(discover(root, "bundle"), [])
            for map_dir in (folder, folder.parent):
                params, remaps, _, _ = resolve(CONFIG, map_dir, "", enable_vgl=False)
                self.assertEqual(params["cuvslam"]["load_map_folder_path"], str(folder))
                self.assertTrue(params["cuvslam"]["localize_on_startup"])
                self.assertFalse(params["cuvslam"]["enable_request_hint"])
                self.assertTrue(remaps["cuvslam"])
