"""Offline report labels and metadata; no ROS execution or inference is mocked."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/kart_bringup"))
spec = importlib.util.spec_from_file_location("evaluate_racing_bag", ROOT / "scripts/evaluate_racing_bag.py")
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)
from kart_bringup.racing_models import get_profile


class RacingBagReportTests(unittest.TestCase):
    def test_coco_ids_use_actual_labels_and_retain_score_and_id(self):
        names = get_profile("coco")["class_names"]
        for class_id, expected in (("0", "person"), ("2", "car"), ("11", "stop sign")):
            with self.subTest(class_id=class_id):
                name = evaluation.class_name(class_id, names)
                self.assertEqual(name, expected)
                detection = dict(class_name=name, class_id=class_id, score=.875)
                self.assertEqual(evaluation.detection_label(detection), f"{expected} [id={class_id}] 0.88")

    def test_unknown_class_is_never_relabelled_as_opponent(self):
        names = get_profile("coco")["class_names"]
        for class_id in ("-1", "80", "999", "opponent", "2.5", None):
            with self.subTest(class_id=class_id):
                self.assertEqual(evaluation.class_name(class_id, names), "unknown")
        self.assertEqual(evaluation.class_name("0", get_profile("opponent")["class_names"]), "opponent")

    def test_report_retains_sample_timestamps_and_records_model_labels(self):
        profile = get_profile("coco")
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            engine = output / "coco.plan"
            engine.write_bytes(b"report fixture only")
            args = SimpleNamespace(bag=output / "bag", engine=engine,
                                   topic="/camera/rgb", stride=30)
            detection = dict(cx=100., cy=200., w=40., h=50., score=.875,
                             class_id="2", class_name=evaluation.class_name("2", profile["class_names"]))
            frames = [dict(index=60, stamp_ns=1234567890123456789, frame_id="camera",
                           image="frame_0000060.jpg", detections=[detection])]
            evaluation.write_report(args, output, frames, profile)
            report = json.loads((output / "report.json").read_text())
            self.assertEqual(report["model_profile"], "coco")
            self.assertEqual(report["class_names"], list(profile["class_names"]))
            self.assertEqual(report["frames"], frames)
            self.assertEqual(report["stride"], 30)
            self.assertEqual(report["topic"], "/camera/rgb")
            self.assertEqual(report["engine_sha256"], evaluation.engine_hash(engine))
            page = (output / "index.html").read_text()
            self.assertIn("car [id=2] 0.88", page)
            self.assertIn("1234567890123456789 ns", page)
            self.assertIn("Model: coco", page)
            self.assertNotIn("opponent", page)


if __name__ == "__main__":
    unittest.main()
