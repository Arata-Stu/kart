"""UI workers and remote publication tested locally; no SSH or GPU access."""

import hashlib
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_studio.e2e import Learning
from kart_studio.jobs import Cancelled
from kart_studio.remote_agent import execute
from kart_studio.storage import atomic_json

REPO = Path(__file__).resolve().parents[3]


class FakeJob:
    def __init__(self, cancelled=False, fail=False):
        self.cancelled, self.fail = cancelled, fail
        self.args = None

    def log(self, _):
        pass

    def check(self):
        if self.cancelled:
            raise Cancelled()

    def run(self, args, env=None):
        self.args = args
        if self.fail:
            raise ValueError("worker failed")
        if args[0] == "scp":
            target = args[-1].split(":", 1)[1]
            shutil.copytree(args[-2], Path(target) / "bundle")
            return
        module = args[2]
        output = Path(
            args[4]
            if module.endswith("preprocess_bag")
            else args[args.index("--output") + 1]
        )
        output.mkdir()
        if module.endswith("preprocess_bag"):
            atomic_json(output / "metadata.json", dict(samples=2, bag=args[3]))
            (output / "samples.jsonl").write_text("{}\n{}\n")
        else:
            (output / "best.pt").write_bytes(b"fixture")
            (output / "metrics.jsonl").write_text('{"epoch":1,"validation_mse":0.1}\n')


def remote_request(profile, action, root, relative="", **extra):
    return execute(dict(action=action, root=root, relative=relative, **extra))


class LearningTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.records = self.base / "record"
        (self.records / "bag").mkdir(parents=True)
        (self.records / "bag/metadata.yaml").write_text("metadata")
        self.service = Learning(REPO, self.base, self.records, self.base / ".state")
        self.settings = self.service.settings()
        self.settings.update(
            python=sys.executable,
            inference_python=sys.executable,
            encoder_repo=str(self.base),
            weights=str(self.base / "weights.pt"),
            device="cpu",
        )
        (self.base / "weights.pt").write_bytes(b"fixture")
        self.service.save_settings(self.settings)

    def tearDown(self):
        self.temp.cleanup()

    def dataset(self, name="data"):
        plan = self.service.prepare("dataset", dict(name=name, bag="bag"))
        self.service.execute(FakeJob(), plan)
        return self.service.folder("datasets", name)

    def test_missing_encoder_and_weights_identify_path(self):
        self.dataset()
        body = dict(name="run", train=["data"], validation=["data"])
        missing = str(self.base / "missing")
        for field, message in (
            ("encoder_repo", "DINOv3ソース"),
            ("weights", "DINOv3公式重み"),
        ):
            with self.subTest(field=field):
                self.service.save_settings(dict(self.settings, **{field: missing}))
                with self.assertRaisesRegex(ValueError, message) as error:
                    self.service.prepare("train", body)
                self.assertIn(missing, str(error.exception))

    def test_evaluation_publishes_report_without_dataset(self):
        model = self.service.root / "models/example"
        model.mkdir()
        atomic_json(model / "artifact.json", {"created": 1})
        with patch.object(self.service, "contract", return_value=({}, "steer_only")):
            plan = self.service.prepare(
                "evaluate", dict(name="check-v1", model="example", bag="bag")
            )
        self.assertIn("--clock", plan["args"])
        self.assertIn("e2e:models:example", plan["resources"])
        report = dict(
            samples=3, mode="steer_only", metrics={"steering_command": {"mae": 0.1}}
        )

        class Worker(FakeJob):
            def run(inner, args, env=None):
                output = Path(args[args.index("--output") + 1])
                output.mkdir()
                atomic_json(output / "report.json", report)
                (output / "predictions.csv").write_text("fixture")

        result = self.service.execute(Worker(), plan)
        self.assertEqual(result["kind"], "evaluations")
        self.assertEqual(self.service.catalog()["evaluations"][0]["samples"], 3)
        self.assertEqual(self.service.catalog()["datasets"], [])

    def test_dataset_publication_and_overwrite(self):
        folder = self.dataset()
        self.assertTrue((folder / "artifact.json").is_file())
        self.assertEqual(self.service.catalog()["datasets"][0]["samples"], 2)
        with self.assertRaises(ValueError):
            self.dataset()
        with self.assertRaises(ValueError):
            self.service.prepare("dataset", dict(name="bad", bag="../escape"))

    def test_cancel_and_failure_do_not_publish(self):
        for job in (FakeJob(cancelled=True), FakeJob(fail=True)):
            plan = self.service.prepare("dataset", dict(name="cancel", bag="bag"))
            with self.assertRaises((Cancelled, ValueError)):
                self.service.execute(job, plan)
            self.assertFalse(plan["target"].exists())
            self.assertFalse(list(plan["target"].parent.glob(".work-*")))

    def test_changing_bag_does_not_publish(self):
        plan = self.service.prepare("dataset", dict(name="changing", bag="bag"))
        job = FakeJob()
        run = job.run

        def mutate(args, env=None):
            run(args, env)
            (self.records / "bag/metadata.yaml").write_text("changed recording")

        job.run = mutate
        with self.assertRaisesRegex(ValueError, "bagが更新"):
            self.service.execute(job, plan)
        self.assertFalse(plan["target"].exists())
        self.assertFalse(list(plan["target"].parent.glob(".work-*")))

    def test_split_and_training_arguments(self):
        self.dataset("train")
        self.dataset("val")
        body = dict(
            name="run",
            train=["train"],
            validation=["val"],
            mode="steer_only",
            fixed_throttle=0.12,
            max_throttle=0.25,
            epochs=2,
            finetune=True,
        )
        # Both datasets originate from the same bag; identical dataset selection is also allowed.
        self.service.prepare("train", {**body, "validation": ["train"]})
        plan = self.service.prepare("train", body)
        job = FakeJob()
        self.service.execute(job, plan)
        self.assertEqual(job.args[job.args.index("--fixed-throttle") + 1], "0.12")
        self.assertEqual(job.args[job.args.index("--max-throttle") + 1], "0.25")
        self.assertIn("--finetune", job.args)
        self.assertIn("steer_only", job.args)
        self.assertEqual(self.service.catalog()["runs"][0]["metrics"]["epoch"], 1)
        with self.assertRaises(ValueError):
            self.service.prepare(
                "train", {**body, "name": "bad_runtime", "fixed_throttle": 0.9}
            )
        export = self.service.prepare("export", dict(name="model", run="run"))
        self.assertEqual(export["module"], "export_onnx")
        self.assertTrue(export["args"][0].endswith("/run/best.pt"))
        with self.assertRaises(ValueError):
            self.service.prepare(
                "train", {**body, "name": "nan", "learning_rate": float("nan")}
            )

    def model(self):
        spec = __import__("importlib.util", fromlist=[""]).spec_from_file_location(
            "fixture_contract", REPO / "ros2_ws/src/kart_e2e/kart_e2e/contract.py"
        )
        module = __import__("importlib.util", fromlist=[""]).module_from_spec(spec)
        spec.loader.exec_module(module)
        root = self.service.root / "models/model"
        root.mkdir()
        (root / "model.onnx").write_bytes(b"fixture")
        atomic_json(
            root / "metadata.json",
            dict(
                schema=1,
                model_spec=module.SPEC,
                output_mode="steer_only",
                input_name="image",
                input_shape=[1, 3, 120, 212],
                output_name="control",
                output_shape=[1, 1],
                onnx_sha256=hashlib.sha256(b"fixture").hexdigest(),
                ort_max_abs_error=0,
            ),
        )
        atomic_json(root / "artifact.json", dict(kind="models", created=1))
        return root

    @patch("kart_studio.transfers.request", side_effect=remote_request)
    def test_transfer_only_onnx_with_hash_validation(self, request):
        source = self.model()
        (source / "old.plan").write_bytes(b"excluded")
        remote = self.base / "remote"
        remote.mkdir()
        profile = dict(
            user="tester",
            host="jetson.local",
            port=22,
            record_root="/record",
            map_root="/map",
        )
        result = self.service.push(FakeJob(), profile, "model", str(remote))
        self.assertEqual(
            {p.name for p in Path(result["path"]).iterdir()},
            {"model.onnx", "metadata.json"},
        )
        self.assertEqual(request.call_args.args[1], "publish-model")
        (source / "model.onnx").write_bytes(b"corrupted")
        with self.assertRaises(ValueError):
            self.service.push(FakeJob(), profile, "model", str(remote))

    def test_remote_corruption_rejected(self):
        source = self.model()
        remote = self.base / "remote"
        remote.mkdir()
        stage = ".kart-upload-" + "a" * 32
        execute(dict(action="reserve", root=str(remote), relative=stage))
        bundle = remote / stage / "bundle"
        bundle.mkdir()
        for file in ("model.onnx", "metadata.json"):
            shutil.copyfile(source / file, bundle / file)
        from kart_studio.remote_agent import model_manifest

        hashes = model_manifest(bundle)
        (bundle / "model.onnx").write_bytes(b"corrupt")
        with self.assertRaises(ValueError):
            execute(
                dict(
                    action="publish-model",
                    root=str(remote),
                    relative=stage,
                    destination="model",
                    hashes=hashes,
                )
            )
        self.assertFalse((remote / "model").exists())
