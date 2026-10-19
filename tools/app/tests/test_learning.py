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
        return self.service.folder("datasets", plan["key"])

    def test_loss_history_and_invalid_records(self):
        import json
        run = self.service.root / "runs/history"
        run.mkdir()
        atomic_json(run / "artifact.json", dict(created=1, kind="runs"))
        rows = [dict(epoch=1, train_mse=.4, validation_mse=.3),
                dict(epoch=2, train_mse=.1, validation_mse=.2)]
        metrics = run / "metrics.jsonl"
        metrics.write_text("\n".join(map(json.dumps, rows)))
        self.assertEqual(self.service.loss("history")["metrics"], rows)
        for bad in ([], [dict(epoch=1, validation_mse=.1)],
                    [dict(epoch=1, train_mse=float('nan'), validation_mse=.1)],
                    [dict(epoch=1, train_mse=-.1, validation_mse=.1)],
                    [rows[1], rows[0]], [rows[0], rows[0]]):
            metrics.write_text("\n".join(map(json.dumps, bad)))
            with self.assertRaises(ValueError):
                self.service.loss("history")
        with self.assertRaises(ValueError):
            self.service.loss("../history")

    def test_tiny_distill_plan_publication_and_control_training(self):
        import json
        data = self.dataset().relative_to(self.service.root / "datasets").as_posix()
        plan = self.service.prepare("distill", dict(name="tiny", train=[data], validation=[data]))
        self.assertEqual(plan["kind"], "encoders")
        self.assertEqual(len(plan["key"].split("/")), 3)
        self.assertIn("--teacher-weights", plan["args"])
        self.assertIn("--feature-cache", plan["args"])
        class Worker(FakeJob):
            def run(inner,args,env=None):
                output=Path(args[args.index("--output")+1]);output.mkdir()
                (output/'encoder.pt').write_bytes(b'fixture')
                (output/'metrics.jsonl').write_text(json.dumps(dict(epoch=1,train_loss=.3,validation_loss=.2))+'\n')
        result=self.service.execute(Worker(),plan)
        self.assertEqual(self.service.loss(result['id'], 'encoders')['metric'], 'distillation_loss')
        train=self.service.prepare("train", dict(name="tiny-control", architecture="dinov3_vitt16",
                                                encoder=result['id'],train=[data],validation=[data]))
        self.assertIn("e2e:encoders:"+result['id'],train['resources'])
        self.assertIn(str(plan['target']/'encoder.pt'),train['args'])
        self.assertEqual(train['provenance']['architecture'],'dinov3_vitt16')
        from kart_studio import e2e_trash
        moved=e2e_trash.move(self.service,'encoders',result['id'])
        e2e_trash.restore(self.service,moved['token'])
        self.assertTrue((plan['target']/'encoder.pt').exists())
        with self.assertRaises(ValueError):
            self.service.prepare('train',dict(name='bad',architecture='unknown',train=[data],validation=[data]))

    def test_missing_encoder_and_weights_identify_path(self):
        dataset = self.dataset().relative_to(self.service.root / "datasets").as_posix()
        body = dict(name="run", train=[dataset], validation=[dataset])
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
        with patch("kart_studio.e2e.datetime") as clock:
            clock.now.return_value.strftime.return_value = "2026-10-19/12-34-56/"
            folder = self.dataset()
            with self.assertRaises(ValueError):
                self.dataset()
        self.assertEqual(folder.relative_to(self.service.root / "datasets").as_posix(), "2026-10-19/12-34-56/data")
        self.assertTrue((folder / "artifact.json").is_file())
        self.assertEqual(self.service.catalog()["datasets"][0]["samples"], 2)
        with self.assertRaises(ValueError):
            self.service.prepare("dataset", dict(name="bad", bag="../escape"))

    def test_dataset_legacy_and_new_catalog_training_and_confinement(self):
        legacy = self.service.root / "datasets/legacy"
        legacy.mkdir()
        atomic_json(legacy / "artifact.json", dict(created=1, samples=2, bag="bag"))
        with patch("kart_studio.e2e.datetime") as clock:
            clock.now.return_value.strftime.return_value = "2026-10-19/12-34-56/"
            first = self.dataset("same")
            clock.now.return_value.strftime.return_value = "2026-10-20/01-02-03/"
            second = self.dataset("same")
        self.assertNotEqual(first, second)
        rows = self.service.catalog()["datasets"]
        self.assertEqual(len(rows), 3)
        self.assertEqual(sum(row["name"] == "same" for row in rows), 2)
        self.assertEqual(self.service.folder("datasets", "legacy"), legacy)
        key = second.relative_to(self.service.root / "datasets").as_posix()
        plan = self.service.prepare("train", dict(name="mixed", train=[key], validation=["legacy"]))
        self.assertIn(str(second), plan["args"])
        self.assertIn(str(legacy), plan["args"])
        for bad in ("../escape", "/tmp/escape", "2026-10-19/12-34-56/../legacy", "a/b/c"):
            with self.assertRaises(ValueError):
                self.service.folder("datasets", bad)
        linked = self.service.root / "datasets/2026-10-21"
        linked.symlink_to(first.parent.parent, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.service.folder("datasets", "2026-10-21/12-34-56/same")
        self.assertEqual(len(self.service.catalog()["datasets"]), 3)

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
        train = self.dataset("train").relative_to(self.service.root / "datasets").as_posix()
        val = self.dataset("val").relative_to(self.service.root / "datasets").as_posix()
        body = dict(
            name="run",
            train=[train],
            validation=[val],
            mode="steer_only",
            fixed_throttle=0.12,
            max_throttle=0.25,
            epochs=2,
            finetune=True,
        )
        # Both datasets originate from the same bag; identical dataset selection is also allowed.
        self.service.prepare("train", {**body, "validation": [train]})
        plan = self.service.prepare("train", body)
        defaults = self.service.prepare("train", {key: value for key, value in body.items() if key != "epochs"})
        self.assertEqual(defaults["args"][defaults["args"].index("--epochs") + 1], "10")
        self.assertEqual(defaults["args"][defaults["args"].index("--learning-rate") + 1], "1e-05")
        self.assertTrue(defaults["provenance"]["finetune"])
        self.assertEqual(defaults["provenance"]["num_workers"], 8)
        self.assertIn("--feature-cache", defaults["args"])
        self.assertFalse(defaults["provenance"]["feature_cache_effective"])
        uncached = self.service.prepare("train", {**body, "feature_cache": False, "num_workers": 0})
        self.assertIn("--no-feature-cache", uncached["args"])
        self.assertEqual(uncached["provenance"]["num_workers"], 0)
        for bad in ({"feature_cache": "false"}, {"num_workers": -1}):
            with self.assertRaises(ValueError):
                self.service.prepare("train", {**body, **bad})

        frozen = self.service.prepare("train", {**body, "finetune": False})
        self.assertNotIn("--finetune", frozen["args"])
        self.assertEqual(frozen["provenance"]["learning_rate"], 1e-4)
        explicit = self.service.prepare("train", {**body, "learning_rate": 2e-6})
        self.assertEqual(explicit["provenance"]["learning_rate"], 2e-6)

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

    def test_dated_export_evaluation_and_source_locks(self):
        run = self.service.root / 'runs/run'
        run.mkdir()
        atomic_json(run / 'artifact.json', dict(created=1))
        (run / 'best.pt').write_bytes(b'fixture')
        with patch('kart_studio.e2e.datetime') as clock:
            clock.now.return_value.strftime.return_value = '2026-10-19/12-34-56/'
            export = self.service.prepare('export', dict(name='policy', run='run'))
            self.assertEqual(export['key'], '2026-10-19/12-34-56/policy')
            class Worker(FakeJob):
                def run(inner, args, env=None):
                    output = Path(args[4])
                    output.mkdir()
                    (output / 'model.onnx').write_bytes(b'fixture')
                    atomic_json(output / 'metadata.json', dict(ort_max_abs_error=0))
            with patch.object(self.service, 'contract', return_value=({}, 'steer_only')):
                self.service.execute(Worker(), export)
                evaluation = self.service.prepare('evaluate', dict(name='check', model=export['key'], bag='bag'))
                engine = self.service.prepare('build-engine', dict(name='benchmark', model=export['key']))
            self.assertEqual(evaluation['key'], '2026-10-19/12-34-56/check')
            self.assertEqual(evaluation['provenance']['model'], export['key'])
            self.assertIn('e2e:models:'+export['key'], evaluation['resources'])
            self.assertIn('e2e:models:'+export['key'], engine['resources'])
            self.assertEqual(self.service.catalog()['models'][0]['id'], export['key'])
            self.assertEqual(self.service.folder('models', export['key']), export['target'])

    def test_trash_roundtrip_all_artifact_kinds(self):
        from types import SimpleNamespace
        from kart_studio.server import Studio
        from kart_studio.jobs import Jobs
        from kart_studio.e2e import KINDS
        studio = SimpleNamespace(learning=self.service, jobs=Jobs(self.base/'jobs'))
        for kind in KINDS:
            target = self.service.target(kind, 'cleanup')
            target.mkdir(parents=True)
            atomic_json(target / 'artifact.json', dict(created=1))
            (target / 'payload.bin').write_bytes(b'keep original contents')
            key = target.relative_to(self.service.root/kind).as_posix()
            removed = Studio.post(studio, '/api/e2e/trash', dict(kind=kind, id=key))
            self.assertFalse(target.exists())
            self.assertFalse(self.service.catalog()[kind])
            self.assertEqual(Studio.get(studio, '/api/e2e/trash', {})[0]['id'], key)
            restored = Studio.post(studio, '/api/e2e/restore', dict(token=removed['token']))
            self.assertEqual(restored['id'], key)
            self.assertEqual((target/'payload.bin').read_bytes(), b'keep original contents')
            self.assertFalse(Studio.get(studio, '/api/e2e/trash', {}))

    def test_trash_busy_and_conflicting_restore_are_rejected(self):
        from types import SimpleNamespace
        from kart_studio.server import Studio
        from kart_studio.jobs import Jobs
        from kart_studio import e2e_trash
        path = self.dataset()
        key = path.relative_to(self.service.root/'datasets').as_posix()
        studio = SimpleNamespace(learning=self.service, jobs=Jobs(self.base/'jobs'))
        for resource in ('e2e-compute', 'e2e:datasets:'+key):
            with studio.jobs.guard(resource), self.assertRaises(ValueError):
                Studio.post(studio, '/api/e2e/trash', dict(kind='datasets', id=key))
            self.assertTrue(path.exists())
        removed = Studio.post(studio, '/api/e2e/trash', dict(kind='datasets', id=key))
        path.mkdir(parents=True)
        with self.assertRaises(ValueError):
            Studio.post(studio, '/api/e2e/restore', dict(token=removed['token']))
        self.assertEqual(len(e2e_trash.listing(self.service)), 1)
        path.rmdir()
        e2e_trash.restore(self.service, removed['token'])
        for token in ('../escape', '/tmp/escape', 'bad'):
            with self.assertRaises(ValueError):
                e2e_trash.restore(self.service, token)
        with self.assertRaises(ValueError):
            e2e_trash.move(self.service, '../models', key)

    def test_cleanup_rejects_symlink_artifact_root(self):
        from kart_studio import e2e_trash
        original = self.service.root / 'models'
        original.rename(self.service.root / 'models-backup')
        outside = self.base / 'outside'
        (outside / 'policy').mkdir(parents=True)
        atomic_json(outside / 'policy/artifact.json', dict(created=1))
        original.symlink_to(outside, target_is_directory=True)
        with self.assertRaises(ValueError):
            e2e_trash.move(self.service, 'models', 'policy')
        self.assertTrue((outside/'policy/artifact.json').is_file())
        self.assertFalse(self.service.catalog()['models'])

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
        dated = self.service.root / 'models/2026-10-19/12-34-56/model'
        dated.parent.mkdir(parents=True)
        source.rename(dated)
        source = dated
        key = '2026-10-19/12-34-56/model'
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
        result = self.service.push(FakeJob(), profile, key, str(remote))
        self.assertEqual(
            {p.name for p in Path(result["path"]).iterdir()},
            {"model.onnx", "metadata.json"},
        )
        self.assertEqual(request.call_args.args[1], "publish-model")
        (source / "model.onnx").write_bytes(b"corrupted")
        with self.assertRaises(ValueError):
            self.service.push(FakeJob(), profile, key, str(remote))

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
