"""Offline learning jobs and model bundles. No vehicle startup or arbitrary shell commands."""

from datetime import datetime
import importlib.util
import math
import os
import re
import shutil
import sys
import tempfile
import time
import uuid
from pathlib import Path

from . import transfers
from .storage import atomic_json, name, read_json, within


KINDS = ("datasets", "runs", "models", "evaluations", "engines", "features", "encoders")
DATED = ("datasets", "models", "evaluations", "features", "encoders")


class Learning:
    def __init__(self, repo, base, records, state):
        self.repo, self.records = repo, records
        self.root = (base / "e2e").resolve()
        self.settings_file = state / "e2e.json"
        for kind in KINDS:
            (self.root / kind).mkdir(parents=True, exist_ok=True)
        spec = importlib.util.spec_from_file_location(
            "kart_ui_e2e_contract", repo / "ros2_ws/src/kart_e2e/kart_e2e/contract.py"
        )
        contract = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(contract)
        self.contract = contract.model_contract

    def settings(self):
        result = dict(
            python=os.environ.get("KART_E2E_PYTHON", sys.executable),
            inference_python="/opt/inference/bin/python",
            evs_raw_python=os.environ.get("KART_EVS_RAW_PYTHON", str(self.repo / ".venv/evs-raw/bin/python")
                                         if (self.repo / ".venv/evs-raw/bin/python").is_file() else "/opt/evs-raw/bin/python"),
            encoder_repo=str(self.repo / "python_ws/dinov3"),
            weights=str(
                self.repo
                / "weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth"
            ),
            evs_weights=str(self.repo / "weights/evs/event_encoder.pt"),
            evs_json=str(self.repo / "weights/evs/event_encoder.json"),
            device="cuda",
            model_root="/home/kart/workspaces/kart/models",
        )
        if self.settings_file.exists():
            saved = read_json(self.settings_file)
            if not saved.get("weights"):
                saved.pop("weights", None)
            result.update(saved)
        return result

    def save_settings(self, raw):
        if set(raw) != set(self.settings()):
            raise ValueError("学習環境の設定項目が不正です")
        if not all(isinstance(v, str) and len(v) <= 4096 for v in raw.values()):
            raise ValueError("設定は文字列で指定してください")
        if not re.fullmatch(r"cpu|cuda(?::[0-9]+)?", raw["device"]):
            raise ValueError("deviceはcpuまたはcuda[:番号]です")
        transfers.remote_path(raw["model_root"])
        atomic_json(self.settings_file, raw)
        return raw

    def catalog(self):
        result = dict(settings=self.settings(), root=str(self.root))
        for kind in KINDS:
            entries = []
            root = self.root / kind
            artifacts = list(root.glob('*/artifact.json'))
            if kind in DATED:
                artifacts += list(root.glob('*/*/*/artifact.json'))
            for artifact in artifacts:
                path = artifact.parent
                relative = path.relative_to(root).as_posix()
                if any(part.startswith('.') for part in path.relative_to(root).parts):
                    continue
                try:
                    self.folder(kind, relative)
                    entries.append({**read_json(artifact), "id": relative, "name": path.name})
                except (ValueError, OSError):
                    continue
            result[kind] = sorted(
                entries, key=lambda item: item["created"], reverse=True
            )
        return result

    def relative_key(self, kind, key):
        if kind not in KINDS:
            raise ValueError("Unknown artifact kind")
        if kind in DATED and isinstance(key, str) and re.fullmatch(
            r"[0-9]{4}-[0-9]{2}-[0-9]{2}/[0-9]{2}-[0-9]{2}-[0-9]{2}/[A-Za-z0-9][A-Za-z0-9_-]{0,63}", key
        ):
            return key
        return name(key)

    def folder(self, kind, key):
        relative = self.relative_key(kind, key)
        root = within(self.root, kind)
        path = within(root, relative)
        within(path, "artifact.json")
        return path

    def loss(self, key, kind="runs"):
        if kind not in ("runs", "encoders"):
            raise ValueError("Invalid loss artifact kind")
        folder = self.folder(kind, key)
        metrics = []
        import json
        for line in within(folder, "metrics.jsonl").read_text().splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            epoch = row.get("epoch")
            if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch <= (metrics[-1]["epoch"] if metrics else 0):
                raise ValueError("Invalid loss epoch sequence")
            fields = ("train_loss", "validation_loss") if kind == "encoders" else ("train_mse", "validation_mse")
            for field in fields:
                value = row.get(field)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                    raise ValueError("Invalid training loss")
            metrics.append({field: row[field] for field in ("epoch", *fields)})
        if not metrics:
            raise ValueError("損失の記録がありません")
        return dict(id=key, kind=kind, metric="distillation_loss" if kind=="encoders" else "mse", metrics=metrics)

    def runtime(self, action=None, modality="rgb"):
        cfg = self.settings()
        field = (
            "evs_raw_python" if action == "dataset" and modality == "evs" else
            "inference_python"
            if action in ("export", "evaluate", "build-engine")
            else "python"
        )
        python = Path(cfg[field]).expanduser()
        if (
            not python.is_absolute()
            or not python.is_file()
            or not os.access(python, os.X_OK)
            or not re.fullmatch(r"python(?:[0-9]+(?:\.[0-9]+)*)?", python.name)
        ):
            raise ValueError(
                f"UI「学習環境」の{field}に実行可能なPythonを指定してください: {python}"
            )
        env = os.environ.copy()
        env["PYTHONPATH"] = (
            str(self.repo / "ros2_ws/src/kart_e2e")
            + os.pathsep
            + env.get("PYTHONPATH", "")
        )
        env["PYTHONUNBUFFERED"] = "1"
        return str(python), cfg, env

    def target(self, kind, key):
        relative = self.relative_key(kind, name(key))
        if kind in DATED:
            relative = datetime.now().strftime('%Y-%m-%d/%H-%M-%S/') + relative
        target = within(within(self.root, kind), relative, exists=False)
        if target.exists():
            raise ValueError("同じ名前の成果物が存在します。別名にしてください")
        return target

    def prepare(self, action, body):
        key = name(body.get("name"))
        python, cfg, env = self.runtime(action, body.get("modality", "rgb"))
        common = dict(key=key, python=python, env=env)
        modality = body.get("modality", "rgb")
        if modality not in ("rgb", "evs"):
            raise ValueError("入力種別はRGBまたはEVSです")
        if action == "dataset":
            target = self.target("datasets", key)
            bag = within(self.records, body.get("bag", ""))
            within(bag, "metadata.yaml")
            clock = body.get("clock", "bag")
            if clock not in ("bag", "header"):
                raise ValueError("時計はbagまたはheaderを指定してください")
            skew = self.number(body.get("max_skew_ms", 100), 1, 1000)
            topics = {
                key: body.get(key, default)
                for key, default in (
                    ("image_topic", "/realsense/color/image_raw"),
                    ("command_topic", "/teleop/control_cmd"),
                    ("mode_topic", "/operation_mode/state"),
                )
            }
            if any(
                not isinstance(v, str) or not re.fullmatch(r"/[A-Za-z0-9_/]+", v)
                for v in topics.values()
            ):
                raise ValueError("topicは絶対名で指定してください")
            if modality == "evs":
                model_json = Path(cfg["evs_json"]).expanduser()
                if not model_json.is_file():
                    raise ValueError("学習環境のEVS encoder JSONを確認してください")
                manifest_value = body.get("raw_manifest", "")
                if not isinstance(manifest_value, str):
                    raise ValueError("RAW manifestはパス文字列で指定してください")
                manifest = Path(manifest_value).expanduser() if manifest_value.strip() else None
                if manifest is not None and (not manifest.is_absolute() or not manifest.is_file()):
                    raise ValueError("RAW manifestの絶対パスを指定してください")
                raw_args = ["--raw-manifest", str(manifest)] if manifest is not None else []
                stride = self.integer(body.get("sample_stride", 10), 1, 100000)
                reorder = self.integer(body.get("max_reorder_us", 5000), 0, 100000)
                return {**common, "kind":"datasets", "target":target,
                        "key":target.relative_to(self.root/"datasets").as_posix(),
                        "module":"preprocess_evs",
                        "args":[str(bag), "@OUTPUT@", *raw_args,
                                "--model-json", str(model_json), "--clock", clock,
                                "--sample-stride", str(stride), "--max-reorder-us", str(reorder), "--max-skew-ms", str(skew),
                                "--command-topic", topics["command_topic"], "--mode-topic", topics["mode_topic"]],
                        "provenance":{"bag":str(bag), "modality":"evs", "raw_manifest":str(manifest) if manifest is not None else None},
                        "resources":["e2e-compute", "record:"+str(bag.relative_to(self.records.resolve()))]}
            args = [str(bag), "@OUTPUT@", "--clock", clock, "--max-skew-ms", str(skew)]
            for topic, value in topics.items():
                args += ["--" + topic.replace("_", "-"), value]
            return {
                **common,
                "kind": "datasets",
                "key": target.relative_to(self.root / "datasets").as_posix(),
                "target": target,
                "module": "preprocess_bag",
                "args": args,
                "provenance": {"bag": str(bag), "modality":"rgb"},
                "resources": ["record:" + str(bag.relative_to(self.records.resolve()))],
            }
        if action in ("train", "distill"):
            target = self.target("runs", key) if action == "train" else None
            groups = []
            for field in ("train", "validation"):
                ids = body.get(field, [])
                if (
                    not isinstance(ids, list)
                    or not 1 <= len(ids) <= 100
                    or len(set(ids)) != len(ids)
                ):
                    raise ValueError(
                        "学習用・検証用データセットをそれぞれ選択してください"
                    )
                groups.append([self.folder("datasets", item) for item in ids])

            for group in groups:
                for dataset in group:
                    meta = read_json(within(dataset, "metadata.json"))
                    if meta.get("modality", "rgb") != modality:
                        raise ValueError("選択したデータセットの入力種別が一致しません")
            if action == "distill" and modality != "rgb":
                raise ValueError("Tiny特徴蒸留はRGBのみ対応しています")
            mode = body.get("mode", "steer_throttle")
            if mode not in ("steer_throttle", "steer_only"):
                raise ValueError("予測モードが不正です")
            repo = Path(cfg["encoder_repo"]).expanduser()
            architecture = body.get("architecture", "dinov3_vits16")
            if architecture not in ("dinov3_vits16", "dinov3_vitt16"):
                raise ValueError("不明なRGBモデルです")
            source_resources = []
            weights = Path(cfg["evs_weights"] if modality == "evs" else cfg["weights"]).expanduser()
            if modality == "evs" and architecture != "dinov3_vits16":
                raise ValueError("EVSは20ch ViT-S encoderを使用します")
            if action == "distill" and body.get("teacher_run"):
                teacher = self.folder("runs", body["teacher_run"])
                weights = within(teacher, "best.pt")
                source_resources.append("e2e:runs:" + body["teacher_run"])
            elif action == "train" and architecture == "dinov3_vitt16":
                source = self.folder("encoders", body.get("encoder"))
                weights = within(source, "encoder.pt")
                source_resources.append("e2e:encoders:" + body["encoder"])
            if not repo.is_dir():
                raise ValueError(
                    f"DINOv3ソースが見つかりません: {repo}。"
                    "Notebookのコンテナ内で cd /workspaces && vcs import . < e2e.repos "
                    "を実行し、UI「学習環境」のDINOv3ソースを確認してください。重みとは別に必要です"
                )
            if not weights.is_file():
                raise ValueError(
                    f"{'EVS encoder重み' if modality == 'evs' else 'DINOv3公式重み'}が見つかりません: {weights}。"
                    "UI「学習環境」の重みパスをコンテナ内のパスで指定してください"
                )
            epochs = self.integer(body.get("epochs", 10), 1, 100000)
            batch = self.integer(body.get("batch_size", 32), 1, 4096)
            fixed = self.number(body.get("fixed_throttle", 0.0), 0, 1)
            maximum = self.number(body.get("max_throttle", 0.2), 0, 1)
            if fixed > maximum:
                raise ValueError("固定スロットルは上限以下にしてください")
            finetune = body.get("finetune", False)
            if not isinstance(finetune, bool):
                raise ValueError("finetuneはboolです")
            feature_cache = body.get("feature_cache", True)
            if not isinstance(feature_cache, bool):
                raise ValueError("feature_cacheはboolです")
            workers = self.integer(body.get("num_workers", 8), 0, 64)
            lr = self.number(body.get("learning_rate", 1e-5 if finetune else 1e-4), 1e-9, 1)
            if action == "distill":
                target = self.target("encoders", key)
                args = ["--train", *map(str, groups[0]), "--validation", *map(str, groups[1]),
                        "--repo", str(repo), "--teacher-weights", str(weights), "--output", "@OUTPUT@",
                        "--epochs", str(epochs), "--batch-size", str(batch), "--learning-rate", str(lr),
                        "--device", cfg["device"], "--num-workers", str(workers),
                        "--cache-dir", str(self.root / '.feature_cache'),
                        "--feature-cache" if feature_cache else "--no-feature-cache"]
                return {**common, "kind":"encoders", "target":target,
                        "key":target.relative_to(self.root/"encoders").as_posix(), "module":"distill", "args":args,
                        "provenance":dict(architecture="dinov3_vitt16", teacher_run=body.get("teacher_run"),
                                          train=body["train"], validation=body["validation"], epochs=epochs),
                        "resources":["e2e-compute", *source_resources,
                                     *["e2e:datasets:"+item for item in (*body["train"],*body["validation"])]]}
            args = [
                "--train",
                *map(str, groups[0]),
                "--validation",
                *map(str, groups[1]),
                "--repo",
                str(repo),
                "--weights",
                str(weights),
                "--output",
                "@OUTPUT@",
                "--mode",
                mode,
                "--fixed-throttle",
                str(fixed),
                "--max-throttle",
                str(maximum),
                "--epochs",
                str(epochs),
                "--batch-size",
                str(batch),
                "--learning-rate",
                str(lr),
                "--device",
                cfg["device"],
            ]
            args += ["--modality", modality, "--architecture", architecture, "--num-workers", str(workers), "--cache-dir", str(self.root / '.feature_cache'),
                     "--feature-cache" if feature_cache else "--no-feature-cache"]
            if finetune:
                args.append("--finetune")
            return {
                **common,
                "kind": "runs",
                "target": target,
                "module": "train",
                "args": args,
                "provenance": {
                    "mode": mode,
                    "modality": modality,
                    "architecture": architecture,
                    "encoder": body.get("encoder") if architecture=="dinov3_vitt16" else None,
                    "train": body["train"],
                    "validation": body["validation"],
                    "epochs": epochs,
                    "finetune": finetune,
                    "learning_rate": lr,
                    "num_workers": workers,
                    "feature_cache": feature_cache,
                    "feature_cache_effective": feature_cache and not finetune,
                },
                "resources": ["e2e-compute", *source_resources, *["e2e:datasets:" + item for item in (*body["train"], *body["validation"])]],
            }
        if action == "build-engine":
            model = self.folder("models", body.get("model"))
            self.contract(model)
            return {
                **common,
                "kind": "engines",
                "target": self.target("engines", key),
                "module": "build_engine",
                "args": ["--model", str(model), "--output", "@OUTPUT@"],
                "provenance": {"model": body["model"]},
                "resources": ["e2e-compute", "e2e:models:" + body["model"]],
            }
        if action == "features":
            run = self.folder("runs", body.get("run"))
            dataset = self.folder("datasets", body.get("dataset"))
            checkpoint = within(run, "best.pt")
            within(dataset, "samples.jsonl")
            run_modality = read_json(within(run, "artifact.json")).get("modality", "rgb")
            dataset_meta = within(dataset, "metadata.json", exists=False)
            dataset_modality = read_json(dataset_meta if dataset_meta.is_file() else within(dataset, "artifact.json")).get("modality", "rgb")
            if run_modality != dataset_modality:
                raise ValueError("学習済み実験とデータセットの入力種別が一致しません")
            target = self.target("features", key)
            maximum = self.integer(body.get("max_frames", 12), 1, 64)
            repo = Path(cfg["encoder_repo"]).expanduser()
            if not repo.is_dir():
                raise ValueError("DINOv3ソースのパスを確認してください")
            return {**common, "kind": "features", "target": target,
                    "key": target.relative_to(self.root / "features").as_posix(),
                    "module": "features",
                    "args": ["--checkpoint", str(checkpoint), "--dataset", str(dataset),
                             "--repo", str(repo), "--output", "@OUTPUT@",
                             "--device", cfg["device"], "--max-frames", str(maximum)],
                    "provenance": {"run": body["run"], "dataset": body["dataset"]},
                    "resources": ["e2e-compute", "e2e:runs:" + body["run"],
                                  "e2e:datasets:" + body["dataset"]]}
        if action == "evaluate":
            target = self.target("evaluations", key)
            model = self.folder("models", body.get("model"))
            self.contract(model)
            if read_json(within(model, "metadata.json")).get("modality") == "evs":
                dataset = self.folder("datasets", body.get("dataset"))
                if read_json(within(dataset, "metadata.json")).get("modality") != "evs":
                    raise ValueError("EVS評価用データセットを選択してください")
                return {**common, "kind":"evaluations", "target":target,
                        "key":target.relative_to(self.root/"evaluations").as_posix(),
                        "module":"evaluate", "args":["--dataset",str(dataset),"--model",str(model),"--output","@OUTPUT@"],
                        "provenance":{"dataset":body["dataset"],"model":body["model"],"modality":"evs"},
                        "resources":["e2e-compute","e2e:models:"+body["model"],"e2e:datasets:"+body["dataset"]]}
            # Share extraction validation with dataset creation; do not publish images.
            extraction = self.prepare(
                "dataset", dict(body, name="eval-" + uuid.uuid4().hex)
            )
            args = [
                "--bag",
                extraction["args"][0],
                "--model",
                str(model),
                "--output",
                "@OUTPUT@",
                *extraction["args"][2:],
            ]
            return {
                **common,
                "kind": "evaluations",
                "target": target,
                "key": target.relative_to(self.root / "evaluations").as_posix(),
                "module": "evaluate",
                "args": args,
                "provenance": {
                    "bag": extraction["provenance"]["bag"],
                    "model": body["model"],
                },
                "resources": [
                    "e2e-compute",
                    "e2e:models:" + body["model"],
                    *extraction["resources"],
                ],
            }
        if action == "export":
            run = self.folder("runs", body.get("run"))
            checkpoint = within(run, "best.pt")
            target = self.target("models", key)
            if not Path(cfg["encoder_repo"]).expanduser().is_dir():
                raise ValueError("DINOv3ソースのパスを確認してください")
            return {
                **common,
                "kind": "models",
                "key": target.relative_to(self.root / "models").as_posix(),
                "target": target,
                "module": "export_onnx",
                "args": [
                    str(checkpoint),
                    "@OUTPUT@",
                    "--repo",
                    str(Path(cfg["encoder_repo"]).expanduser()),
                ],
                "provenance": {"run": run.name},
                "resources": ["e2e-compute", "e2e:runs:" + run.name],
            }
        raise ValueError("不明な学習操作です")

    @staticmethod
    def number(value, minimum, maximum):
        value = float(value)
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"値は{minimum}〜{maximum}で指定してください")
        return value

    @classmethod
    def integer(cls, value, minimum, maximum):
        parsed = cls.number(value, minimum, maximum)
        if int(parsed) != parsed:
            raise ValueError("整数で指定してください")
        return int(parsed)

    def execute(self, job, plan):
        # Recheck confinement before creating the timestamp parents/staging directory.
        within(within(self.root, plan["kind"]), plan["target"].relative_to(self.root / plan["kind"]), exists=False)
        plan["target"].parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix=".work-", dir=plan["target"].parent
        ) as directory:
            output = Path(directory) / "output"
            args = [str(output) if v == "@OUTPUT@" else v for v in plan["args"]]
            from .remote_agent import signature

            before = (
                signature(Path(plan["provenance"]["bag"]), ignore_raw_indexes=plan["module"] == "preprocess_evs")
                if "bag" in plan["provenance"] and plan["kind"] in ("datasets", "evaluations")
                else None
            )
            job.run(
                [plan["python"], "-m", "kart_e2e." + plan["module"], *args],
                env=plan["env"],
            )
            if before is not None and before != signature(
                Path(plan["provenance"]["bag"]), ignore_raw_indexes=plan["module"] == "preprocess_evs"
            ):
                raise ValueError(
                    "処理中にbagが更新されました。録画停止後に再実行してください"
                )
            metadata = {
                **plan["provenance"],
                "created": time.time(),
                "kind": plan["kind"],
            }
            if plan["kind"] == "datasets":
                data = read_json(within(output, "metadata.json"))
                within(output, "samples.jsonl")
                if data.get("samples", 0) <= 0:
                    raise ValueError("有効なデータがありません")
                metadata["samples"] = data["samples"]
                metadata["image_format"] = data.get("image_format", "png")
            elif plan["kind"] == "engines":
                report = read_json(within(output, "report.json"))
                if within(output, "model.plan").stat().st_size <= 0:
                    raise ValueError("Empty TensorRT engine")
                metadata.update(
                    mode=report["mode"], engine_size_bytes=report["engine_size_bytes"]
                )
            elif plan["kind"] == "evaluations":
                report = read_json(within(output, "report.json"))
                within(output, "predictions.csv")
                if report.get("samples", 0) <= 0:
                    raise ValueError("評価サンプルがありません")
                metadata.update(
                    samples=report["samples"],
                    mode=report["mode"],
                    metrics=report["metrics"],
                )
            elif plan["kind"] == "features":
                report = read_json(within(output, "report.json"))
                if report.get("schema") not in ("kart.rgb_features.v1", "kart.evs_features.v1") or not 1 <= report.get("frame_count", 0) <= 64:
                    raise ValueError("Invalid feature report")
                for frame in report["frames"]:
                    for kind in (("rgb", "pca", "saliency", "channels") if report["schema"] == "kart.evs_features.v1" else ("rgb", "pca", "saliency")):
                        within(output, frame[kind])
                metadata.update(frame_count=report["frame_count"], mode=report["mode"], modality=report.get("modality", "rgb"))
            elif plan["kind"] in ("runs", "encoders"):
                within(output, "encoder.pt" if plan["kind"] == "encoders" else "best.pt")
                rows = within(output, "metrics.jsonl").read_text().splitlines()
                import json

                metadata["metrics"] = json.loads(rows[-1])
            else:
                _, mode = self.contract(output)
                metadata["mode"] = mode
                metadata["modality"] = read_json(output / "metadata.json").get("modality", "rgb")
                metadata["ort_max_abs_error"] = read_json(output / "metadata.json")[
                    "ort_max_abs_error"
                ]
            atomic_json(output / "artifact.json", metadata)
            job.check()
            if plan["target"].exists():
                raise ValueError("同名の成果物があります")
            os.rename(output, plan["target"])
            return {
                "kind": plan["kind"],
                "id": plan["key"],
                "path": str(plan["target"]),
            }

    def push(self, job, raw, key, model_root):
        p = transfers.profile(raw)
        root = transfers.remote_path(model_root)
        source = self.folder("models", key)
        self.contract(source)
        stage = ".kart-upload-" + uuid.uuid4().hex
        destination = name(source.name) + "_" + uuid.uuid4().hex[:8]
        with tempfile.TemporaryDirectory(prefix="kart-model-") as directory:
            bundle = Path(directory) / "bundle"
            bundle.mkdir()
            for filename in ("model.onnx", "metadata.json"):
                shutil.copyfile(within(source, filename), bundle / filename)
            self.contract(bundle)
            from .remote_agent import model_manifest

            hashes = model_manifest(bundle)
            job.check()
            transfers.request(p, "reserve", root, stage)
            job.log(
                f"ONNX送信先: {root}/{destination}（TensorRT engineはJetsonで生成）"
            )
            transfers.run_transfer(
                job,
                [
                    "scp",
                    *transfers.SSH_OPTIONS,
                    "-P",
                    str(p["port"]),
                    "-r",
                    str(bundle),
                    f"{p['user']}@{p['host']}:{root}/{stage}/",
                ],
                sum(p.stat().st_size for p in bundle.iterdir()),
                lambda: transfers.request(p, "progress", root, stage)["bytes"],
            )
            job.check()
            return transfers.request(
                p, "publish-model", root, stage, destination=destination, hashes=hashes
            )
