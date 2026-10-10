"""Offline learning jobs and model bundles. No vehicle startup or arbitrary shell commands."""

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


class Learning:
    def __init__(self, repo, base, records, state):
        self.repo, self.records = repo, records
        self.root = base / "e2e"
        self.settings_file = state / "e2e.json"
        for kind in ("datasets", "runs", "models", "evaluations", "engines"):
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
            encoder_repo=str(self.repo / "python_ws/dinov3"),
            weights=str(
                self.repo
                / "weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth"
            ),
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
        for kind in ("datasets", "runs", "models", "evaluations", "engines"):
            entries = []
            for path in (self.root / kind).iterdir():
                if path.name.startswith(".") or path.is_symlink() or not path.is_dir():
                    continue
                try:
                    entries.append(
                        {**read_json(within(path, "artifact.json")), "id": path.name}
                    )
                except (ValueError, OSError):
                    continue
            result[kind] = sorted(
                entries, key=lambda item: item["created"], reverse=True
            )
        return result

    def folder(self, kind, key):
        path = within(self.root / kind, name(key))
        within(path, "artifact.json")
        return path

    def runtime(self, action=None):
        cfg = self.settings()
        field = (
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
        target = within(self.root / kind, name(key), exists=False)
        if target.exists():
            raise ValueError("同じ名前の成果物が存在します。別名にしてください")
        return target

    def prepare(self, action, body):
        key = name(body.get("name"))
        python, cfg, env = self.runtime(action)
        common = dict(key=key, python=python, env=env)
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
            args = [str(bag), "@OUTPUT@", "--clock", clock, "--max-skew-ms", str(skew)]
            for topic, value in topics.items():
                args += ["--" + topic.replace("_", "-"), value]
            return {
                **common,
                "kind": "datasets",
                "target": target,
                "module": "preprocess_bag",
                "args": args,
                "provenance": {"bag": str(bag)},
                "resources": ["record:" + str(bag.relative_to(self.records.resolve()))],
            }
        if action == "train":
            target = self.target("runs", key)
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

            mode = body.get("mode", "steer_throttle")
            if mode not in ("steer_throttle", "steer_only"):
                raise ValueError("予測モードが不正です")
            repo = Path(cfg["encoder_repo"]).expanduser()
            weights = Path(cfg["weights"]).expanduser()
            if not repo.is_dir():
                raise ValueError(
                    f"DINOv3ソースが見つかりません: {repo}。"
                    "Notebookのコンテナ内で cd /workspaces && vcs import . < e2e.repos "
                    "を実行し、UI「学習環境」のDINOv3ソースを確認してください。重みとは別に必要です"
                )
            if not weights.is_file():
                raise ValueError(
                    f"DINOv3公式重みが見つかりません: {weights}。"
                    "UI「学習環境」の重みパスをコンテナ内のパスで指定してください"
                )
            epochs = self.integer(body.get("epochs", 20), 1, 100000)
            batch = self.integer(body.get("batch_size", 32), 1, 4096)
            lr = self.number(body.get("learning_rate", 0.0001), 1e-9, 1)
            fixed = self.number(body.get("fixed_throttle", 0.0), 0, 1)
            maximum = self.number(body.get("max_throttle", 0.2), 0, 1)
            if fixed > maximum:
                raise ValueError("固定スロットルは上限以下にしてください")
            finetune = body.get("finetune", False)
            if not isinstance(finetune, bool):
                raise ValueError("finetuneはboolです")
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
                    "train": body["train"],
                    "validation": body["validation"],
                    "epochs": epochs,
                },
                "resources": ["e2e-compute"],
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
                "provenance": {"model": model.name},
                "resources": ["e2e-compute", "e2e:models:" + model.name],
            }
        if action == "evaluate":
            model = self.folder("models", body.get("model"))
            self.contract(model)
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
                "target": self.target("evaluations", key),
                "module": "evaluate",
                "args": args,
                "provenance": {
                    "bag": extraction["provenance"]["bag"],
                    "model": model.name,
                },
                "resources": [
                    "e2e-compute",
                    "e2e:models:" + model.name,
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
        with tempfile.TemporaryDirectory(
            prefix=".work-", dir=plan["target"].parent
        ) as directory:
            output = Path(directory) / "output"
            args = [str(output) if v == "@OUTPUT@" else v for v in plan["args"]]
            from .remote_agent import signature

            before = (
                signature(Path(plan["provenance"]["bag"]))
                if plan["kind"] in ("datasets", "evaluations")
                else None
            )
            job.run(
                [plan["python"], "-m", "kart_e2e." + plan["module"], *args],
                env=plan["env"],
            )
            if before is not None and before != signature(
                Path(plan["provenance"]["bag"])
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
            elif plan["kind"] == "runs":
                within(output, "best.pt")
                rows = within(output, "metrics.jsonl").read_text().splitlines()
                import json

                metadata["metrics"] = json.loads(rows[-1])
            else:
                _, mode = self.contract(output)
                metadata["mode"] = mode
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
        destination = name(key) + "_" + uuid.uuid4().hex[:8]
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
            job.run(
                [
                    "scp",
                    *transfers.SSH_OPTIONS,
                    "-P",
                    str(p["port"]),
                    "-r",
                    str(bundle),
                    f"{p['user']}@{p['host']}:{root}/{stage}/",
                ]
            )
            job.check()
            return transfers.request(
                p, "publish-model", root, stage, destination=destination, hashes=hashes
            )
