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


KINDS = ("datasets", "runs", "models", "evaluations", "engines", "features", "encoders", "routings", "trajectories")
DATED = ("datasets", "models", "evaluations", "features", "encoders", "routings", "trajectories")


class Learning:
    def __init__(self, repo, base, records, state):
        self.repo, self.records = repo, records
        self.root = (base / "e2e").resolve()
        self.settings_file = state / "e2e.json"
        for kind in KINDS:
            (self.root / kind).mkdir(parents=True, exist_ok=True)
        for package in ('kart_e2e','kart_hdmap'):
            path=str(repo/'ros2_ws/src'/package)
            if path not in sys.path:sys.path.insert(0,path)
        spec = importlib.util.spec_from_file_location(
            "kart_e2e._ui_contract", repo / "ros2_ws/src/kart_e2e/kart_e2e/contract.py"
        )
        contract = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(contract)
        self.contract = contract.model_contract
        self.evs_spec = contract.evs_spec
        self.evs_policy_spec = contract.evs_policy_spec

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
            base_weights=str(self.repo / "weights/dinov3/dinov3_vitb16.pth"),
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
                    row={**read_json(artifact), "id": relative, "name": path.name, "pipeline": self.artifact_pipeline(kind,relative)}
                    extra=within(path,'metadata.json',exists=False)
                    if kind in ('datasets','models') and extra.is_file():
                        info=read_json(extra)
                        for field in ('routing','routing_sha256','section_counts','selected_sections','section_assignment'):
                            if field in info:row[field]=info[field]
                    entries.append(row)
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
            values = {field: row[field] for field in ("epoch", *fields)}
            for field in ("train_loss", "validation_loss", "learning_rate", "next_learning_rate"):
                if field in row:
                    value = row[field]
                    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                        raise ValueError("Invalid loss/LR metric")
                    values[field] = value
            metrics.append(values)
        if not metrics:
            raise ValueError("損失の記録がありません")
        return dict(id=key, kind=kind, metric="distillation_loss" if kind=="encoders" else "mse", metrics=metrics)

    def runtime(self, action=None, modality="rgb"):
        cfg = self.settings()
        field = (
            "evs_raw_python" if action == "dataset" and modality == "evs" else
            "inference_python"
            if action in ("export", "evaluate", "build-engine", "quantize")
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

    def artifact_pipeline(self, kind, key, depth=0):
        if depth>3:raise ValueError('Invalid artifact reference chain')
        folder=self.folder(kind,key)
        row=read_json(folder/'artifact.json')
        if kind in ('routings','trajectories'):return 'multi'
        extra=within(folder,'run.json' if kind=='runs' else 'metadata.json',exists=False)
        info=read_json(extra) if extra.is_file() else {}
        if row.get('section_heads') or row.get('routing') or row.get('routing_sha256') or info.get('routing') or info.get('routing_sha256'):
            return 'multi'
        if kind=='datasets':return 'shared'
        reference='run' if kind=='features' else 'model' if kind in ('engines','evaluations') else None
        if reference and row.get(reference):
            source='runs' if reference=='run' else 'models'
            try:return self.artifact_pipeline(source,row[reference],depth+1)
            except (OSError,ValueError):pass
        return 'multi' if row.get('pipeline')=='multi' else 'single'

    def prepare(self, action, body):
        requested=body.get('pipeline')
        if requested not in (None,'single','multi'):raise ValueError('学習pipelineは通常またはMulti-headを選択してください')
        plan=self._prepare(action,body)
        if action=='dataset':actual='shared'
        elif action.startswith('section-'):actual='multi'
        elif action=='train':actual='multi' if plan.get('routing') else 'single'
        elif action=='distill':actual='single'
        elif action in ('export','features'):actual=self.artifact_pipeline('runs',body['run'])
        else:actual=self.artifact_pipeline('models',body['model'])
        if requested and actual!='shared' and requested!=actual:
            raise ValueError('通常モデルとMulti-headの入力は混在できません。上部で該当pipelineを選択してください')
        if actual=='multi' and action in ('features','quantize'):
            raise ValueError('Multi-headでは特徴量可視化・INT8比較は未対応です')
        plan['pipeline']=actual
        return plan

    def _prepare(self, action, body):
        if action in ('section-routing','section-offline','section-dataset'):
            from .section_learning import prepare
            return prepare(self,action,body)
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
                        "resources":["read:record:"+str(bag.relative_to(self.records.resolve()))]}
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
                "resources": ["read:record:" + str(bag.relative_to(self.records.resolve()))],
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
            from .section_learning import training_routing
            section_routing = training_routing(self,groups,action)
            if action == "distill" and modality != "rgb":
                raise ValueError("Tiny特徴蒸留はRGBのみ対応しています")
            mode = body.get("mode", "steer_throttle")
            if mode not in ("steer_throttle", "steer_only"):
                raise ValueError("予測モードが不正です")
            repo = Path(cfg["encoder_repo"]).expanduser()
            architecture = body.get("architecture", "dinov3_vits16")
            if architecture not in ("dinov3_vits16", "dinov3_vitt16", "dinov3_vitb16", "simple_cnn"):
                raise ValueError("不明なモデルです")
            initialization = body.get("initialization", "pretrained")
            if initialization not in ("pretrained", "scratch"):
                raise ValueError("初期化方法が不正です")
            scratch = initialization == "scratch"
            if scratch and (action != "train" or (modality == "evs" and architecture != "simple_cnn")):
                raise ValueError("scratchはRGBまたはEVS CNNの制御学習に対応しています")
            if architecture == "simple_cnn" and not scratch:
                raise ValueError("CNNはscratchを選択してください")
            source_resources = []
            weights = Path(cfg["evs_weights"] if modality == "evs" else cfg["base_weights"] if architecture == "dinov3_vitb16" else cfg["weights"]).expanduser()
            if modality == "evs" and architecture not in ("dinov3_vits16", "simple_cnn"):
                raise ValueError("EVSは20ch ViT-SまたはPilotNet風CNNを選択してください")
            if modality == "evs" and architecture == "simple_cnn":
                for group in groups:
                    for dataset in group:
                        data_meta = read_json(dataset / "metadata.json")
                        input_spec = self.evs_spec(data_meta['evs_metadata'])
                        self.evs_policy_spec(data_meta['evs_metadata'], 'simple_cnn')
                        if data_meta.get('model_spec') != input_spec:
                            raise ValueError("EVS datasetの入力契約が一致しません")
            if action == "distill" and body.get("teacher_run"):
                teacher = self.folder("runs", body["teacher_run"])
                weights = within(teacher, "best.pt")
                source_resources.append("read:e2e:runs:" + body["teacher_run"])
            elif action == "train" and architecture == "dinov3_vitt16" and not scratch:
                source = self.folder("encoders", body.get("encoder"))
                weights = within(source, "encoder.pt")
                source_resources.append("read:e2e:encoders:" + body["encoder"])
            if architecture != "simple_cnn" and not repo.is_dir():
                raise ValueError(
                    f"DINOv3ソースが見つかりません: {repo}。"
                    "Notebookのコンテナ内で cd /workspaces && vcs import . < e2e.repos "
                    "を実行し、UI「学習環境」のDINOv3ソースを確認してください。重みとは別に必要です"
                )
            if not scratch and not weights.is_file():
                raise ValueError(
                    f"{'EVS encoder重み' if modality == 'evs' else 'DINOv3公式重み'}が見つかりません: {weights}。"
                    "UI「学習環境」の重みパスをコンテナ内のパスで指定してください"
                )
            epochs = self.integer(body.get("epochs", 10 if action == "distill" else 100), 1, 100000)
            batch = self.integer(body.get("batch_size", 32), 1, 4096)
            fixed = self.number(body.get("fixed_throttle", 0.0), 0, 1)
            maximum = self.number(body.get("max_throttle", 0.2), 0, 1)
            if fixed > maximum:
                raise ValueError("固定スロットルは上限以下にしてください")
            finetune = body.get("finetune", False)
            if not isinstance(finetune, bool):
                raise ValueError("finetuneはboolです")
            if scratch:
                finetune = True
            feature_cache = body.get("feature_cache", True)
            if not isinstance(feature_cache, bool):
                raise ValueError("feature_cacheはboolです")
            ram_preload = body.get("ram_preload", True)
            if not isinstance(ram_preload, bool):
                raise ValueError("ram_preloadはboolです")
            workers = self.integer(body.get("num_workers", 8), 0, 64)
            lr = self.number(body.get("learning_rate", 1e-4 if scratch else (1e-5 if finetune else 1e-4)), 1e-9, 1)
            if action == "distill":
                target = self.target("encoders", key)
                args = ["--train", *map(str, groups[0]), "--validation", *map(str, groups[1]),
                        "--repo", str(repo), "--teacher-weights", str(weights), "--output", "@OUTPUT@",
                        "--epochs", str(epochs), "--batch-size", str(batch), "--learning-rate", str(lr),
                        "--device", cfg["device"], "--num-workers", str(workers),
                        "--cache-dir", str(self.root / '.feature_cache'),
                        "--feature-cache" if feature_cache else "--no-feature-cache",
                        "--ram-preload" if ram_preload else "--no-ram-preload"]
                return {**common, "kind":"encoders", "target":target,
                        "key":target.relative_to(self.root/"encoders").as_posix(), "module":"distill", "args":args,
                        "provenance":dict(architecture="dinov3_vitt16", teacher_run=body.get("teacher_run"),
                                          train=body["train"], validation=body["validation"], epochs=epochs, ram_preload=ram_preload),
                        "resources":[*source_resources,
                                     *["read:e2e:datasets:"+item for item in (*body["train"],*body["validation"])]]}
            loss = body.get("loss", "smooth_l1")
            scheduler = body.get("lr_scheduler", "plateau")
            if loss not in ("smooth_l1", "mse") or scheduler not in ("none", "plateau"):
                raise ValueError("損失関数またはschedulerが不正です")
            beta = self.number(body.get("smooth_l1_beta", 0.1), 1e-9, 100)
            factor = self.number(body.get("lr_factor", 0.5), 1e-9, 0.999999)
            patience = self.integer(body.get("lr_patience", 2), 0, 100000)
            minimum_lr = self.number(body.get("min_lr", 1e-7), 0, 1)
            if scheduler != "none" and minimum_lr > lr:
                raise ValueError("最低学習率は初期学習率以下にしてください")
            args = [
                "--train",
                *map(str, groups[0]),
                "--validation",
                *map(str, groups[1]),
                "--repo",
                str(repo),
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
            precision = body.get("precision", "auto")
            if precision not in ("auto", "fp32", "bf16", "fp16"):
                raise ValueError("学習精度はauto / fp32 / bf16 / fp16です")
            if precision in ("bf16", "fp16") and not cfg["device"].startswith("cuda"):
                raise ValueError("BF16/FP16の明示指定にはCUDAが必要です。CPUでは自動またはFP32を選択してください")
            profile_batches = self.integer(body.get("profile_batches", 20), 0, 1000)
            log_interval = self.integer(body.get("log_interval", 50), 1, 100000)
            args += ["--precision", precision, "--profile-batches", str(profile_batches), "--log-interval", str(log_interval)]
            seed = self.integer(body.get("seed", 42), 0, 2147483647)
            args += ["--initialization", initialization, "--seed", str(seed)]
            if not scratch:
                args += ["--weights", str(weights)]
            args += ["--loss", loss, "--smooth-l1-beta", str(beta), "--lr-scheduler", scheduler,
                     "--lr-factor", str(factor), "--lr-patience", str(patience), "--min-lr", str(minimum_lr)]
            args += ["--modality", modality, "--architecture", architecture, "--num-workers", str(workers), "--cache-dir", str(self.root / '.feature_cache'),
                     "--feature-cache" if feature_cache else "--no-feature-cache",
                     "--ram-preload" if ram_preload else "--no-ram-preload"]
            if finetune:
                args.append("--finetune")
            if section_routing:
                args += ['--section-routing','@ROUTING@']
            return {
                **common,
                "kind": "runs",
                "routing": section_routing,
                "target": target,
                "module": "train",
                "args": args,
                "provenance": {
                    "mode": mode,
                    "section_heads": section_routing["heads"] if section_routing else [],
                    "modality": modality,
                    "architecture": architecture,
                    "initialization": initialization, "seed": seed, "precision": precision,
                    "profile_batches": profile_batches, "log_interval": log_interval,
                    "encoder": body.get("encoder") if architecture=="dinov3_vitt16" and not scratch else None,
                    "train": body["train"],
                    "validation": body["validation"],
                    "epochs": epochs,
                    "finetune": finetune,
                    "learning_rate": lr,
                    "loss": loss, "smooth_l1_beta": beta, "lr_scheduler": scheduler,
                    "lr_factor": factor, "lr_patience": patience, "min_lr": minimum_lr,
                    "num_workers": workers, "ram_preload": ram_preload,
                    "feature_cache": feature_cache,
                    "feature_cache_effective": feature_cache and not finetune,
                },
                "resources": [*source_resources, *["read:e2e:datasets:" + item for item in (*body["train"], *body["validation"])]],
            }
        if action == "quantize":
            model = self.folder("models", body.get("model"))
            self.contract(model)
            meta = read_json(within(model, "metadata.json"))
            if meta.get("quantization"):
                raise ValueError("INT8校正には元のFP32モデルを選択してください")
            datasets = []
            for field in ("calibration", "evaluation"):
                dataset = self.folder("datasets", body.get(field))
                dataset_meta = read_json(within(dataset, "metadata.json"))
                if dataset_meta.get("modality", "rgb") != meta.get("modality", "rgb"):
                    raise ValueError("モデルと校正・評価データセットの入力種別が一致しません")
                if meta.get("modality") == "evs":
                    if dataset_meta.get("model_spec") != self.evs_spec(meta["evs_metadata"]):
                        raise ValueError("EVS校正・評価データのSPECが一致しません")
                datasets.append(dataset)
            target = self.target("models", key)
            calibration_samples = self.integer(body.get("calibration_samples", 256), 1, 100000)
            evaluation_samples = self.integer(body.get("evaluation_samples", 256), 1, 100000)
            seed = self.integer(body.get("seed", 42), 0, 2147483647)
            warmup = self.integer(body.get("warmup", 10), 1, 1000)
            repeats = self.integer(body.get("repeats", 3), 1, 20)
            return {**common, "kind":"models", "target":target,
                    "key":target.relative_to(self.root/"models").as_posix(), "module":"int8_benchmark",
                    "args":["--model",str(model),"--calibration",str(datasets[0]),"--evaluation",str(datasets[1]),
                            "--output","@OUTPUT@","--calibration-samples",str(calibration_samples),
                            "--evaluation-samples",str(evaluation_samples),"--seed",str(seed),
                            "--warmup",str(warmup),"--repeats",str(repeats)],
                    "provenance":{"source_model":body["model"],"calibration":body["calibration"],
                                  "evaluation":body["evaluation"],"quantization":"int8"},
                    "resources":["read:e2e:models:"+body["model"],
                                 "read:e2e:datasets:"+body["calibration"],"read:e2e:datasets:"+body["evaluation"]]}
        if action == "build-engine":
            precision = body.get("precision", "fp32")
            if precision not in ("fp16", "fp32"):
                raise ValueError("TensorRT build精度はfp16またはfp32です")
            model = self.folder("models", body.get("model"))
            self.contract(model)
            if read_json(within(model, "metadata.json")).get("quantization"):
                raise ValueError("校正済みINT8モデルは元のFP32/FP16モデルへ戻せません")
            return {
                **common,
                "kind": "engines",
                "target": self.target("engines", key),
                "module": "build_engine",
                "args": ["--model", str(model), "--output", "@OUTPUT@", "--precision", precision],
                "provenance": {"model": body["model"], "precision": precision},
                "resources": ["read:e2e:models:" + body["model"]],
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
            if read_json(within(run, "artifact.json")).get("architecture") != "simple_cnn" and not repo.is_dir():
                raise ValueError("DINOv3ソースのパスを確認してください")
            return {**common, "kind": "features", "target": target,
                    "key": target.relative_to(self.root / "features").as_posix(),
                    "module": "features",
                    "args": ["--checkpoint", str(checkpoint), "--dataset", str(dataset),
                             "--repo", str(repo), "--output", "@OUTPUT@",
                             "--device", cfg["device"], "--max-frames", str(maximum)],
                    "provenance": {"run": body["run"], "dataset": body["dataset"]},
                    "resources": ["read:e2e:runs:" + body["run"],
                                  "read:e2e:datasets:" + body["dataset"]]}
        if action == "evaluate":
            target = self.target("evaluations", key)
            model = self.folder("models", body.get("model"))
            self.contract(model)
            if read_json(within(model, "metadata.json")).get("modality") == "evs" or read_json(within(model, "metadata.json")).get("routing"):
                dataset = self.folder("datasets", body.get("dataset"))
                if read_json(within(dataset, "metadata.json")).get("modality", "rgb") != read_json(within(model,"metadata.json")).get("modality", "rgb"):
                    raise ValueError("EVS評価用データセットを選択してください" if read_json(model/"metadata.json").get("modality")=="evs" else "モデルと同じ入力種別の評価datasetを選択してください")
                return {**common, "kind":"evaluations", "target":target,
                        "key":target.relative_to(self.root/"evaluations").as_posix(),
                        "module":"evaluate", "args":["--dataset",str(dataset),"--model",str(model),"--output","@OUTPUT@"],
                        "provenance":{"dataset":body["dataset"],"model":body["model"],"modality":read_json(model/"metadata.json").get("modality","rgb")},
                        "resources":["read:e2e:models:"+body["model"],"read:e2e:datasets:"+body["dataset"]]}
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
                    "read:e2e:models:" + body["model"],
                    *extraction["resources"],
                ],
            }
        if action == "export":
            run = self.folder("runs", body.get("run"))
            checkpoint = within(run, "best.pt")
            target = self.target("models", key)
            if read_json(within(run, "artifact.json")).get("architecture") != "simple_cnn" and not Path(cfg["encoder_repo"]).expanduser().is_dir():
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
                "resources": ["read:e2e:runs:" + run.name],
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
            replacements={'@OUTPUT@':str(output)}
            if plan.get('request') is not None:
                request=Path(directory)/'request.json';atomic_json(request,plan['request'])
                replacements['@REQUEST@']=str(request)
            if plan.get('routing') is not None:
                routing=Path(directory)/'routing.json';atomic_json(routing,plan['routing'])
                replacements['@ROUTING@']=str(routing)
            args = [replacements.get(v,v) for v in plan["args"]]
            from .remote_agent import signature

            before = (
                signature(Path(plan["provenance"]["bag"]), ignore_raw_indexes=plan["module"] == "preprocess_evs")
                if "bag" in plan["provenance"] and plan["module"] in ("preprocess_bag", "preprocess_evs", "evaluate", "section_workflow")
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
                "pipeline": plan.get("pipeline", "single"),
            }
            if plan["kind"] == "datasets":
                data = read_json(within(output, "metadata.json"))
                within(output, "samples.jsonl")
                if data.get("samples", 0) <= 0:
                    raise ValueError("有効なデータがありません")
                metadata["samples"] = data["samples"]
                metadata["image_format"] = data.get("image_format", "png")
                for field in ('routing_sha256','routing','section_counts','selected_sections','section_assignment'):
                    if field in data:metadata[field]=data[field]
            elif plan["kind"] in ('routings','trajectories'):
                data=read_json(within(output,'metadata.json'))
                if plan['kind']=='routings':
                    from .section_learning import modules
                    modules(self)[0](read_json(within(output,'routing.json')))
                elif data.get('valid_poses',0)<1:
                    raise ValueError('既存mapに局在した有効poseがありません')
                metadata.update(data)
            elif plan["kind"] == "engines":
                report = read_json(within(output, "report.json"))
                if within(output, "model.plan").stat().st_size <= 0:
                    raise ValueError("Empty TensorRT engine")
                metadata.update(
                    mode=report["mode"], engine_size_bytes=report["engine_size_bytes"], precision=report.get("precision", "fp32")
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
                run_info = within(output, "run.json", exists=False)
                if plan["kind"] == "runs" and run_info.is_file():
                    metadata["effective_precision"] = read_json(run_info).get("effective_precision", "fp32")
            else:
                if plan["module"] == "int8_benchmark":
                    comparison = read_json(within(output, "benchmark/report.json"))
                    if comparison.get("schema") != "kart.int8_benchmark.v1" or comparison.get("samples", 0) < 1:
                        raise ValueError("Invalid INT8 benchmark report")
                    for filename in ("benchmark/fp32/model.plan", "benchmark/int8/model.plan", "benchmark/predictions.csv"):
                        within(output, filename)
                    metadata["benchmark_summary"] = dict(
                        steering_mae_delta=comparison["metric_delta_int8_minus_fp32"]["steering_command"]["mae"],
                        speedup_wall_median=comparison["speedup_wall_median"],
                        speedup_gpu_median=comparison["speedup_gpu_median"])
                _, mode = self.contract(output)
                metadata["mode"] = mode
                metadata["modality"] = read_json(output / "metadata.json").get("modality", "rgb")
                if read_json(output / "metadata.json").get("routing"):
                    metadata["routing"] = read_json(output / "metadata.json")["routing"]
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
