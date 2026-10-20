"""Small DINOv3 behavior-cloning trainer, with explicit session-level validation."""

import argparse
import json
import math
import random
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler

from .model import SPEC, Policy, backbone, preprocess
from .contract import RGB_ENCODERS
from .data import load_rgb


def regression_loss(prediction, target, kind="smooth_l1", beta=0.1):
    if kind == "mse":
        return torch.nn.functional.mse_loss(prediction, target)
    if kind != "smooth_l1" or not math.isfinite(beta) or beta <= 0:
        raise ValueError("Expected mse or smooth_l1 with beta > 0")
    return torch.nn.functional.smooth_l1_loss(prediction, target, beta=beta)


def plateau_scheduler(optimizer, *, factor=0.5, patience=2, min_lr=1e-7):
    return torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=factor, patience=patience, min_lr=min_lr)


class Samples(Dataset):
    def __init__(self, roots, mode, evs_spec=None, routing=None):
        self.items = []
        self.mode = mode
        self.evs_spec = evs_spec
        self.routing = routing
        for root in roots:
            if not routing and (root/'metadata.json').exists():
                if json.loads((root/'metadata.json').read_text()).get('routing_sha256'):
                    raise ValueError('Section-labeled datasets require --section-routing')
            if routing:
                from .sections import digest
                meta = json.loads((root / 'metadata.json').read_text())
                if meta.get('routing_sha256') != digest(routing):
                    raise ValueError('Section dataset routing differs from training routing')
            if evs_spec is None and (root / 'metadata.json').exists():
                if json.loads((root / 'metadata.json').read_text()).get('modality', 'rgb') != 'rgb':
                    raise ValueError('RGB training cannot consume an EVS dataset; use --modality evs')
            if evs_spec is not None:
                meta = json.loads((root / 'metadata.json').read_text())
                if meta.get('modality') != 'evs' or meta.get('model_spec') != evs_spec:
                    raise ValueError('EVS dataset/encoder contract mismatch')
            for line in (root / "samples.jsonl").read_text().splitlines():
                row = json.loads(line)
                image = (root / row["counts" if evs_spec is not None else "image"]).resolve(strict=True)
                if not image.is_relative_to(root.resolve()):
                    raise ValueError("Sample outside dataset")
                values = [float(row["steering"]), float(row["throttle"])]
                if (
                    not all(math.isfinite(v) for v in values)
                    or not -1 <= values[0] <= 1
                    or not 0 <= values[1] <= 1
                ):
                    raise ValueError("Invalid control label")
                if routing:
                    head = row.get('head_id')
                    if head not in routing['heads']:
                        raise ValueError('Sample lacks a valid head_id')
                    values.append(routing['heads'].index(head))
                self.items.append((image, values))
        if not self.items:
            raise ValueError("Empty dataset")

    def __len__(self):
        return len(self.items)

    def __getitem__(self, index):
        path, label = self.items[index]
        if self.evs_spec is not None:
            counts = np.load(path, allow_pickle=False)
            spec = self.evs_spec
            if (counts.shape != (20, spec['height'], spec['width'])
                    or counts.dtype != np.float32 or not np.isfinite(counts).all()
                    or (counts < 0).any()):
                raise ValueError('Invalid raw EVS counts')
            value = torch.from_numpy(counts)
        else:
            value = preprocess(load_rgb(path))
        target = label[:1] if self.mode == "steer_only" else label[:2]
        if self.routing: target = target + [label[-1]]
        return value, torch.tensor(target)


def main():
    setup_started = time.perf_counter()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--train", type=Path, nargs="+", required=True)
    p.add_argument("--validation", type=Path, nargs="+", required=True)
    p.add_argument("--repo", required=True)
    p.add_argument(
        "--weights",
        type=Path,
        required=False,
        help="Local pretrained encoder weights; not used with --initialization scratch",
    )
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--mode", choices=["steer_throttle", "steer_only"], default="steer_throttle"
    )
    p.add_argument(
        "--fixed-throttle",
        type=float,
        default=0.0,
        help="Steer-only deployment throttle",
    )
    p.add_argument(
        "--max-throttle", type=float, default=0.2, help="Deployment output ceiling"
    )
    p.add_argument("--architecture", choices=RGB_ENCODERS, default="dinov3_vits16")
    p.add_argument("--modality", choices=["rgb", "evs"], default="rgb")
    p.add_argument("--initialization", choices=["pretrained", "scratch"], default="pretrained")
    p.add_argument("--section-routing", type=Path, help="Jointly train shared encoder and section-assigned heads")
    p.add_argument("--finetune", action="store_true", help="Update the full backbone and control head")
    p.add_argument("--device", default="cuda")
    p.add_argument("--precision", choices=["auto", "fp32", "bf16", "fp16"], default="auto",
                   help="CUDA AMP: auto prefers native BF16, otherwise FP16; CPU auto uses FP32")
    p.add_argument("--epochs", type=int, default=100, help="Control training budget (default: 100 for pretrained and scratch)")
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--learning-rate", type=float, default=None,
                   help="AdamW learning rate for all trainable parameters (default: 1e-4 frozen/scratch, 1e-5 pretrained finetune)")
    p.add_argument("--loss", choices=["smooth_l1", "mse"], default="smooth_l1")
    p.add_argument("--smooth-l1-beta", type=float, default=0.1)
    p.add_argument("--lr-scheduler", choices=["none", "plateau"], default="plateau")
    p.add_argument("--lr-factor", type=float, default=0.5)
    p.add_argument("--lr-patience", type=int, default=2)
    p.add_argument("--min-lr", type=float, default=1e-7)
    p.add_argument("--num-workers", type=int, default=8)
    p.add_argument("--feature-cache", action=argparse.BooleanOptionalAction, default=True,
                   help="Cache frozen encoder features; bypassed during finetune")
    p.add_argument("--ram-preload", action=argparse.BooleanOptionalAction, default=True,
                   help="Preload training tensors in RAM; --no-ram-preload streams from disk")
    p.add_argument("--cache-dir", type=Path, default=None)
    p.add_argument("--profile-batches", type=int, default=20, help="CUDA event samples per phase; 0 disables GPU event sampling")
    p.add_argument("--log-interval", type=int, default=50, help="Detailed timing.jsonl snapshot every N batches; console progress refreshes at most every 0.5s")
    p.add_argument("--seed", type=int, default=42)
    a = p.parse_args()
    routing = None
    if a.section_routing:
        from .sections import load as load_routing
        routing = load_routing(a.section_routing)
    scratch = a.initialization == "scratch"
    if scratch and a.modality != "rgb":
        p.error("Scratch baselines currently support RGB only")
    if a.architecture == "simple_cnn" and not scratch:
        p.error("simple_cnn requires --initialization scratch")
    if scratch and a.weights is not None:
        p.error("Scratch must not load --weights; omit it")
    if not scratch and a.weights is None:
        p.error("Pretrained training requires --weights")
    if a.learning_rate is None:
        a.learning_rate = 1e-4 if scratch else (1e-5 if a.finetune else 1e-4)
    if scratch:
        a.finetune = True
    if (
        not 0 <= a.profile_batches <= 1000
        or not 1 <= a.log_interval <= 100000
        or a.num_workers < 0
        or a.epochs <= 0
        or a.batch_size <= 0
        or not math.isfinite(a.learning_rate)
        or a.learning_rate <= 0
    ):
        p.error("epochs/batch-size/lr positive; profile-batches 0..1000; log-interval 1..100000 required")

    if (not math.isfinite(a.smooth_l1_beta) or a.smooth_l1_beta <= 0
            or not math.isfinite(a.lr_factor) or not 0 < a.lr_factor < 1
            or a.lr_patience < 0 or not math.isfinite(a.min_lr) or a.min_lr < 0
            or (a.lr_scheduler != "none" and a.min_lr > a.learning_rate)):
        p.error("beta > 0, 0 < lr-factor < 1, patience >= 0, 0 <= min-lr <= learning-rate required")

    from .precision import resolve_precision, autocast_context
    effective_precision = resolve_precision(a.precision, a.device)
    scaler = torch.amp.GradScaler("cuda", enabled=effective_precision == "fp16")
    print(f"Precision: requested={a.precision}; effective={effective_precision}; grad_scaler={scaler.is_enabled()}", flush=True)

    from .contract import validate_runtime

    runtime = validate_runtime(
        {"fixed_throttle": a.fixed_throttle, "max_throttle": a.max_throttle}
    )
    if not 0 <= a.seed <= 2147483647:
        p.error("seed must be 0..2147483647")
    random.seed(a.seed)
    np.random.seed(a.seed)
    torch.manual_seed(a.seed)
    from .feature_cache import cached_samples, digest
    weights_sha = None if scratch else digest(a.weights)
    if a.modality == 'evs':
        if a.architecture != 'dinov3_vits16':
            p.error('ViT-Tiny distillation currently supports RGB only')
        from .evs_policy import pretrained_policy
        model = pretrained_policy(a.weights, a.repo, a.mode, frozen=not a.finetune).to(a.device)
        datasets = [Samples(r, a.mode, model.spec, routing) for r in (a.train, a.validation)]
    else:
        datasets = [Samples(a.train, a.mode, routing=routing), Samples(a.validation, a.mode, routing=routing)]
        if scratch:
            encoder = backbone(a.repo, a.architecture)
        elif a.architecture == 'dinov3_vits16':
            encoder = backbone(a.repo)
            encoder.load_state_dict(torch.load(a.weights, map_location="cpu", weights_only=True), strict=True)
        else:
            from .model import load_rgb_encoder
            encoder = load_rgb_encoder(a.weights, a.repo, a.architecture)
        model = Policy(encoder, a.mode, frozen=not a.finetune, architecture=a.architecture).to(a.device)
    head_ids = None
    if routing:
        model.configure_heads(routing).to(a.device)
        head_ids = [[int(label[-1]) for _, label in d.items] for d in datasets]
        if any(set(ids) != set(range(len(routing['heads']))) for ids in head_ids):
            raise ValueError('Every head requires both training and validation samples')
    model.runtime = runtime
    optimizer = torch.optim.AdamW(
        (v for v in model.parameters() if v.requires_grad), lr=a.learning_rate
    )
    scheduler = (plateau_scheduler(optimizer, factor=a.lr_factor, patience=a.lr_patience, min_lr=a.min_lr)
                 if a.lr_scheduler == "plateau" else None)
    if not scratch and digest(a.weights) != weights_sha:
        raise ValueError("Weights changed while loading")
    effective_cache = a.feature_cache and not a.finetune
    print(f"Feature cache: requested={a.feature_cache}; effective={effective_cache}; workers={a.num_workers}", flush=True)
    if a.output.exists():
        raise FileExistsError(a.output)

    from .ram_dataset import preload_datasets, loader_workers

    def loader_factory(dataset, shuffle, persistent=True, seed=a.seed):
        workers = loader_workers(dataset, a.num_workers)
        sampler = None
        if shuffle and head_ids is not None:
            ids = head_ids[0]
            counts = np.bincount(ids, minlength=len(routing['heads']))
            sampler = WeightedRandomSampler([1./counts[i] for i in ids], len(ids), replacement=True,
                                             generator=torch.Generator().manual_seed(seed))
        options = dict(batch_size=a.batch_size, shuffle=shuffle and sampler is None, sampler=sampler, num_workers=workers,
                       pin_memory=str(a.device).startswith('cuda'),
                       generator=torch.Generator().manual_seed(seed))
        if workers:
            options.update(multiprocessing_context='spawn', persistent_workers=persistent)
        return DataLoader(dataset, **options)

    a.output.mkdir(parents=True, exist_ok=False)
    from .training_timing import TrainingTiming
    timing = TrainingTiming(a.output, a.device, profile_batches=a.profile_batches, log_interval=a.log_interval, epochs=a.epochs)
    timing.record("setup", time.perf_counter()-setup_started, feature_cache=effective_cache, num_workers=a.num_workers)
    cache_keys = []
    if effective_cache:
        cached = []
        cache_dir = a.cache_dir or a.output.parent/'.feature_cache'
        for index, dataset in enumerate(datasets):
            data, key = cached_samples(dataset, model, cache_dir, weights_sha, loader_factory, a.device, timing=timing, timing_phase="cache_"+("train" if index==0 else "validation"))
            cached.append(data); cache_keys.append(key)
        datasets = cached
    if a.ram_preload:
        datasets = preload_datasets(datasets, timing=timing)
    effective_workers = loader_workers(datasets[0], a.num_workers)
    print(f"Data loading: ram_preload={a.ram_preload}; requested_workers={a.num_workers}; effective_workers={effective_workers}", flush=True)
    loaders = [loader_factory(d, i==0, seed=a.seed+i) for i,d in enumerate(datasets)]
    provenance = {
        key: str(value) if isinstance(value, Path) else value
        for key, value in vars(a).items()
    }
    provenance["train"] = [str(r.resolve()) for r in a.train]
    provenance["validation"] = [str(r.resolve()) for r in a.validation]
    provenance["trainable_parameters"] = sum(v.numel() for v in model.parameters() if v.requires_grad)
    provenance["total_parameters"] = sum(v.numel() for v in model.parameters())
    print(f"Training: finetune={a.finetune}; lr={a.learning_rate:g}; "
          f"trainable={provenance['trainable_parameters']}/{provenance['total_parameters']}", flush=True)
    provenance["effective_precision"] = effective_precision
    provenance["amp_enabled"] = effective_precision != "fp32"
    provenance["grad_scaler_enabled"] = scaler.is_enabled()
    provenance["feature_cache_precision"] = "fp32" if effective_cache else None
    provenance["selection_metric"] = "validation_mse"
    provenance["head_sampling"] = "equal_head_replacement" if routing else None
    provenance["routing"] = routing
    provenance["scheduler_metric"] = ("validation_mse" if routing else "validation_loss") if scheduler else None
    provenance["weights_sha256"] = weights_sha
    provenance["feature_cache_effective"] = effective_cache
    provenance["feature_cache_keys"] = cache_keys
    provenance["effective_num_workers"] = effective_workers
    provenance["ram_preload_bytes"] = sum(d.bytes for d in {id(value):value for value in datasets}.values()) if a.ram_preload else 0
    (a.output / "run.json").write_text(json.dumps(provenance, indent=2))
    best = float("inf")
    for epoch in range(a.epochs):
        epoch_started = time.perf_counter()
        metrics = {"epoch": epoch + 1, "learning_rate": optimizer.param_groups[0]["lr"]}
        for index, loader in enumerate(loaders):
            model.train(index == 0)
            total = 0.0
            total_mse = 0.0
            mae = torch.zeros(1 if a.mode == "steer_only" else 2, device=a.device)
            head_error = torch.zeros(len(routing["heads"]), device=a.device) if routing else None
            head_mae = torch.zeros_like(head_error) if routing else None
            head_count = torch.zeros_like(head_error) if routing else None
            phase = timing.phase("train" if index==0 else "validation", epoch+1)
            with torch.set_grad_enabled(index == 0):
                for images, target in phase.batches(loader):
                    with phase.measure("transfer"):
                        target = target.to(a.device, non_blocking=True)
                        selected_heads = target[:, -1].long() if routing else None
                        if routing: target = target[:, :-1]
                        inputs = images.to(a.device, non_blocking=True)
                    with autocast_context(effective_precision):
                        if effective_cache:
                            features = inputs
                        else:
                            with phase.measure("encoder"):
                                features = model.extract_features(inputs)
                        with phase.measure("head"):
                            prediction = model.predict_features(features)
                            if routing:
                                prediction = prediction.reshape(len(inputs), len(routing['heads']), -1)[
                                    torch.arange(len(inputs), device=a.device), selected_heads]
                    with phase.measure("loss"):
                        # Includes the pre-existing finite-check synchronization.
                        prediction = prediction.float()
                        loss = regression_loss(prediction, target.float(), a.loss, a.smooth_l1_beta)
                        if not torch.isfinite(loss):
                            raise RuntimeError("Non-finite training/validation loss")
                    if index == 0:
                        with phase.measure("zero_grad", gpu=False):
                            optimizer.zero_grad(set_to_none=True)
                        with phase.measure("backward"):
                            scaler.scale(loss).backward()
                        with phase.measure("optimizer"):
                            scaler.step(optimizer)
                            scaler.update()
                    with phase.measure("metrics"):
                        total += loss.item() * len(images)
                        total_mse += torch.nn.functional.mse_loss(prediction.detach(), target).item() * len(images)
                        mae += (prediction.detach() - target).abs().sum(0)
                        if routing:
                            errors = prediction.detach() - target
                            head_error.scatter_add_(0, selected_heads, errors.square().mean(1))
                            head_mae.scatter_add_(0, selected_heads, errors[:, 0].abs())
                            head_count.scatter_add_(0, selected_heads, torch.ones_like(selected_heads, dtype=torch.float32))
                    phase.loss = total / (phase.samples + len(images))
                    phase.mse = total_mse / (phase.samples + len(images))
            phase_report = phase.report()
            metrics[("train" if index==0 else "validation")+"_seconds"] = phase_report["wall_seconds"]
            metrics[("train" if index==0 else "validation")+"_samples_per_second"] = phase_report["samples_per_second"]
            key = "train" if index == 0 else "validation"
            metrics[key + "_loss"] = total / len(loader.dataset)
            metrics[key + "_mse"] = total_mse / len(loader.dataset)
            metrics[key + "_mae"] = (mae / len(loader.dataset)).tolist()
            if routing:
                count = head_count.cpu().tolist()
                mse = (head_error / head_count.clamp_min(1)).cpu().tolist()
                steer = (head_mae / head_count.clamp_min(1)).cpu().tolist()
                metrics[key + '_heads'] = {head: dict(samples=int(count[i]), mse=mse[i] if count[i] else None,
                                                     steering_mae=steer[i] if count[i] else None)
                                           for i,head in enumerate(routing['heads'])}
                metrics[key + '_micro_mse'] = metrics[key + '_mse']
                metrics[key + '_mse'] = sum(mse[i] for i in range(len(mse)) if count[i])/sum(c>0 for c in count)

        if scheduler:
            scheduler.step(metrics["validation_mse" if routing else "validation_loss"])
        metrics["next_learning_rate"] = optimizer.param_groups[0]["lr"]
        if scaler.is_enabled():
            metrics["grad_scale"] = scaler.get_scale()
        metrics["epoch_seconds"] = time.perf_counter()-epoch_started
        metrics["elapsed_seconds"] = time.perf_counter()-setup_started
        with (a.output / "metrics.jsonl").open("a") as stream:
            stream.write(json.dumps(metrics) + "\n")
        improved = metrics["validation_mse"] < best
        if improved:
            best = metrics["validation_mse"]
            state = {
                "spec": getattr(model, "spec", SPEC),
                "mode": a.mode,
                "state_dict": model.state_dict(),
                "runtime": runtime,
                "metrics": metrics,
                "provenance": provenance,
            }
            if routing:
                state["routing"] = routing
            if a.modality == 'evs':
                state.update(modality='evs', evs_metadata=model.evs_metadata)
            save_started = time.perf_counter()
            temporary = a.output / "best.tmp"
            torch.save(state, temporary)
            temporary.replace(a.output / "best.pt")
            timing.record("checkpoint_save", time.perf_counter()-save_started, epoch=epoch+1, bytes=(a.output/"best.pt").stat().st_size)
        timing.progress.summary(epoch+1, metrics, elapsed=time.perf_counter()-epoch_started, best=improved)


if __name__ == "__main__":
    main()
