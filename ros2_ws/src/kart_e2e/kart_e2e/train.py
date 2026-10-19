"""Small DINOv3 behavior-cloning trainer, with explicit session-level validation."""

import argparse
import json
import math
import random
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset

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
    def __init__(self, roots, mode, evs_spec=None):
        self.items = []
        self.mode = mode
        self.evs_spec = evs_spec
        for root in roots:
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
        return value, torch.tensor(label[:1] if self.mode == "steer_only" else label)


def main():
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
    p.add_argument("--finetune", action="store_true", help="Update the full backbone and control head")
    p.add_argument("--device", default="cuda")
    p.add_argument("--precision", choices=["auto", "fp32", "bf16", "fp16"], default="auto",
                   help="CUDA AMP: auto prefers native BF16, otherwise FP16; CPU auto uses FP32")
    p.add_argument("--epochs", type=int, default=None, help="Default 10 pretrained / 50 scratch")
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--learning-rate", type=float, default=None,
                   help="AdamW learning rate for all trainable parameters (default: 1e-4 frozen/scratch, 1e-5 pretrained finetune)")
    p.add_argument("--loss", choices=["smooth_l1", "mse"], default="smooth_l1")
    p.add_argument("--smooth-l1-beta", type=float, default=0.1)
    p.add_argument("--lr-scheduler", choices=["none", "plateau"], default="none")
    p.add_argument("--lr-factor", type=float, default=0.5)
    p.add_argument("--lr-patience", type=int, default=2)
    p.add_argument("--min-lr", type=float, default=1e-7)
    p.add_argument("--num-workers", type=int, default=8)
    p.add_argument("--feature-cache", action=argparse.BooleanOptionalAction, default=True,
                   help="Cache frozen encoder features; bypassed during finetune")
    p.add_argument("--cache-dir", type=Path, default=None)
    p.add_argument("--seed", type=int, default=42)
    a = p.parse_args()
    scratch = a.initialization == "scratch"
    if scratch and a.modality != "rgb":
        p.error("Scratch baselines currently support RGB only")
    if a.architecture == "simple_cnn" and not scratch:
        p.error("simple_cnn requires --initialization scratch")
    if scratch and a.weights is not None:
        p.error("Scratch must not load --weights; omit it")
    if not scratch and a.weights is None:
        p.error("Pretrained training requires --weights")
    if a.epochs is None:
        a.epochs = 50 if scratch else 10
    if a.learning_rate is None:
        a.learning_rate = 1e-4 if scratch else (1e-5 if a.finetune else 1e-4)
    if scratch:
        a.finetune = True
    if (
        a.num_workers < 0
        or a.epochs <= 0
        or a.batch_size <= 0
        or not math.isfinite(a.learning_rate)
        or a.learning_rate <= 0
    ):
        p.error("epochs, batch-size and learning-rate must be positive")

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
        datasets = [Samples(r, a.mode, model.spec) for r in (a.train, a.validation)]
    else:
        datasets = [Samples(a.train, a.mode), Samples(a.validation, a.mode)]
        if scratch:
            encoder = backbone(a.repo, a.architecture)
        elif a.architecture == 'dinov3_vits16':
            encoder = backbone(a.repo)
            encoder.load_state_dict(torch.load(a.weights, map_location="cpu", weights_only=True), strict=True)
        else:
            from .model import load_rgb_encoder
            encoder = load_rgb_encoder(a.weights, a.repo, a.architecture)
        model = Policy(encoder, a.mode, frozen=not a.finetune, architecture=a.architecture).to(a.device)
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

    def loader_factory(dataset, shuffle, persistent=True, seed=a.seed):
        options = dict(batch_size=a.batch_size, shuffle=shuffle, num_workers=a.num_workers,
                       pin_memory=str(a.device).startswith('cuda'),
                       generator=torch.Generator().manual_seed(seed))
        if a.num_workers:
            options.update(multiprocessing_context='spawn', persistent_workers=persistent)
        return DataLoader(dataset, **options)

    cache_keys = []
    if effective_cache:
        cached = []
        cache_dir = a.cache_dir or a.output.parent/'.feature_cache'
        for dataset in datasets:
            data, key = cached_samples(dataset, model, cache_dir, weights_sha, loader_factory, a.device)
            cached.append(data); cache_keys.append(key)
        datasets = cached
    loaders = [loader_factory(d, i==0, seed=a.seed+i) for i,d in enumerate(datasets)]
    a.output.mkdir(parents=True, exist_ok=False)
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
    provenance["scheduler_metric"] = "validation_loss" if scheduler else None
    provenance["weights_sha256"] = weights_sha
    provenance["feature_cache_effective"] = effective_cache
    provenance["feature_cache_keys"] = cache_keys
    (a.output / "run.json").write_text(json.dumps(provenance, indent=2))
    best = float("inf")
    for epoch in range(a.epochs):
        metrics = {"epoch": epoch + 1, "learning_rate": optimizer.param_groups[0]["lr"]}
        for index, loader in enumerate(loaders):
            model.train(index == 0)
            total = 0.0
            total_mse = 0.0
            mae = torch.zeros(1 if a.mode == "steer_only" else 2, device=a.device)
            with torch.set_grad_enabled(index == 0):
                for images, target in loader:
                    target = target.to(a.device, non_blocking=True)
                    inputs = images.to(a.device, non_blocking=True)
                    with autocast_context(effective_precision):
                        prediction = model.predict_features(inputs) if effective_cache else model(inputs)
                    # Loss and comparison metrics stay FP32; casts preserve gradients.
                    prediction = prediction.float()
                    loss = regression_loss(prediction, target.float(), a.loss, a.smooth_l1_beta)
                    if not torch.isfinite(loss):
                        raise RuntimeError("Non-finite training/validation loss")
                    if index == 0:
                        optimizer.zero_grad(set_to_none=True)
                        scaler.scale(loss).backward()
                        scaler.step(optimizer)
                        scaler.update()
                    total += loss.item() * len(images)
                    total_mse += torch.nn.functional.mse_loss(prediction.detach(), target).item() * len(images)
                    mae += (prediction.detach() - target).abs().sum(0)
            key = "train" if index == 0 else "validation"
            metrics[key + "_loss"] = total / len(loader.dataset)
            metrics[key + "_mse"] = total_mse / len(loader.dataset)
            metrics[key + "_mae"] = (mae / len(loader.dataset)).tolist()
        if scheduler:
            scheduler.step(metrics["validation_loss"])
        metrics["next_learning_rate"] = optimizer.param_groups[0]["lr"]
        if scaler.is_enabled():
            metrics["grad_scale"] = scaler.get_scale()
        with (a.output / "metrics.jsonl").open("a") as stream:
            stream.write(json.dumps(metrics) + "\n")
        print(metrics, flush=True)
        if metrics["validation_mse"] < best:
            best = metrics["validation_mse"]
            state = {
                "spec": getattr(model, "spec", SPEC),
                "mode": a.mode,
                "state_dict": model.state_dict(),
                "runtime": runtime,
                "metrics": metrics,
                "provenance": provenance,
            }
            if a.modality == 'evs':
                state.update(modality='evs', evs_metadata=model.evs_metadata)
            temporary = a.output / "best.tmp"
            torch.save(state, temporary)
            temporary.replace(a.output / "best.pt")


if __name__ == "__main__":
    main()
