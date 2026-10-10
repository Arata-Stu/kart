"""Small DINOv3 behavior-cloning trainer, with explicit session-level validation."""

import argparse
import hashlib
import json
import math
import random
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import DataLoader, Dataset

from .model import SPEC, Policy, backbone, preprocess


class Samples(Dataset):
    def __init__(self, roots, mode):
        self.items = []
        self.mode = mode
        for root in roots:
            for line in (root / "samples.jsonl").read_text().splitlines():
                row = json.loads(line)
                image = (root / row["image"]).resolve(strict=True)
                if not image.is_relative_to(root.resolve()):
                    raise ValueError("Image outside dataset")
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
        with Image.open(path) as image:
            value = preprocess(image)
        return value, torch.tensor(label[:1] if self.mode == "steer_only" else label)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--train", type=Path, nargs="+", required=True)
    p.add_argument("--validation", type=Path, nargs="+", required=True)
    p.add_argument("--repo", required=True)
    p.add_argument(
        "--weights",
        type=Path,
        required=True,
        help="Official local DINOv3 ViT-S/16 state dict",
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
    p.add_argument("--finetune", action="store_true")
    p.add_argument("--device", default="cuda")
    p.add_argument("--epochs", type=int, default=20)
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--learning-rate", type=float, default=1e-4)
    p.add_argument("--seed", type=int, default=42)
    a = p.parse_args()
    if (
        a.epochs <= 0
        or a.batch_size <= 0
        or not math.isfinite(a.learning_rate)
        or a.learning_rate <= 0
    ):
        p.error("epochs, batch-size and learning-rate must be positive")

    from .contract import validate_runtime

    runtime = validate_runtime(
        {"fixed_throttle": a.fixed_throttle, "max_throttle": a.max_throttle}
    )
    random.seed(a.seed)
    np.random.seed(a.seed)
    torch.manual_seed(a.seed)
    datasets = [Samples(a.train, a.mode), Samples(a.validation, a.mode)]
    encoder = backbone(a.repo)
    encoder.load_state_dict(
        torch.load(a.weights, map_location="cpu", weights_only=True), strict=True
    )
    model = Policy(encoder, a.mode, frozen=not a.finetune).to(a.device)
    model.runtime = runtime
    optimizer = torch.optim.AdamW(
        (v for v in model.parameters() if v.requires_grad), lr=a.learning_rate
    )
    loaders = [
        DataLoader(d, batch_size=a.batch_size, shuffle=i == 0)
        for i, d in enumerate(datasets)
    ]
    a.output.mkdir(parents=True, exist_ok=False)
    provenance = {
        key: str(value) if isinstance(value, Path) else value
        for key, value in vars(a).items()
    }
    provenance["train"] = [str(r.resolve()) for r in a.train]
    provenance["validation"] = [str(r.resolve()) for r in a.validation]
    provenance["weights_sha256"] = hashlib.sha256(a.weights.read_bytes()).hexdigest()
    (a.output / "run.json").write_text(json.dumps(provenance, indent=2))
    best = float("inf")
    for epoch in range(a.epochs):
        metrics = {"epoch": epoch + 1}
        for index, loader in enumerate(loaders):
            model.train(index == 0)
            total = 0.0
            mae = torch.zeros(1 if a.mode == "steer_only" else 2, device=a.device)
            with torch.set_grad_enabled(index == 0):
                for images, target in loader:
                    target = target.to(a.device)
                    prediction = model(images.to(a.device))
                    loss = torch.nn.functional.mse_loss(prediction, target)
                    if not torch.isfinite(loss):
                        raise RuntimeError("Non-finite training/validation loss")
                    if index == 0:
                        optimizer.zero_grad()
                        loss.backward()
                        optimizer.step()
                    total += loss.item() * len(images)
                    mae += (prediction.detach() - target).abs().sum(0)
            key = "train" if index == 0 else "validation"
            metrics[key + "_mse"] = total / len(loader.dataset)
            metrics[key + "_mae"] = (mae / len(loader.dataset)).tolist()
        with (a.output / "metrics.jsonl").open("a") as stream:
            stream.write(json.dumps(metrics) + "\n")
        print(metrics, flush=True)
        if metrics["validation_mse"] < best:
            best = metrics["validation_mse"]
            state = {
                "spec": SPEC,
                "mode": a.mode,
                "state_dict": model.state_dict(),
                "runtime": runtime,
                "metrics": metrics,
                "provenance": provenance,
            }
            temporary = a.output / "best.tmp"
            torch.save(state, temporary)
            temporary.replace(a.output / "best.pt")


if __name__ == "__main__":
    main()
