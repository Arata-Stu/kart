"""One preprocessing/output contract for training and inference; no network downloads."""

import subprocess
import sys
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from .contract import DINOV3_COMMIT, SPEC


def preprocess(image):
    # Match the encoder's linear resize, without PIL downsampling antialiasing.
    value = torch.from_numpy(
        np.array(image.convert("RGB"), dtype=np.float32).transpose(2, 0, 1).copy()
    )
    value = F.interpolate(
        value.unsqueeze(0),
        size=(SPEC["height"], SPEC["width"]),
        mode="bilinear",
        align_corners=False,
        antialias=False,
    )[0]
    value = value.round().clamp(0, 255) / 255
    value = (value - torch.tensor([0.485, 0.456, 0.406])[:, None, None]) / torch.tensor(
        [0.229, 0.224, 0.225]
    )[:, None, None]
    return F.pad(value, (6, 6, 4, 4))  # 212x120 -> 224x128, zero after normalization


def backbone(repo):
    repo = Path(repo).expanduser().resolve(strict=True)
    revision = subprocess.check_output(
        ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
    ).strip()
    dirty = subprocess.check_output(
        ["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=no"],
        text=True,
    )
    if revision != DINOV3_COMMIT or dirty:
        raise ValueError("DINOv3 checkout must be clean and match e2e.repos")
    # Import only the official backbone; hubconf also imports unused depth/detection stacks.
    sys.path.insert(0, str(repo))
    from dinov3.hub import backbones

    if not Path(backbones.__file__).resolve().is_relative_to(repo):
        raise ValueError("A different dinov3 package was already imported")
    return backbones.dinov3_vits16(pretrained=False)


class Policy(nn.Module):
    def __init__(self, encoder, mode="steer_throttle", frozen=True):
        super().__init__()
        if mode not in ("steer_throttle", "steer_only"):
            raise ValueError("mode must be steer_throttle or steer_only")
        self.encoder, self.mode, self.frozen = encoder, mode, frozen
        self.head = nn.Sequential(
            nn.Linear(384, 128),
            nn.GELU(),
            nn.Dropout(0.1),
            nn.Linear(128, 1 if mode == "steer_only" else 2),
        )
        self.encoder.requires_grad_(not frozen)
        self.train()

    def train(self, mode=True):
        super().train(mode)
        if self.frozen:
            self.encoder.eval()
        return self

    def forward(self, value):
        raw = self.head(self.encoder(value))
        steering = raw[:, :1].tanh()
        return (
            steering
            if self.mode == "steer_only"
            else torch.cat((steering, raw[:, 1:2].sigmoid()), 1)
        )


def load_checkpoint(path, repo, device="cpu"):
    state = torch.load(path, map_location="cpu", weights_only=True)
    if state["spec"] != SPEC:
        raise ValueError("Unsupported checkpoint preprocessing/encoder contract")
    model = Policy(backbone(repo), state["mode"])
    model.load_state_dict(state["state_dict"], strict=True)
    return model.to(device).eval()
