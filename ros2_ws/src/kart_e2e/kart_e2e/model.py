"""One preprocessing/output contract for training and inference; no network downloads."""

import subprocess
import sys
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from .contract import DINOV3_COMMIT, SPEC, rgb_spec, validate_rgb_spec


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


def backbone(repo, architecture="dinov3_vits16"):
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
    rgb_spec(architecture)
    if architecture == "dinov3_vits16":
        return backbones.dinov3_vits16(pretrained=False)
    # Same pinned official implementation/config as ViT-S; change only width/heads.
    return backbones._make_dinov3_vit(
        img_size=224, patch_size=16, in_chans=3, embed_dim=192, depth=12, num_heads=3,
        pos_embed_rope_base=100, pos_embed_rope_normalize_coords="separate",
        pos_embed_rope_rescale_coords=2, pos_embed_rope_dtype="fp32",
        ffn_ratio=4, qkv_bias=True, drop_path_rate=0., layerscale_init=1e-5,
        norm_layer="layernormbf16", ffn_layer="mlp", ffn_bias=True, proj_bias=True,
        n_storage_tokens=4, mask_k_bias=True, pretrained=False, compact_arch_name="vitt")


class Policy(nn.Module):
    def __init__(self, encoder, mode="steer_throttle", frozen=True, architecture="dinov3_vits16"):
        super().__init__()
        if mode not in ("steer_throttle", "steer_only"):
            raise ValueError("mode must be steer_throttle or steer_only")
        self.encoder, self.mode, self.frozen = encoder, mode, frozen
        self.spec = rgb_spec(architecture)
        self.feature_dim = 192 if architecture == "dinov3_vitt16" else 384
        self.runtime = {"fixed_throttle": 0.0, "max_throttle": 0.2}
        self.head = nn.Sequential(
            nn.Linear(self.feature_dim, 128),
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

    def extract_features(self, value):
        return self.encoder(value)

    def forward(self, value):
        return self.predict_features(self.extract_features(value))

    def predict_features(self, features):
        raw = self.head(features)
        steering = raw[:, :1].tanh()
        return (
            steering
            if self.mode == "steer_only"
            else torch.cat((steering, raw[:, 1:2].sigmoid()), 1)
        )


def load_checkpoint(path, repo, device="cpu"):
    state = torch.load(path, map_location="cpu", weights_only=True)
    spec = validate_rgb_spec(state["spec"])
    architecture = spec["encoder"]
    model = Policy(backbone(repo, architecture), state["mode"], architecture=architecture)
    model.runtime = state.get("runtime")
    model.load_state_dict(state["state_dict"], strict=True)
    return model.to(device).eval()


def load_rgb_encoder(path, repo, architecture):
    """Raw ViT-S weights or a strictly validated distilled Tiny bundle; no fallback."""
    state = torch.load(path, map_location="cpu", weights_only=True)
    encoder = backbone(repo, architecture)
    if architecture == "dinov3_vitt16":
        if (state.get("format") != "kart.rgb_encoder.v1" or
                state.get("spec") != rgb_spec(architecture)):
            raise ValueError("ViT-Tiny requires a distilled kart.rgb_encoder.v1 encoder.pt")
        state = state["encoder_state_dict"]
    encoder.load_state_dict(state, strict=True)
    return encoder
