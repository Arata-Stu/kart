"""CUDA AMP selection; no model casting or silent explicit-mode fallback."""
from contextlib import nullcontext

import torch

PRECISIONS = ("auto", "fp32", "bf16", "fp16")


def resolve_precision(requested, device):
    if requested not in PRECISIONS:
        raise ValueError("precision must be auto, fp32, bf16 or fp16")
    target = torch.device(device)
    if requested == "fp32":
        return "fp32"
    if target.type != "cuda":
        if requested == "auto":
            return "fp32"
        raise ValueError("Explicit bf16/fp16 AMP requires a CUDA device; use auto or fp32 on CPU")
    if not torch.cuda.is_available():
        raise ValueError("CUDA is unavailable for AMP training")
    with torch.cuda.device(target):
        # Do not select software/emulated BF16 as a speed optimization.
        native_bf16 = torch.cuda.get_device_capability()[0] >= 8 and torch.cuda.is_bf16_supported()
    if requested == "bf16" and not native_bf16:
        raise ValueError("Native BF16 is unavailable on the selected CUDA device; use auto or fp16")
    return ("bf16" if native_bf16 else "fp16") if requested == "auto" else requested


def autocast_context(effective):
    if effective == "fp32":
        return nullcontext()
    if effective not in ("bf16", "fp16"):
        raise ValueError("Invalid resolved precision")
    return torch.autocast(device_type="cuda", dtype=torch.bfloat16 if effective == "bf16" else torch.float16)
