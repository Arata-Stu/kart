"""Raw histogram -> fixed normalization -> EVS patch-token mean -> control head."""

import torch
from torch import nn
from torch.nn import functional as F

from .contract import evs_spec, validate_runtime
from .event_encoder_model import PortableEventEncoder
from .model import Policy, backbone
from .evs_model import load_evs_encoder


class EVSPolicy(Policy):
    def __init__(self, encoder, mode='steer_throttle', frozen=True):
        super().__init__(encoder, mode, frozen)
        self.evs_metadata = encoder.metadata
        self.spec = evs_spec(self.evs_metadata)
        self.modality = 'evs'

    def forward(self, counts):
        # Validation of finite/nonnegative counts belongs to the offline dataset
        # loader and live OpenEB producer. This branch-free graph is also ONNX.
        h, w = self.spec['height'], self.spec['width']
        ph, pw = self.spec['padded_shape'][2:]
        padded = F.pad(counts, ((pw-w)//2, pw-w-(pw-w)//2,
                               (ph-h)//2, ph-h-(ph-h)//2))
        normalized = (padded - self.encoder.mean) / self.encoder.std
        tokens = self.encoder(normalized)
        raw = self.head(tokens.mean(dim=1))
        steering = raw[:, :1].tanh()
        return steering if self.mode == 'steer_only' else torch.cat(
            (steering, raw[:, 1:2].sigmoid()), dim=1)


def pretrained_policy(weights, repo, mode='steer_throttle', frozen=True):
    return EVSPolicy(load_evs_encoder(weights, repo), mode, frozen)


def load_checkpoint(path, repo, device='cpu'):
    ck = torch.load(path, map_location='cpu', weights_only=True)
    meta = ck['evs_metadata']
    spec = evs_spec(meta)
    if ck.get('modality') != 'evs' or ck.get('spec') != spec:
        raise ValueError('Unsupported EVS checkpoint contract')
    encoder = backbone(repo)
    old = encoder.patch_embed.proj
    encoder.patch_embed.proj = nn.Conv2d(
        20, old.out_channels, old.kernel_size, old.stride, old.padding,
        old.dilation, bias=old.bias is not None, padding_mode=old.padding_mode)
    if hasattr(encoder.patch_embed, 'in_chans'):
        encoder.patch_embed.in_chans = 20
    policy = EVSPolicy(PortableEventEncoder(encoder, meta), ck['mode'])
    policy.load_state_dict(ck['state_dict'], strict=True)
    policy.runtime = validate_runtime(ck['runtime'])
    return policy.to(device).eval()
