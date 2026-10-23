"""Raw 20ch histogram -> fixed normalization -> ViT or PilotNet -> control head."""

import torch
from torch import nn
from torch.nn import functional as F

from .contract import evs_spec, evs_policy_spec, validate_runtime
from .event_encoder_model import PortableEventEncoder
from .model import Policy, backbone, pilotnet_blocks, pilotnet_head
from .evs_model import load_evs_encoder


class EVSPolicy(Policy):
    def __init__(self, encoder, mode='steer_throttle', frozen=True):
        super().__init__(encoder, mode, frozen)
        self.evs_metadata = encoder.metadata
        self.spec = evs_spec(self.evs_metadata)
        self.modality = 'evs'

    def extract_features(self, counts):
        # Validation of finite/nonnegative counts belongs to the offline dataset
        # loader and live OpenEB producer. This branch-free graph is also ONNX.
        h, w = self.spec['height'], self.spec['width']
        ph, pw = self.spec['padded_shape'][2:]
        padded = F.pad(counts, ((pw-w)//2, pw-w-(pw-w)//2,
                               (ph-h)//2, ph-h-(ph-h)//2))
        normalized = (padded - self.encoder.mean) / self.encoder.std
        tokens = self.encoder(normalized)
        return tokens.mean(dim=1)


def pretrained_policy(weights, repo, mode='steer_throttle', frozen=True):
    return EVSPolicy(load_evs_encoder(weights, repo), mode, frozen)


class EVSPilotNetEncoder(nn.Module):
    def __init__(self, meta):
        super().__init__()
        self.metadata = meta
        spec = evs_policy_spec(meta, 'simple_cnn')
        self.blocks, self.feature_dim = pilotnet_blocks(20, spec['height'], spec['width'])
        self.register_buffer('mean', torch.tensor(spec['mean']).reshape(1,20,1,1))
        self.register_buffer('std', torch.tensor(spec['std']).reshape(1,20,1,1))

    def prepare_content(self, counts):
        # OpenEB produces raw counts. Normalize all 20 channels in the ONNX graph;
        # no RGB conversion, resize, temporal aggregation or ViT padding.
        return (counts - self.mean) / self.std

    def forward(self, normalized):
        return self.blocks(normalized).flatten(2).transpose(1,2)


class EVSPilotNetPolicy(Policy):
    def __init__(self, meta, mode='steer_throttle'):
        nn.Module.__init__(self)
        if mode not in ('steer_only', 'steer_throttle'):
            raise ValueError('Unsupported EVS control mode')
        self.encoder = EVSPilotNetEncoder(meta)
        self.mode, self.frozen, self.modality = mode, False, 'evs'
        self.evs_metadata = meta
        self.spec = evs_policy_spec(meta, 'simple_cnn')
        self.feature_dim = self.encoder.feature_dim
        self.head = pilotnet_head(self.feature_dim, 1 if mode == 'steer_only' else 2)
        self.runtime = {'fixed_throttle': 0.0, 'max_throttle': 0.2}
        self.train()

    def extract_features(self, counts):
        spatial = self.encoder.blocks(self.encoder.prepare_content(counts))
        return spatial.flatten(1)


def load_checkpoint(path, repo, device='cpu'):
    ck = torch.load(path, map_location='cpu', weights_only=True)
    meta = ck['evs_metadata']
    stored_spec = ck.get('spec')
    if not isinstance(stored_spec, dict):
        raise ValueError('Missing EVS checkpoint spec')
    spec = evs_policy_spec(meta, stored_spec.get('encoder'))
    if ck.get('modality') != 'evs' or ck.get('spec') != spec:
        raise ValueError('Unsupported EVS checkpoint contract')
    if spec['encoder'] == 'simple_cnn':
        policy = EVSPilotNetPolicy(meta, ck['mode'])
    else:
        encoder = backbone(repo)
        old = encoder.patch_embed.proj
        encoder.patch_embed.proj = nn.Conv2d(
            20, old.out_channels, old.kernel_size, old.stride, old.padding,
            old.dilation, bias=old.bias is not None, padding_mode=old.padding_mode)
        if hasattr(encoder.patch_embed, 'in_chans'):
            encoder.patch_embed.in_chans = 20
        policy = EVSPolicy(PortableEventEncoder(encoder, meta), ck['mode'])
    if ck.get('routing') is not None:
        policy.configure_heads(ck['routing'])
    policy.load_state_dict(ck['state_dict'], strict=True)
    policy.runtime = validate_runtime(ck['runtime'])
    return policy.to(device).eval()
