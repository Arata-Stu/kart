"""Offline EVS encoder adapter. ROS driver continues to emit raw histograms."""
from .contract import evs_spec
from .event_encoder_model import load_encoder
from .model import backbone


def load_evs_encoder(weights, repo, device='cpu'):
    import torch
    bundle = torch.load(weights, map_location='cpu', weights_only=True)
    evs_spec(bundle['metadata'])
    # Existing kart builder verifies the pinned local source, with no downloads.
    return load_encoder(weights, device=device, backbone_factory=lambda: backbone(repo))


def encode_openeb_counts(encoder, counts):
    """[B,20,Hcontent,Wcontent] raw FP32 -> [B,N,384] patch features.

    The caller must preserve the driver's tensor timestamp. This is not a ROS node.
    """
    return encoder(encoder.prepare_content(counts))
