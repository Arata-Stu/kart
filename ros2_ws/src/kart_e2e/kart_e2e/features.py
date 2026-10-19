"""Offline RGB checkpoint patch features and steering input sensitivity; no downloads."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tempfile

import numpy as np
from PIL import Image
import torch

from .model import load_checkpoint, preprocess
from .data import load_rgb


def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def sample_rows(dataset, maximum, modality="rgb"):
    manifest = Path(dataset) / 'samples.jsonl'
    with manifest.open() as stream:
        count = sum(bool(line.strip()) for line in stream)
    if not count:
        raise ValueError('Empty dataset')
    indexes = set(np.linspace(0, count-1, min(maximum, count), dtype=int).tolist())
    selected, index = [], 0
    with manifest.open() as stream:
        for line in stream:
            if not line.strip():
                continue
            if index in indexes:
                row = json.loads(line)
                if ('counts' if modality == 'evs' else 'image') not in row:
                    raise ValueError(f'Feature visualization requires a {modality} dataset')
                if any(not np.isfinite(float(row[key])) for key in ('steering','throttle')):
                    raise ValueError('Non-finite teacher label')
                selected.append((index, row))
            index += 1
    return selected


def pca_colors(features):
    """One shared PCA/color scale across this report, not independent per frame."""
    values = np.concatenate(features).astype(np.float64)
    if values.ndim != 2 or values.shape[1] < 3 or not np.isfinite(values).all():
        raise ValueError('Expected finite patch features [patches, channels>=3]')
    mean = values.mean(axis=0)
    _, singular, basis = np.linalg.svd(values-mean, full_matrices=False)
    basis = basis[:3].copy()
    # Fix PCA's arbitrary sign for reproducible colors.
    for component in basis:
        if component[np.abs(component).argmax()] < 0:
            component *= -1
    scores = (values-mean) @ basis.T
    low, high = np.percentile(scores, [2,98], axis=0)
    span = high-low
    colors = np.full_like(scores, .5)
    np.divide(scores-low, span, out=colors, where=span>1e-12)
    colors = (np.clip(colors,0,1)*255).round().astype(np.uint8)
    energy = float(np.square(singular).sum())
    explained = (np.square(singular[:3])/energy).tolist() if energy else [0.,0.,0.]
    return colors, dict(mean=mean, components=basis, low=low, high=high), explained


def feature_image(colors, grid):
    h,w = grid
    canvas = Image.fromarray(colors.reshape(h,w,3)).resize((w*16,h*16), Image.Resampling.BILINEAR)
    # Identical padded input coordinates to Policy/preprocess; exclude padding.
    return canvas.crop((6,4,6+212,4+120))


def visualize(checkpoint, dataset, repo, output, *, device='cpu', max_frames=12):
    if not 1 <= max_frames <= 64:
        raise ValueError('max_frames must be 1..64')
    checkpoint, dataset, output = Path(checkpoint), Path(dataset).resolve(), Path(output)
    if output.exists():
        raise FileExistsError(output)
    metadata = dataset / 'metadata.json'
    if metadata.exists() and json.loads(metadata.read_text()).get('modality','rgb') == 'evs':
        from .evs_features import visualize as visualize_evs
        return visualize_evs(checkpoint, dataset, repo, output, device=device, max_frames=max_frames)
    checkpoint_sha, manifest_sha = sha256(checkpoint), sha256(dataset/'samples.jsonl')
    selected = sample_rows(dataset, max_frames)
    model = load_checkpoint(checkpoint, repo, device)
    model.requires_grad_(False)
    patches, frames, images, gradients = [], [], [], []
    for index,row in selected:
        source = (dataset / row['image']).resolve(strict=True)
        if not source.is_relative_to(dataset):
            raise ValueError('Image outside dataset')
        raw = source.read_bytes()
        value = preprocess(load_rgb(io.BytesIO(raw), suffix=source.suffix)).unsqueeze(0).to(device)
        h,w = value.shape[-2:]
        grid = (h//16,w//16)
        with torch.no_grad():
            features = model.encoder.forward_features(value)['x_norm_patchtokens'][0]
        if features.ndim != 2 or features.shape[0] != grid[0]*grid[1]:
            raise ValueError('Unexpected DINO patch-token layout')
        patches.append(features.detach().float().cpu().numpy())
        value.requires_grad_(True)
        prediction = model(value)
        if prediction.shape != (1,1 if model.mode=='steer_only' else 2) or not torch.isfinite(prediction).all():
            raise ValueError('Invalid policy prediction')
        derivative, = torch.autograd.grad(prediction[0,0], value)
        magnitude = derivative[0,:,4:-4,6:-6].abs().mean(dim=0).detach().cpu().numpy()
        if not np.isfinite(magnitude).all():
            raise ValueError('Non-finite steering input gradient')
        gradients.append(magnitude)
        # Show the exact resized RGB content used by the model (before normalization).
        content = value.detach()[0,:,4:-4,6:-6].cpu()
        rgb = (content*torch.tensor([.229,.224,.225])[:,None,None] +
               torch.tensor([.485,.456,.406])[:,None,None]).permute(1,2,0).numpy()
        images.append((rgb.clip(0,1)*255).round().astype(np.uint8))
        frames.append(dict(sample_index=index, stamp_ns=int(row['stamp_ns']),
                           steering_label=float(row['steering']), throttle_label=float(row['throttle']),
                           steering_prediction=float(prediction[0,0].detach().cpu()),
                           image=row['image'], image_sha256=hashlib.sha256(raw).hexdigest()))
        print(f'Feature extraction {len(frames)}/{len(selected)}', flush=True)
    colors,basis,explained = pca_colors(patches)
    gradient_scale = float(np.percentile(np.stack(gradients),99))
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.features-', dir=output.parent) as temporary:
        stage = Path(temporary)/'output'
        stage.mkdir()
        offset = 0
        for i,(row,rgb,gradient,patch) in enumerate(zip(frames,images,gradients,patches)):
            folder = stage/'frames'/f'{i:04d}'
            folder.mkdir(parents=True)
            Image.fromarray(rgb).save(folder/'rgb.png')
            feature_image(colors[offset:offset+len(patch)],grid).save(folder/'pca.png')
            offset += len(patch)
            strength = np.clip(gradient/gradient_scale,0,1) if gradient_scale>0 else np.zeros_like(gradient)
            alpha = .7*strength[...,None]
            red = np.zeros_like(rgb,dtype=np.float64)
            red[...,0] = 255
            overlay = (rgb*(1-alpha)+red*alpha).round().astype(np.uint8)
            Image.fromarray(overlay).save(folder/'saliency.png')
            row.update({kind:f'frames/{i:04d}/{kind}.png' for kind in ('rgb','pca','saliency')})
        np.savez(stage/'pca_basis.npz', **basis)
        report = dict(schema='kart.rgb_features.v1', modality='rgb', mode=model.mode,
                      checkpoint=str(checkpoint.resolve()), checkpoint_sha256=checkpoint_sha,
                      dataset=str(dataset), dataset_manifest_sha256=manifest_sha,
                      frames=frames, frame_count=len(frames), patch_grid=list(grid),
                      pca_explained_variance=explained, sensitivity_scale_p99=gradient_scale,
                      pca_scope='shared_basis_and_2_98_percentile_colors_within_report',
                      sensitivity='absolute_mean_input_gradient_of_steering; red_overlay; shared_p99_scale',
                      scope='offline_pytorch_checkpoint; qualitative_features_and_local_sensitivity_not_driving_accuracy')
        if sha256(checkpoint)!=checkpoint_sha or sha256(dataset/'samples.jsonl')!=manifest_sha:
            raise ValueError('Checkpoint or dataset manifest changed during visualization')
        (stage/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        stage.rename(output)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('checkpoint','dataset','repo','output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--max-frames',type=int,default=12)
    args = parser.parse_args()
    visualize(**vars(args))


if __name__=='__main__':
    main()
