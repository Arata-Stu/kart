"""Offline raw 20ch inputs, shared patch PCA, and steering gradients."""
import hashlib
import io
import json
from pathlib import Path
import tempfile

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

from .evs_policy import load_checkpoint
from .contract import evs_spec
from .features import sha256, sample_rows, pca_colors


def event_images(counts, scale):
    """Display-only log intensity; never used as model input."""
    strength = np.clip(np.log1p(counts) / scale, 0, 1) if scale > 0 else np.zeros_like(counts)
    positive = np.clip(np.log1p(counts[:10].sum(0))/scale,0,1) if scale>0 else np.zeros_like(counts[0])
    negative = np.clip(np.log1p(counts[10:].sum(0))/scale,0,1) if scale>0 else np.zeros_like(counts[0])
    image = np.stack((positive, np.zeros_like(positive), negative), axis=-1)
    image = (255 * image).round().astype(np.uint8)
    h, w = counts.shape[-2:]
    header = max(20,w//5)
    tiles = Image.new('RGB', (w * 10, (h+header) * 2))
    font = ImageFont.load_default(size=max(12,w//7))
    draw = ImageDraw.Draw(tiles)
    for channel in range(20):
        tint = np.zeros((h,w,3),dtype=np.uint8)
        tint[...,0 if channel < 10 else 2] = (255*strength[channel]).round().astype(np.uint8)
        x, y = (channel % 10) * w, (channel // 10) * (h+header)
        draw.text((x+3,y+3), f'{"+" if channel < 10 else "-"} bin {channel % 10}', fill='white',font=font)
        tiles.paste(Image.fromarray(tint), (x,y+header))
    return image, tiles


def visualize(checkpoint, dataset, repo, output, *, device='cpu', max_frames=12):
    checkpoint, dataset, output = Path(checkpoint), Path(dataset).resolve(), Path(output)
    if not 1 <= max_frames <= 64:
        raise ValueError('max_frames must be 1..64')
    if output.exists():
        raise FileExistsError(output)
    metadata = dataset / 'metadata.json'
    meta = json.loads(metadata.read_text())
    checkpoint_sha, manifest_sha, metadata_sha = sha256(checkpoint), sha256(dataset/'samples.jsonl'), sha256(metadata)
    model = load_checkpoint(checkpoint, repo, device)
    if meta.get('modality') != 'evs' or meta.get('model_spec') != evs_spec(model.evs_metadata):
        raise ValueError('EVS dataset and checkpoint spec mismatch')
    model.requires_grad_(False)
    selected = sample_rows(dataset, max_frames, modality='evs')
    h,w = model.spec['height'],model.spec['width']
    ph,pw = model.spec['padded_shape'][2:]
    top,left = (ph-h)//2,(pw-w)//2
    grid = (ph//16,pw//16)
    cnn = model.spec['encoder'] == 'simple_cnn'
    if cnn:
        # Conv4 downsamples again; final overlapping receptive fields are a
        # coarse feature grid, not ViT patches aligned to the input pixels.
        gh, gw = h, w
        for kernel, stride in ((5,2),(5,2),(5,2),(3,2),(3,1)):
            gh, gw = (gh-kernel)//stride+1, (gw-kernel)//stride+1
        grid = (gh,gw)
    frames, patches, inputs, gradients = [],[],[],[]
    for index,row in selected:
        source = (dataset/row['counts']).resolve(strict=True)
        if not source.is_relative_to(dataset):
            raise ValueError('Counts outside dataset')
        raw = source.read_bytes()
        counts = np.load(io.BytesIO(raw),allow_pickle=False)
        if counts.shape != (20,h,w) or counts.dtype != np.float32 or not np.isfinite(counts).all() or (counts < 0).any():
            raise ValueError('Expected finite nonnegative float32 raw counts [20,H,W]')
        value = torch.from_numpy(counts).unsqueeze(0).to(device)
        with torch.no_grad():
            tokens = model.encoder(model.encoder.prepare_content(value))[0]
        if tokens.shape != (grid[0]*grid[1],64 if cnn else 384) or not torch.isfinite(tokens).all():
            raise ValueError('Unexpected EVS patch-token layout')
        patches.append(tokens.detach().float().cpu().numpy())
        value.requires_grad_(True)
        prediction = model(value)
        if prediction.shape != (1,1 if model.mode=='steer_only' else 2) or not torch.isfinite(prediction).all():
            raise ValueError('Invalid policy prediction')
        derivative, = torch.autograd.grad(prediction[0,0],value)
        gradient = derivative[0].abs().mean(0).detach().cpu().numpy()
        if not np.isfinite(gradient).all():
            raise ValueError('Non-finite steering input gradient')
        inputs.append(counts); gradients.append(gradient)
        frames.append(dict(sample_index=index,stamp_ns=int(row['stamp_ns']),
                           steering_label=float(row['steering']),throttle_label=float(row['throttle']),
                           steering_prediction=float(prediction[0,0].detach().cpu()),
                           counts=row['counts'],counts_sha256=hashlib.sha256(raw).hexdigest()))
        print(f'EVS feature extraction {len(frames)}/{len(selected)}',flush=True)
    colors,basis,explained = pca_colors(patches)
    logcounts = np.log1p(np.stack(inputs))
    nonzero = logcounts[logcounts > 0]
    scale = float(np.percentile(nonzero,99)) if nonzero.size else 0.
    gradient_scale = float(np.percentile(np.stack(gradients),99))
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.features-',dir=output.parent) as temporary:
        stage = Path(temporary)/'output'; stage.mkdir()
        offset = 0
        for i,(row,counts,gradient,patch) in enumerate(zip(frames,inputs,gradients,patches)):
            folder=stage/'frames'/f'{i:04d}'; folder.mkdir(parents=True)
            image,tiles = event_images(counts,scale)
            Image.fromarray(image).save(folder/'rgb.png') # legacy viewer input-image slot
            tiles.save(folder/'channels.png')
            pca = Image.fromarray(colors[offset:offset+len(patch)].reshape(*grid,3)).resize((pw,ph),Image.Resampling.BILINEAR)
            pca.crop((left,top,left+w,top+h)).save(folder/'pca.png'); offset += len(patch)
            strength = np.clip(gradient/gradient_scale,0,1) if gradient_scale>0 else np.zeros_like(gradient)
            alpha=.7*strength[...,None]
            red=np.zeros_like(image,dtype=np.float64); red[...,0]=255
            Image.fromarray((image*(1-alpha)+red*alpha).round().astype(np.uint8)).save(folder/'saliency.png')
            row.update({kind:f'frames/{i:04d}/{kind}.png' for kind in ('rgb','pca','saliency','channels')})
        np.savez(stage/'pca_basis.npz',**basis)
        report=dict(schema='kart.evs_features.v1',modality='evs',mode=model.mode,
                    checkpoint=str(checkpoint.resolve()),checkpoint_sha256=checkpoint_sha,
                    dataset=str(dataset),dataset_manifest_sha256=manifest_sha,dataset_metadata_sha256=metadata_sha,
                    frames=frames,frame_count=len(frames),patch_grid=list(grid),
                    spatial_layout=('overlapping_conv_features_resized_for_display_not_pixel_aligned' if cnn else 'stride16_input_aligned'),
                    pca_explained_variance=explained,sensitivity_scale_p99=gradient_scale,
                    input_log1p_scale_p99=scale,
                    input_display='log1p raw counts; positive red / negative blue; sum over 10 bins; shared p99 scale',
                    channels_display='positive bins 0..9 top; negative bins 0..9 bottom; shared log1p p99 scale',
                    pca_scope='shared_basis_and_2_98_percentile_colors_within_report',
                    sensitivity='absolute_mean_gradient_over_20_raw_count_channels_of_steering; shared_p99_scale',
                    scope='offline_pytorch_checkpoint; qualitative_features_and_local_sensitivity_not_driving_accuracy')
        if sha256(checkpoint)!=checkpoint_sha or sha256(dataset/'samples.jsonl')!=manifest_sha or sha256(metadata)!=metadata_sha:
            raise ValueError('Checkpoint or dataset changed during visualization')
        (stage/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)); stage.rename(output)
    return report
