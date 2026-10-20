"""Content-addressed, atomic FP32 encoder features for frozen-backbone training."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time

import numpy as np
import torch
from torch.utils.data import Dataset
from .contract import SPEC


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


class CachedSamples(Dataset):
    def __init__(self, path, items, mode):
        self.path = str(path)
        self.labels = np.asarray([label[:1] if mode == 'steer_only' else label for _,label in items], dtype=np.float32)
        self.features = None

    def __len__(self):
        return len(self.labels)

    def __getstate__(self):
        return {**self.__dict__, 'features': None}

    def __getitem__(self, index):
        if self.features is None:
            self.features = np.load(self.path, mmap_mode='r', allow_pickle=False)
        # Writable copy: torch must never mutate the read-only memory mapping.
        return torch.from_numpy(self.features[index].copy()), torch.from_numpy(self.labels[index].copy())


def cached_samples(dataset, model, root, weights_sha, loader_factory, device, *, timing=None, timing_phase="cache"):
    started = time.perf_counter()
    if timing:
        timing.write(dict(event="operation_start", phase=timing_phase, samples=len(dataset)))
        print(f"TIMING {timing_phase}: hashing/validating {len(dataset)} inputs", flush=True)
    dimension = getattr(model, "feature_dim", 384)
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    sources = {str(path): digest(path) for path,_ in dataset.items}
    code = Path(__file__).parent
    identity = dict(schema='kart.frozen_features.v1', weights_sha256=weights_sha,
                    spec=getattr(model,'spec',SPEC), torch_version=str(torch.__version__),
                    device=str(device), inputs=[sources[str(path)] for path,_ in dataset.items],
                    implementation={name:digest(code/name) for name in
                                    ('feature_cache.py','train.py','model.py','evs_policy.py','event_encoder_model.py','contract.py','data.py')})
    if timing: timing.record(timing_phase+"_hash_inputs", time.perf_counter()-started, samples=len(dataset))
    key = hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    folder = root/key
    with (root/(key+'.lock')).open('a') as lock:
        lock_started = time.perf_counter()
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        if timing: timing.record(timing_phase+"_lock_wait", time.perf_counter()-lock_started)
        reused = folder.exists()
        if folder.is_symlink():
            raise ValueError('Feature cache must not be a symlink')
        if folder.exists():
            report = json.loads((folder/'cache.json').read_text())
            features = folder/'features.npy'
            if features.is_symlink() or report.get('identity') != identity or report.get('sha256') != digest(features):
                raise ValueError(f'Invalid feature cache; remove this cache and retry: {folder}')
            values = np.load(features,mmap_mode='r',allow_pickle=False)
            if values.shape != (len(dataset),dimension) or values.dtype != np.float32:
                raise ValueError('Invalid cached feature layout')
            print(f'Feature cache reused: {key[:12]} ({len(dataset)} samples)',flush=True)
        else:
            with tempfile.TemporaryDirectory(prefix='.building-',dir=root) as temporary:
                stage = Path(temporary)/'data'; stage.mkdir()
                path = stage/'features.npy'
                values = np.lib.format.open_memmap(path,mode='w+',dtype=np.float32,shape=(len(dataset),dimension))
                offset = 0
                model.eval()
                loader = loader_factory(dataset,False,persistent=False)
                phase = timing.phase(timing_phase+"_extract") if timing else None
                from contextlib import nullcontext
                measure = phase.measure if phase else lambda *args,**kwargs: nullcontext()
                with torch.inference_mode():
                    for batch,(images,_) in enumerate(phase.batches(loader) if phase else loader):
                        with measure("transfer"):
                            inputs = images.to(device,non_blocking=True)
                        with measure("encoder"):
                            encoded = model.extract_features(inputs)
                        with measure("download"):
                            features = encoded.float().cpu().numpy()
                        if features.shape != (len(images),dimension) or not np.isfinite(features).all():
                            raise ValueError('Invalid encoder features')
                        with measure("write_features", gpu=False):
                            values[offset:offset+len(images)] = features
                        offset += len(images)
                        if batch%100==0 or offset==len(dataset):
                            print(f'Feature cache extraction: {offset}/{len(dataset)}',flush=True)
                if phase: phase.report()
                if offset != len(dataset) or any(digest(path)!=sha for path,sha in sources.items()):
                    raise ValueError('Dataset changed during feature extraction')
                values.flush(); del values
                report = dict(identity=identity,sha256=digest(path),samples=offset)
                (stage/'cache.json').write_text(json.dumps(report,indent=2))
                os.rename(stage,folder)
            print(f'Feature cache created: {key[:12]} ({len(dataset)} samples)',flush=True)
    if timing: timing.record(timing_phase+"_total", time.perf_counter()-started, cache_hit=reused, key=key, samples=len(dataset))
    return CachedSamples(folder/'features.npy',dataset.items,dataset.mode), key
