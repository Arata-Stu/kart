"""Preload deterministic dataset tensors once in process RAM; no worker replication."""
import json
import os
from pathlib import Path
import time

import torch
from torch.utils.data import Dataset


def available_memory():
    """Best-effort available bytes, bounded by Linux cgroup container headroom."""
    limits = []
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemAvailable:'): limits.append(int(line.split()[1])*1024)
    except (OSError, ValueError):
        try: limits.append(os.sysconf('SC_AVPHYS_PAGES')*os.sysconf('SC_PAGE_SIZE'))
        except (OSError, ValueError): pass
    pairs = [('/sys/fs/cgroup/memory.max','/sys/fs/cgroup/memory.current'),
             ('/sys/fs/cgroup/memory/memory.limit_in_bytes','/sys/fs/cgroup/memory/memory.usage_in_bytes')]
    # Also cover non-root cgroup mounts used by some container runtimes.
    try:
        for row in Path('/proc/self/cgroup').read_text().splitlines():
            hierarchy, controllers, relative = row.split(':',2)
            relative = relative.lstrip('/')
            if not controllers:
                base = Path('/sys/fs/cgroup')/relative
                pairs.append((str(base/'memory.max'),str(base/'memory.current')))
            elif 'memory' in controllers.split(','):
                base = Path('/sys/fs/cgroup/memory')/relative
                pairs.append((str(base/'memory.limit_in_bytes'),str(base/'memory.usage_in_bytes')))
    except (OSError, ValueError): pass
    for maximum, used in pairs:
        try:
            raw = Path(maximum).read_text().strip()
            if raw != 'max': limits.append(max(0,int(raw)-int(Path(used).read_text())))
        except (OSError, ValueError): pass
    return max(0,min(limits)) if limits else None


def identity(dataset):
    if hasattr(dataset, 'items'):
        return ('samples',tuple((str(path),tuple(label)) for path,label in dataset.items),
                dataset.mode,json.dumps(dataset.evs_spec,sort_keys=True))
    if hasattr(dataset,'path') and hasattr(dataset,'labels'):
        return ('features',str(Path(dataset.path).resolve()),dataset.labels.shape,dataset.labels.tobytes())
    if hasattr(dataset,'images') and hasattr(dataset,'features'):
        return ('aligned',identity(dataset.images),identity(dataset.features))
    return ('object',id(dataset))


class RamSamples(Dataset):
    ram_loaded = True

    def __init__(self, source, *, name='dataset', timing=None):
        started = time.perf_counter()
        if not len(source): raise ValueError('Cannot preload an empty dataset')
        first, target = source[0]
        if first.device.type != 'cpu' or target.device.type != 'cpu':
            raise ValueError('RAM preload requires CPU tensors')
        self.bytes = len(source)*(first.numel()*first.element_size()+target.numel()*target.element_size())
        available = available_memory()
        print(f'RAM preload {name}: {len(source)} samples, {self.bytes/1048576:.1f} MiB '
              f'(available={available/1048576 if available is not None else "unknown"} MiB)',flush=True)
        if timing: timing.write(dict(event='operation_start',phase='ram_preload',dataset=name,
                                     samples=len(source),estimated_bytes=self.bytes,available_bytes=available))
        if available is not None and self.bytes > available*.8:
            raise MemoryError('RAM preload needs more than 80% of available RAM/container headroom. '
                              'Disable RAM preload in the GUI or pass --no-ram-preload to use streaming.')
        try:
            self.inputs = torch.empty((len(source),*first.shape),dtype=first.dtype)
            self.targets = torch.empty((len(source),*target.shape),dtype=target.dtype)
            for index in range(len(source)):
                value,label = (first,target) if index==0 else source[index]
                if value.shape!=first.shape or value.dtype!=first.dtype or label.shape!=target.shape or label.dtype!=target.dtype:
                    raise ValueError('Dataset tensor layout changed during RAM preload')
                self.inputs[index].copy_(value);self.targets[index].copy_(label)
                if (index+1)%1000==0 or index+1==len(source):
                    print(f'RAM preload {name}: {index+1}/{len(source)}',flush=True)
        except (MemoryError, RuntimeError) as error:
            raise RuntimeError('RAM preload failed; disable RAM preload or use --no-ram-preload. '+str(error)) from error
        if timing: timing.record('ram_preload',time.perf_counter()-started,dataset=name,
                                 samples=len(source),bytes=self.bytes,available_bytes=available)

    def __len__(self): return len(self.targets)
    def __getitem__(self,index): return self.inputs[index],self.targets[index]


def preload_datasets(datasets, *, timing=None):
    """Reuse identical train/validation data without allocating a second tensor copy."""
    result, unique = [], {}
    for index, source in enumerate(datasets):
        key = identity(source)
        name = 'train' if index==0 else 'validation'
        if key not in unique:
            unique[key] = RamSamples(source,name=name,timing=timing)
        else:
            print(f'RAM preload {name}: reusing identical dataset in RAM',flush=True)
            if timing: timing.record('ram_preload_reuse',0.,dataset=name,samples=len(source))
        result.append(unique[key])
    return result


def loader_workers(dataset, requested):
    return 0 if getattr(dataset,'ram_loaded',False) else requested
