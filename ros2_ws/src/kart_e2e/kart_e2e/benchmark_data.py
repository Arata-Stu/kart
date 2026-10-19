"""Deterministic, contract-checked model inputs for calibration and paired benchmarks."""
import hashlib
import io
import json
from pathlib import Path

import numpy as np

from .contract import input_contract
from .data import load_rgb


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


class DatasetInputs:
    def __init__(self, model, dataset, maximum=256, seed=42):
        if type(maximum) is not int or not 1 <= maximum <= 100000:
            raise ValueError('sample count must be 1..100000')
        if type(seed) is not int or not 0 <= seed <= 2147483647:
            raise ValueError('seed must be 0..2147483647')
        self.root=Path(dataset).resolve(strict=True)
        self.modality,self.name,self.shape=input_contract(model)
        metadata=json.loads((self.root/'metadata.json').read_text())
        model_meta=json.loads((Path(model)/'metadata.json').read_text())
        if metadata.get('modality','rgb') != self.modality:
            raise ValueError('Dataset/model modality mismatch')
        if self.modality=='evs' and metadata.get('model_spec') != model_meta['model_spec']:
            raise ValueError('EVS dataset/model spec mismatch')
        manifest=self.root/'samples.jsonl'
        self.manifest_hash=digest(manifest);self.metadata_hash=digest(self.root/'metadata.json')
        rows=[json.loads(line) for line in manifest.read_text().splitlines() if line.strip()]
        if not rows: raise ValueError('Empty benchmark/calibration dataset')
        indices=sorted(np.random.default_rng(seed).choice(len(rows),min(maximum,len(rows)),replace=False).tolist())
        self.rows=[rows[i] for i in indices];self.indices=indices;self.source_hashes={}
        self.paths=[];self.labels=[]
        for row in self.rows:
            path=(self.root/row['counts' if self.modality=='evs' else 'image']).resolve(strict=True)
            if not path.is_relative_to(self.root): raise ValueError('Input outside dataset')
            label=[float(row['steering']),float(row['throttle'])]
            if not np.isfinite(label).all() or not -1<=label[0]<=1 or not 0<=label[1]<=1:
                raise ValueError('Invalid benchmark teacher')
            self.paths.append(path);self.labels.append(label)
        self.labels=np.asarray(self.labels,dtype=np.float64)
        if self.modality=='rgb':
            from .model import preprocess
            self.preprocess=preprocess

    def __len__(self): return len(self.rows)

    def tensor(self,index):
        path=self.paths[index];raw=path.read_bytes();sha=hashlib.sha256(raw).hexdigest()
        if str(path) in self.source_hashes and self.source_hashes[str(path)] != sha:
            raise ValueError('Input changed during calibration/benchmark')
        self.source_hashes[str(path)]=sha
        if self.modality=='evs':
            value=np.load(io.BytesIO(raw),allow_pickle=False)[None]
            if value.dtype != np.float32 or (value<0).any(): raise ValueError('Invalid EVS counts')
        else:
            value=self.preprocess(load_rgb(io.BytesIO(raw),suffix=path.suffix))[:,4:-4,6:-6].unsqueeze(0).numpy()
        if list(value.shape)!=self.shape or not np.isfinite(value).all(): raise ValueError('Invalid model input')
        return np.ascontiguousarray(value,dtype=np.float32)

    def verify(self):
        if digest(self.root/'samples.jsonl')!=self.manifest_hash or digest(self.root/'metadata.json')!=self.metadata_hash:
            raise ValueError('Dataset metadata/manifest changed during benchmark')
        for path,sha in self.source_hashes.items():
            if digest(path)!=sha: raise ValueError('Dataset input changed during benchmark')

    def provenance(self):
        return dict(dataset=str(self.root),manifest_sha256=self.manifest_hash,metadata_sha256=self.metadata_hash,
                    samples=len(self),sample_indices=self.indices,input_sha256=self.source_hashes)
