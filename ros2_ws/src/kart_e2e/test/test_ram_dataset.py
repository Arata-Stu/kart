"""RAM/stream tensor parity, no later IO, reuse, capacity guard and worker policy."""
import json
from unittest.mock import patch

import numpy as np
from PIL import Image
import pytest
import torch
from torch.utils.data import DataLoader

from kart_e2e.train import Samples
from kart_e2e.feature_cache import CachedSamples
from kart_e2e.ram_dataset import RamSamples, preload_datasets, loader_workers, available_memory


@pytest.mark.parametrize('kind',['png','npy','evs'])
def test_tensor_parity_no_reloads_and_identical_dataset_reuse(tmp_path,kind):
    torch.set_num_threads(2)
    rows=[];spec=None
    if kind=='evs':
        spec=dict(height=2,width=3)
        (tmp_path/'metadata.json').write_text(json.dumps(dict(modality='evs',model_spec=spec)))
    for i in range(3):
        path=tmp_path/f'{i}.{kind if kind!="evs" else "npy"}'
        if kind=='evs':np.save(path,np.full((20,2,3),i,np.float32))
        else:
            rgb=np.random.default_rng(i).integers(0,256,(125,219,3),np.uint8)
            if kind=='png':Image.fromarray(rgb).save(path)
            else:np.save(path,rgb)
        rows.append(dict(**{'counts' if kind=='evs' else 'image':path.name},steering=.1*i,throttle=.12))
    (tmp_path/'samples.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
    source=Samples([tmp_path],'steer_throttle',spec)
    expected=[source[i] for i in range(3)]
    with patch('kart_e2e.ram_dataset.available_memory',return_value=10**12):
        train,val=preload_datasets([source,Samples([tmp_path],'steer_throttle',spec)])
    assert train is val and loader_workers(train,8)==0 and loader_workers(source,8)==8
    for i in range(3):
        (tmp_path/rows[i]['counts' if kind=='evs' else 'image']).unlink()
        for actual,want in zip(train[i],expected[i]):torch.testing.assert_close(actual,want,rtol=0,atol=0)
    assert sum(len(batch[0]) for batch in DataLoader(train,batch_size=2,num_workers=loader_workers(train,8)))==3
    assert train.bytes==train.inputs.numel()*4+train.targets.numel()*4
    with pytest.raises(FileNotFoundError):source[0]


def test_feature_cache_preload_reuses_data_but_preserves_labels(tmp_path):
    path=tmp_path/'features.npy';np.save(path,np.arange(12,dtype=np.float32).reshape(3,4))
    items=[(tmp_path/f'{i}.png',[i*.1,.1]) for i in range(3)]
    first=CachedSamples(path,items,'steer_only');second=CachedSamples(path,items,'steer_only')
    a,b=preload_datasets([first,second]);assert a is b
    path.unlink()
    torch.testing.assert_close(a[2][0],torch.tensor([8.,9.,10.,11.]))
    assert a[2][1].shape==(1,)
    different=CachedSamples(tmp_path/'other.npy',items,'steer_only')
    np.save(different.path,np.zeros((3,4),np.float32))
    train,val=preload_datasets([first,different]);assert train is not val
    assert train.inputs[0,1]==1 and val.inputs[0,1]==0


def test_capacity_guard_precedes_large_allocation(tmp_path):
    source=[(torch.zeros(3,4),torch.zeros(2))]*4
    with patch('kart_e2e.ram_dataset.available_memory',return_value=100),patch('kart_e2e.ram_dataset.torch.empty') as empty:
        with pytest.raises(MemoryError,match='no-ram-preload'):RamSamples(source)
        empty.assert_not_called()


def test_linux_available_memory_respects_container_limit():
    files={'/proc/meminfo':'MemAvailable: 100000 kB\n',
           '/sys/fs/cgroup/memory.max':'1000000','/sys/fs/cgroup/memory.current':'700000'}
    def read(path):
        if str(path) not in files:raise FileNotFoundError(path)
        return files[str(path)]
    with patch('pathlib.Path.read_text',read):assert available_memory()==300000
