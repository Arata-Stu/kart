"""Cache reuse, content invalidation and policy head equivalence without downloads."""
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
import pytest
import torch
from torch.utils.data import DataLoader
from PIL import Image
from kart_e2e.train import Samples
from kart_e2e.model import Policy
from kart_e2e.feature_cache import cached_samples

class TinyEncoder(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.proj=torch.nn.Linear(3,384)
    def forward(self,x):
        return self.proj(x.mean((2,3)))


def test_cache_reuse_identity_and_corruption(tmp_path):
    data=tmp_path/'data'; data.mkdir()
    image=data/'image.png';Image.new('RGB',(212,120),(110,70,40)).save(image)
    (data/'samples.jsonl').write_text(json.dumps(dict(image='image.png',steering=.2,throttle=.1))+'\n')
    dataset=Samples([data],'steer_only')
    policy=Policy(TinyEncoder(),'steer_only').eval()
    calls=[]
    def loader(dataset,shuffle,persistent=False):
        calls.append(1)
        return DataLoader(dataset,batch_size=1,shuffle=shuffle)
    root=tmp_path/'cache'
    cached,key=cached_samples(dataset,policy,root,'weight-A',loader,'cpu')
    value,label=dataset[0]
    torch.testing.assert_close(policy.predict_features(cached[0][0].unsqueeze(0)),policy(value.unsqueeze(0)))
    torch.testing.assert_close(cached[0][1],label)
    with patch.object(policy,'extract_features',side_effect=AssertionError('Must reuse')):
        again,key2=cached_samples(dataset,policy,root,'weight-A',loader,'cpu')
        assert key==key2 and len(calls)==1
        torch.testing.assert_close(again[0][0],cached[0][0])
    _,different_weights=cached_samples(dataset,policy,root,'weight-B',loader,'cpu')
    assert different_weights!=key
    Image.new('RGB',(212,120),(12,5,9)).save(image)
    _,different_image=cached_samples(dataset,policy,root,'weight-A',loader,'cpu')
    assert different_image!=key
    path=root/different_image/'features.npy';path.write_bytes(b'corrupt')
    with pytest.raises(ValueError,match='Invalid feature cache'):
        cached_samples(dataset,policy,root,'weight-A',loader,'cpu')
    assert not list(root.glob('.building-*'))
