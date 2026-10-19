"""Offline visualization contracts, no downloaded weights or ROS required."""
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest
import torch
from PIL import Image
from kart_e2e.features import visualize, pca_colors, sample_rows

class Encoder(torch.nn.Module):
    def forward_features(self, x):
        return {'x_norm_patchtokens': torch.nn.functional.avg_pool2d(x,16).flatten(2).transpose(1,2)}

class Policy(torch.nn.Module):
    mode = 'steer_only'
    def __init__(self):
        super().__init__()
        self.encoder = Encoder()
    def forward(self,x):
        return x.mean((1,2,3)).tanh().unsqueeze(1)


def test_shared_pca_constant_and_reproducible():
    rng = np.random.default_rng(2)
    values = rng.normal(size=(224,6))
    a,b,ratio = pca_colors([values[:112],values[112:]])
    assert np.array_equal(a,pca_colors([values])[0])
    assert len(ratio)==3 and 0 < sum(ratio) <= 1
    assert np.all(pca_colors([np.ones((112,4))])[0] == 128)


@pytest.mark.parametrize("image_format", ["png","npy"])
def test_checkpoint_report_and_images(tmp_path,image_format):
    dataset=tmp_path/'data'; dataset.mkdir()
    from kart_e2e.data import save_rgb
    filename='image.'+image_format
    save_rgb(Image.fromarray(np.random.default_rng(1).integers(0,256,(120,212,3),dtype=np.uint8)),dataset/filename,image_format)
    rows=[dict(image=filename,stamp_ns=i*100000000,steering=.1,throttle=.2) for i in range(5)]
    (dataset/'samples.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    checkpoint=tmp_path/'best.pt'; checkpoint.write_bytes(b'fixture')
    output=tmp_path/'result'
    with patch('kart_e2e.features.load_checkpoint',return_value=Policy()):
        report=visualize(checkpoint,dataset,tmp_path,output,max_frames=3)
    assert [r['sample_index'] for r in report['frames']]==[0,2,4]
    assert report['frame_count']==3
    for row in report['frames']:
        for key in ('rgb','pca','saliency'):
            assert Image.open(output/row[key]).size==(212,120)
    assert (output/'pca_basis.npz').exists()
    assert json.loads((output/'report.json').read_text())['checkpoint_sha256']==report['checkpoint_sha256']
    with pytest.raises(FileExistsError):
        visualize(checkpoint,dataset,tmp_path,output)


def test_dataset_image_cannot_escape(tmp_path):
    data=tmp_path/'data'; data.mkdir()
    Image.new('RGB',(212,120)).save(tmp_path/'outside.png')
    (data/'samples.jsonl').write_text(json.dumps(dict(image='../outside.png',stamp_ns=0,steering=0,throttle=0))+'\n')
    checkpoint=tmp_path/'best.pt'; checkpoint.write_bytes(b'fixture')
    with patch('kart_e2e.features.load_checkpoint',return_value=Policy()):
        with pytest.raises(ValueError,match='outside dataset'):
            visualize(checkpoint,data,tmp_path,tmp_path/'output')
    assert not (tmp_path/'output').exists()

@pytest.mark.parametrize('width', [160,320])
def test_evs_features_input_pca_gradient_and_channels(tmp_path,width):
    from evs_fixtures import metadata
    from kart_e2e.contract import evs_spec
    from kart_e2e.event_encoder_model import PortableEventEncoder
    from kart_e2e.evs_policy import EVSPolicy
    class Backbone(torch.nn.Module):
        def forward_features(self,x):
            patches=torch.nn.functional.avg_pool2d(x.mean(1,keepdim=True),16).flatten(2).transpose(1,2)
            return {'x_norm_patchtokens':patches.expand(-1,-1,384)}
    model=EVSPolicy(PortableEventEncoder(Backbone(),metadata(width)),mode='steer_only').eval()
    data=tmp_path/'data'; data.mkdir()
    (data/'metadata.json').write_text(json.dumps(dict(modality='evs',model_spec=model.spec)))
    h,w=model.spec['height'],model.spec['width']
    counts=np.zeros((20,h,w),dtype=np.float32); counts[0,10:20,10:20]=4; counts[19,30:40,30:40]=2
    np.save(data/'counts.npy',counts)
    rows=[dict(counts='counts.npy',stamp_ns=i*100000000,steering=.1,throttle=.2) for i in range(3)]
    (data/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    checkpoint=tmp_path/'best.pt';checkpoint.write_bytes(b'fixture')
    with patch('kart_e2e.evs_features.load_checkpoint',return_value=model):
        report=visualize(checkpoint,data,tmp_path,tmp_path/'out',max_frames=2)
    assert report['schema']=='kart.evs_features.v1' and report['frame_count']==2
    assert report['sensitivity_scale_p99']>0
    for row in report['frames']:
        for key in ('rgb','pca','saliency'):
            assert Image.open(tmp_path/'out'/row[key]).size==(w,h)
        assert Image.open(tmp_path/'out'/row['channels']).size==(10*w,2*(h+max(20,w//5)))
        assert np.asarray(Image.open(tmp_path/'out'/row['rgb'])).max()>0
    bad=dict(model.spec,width=1)
    (data/'metadata.json').write_text(json.dumps(dict(modality='evs',model_spec=bad)))
    with patch('kart_e2e.evs_features.load_checkpoint',return_value=model),pytest.raises(ValueError,match='spec mismatch'):
        visualize(checkpoint,data,tmp_path,tmp_path/'bad')
