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


def test_evs_and_path_escape_rejected(tmp_path):
    data=tmp_path/'data';data.mkdir()
    (data/'metadata.json').write_text('{"modality":"evs"}')
    with pytest.raises(ValueError,match='RGB'):
        visualize(tmp_path/'best.pt',data,tmp_path,tmp_path/'out')


def test_dataset_image_cannot_escape(tmp_path):
    data=tmp_path/'data'; data.mkdir()
    Image.new('RGB',(212,120)).save(tmp_path/'outside.png')
    (data/'samples.jsonl').write_text(json.dumps(dict(image='../outside.png',stamp_ns=0,steering=0,throttle=0))+'\n')
    checkpoint=tmp_path/'best.pt'; checkpoint.write_bytes(b'fixture')
    with patch('kart_e2e.features.load_checkpoint',return_value=Policy()):
        with pytest.raises(ValueError,match='outside dataset'):
            visualize(checkpoint,data,tmp_path,tmp_path/'output')
    assert not (tmp_path/'output').exists()
