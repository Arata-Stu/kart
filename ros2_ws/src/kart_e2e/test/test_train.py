"""CLI defaults and optimizer scope exercised through a one-epoch local run."""
import json
import sys
from unittest.mock import patch
import pytest
import torch
from PIL import Image
from kart_e2e.train import main

class TinyEncoder(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.proj = torch.nn.Linear(3,384)
    def forward(self,value):
        return self.proj(value.mean((2,3)))

@pytest.mark.parametrize('image_format', ['png','npy'])
@pytest.mark.parametrize('finetune,explicit,expected,cache', [(False,None,1e-4,True),(False,None,1e-4,False),(True,None,1e-5,True),(True,2e-6,2e-6,True)])
def test_cli_update_scope_and_provenance(tmp_path,finetune,explicit,expected,cache,image_format):
    torch.set_num_threads(2)
    torch.manual_seed(42)
    encoder=TinyEncoder()
    initial={k:v.clone() for k,v in encoder.state_dict().items()}
    weights=tmp_path/'weights.pt';torch.save(initial,weights)
    data=tmp_path/'data';data.mkdir()
    from kart_e2e.data import save_rgb
    filename='image.'+image_format
    save_rgb(Image.new('RGB',(212,120),(200,100,20)),data/filename,image_format)
    (data/'samples.jsonl').write_text(json.dumps(dict(image=filename,steering=.8,throttle=.1))+'\n')
    output=tmp_path/'run'
    args=['train','--train',str(data),'--validation',str(data),'--repo',str(tmp_path),
          '--weights',str(weights),'--output',str(output),'--device','cpu','--epochs','1','--num-workers','0','--mode','steer_only']
    if not cache: args+=['--no-feature-cache']
    if finetune: args+=['--finetune']
    if explicit is not None: args+=['--learning-rate',str(explicit)]
    with patch.object(sys,'argv',args),patch('kart_e2e.train.backbone',return_value=encoder):
        main()
    report=json.loads((output/'run.json').read_text())
    assert report['learning_rate']==expected and report['finetune']==finetune
    assert report['feature_cache_effective']==(cache and not finetune)
    assert bool(report['feature_cache_keys'])==(cache and not finetune)
    saved=torch.load(output/'best.pt',weights_only=True)
    changed=not torch.equal(initial['proj.weight'],saved['state_dict']['encoder.proj.weight'])
    assert changed==finetune
    assert (report['trainable_parameters']==report['total_parameters'])==finetune
