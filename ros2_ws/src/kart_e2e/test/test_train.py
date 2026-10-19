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
    if image_format == 'npy': args+=['--loss','mse','--lr-scheduler','plateau','--lr-patience','0']
    if not cache: args+=['--no-feature-cache']
    if finetune: args+=['--finetune']
    if explicit is not None: args+=['--learning-rate',str(explicit)]
    with patch.object(sys,'argv',args),patch('kart_e2e.train.backbone',return_value=encoder):
        main()
    report=json.loads((output/'run.json').read_text())
    assert report['effective_precision']=='fp32' and not report['amp_enabled']
    assert not report['grad_scaler_enabled']
    assert report['precision']=='auto'
    assert report['learning_rate']==expected and report['finetune']==finetune
    assert report['loss']==('mse' if image_format=='npy' else 'smooth_l1')
    assert report['selection_metric']=='validation_mse'
    metrics=json.loads((output/'metrics.jsonl').read_text().strip())
    assert metrics['learning_rate']==expected
    assert metrics['next_learning_rate']==expected
    assert metrics['train_loss']>=0 and metrics['validation_loss']>=0
    if image_format=='npy': assert metrics['validation_loss']==metrics['validation_mse']
    assert report['feature_cache_effective']==(cache and not finetune)
    assert bool(report['feature_cache_keys'])==(cache and not finetune)
    saved=torch.load(output/'best.pt',weights_only=True)
    changed=not torch.equal(initial['proj.weight'],saved['state_dict']['encoder.proj.weight'])
    assert changed==finetune
    assert (report['trainable_parameters']==report['total_parameters'])==finetune


def test_smooth_l1_values_gradients_and_mse_comparison():
    from kart_e2e.train import regression_loss
    prediction=torch.tensor([0.05,0.2],requires_grad=True)
    target=torch.zeros(2)
    loss=regression_loss(prediction,target,'smooth_l1',.1)
    torch.testing.assert_close(loss,torch.tensor((.0125+.15)/2))
    loss.backward()
    torch.testing.assert_close(prediction.grad,torch.tensor([.25,.5]))
    torch.testing.assert_close(regression_loss(prediction,target,'mse'),torch.tensor((.0025+.04)/2))
    with pytest.raises(ValueError):
        regression_loss(prediction,target,'smooth_l1',0)


def test_plateau_patience_floor_and_improvement():
    from kart_e2e.train import plateau_scheduler
    parameter=torch.nn.Parameter(torch.tensor(1.))
    optimizer=torch.optim.AdamW([parameter],lr=.001)
    scheduler=plateau_scheduler(optimizer,factor=.5,patience=2,min_lr=.0003)
    for value in [1.,1.,1.]: scheduler.step(value)
    assert optimizer.param_groups[0]['lr']==.001
    scheduler.step(1.)
    assert optimizer.param_groups[0]['lr']==.0005
    scheduler.step(.5) # improvement resets patience
    for _ in range(2): scheduler.step(.5)
    assert optimizer.param_groups[0]['lr']==.0005
    scheduler.step(.5)
    assert optimizer.param_groups[0]['lr']==.0003
    for _ in range(5): scheduler.step(.5)
    assert optimizer.param_groups[0]['lr']==.0003
