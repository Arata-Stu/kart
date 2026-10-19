"""Precision resolution and optional CUDA end-to-end AMP training."""
from contextlib import nullcontext
import json
import sys
from unittest.mock import patch

from PIL import Image
import pytest
import torch

from kart_e2e.precision import resolve_precision, autocast_context


def test_cpu_auto_fp32_and_explicit_modes_fail():
    assert resolve_precision('auto','cpu')=='fp32'
    assert resolve_precision('fp32','cpu')=='fp32'
    with autocast_context('fp32'):
        assert torch.ones(1).dtype==torch.float32
    for requested in ('bf16','fp16','invalid'):
        with pytest.raises(ValueError): resolve_precision(requested,'cpu')
    with patch('torch.cuda.is_available',return_value=False),pytest.raises(ValueError,match='unavailable'):
        resolve_precision('auto','cuda')


@pytest.mark.parametrize('major,supported,expected', [(8,True,'bf16'),(7,True,'fp16'),(8,False,'fp16')])
def test_cuda_mode_resolution_targets_requested_device(major,supported,expected):
    with patch('torch.cuda.is_available',return_value=True), \
         patch('torch.cuda.device',return_value=nullcontext()) as selected, \
         patch('torch.cuda.get_device_capability',return_value=(major,0)), \
         patch('torch.cuda.is_bf16_supported',return_value=supported):
        assert resolve_precision('auto','cuda:1')==expected
        selected.assert_called_with(torch.device('cuda:1'))
        assert resolve_precision('fp16','cuda:1')=='fp16'
        if expected=='fp16':
            with pytest.raises(ValueError,match='Native BF16'): resolve_precision('bf16','cuda:1')
        else:
            assert resolve_precision('bf16','cuda:1')=='bf16'


@pytest.mark.parametrize('precision,dtype',[('bf16',torch.bfloat16),('fp16',torch.float16)])
def test_autocast_configuration(precision,dtype):
    with patch('torch.autocast',return_value=nullcontext()) as autocast:
        with autocast_context(precision): pass
        autocast.assert_called_once_with(device_type='cuda',dtype=dtype)


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA GPU required for native AMP training')
@pytest.mark.parametrize('precision',['fp16','bf16'])
def test_cuda_amp_train_fp32_checkpoint_and_metrics(tmp_path,precision):
    from kart_e2e.train import main
    from kart_e2e.model import load_checkpoint
    if precision=='bf16' and (torch.cuda.get_device_capability()[0]<8 or not torch.cuda.is_bf16_supported()):
        pytest.skip('native BF16 GPU required')
    data=tmp_path/'data';data.mkdir()
    Image.new('RGB',(212,120),(80,120,160)).save(data/'image.png')
    (data/'samples.jsonl').write_text(json.dumps(dict(image='image.png',steering=.1,throttle=.2))+'\n')
    run=tmp_path/'run'
    args=['train','--train',str(data),'--validation',str(data),'--repo',str(tmp_path),
          '--architecture','simple_cnn','--initialization','scratch','--output',str(run),
          '--device','cuda','--precision',precision,'--epochs','1','--num-workers','0']
    with patch.object(sys,'argv',args): main()
    ck=torch.load(run/'best.pt',map_location='cpu',weights_only=True)
    assert ck['provenance']['effective_precision']==precision
    assert ck['provenance']['amp_enabled']
    assert ck['provenance']['grad_scaler_enabled']==(precision=='fp16')
    assert all(v.dtype==torch.float32 for v in ck['state_dict'].values() if v.is_floating_point())
    metrics=ck['metrics']
    assert all(torch.isfinite(torch.tensor(metrics[key])) for key in ('train_loss','validation_loss','validation_mse'))
    assert ('grad_scale' in metrics)==(precision=='fp16')
    model=load_checkpoint(run/'best.pt',tmp_path)
    assert torch.isfinite(model(torch.zeros(1,3,128,224))).all()
