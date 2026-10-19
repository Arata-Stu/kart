"""Two-stage DINOv3 Tiny compression, checkpoint/export, no downloaded weights."""
import json
import os
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

import numpy as np
import pytest
import torch
from PIL import Image
from kart_e2e.contract import rgb_spec, model_contract
from kart_e2e.distill import alignment_loss


def test_alignment_loss_and_gradient():
    student=torch.randn(2,192,requires_grad=True)
    projection=torch.nn.Linear(192,384)
    teacher=torch.randn(2,384)
    loss,mse,cosine=alignment_loss(projection(student),teacher)
    assert torch.isfinite(loss) and mse>=0
    loss.backward()
    assert student.grad.abs().sum()>0 and projection.weight.grad.abs().sum()>0
    assert teacher.grad is None
    exact=torch.randn(2,384)
    assert alignment_loss(exact,exact)[0].abs()<1e-6


@pytest.mark.skipif(not os.environ.get('KART_DINOV3_REPO'),reason='Set KART_DINOV3_REPO for official architecture smoke')
def test_real_tiny_two_stages_export_and_finetune(tmp_path):
    from kart_e2e.model import backbone,load_checkpoint
    from kart_e2e.export_onnx import export
    from kart_e2e.features import visualize
    repo=os.environ['KART_DINOV3_REPO']
    torch.set_num_threads(2);torch.manual_seed(42)
    teacher=backbone(repo)
    initial={k:v.clone() for k,v in teacher.state_dict().items()}
    weights=tmp_path/'teacher.pt';torch.save(initial,weights)
    data=tmp_path/'data';data.mkdir()
    for i in range(2):
        np.save(data/f'{i}.npy',np.random.default_rng(i).integers(0,256,(120,212,3),dtype=np.uint8),allow_pickle=False)
    (data/'samples.jsonl').write_text(''.join(json.dumps(dict(image=f'{i}.npy',steering=.2,throttle=.1,stamp_ns=i*100000000))+'\n' for i in range(2)))
    env=dict(os.environ,OMP_NUM_THREADS='2',MKL_NUM_THREADS='2')
    args=['--train',str(data),'--validation',str(data),'--repo',repo,'--device','cpu','--epochs','1','--num-workers','0','--batch-size','2']
    result=subprocess.run([sys.executable,'-m','kart_e2e.distill',*args,'--teacher-weights',str(weights),'--output',str(tmp_path/'encoder')],env=env,check=True,capture_output=True,text=True)
    assert 'Feature cache reused' in result.stdout
    ck=torch.load(tmp_path/'encoder/encoder.pt',weights_only=True)
    assert ck['spec']==rgb_spec('dinov3_vitt16')
    assert ck['projection_state_dict']['weight'].shape==(384,192)
    after=torch.load(weights,weights_only=True)
    assert all(torch.equal(v,after[k]) for k,v in initial.items())
    assert ck['metrics']['validation_loss']>=0
    # A trained ViT-S checkpoint is also accepted; teacher cache can be disabled.
    from kart_e2e.model import Policy
    teacher_policy=Policy(teacher,'steer_only').eval()
    teacher_checkpoint=tmp_path/'teacher-policy.pt'
    torch.save(dict(spec=rgb_spec(),mode='steer_only',state_dict=teacher_policy.state_dict(),runtime=teacher_policy.runtime),teacher_checkpoint)
    subprocess.run([sys.executable,'-m','kart_e2e.distill',*args,'--teacher-weights',str(teacher_checkpoint),
                    '--no-feature-cache','--output',str(tmp_path/'uncached-encoder')],env=env,check=True,capture_output=True,text=True)
    assert json.loads((tmp_path/'uncached-encoder/run.json').read_text())['feature_cache_keys']==[]
    rejected=subprocess.run([sys.executable,'-m','kart_e2e.distill',*args,'--teacher-weights',str(tmp_path/'encoder/encoder.pt'),
                             '--output',str(tmp_path/'bad-teacher')],env=env,capture_output=True,text=True)
    assert rejected.returncode!=0 and 'Teacher must be an RGB ViT-S checkpoint' in rejected.stderr
    assert not (tmp_path/'bad-teacher').exists()
    for mode in ('steer_only','steer_throttle'):
        run=tmp_path/mode
        subprocess.run([sys.executable,'-m','kart_e2e.train',*args,'--architecture','dinov3_vitt16',
                        '--weights',str(tmp_path/'encoder/encoder.pt'),'--mode',mode,'--output',str(run)],env=env,check=True,capture_output=True,text=True)
        policy=load_checkpoint(run/'best.pt',repo)
        assert policy.feature_dim==192 and policy.encoder.num_heads==3 and len(policy.encoder.blocks)==12
        assert policy.encoder.n_storage_tokens==4
        assert 5000000<sum(p.numel() for p in policy.encoder.parameters())<6000000
        assert all(torch.equal(v,policy.encoder.state_dict()[k]) for k,v in ck['encoder_state_dict'].items())
        bundle=tmp_path/(mode+'-onnx');meta=export(policy,bundle)
        assert model_contract(bundle)[1]==mode and meta['model_spec']==ck['spec']
        assert meta['ort_max_abs_error']<1e-5
    # Stage 2 full fine-tuning and qualitative feature display remain connected.
    finetune=tmp_path/'finetune'
    subprocess.run([sys.executable,'-m','kart_e2e.train',*args,'--architecture','dinov3_vitt16','--finetune',
                    '--weights',str(tmp_path/'encoder/encoder.pt'),'--output',str(finetune)],env=env,check=True,capture_output=True,text=True)
    policy=load_checkpoint(finetune/'best.pt',repo)
    assert not torch.equal(policy.encoder.patch_embed.proj.weight,ck['encoder_state_dict']['patch_embed.proj.weight'])
    assert not list((tmp_path/'finetune').glob('projection*'))
    report=visualize(tmp_path/'steer_only/best.pt',data,repo,tmp_path/'features',max_frames=1)
    assert report['frame_count']==1
