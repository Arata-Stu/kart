"""Scratch isolation, CNN portability, and optional official ViT-B smoke."""
import json
import os
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
from PIL import Image
import pytest
import torch

from kart_e2e.contract import rgb_spec, model_contract
from kart_e2e.model import backbone, Policy, load_checkpoint
from kart_e2e.train import main


def dataset(root):
    root.mkdir()
    Image.new('RGB',(212,120),(120,60,30)).save(root/'image.png')
    (root/'metadata.json').write_text(json.dumps(dict(modality='rgb',samples=1)))
    (root/'samples.jsonl').write_text(json.dumps(dict(image='image.png',stamp_ns=1000000000,steering=.2,throttle=.1))+'\n')
    return root


def test_cnn_scratch_defaults_train_restore_export_evaluate_features(tmp_path):
    from kart_e2e.export_onnx import export
    from kart_e2e.evaluate import evaluate
    from kart_e2e.features import visualize
    torch.set_num_threads(2)
    data=dataset(tmp_path/'data'); run=tmp_path/'run'; missing=tmp_path/'no-dino'
    torch.manual_seed(42)
    before=backbone(missing,'simple_cnn').blocks[0].weight.detach().clone()
    argv=['train','--train',str(data),'--validation',str(data),'--repo',str(missing),
          '--architecture','simple_cnn','--initialization','scratch','--output',str(run),
          '--device','cpu','--num-workers','0','--seed','42']
    # No weights and no epochs: actually exercises the default 50 epochs.
    with patch.object(sys,'argv',argv): main()
    provenance=json.loads((run/'run.json').read_text())
    assert provenance['epochs']==50 and provenance['initialization']=='scratch'
    assert provenance['finetune'] and not provenance['feature_cache_effective']
    assert provenance['weights_sha256'] is None
    assert provenance['learning_rate']==1e-4
    assert provenance['total_parameters']==provenance['trainable_parameters']
    assert len((run/'metrics.jsonl').read_text().splitlines())==50
    policy=load_checkpoint(run/'best.pt',missing)
    assert not torch.equal(before,policy.encoder.blocks[0].weight)
    assert policy.spec==rgb_spec('simple_cnn') and policy.feature_dim==192
    bundle=tmp_path/'onnx'; report=export(policy,bundle)
    assert report['ort_max_abs_error']<1e-5
    assert model_contract(bundle)[1]=='steer_throttle'
    assert evaluate(bundle,data,tmp_path/'eval')['samples']==1
    assert visualize(run/'best.pt',data,missing,tmp_path/'features',max_frames=1)['frame_count']==1


@pytest.mark.parametrize('extra', [[],['--initialization','scratch','--weights','unused.pt'],
                                    ['--initialization','scratch','--modality','evs']])
def test_invalid_initialization_rejected_before_data_access(tmp_path,extra):
    args=['train','--train','missing','--validation','missing','--repo','missing',
          '--architecture','simple_cnn','--output',str(tmp_path/'output'),*extra]
    with patch.object(sys,'argv',args),pytest.raises(SystemExit): main()
    assert not (tmp_path/'output').exists()


@pytest.mark.skipif(not os.environ.get('KART_DINOV3_REPO'),reason='local pinned DINOv3 required')
def test_official_vit_base_scratch_checkpoint_and_export(tmp_path):
    from kart_e2e.export_onnx import export
    torch.set_num_threads(2)
    repo=os.environ['KART_DINOV3_REPO']
    data=dataset(tmp_path/'data');run=tmp_path/'base'
    argv=['train','--train',str(data),'--validation',str(data),'--repo',repo,
          '--architecture','dinov3_vitb16','--initialization','scratch','--output',str(run),
          '--device','cpu','--num-workers','0','--epochs','1','--batch-size','1','--mode','steer_only']
    with patch.object(sys,'argv',argv): main()
    policy=load_checkpoint(run/'best.pt',repo)
    assert policy.feature_dim==768 and policy.head[0].in_features==768
    assert len(policy.encoder.blocks)==12
    assert 80_000_000 < sum(p.numel() for p in policy.encoder.parameters()) < 90_000_000
    ck=torch.load(run/'best.pt',weights_only=True)
    assert ck['provenance']['initialization']=='scratch' and ck['provenance']['finetune']
    result=export(policy,tmp_path/'base-onnx')
    assert result['ort_max_abs_error']<1e-5
    assert model_contract(tmp_path/'base-onnx')[1]=='steer_only'


@pytest.mark.parametrize('architecture', ['dinov3_vits16','dinov3_vitt16'])
@pytest.mark.skipif(not os.environ.get('KART_DINOV3_REPO'),reason='local pinned DINOv3 required')
def test_scratch_other_vit_sizes_without_weights(tmp_path,architecture):
    torch.set_num_threads(2)
    data=dataset(tmp_path/'data');run=tmp_path/'run'
    args=['train','--train',str(data),'--validation',str(data),'--repo',os.environ['KART_DINOV3_REPO'],
          '--architecture',architecture,'--initialization','scratch','--output',str(run),
          '--device','cpu','--num-workers','0','--epochs','1','--mode','steer_only']
    with patch.object(sys,'argv',args): main()
    ck=torch.load(run/'best.pt',weights_only=True)
    assert ck['provenance']['initialization']=='scratch'
    assert ck['provenance']['trainable_parameters']==ck['provenance']['total_parameters']
    assert not ck['provenance']['feature_cache_effective']
