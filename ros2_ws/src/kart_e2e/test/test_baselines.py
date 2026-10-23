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

from kart_e2e.contract import (
    FLATTEN_SIMPLE_CNN_COMMIT, LEGACY_SIMPLE_CNN_COMMIT, SIMPLE_CNN_COMMIT,
    rgb_spec, model_contract,
)
from kart_e2e.model import backbone, Policy, load_checkpoint
from kart_e2e.train import main


def dataset(root):
    root.mkdir()
    Image.new('RGB',(212,120),(120,60,30)).save(root/'image.png')
    (root/'metadata.json').write_text(json.dumps(dict(modality='rgb',samples=1)))
    (root/'samples.jsonl').write_text(json.dumps(dict(image='image.png',stamp_ns=1000000000,steering=.2,throttle=.1))+'\n')
    return root


def test_cnn_flatten_retains_spatial_order_and_position():
    torch.set_num_threads(2)
    encoder = backbone('unused', 'simple_cnn')
    # Isolate pooling with equal-mean maps whose activation positions differ.
    maps = torch.zeros(2, 64, 3, 9)
    maps[0, 0, 0, 0] = 1
    maps[1, 0, 0, 1] = 1
    with patch.object(encoder, 'spatial_features', return_value=maps):
        encoded = encoder(torch.zeros(2, 3, 128, 224))
    assert encoded.shape == (2, 1728)
    assert encoded[0, 0] == 1 and encoded[1, 1] == 1
    assert not torch.equal(encoded[0], encoded[1])
    torch.testing.assert_close(maps.mean((2, 3))[0], maps.mean((2, 3))[1])


def test_pilotnet_topology_resolution_and_parameter_budget():
    torch.set_num_threads(2)
    model = Policy(backbone('unused', 'simple_cnn'), 'steer_only',
                   frozen=False, architecture='simple_cnn')
    layers = [layer for layer in model.encoder.blocks if isinstance(layer, torch.nn.Conv2d)]
    assert [(layer.out_channels, layer.kernel_size[0], layer.stride[0], layer.padding[0])
            for layer in layers] == [(24,5,2,0), (36,5,2,0), (48,5,2,0), (64,3,2,0), (64,3,1,0)]
    observed = []
    hook = layers[0].register_forward_pre_hook(lambda _, inputs: observed.append(tuple(inputs[0].shape)))
    spatial = model.encoder.forward_features(torch.zeros(1,3,128,224))
    hook.remove()
    assert observed == [(1,3,120,212)]
    assert spatial['spatial_shape'] == (3,9)
    assert spatial['x_norm_patchtokens'].shape == (1,27,64)
    assert [(layer.in_features, layer.out_features) for layer in model.head
            if isinstance(layer, torch.nn.Linear)] == [(1728,100),(100,50),(50,10),(10,1)]
    assert sum(p.numel() for p in model.parameters()) == 309819


@pytest.mark.parametrize('version', [SIMPLE_CNN_COMMIT, FLATTEN_SIMPLE_CNN_COMMIT, LEGACY_SIMPLE_CNN_COMMIT])
@pytest.mark.parametrize('mode,width', [('steer_only', 1), ('steer_throttle', 2)])
def test_cnn_versions_restore_and_export_with_same_output_contract(tmp_path, version, mode, width):
    from kart_e2e.export_onnx import export
    torch.set_num_threads(2)
    torch.manual_seed(42)
    model = Policy(backbone('unused', 'simple_cnn', cnn_version=version),
                   mode, frozen=False, architecture='simple_cnn').eval()
    spec = rgb_spec('simple_cnn')
    spec['encoder_commit'] = version
    assert model.spec == spec
    image = torch.randn(2, 3, 128, 224)
    expected = model(image).detach()
    assert expected.shape == (2, width)
    assert torch.all(expected[:, 0].abs() <= 1)
    if width == 2:
        assert torch.all((expected[:, 1] >= 0) & (expected[:, 1] <= 1))
    model(image).sum().backward()
    assert model.encoder.blocks[0].weight.grad.abs().sum() > 0
    assert model.head[0].weight.grad.abs().sum() > 0
    checkpoint = tmp_path / 'model.pt'
    torch.save(dict(spec=spec, mode=mode, runtime=model.runtime,
                    state_dict=model.state_dict()), checkpoint)
    restored = load_checkpoint(checkpoint, 'unused')
    assert restored.spec == spec and restored.encoder.encoder_commit == version
    torch.testing.assert_close(restored(image), expected)
    bundle = tmp_path / 'onnx'
    report = export(restored, bundle)
    assert report['model_spec'] == spec and report['output_shape'] == [1, width]
    # export() checks all probes with the shared rtol=1e-4, atol=1e-5 contract.
    assert model_contract(bundle)[1] == mode
    # Relabelling old weights must fail instead of silently changing pooling.
    wrong_spec = dict(spec, encoder_commit=(SIMPLE_CNN_COMMIT
                      if version != SIMPLE_CNN_COMMIT else LEGACY_SIMPLE_CNN_COMMIT))
    torch.save(dict(spec=wrong_spec, mode=mode, state_dict=model.state_dict()), checkpoint)
    with pytest.raises(RuntimeError, match='size mismatch'):
        load_checkpoint(checkpoint, 'unused')


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
    # No weights/epochs/scheduler: actually exercises the default control recipe.
    with patch.object(sys,'argv',argv): main()
    provenance=json.loads((run/'run.json').read_text())
    assert provenance['epochs']==100 and provenance['initialization']=='scratch'
    assert provenance['loss']=='smooth_l1' and provenance['lr_scheduler']=='plateau'
    assert provenance['finetune'] and not provenance['feature_cache_effective']
    assert provenance['weights_sha256'] is None
    assert provenance['learning_rate']==1e-4
    assert provenance['total_parameters']==provenance['trainable_parameters']
    assert len((run/'metrics.jsonl').read_text().splitlines())==100
    policy=load_checkpoint(run/'best.pt',missing)
    assert not torch.equal(before,policy.encoder.blocks[0].weight)
    assert policy.spec==rgb_spec('simple_cnn') and policy.feature_dim==1728
    assert provenance['total_parameters']==309830
    bundle=tmp_path/'onnx'; report=export(policy,bundle)
    assert report['ort_max_abs_error']<1e-5
    assert model_contract(bundle)[1]=='steer_throttle'
    assert evaluate(bundle,data,tmp_path/'eval')['samples']==1
    features = visualize(run/'best.pt',data,missing,tmp_path/'features',max_frames=1)
    assert features['frame_count']==1 and features['patch_grid']==[3,9]
    assert features['spatial_layout']=='overlapping_conv_features_resized_for_display_not_pixel_aligned'
    assert Image.open(tmp_path/'features'/features['frames'][0]['pca']).size == (212,120)


@pytest.mark.parametrize('extra', [[],['--initialization','scratch','--weights','unused.pt'],
                                    ['--initialization','scratch','--modality','evs','--architecture','dinov3_vits16']])
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
