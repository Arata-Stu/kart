"""Portable EVS PilotNet training/export checks; no camera, pretrained weights or GPU."""
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
from PIL import Image
import pytest
import torch

from evs_fixtures import metadata
from kart_e2e.contract import evs_spec, evs_policy_spec, input_contract, model_contract
from kart_e2e.evs_policy import EVSPilotNetPolicy, load_checkpoint


def dataset(root):
    root.mkdir()
    meta = metadata()
    rows = []
    rng = np.random.default_rng(42)
    for i in range(2):
        name = f'{i}.npy'
        np.save(root/name, rng.poisson(.2, (20,120,160)).astype(np.float32))
        rows.append(dict(counts=name,stamp_ns=1_000_000_000+i*100_000_000,
                         steering=.1 if i else -.1,throttle=.1))
    (root/'samples.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    (root/'metadata.json').write_text(json.dumps(dict(modality='evs',samples=2,
                                      model_spec=evs_spec(meta),evs_metadata=meta)))
    return root


def test_evs_cnn_shape_channels_normalization_and_budget():
    torch.set_num_threads(2)
    policy = EVSPilotNetPolicy(metadata(), 'steer_only')
    assert policy.feature_dim == 1152
    assert policy.encoder.blocks[0].in_channels == 20
    assert sum(p.numel() for p in policy.parameters()) == 262419
    counts = torch.zeros(1,20,120,160)
    counts[0,19,3,4] = 8
    normalized = policy.encoder.prepare_content(counts)
    assert normalized.shape == counts.shape
    torch.testing.assert_close(normalized, (counts-.1)/2)
    observed = []
    hook = policy.encoder.blocks[0].register_forward_pre_hook(
        lambda _, inputs: observed.append(inputs[0].detach().clone()))
    assert policy(counts).shape == (1,1)
    hook.remove()
    torch.testing.assert_close(observed[0], normalized)
    assert policy.encoder(normalized).shape == (1,18,64)
    with pytest.raises(ValueError, match='160x120'):
        EVSPilotNetPolicy(metadata(320))
    assert evs_policy_spec(metadata()) == evs_spec(metadata())
    assert evs_policy_spec(metadata(), 'simple_cnn')['encoder_commit'] == 'kart.evs_pilotnet.v1'


@pytest.mark.parametrize('mode', ['steer_only','steer_throttle'])
def test_evs_cnn_train_restore_export_evaluate_visualize_quantize(tmp_path, mode):
    from kart_e2e.train import main
    from kart_e2e.export_onnx import export
    from kart_e2e.evaluate import evaluate
    from kart_e2e.evs_features import visualize
    from kart_e2e.int8_quantize import quantize
    from kart_e2e.benchmark_data import DatasetInputs
    torch.set_num_threads(2)
    data = dataset(tmp_path/'data')
    run = tmp_path/'run'
    args = ['train','--train',str(data),'--validation',str(data),'--repo','unused',
            '--output',str(run),'--modality','evs','--architecture','simple_cnn',
            '--initialization','scratch','--mode',mode,'--device','cpu','--num-workers','0']
    # Exercise the real 100-epoch default, with no encoder checkout or weights.
    with patch.object(sys,'argv',args):
        main()
    provenance = json.loads((run/'run.json').read_text())
    assert provenance['epochs'] == 100 and provenance['finetune']
    assert not provenance['feature_cache_effective'] and provenance['weights_sha256'] is None
    assert provenance['trainable_parameters'] == provenance['total_parameters']
    assert len((run/'metrics.jsonl').read_text().splitlines()) == 100
    model = load_checkpoint(run/'best.pt','unused')
    counts = torch.rand(2,20,120,160)
    expected = model(counts).detach()
    assert expected.shape == (2,1 if mode == 'steer_only' else 2)
    assert (expected[:,0].abs() <= 1).all()
    if mode == 'steer_throttle':
        assert ((expected[:,1] >= 0) & (expected[:,1] <= 1)).all()
    bundle = tmp_path/'onnx'
    report = export(model, bundle)
    assert report['model_spec'] == model.spec
    assert input_contract(bundle) == ('evs','input_tensor',[1,20,120,160])
    assert model_contract(bundle)[1] == mode
    assert evaluate(bundle,data,tmp_path/'eval')['samples'] == 2
    assert DatasetInputs(bundle,data).tensor(0).shape == (1,20,120,160)
    feature_report = visualize(run/'best.pt',data,'unused',tmp_path/'features',max_frames=1)
    assert feature_report['patch_grid'] == [3,6]
    for kind in ('rgb','pca','saliency'):
        assert Image.open(tmp_path/'features'/feature_report['frames'][0][kind]).size == (160,120)
    assert feature_report['spatial_layout'].endswith('not_pixel_aligned')
    # Also cover calibration data/spec matching and actual CPU Q/DQ execution.
    if mode == 'steer_only':
        quantized = quantize(bundle,data,tmp_path/'int8',samples=2)
        assert len(quantized['quantization']['quantized_nodes']) == 5
        assert input_contract(tmp_path/'int8')[2] == [1,20,120,160]
    state = torch.load(run/'best.pt',weights_only=True)
    state['spec']['encoder_commit'] = 'wrong'
    torch.save(state,tmp_path/'invalid.pt')
    with pytest.raises(ValueError,match='contract'):
        load_checkpoint(tmp_path/'invalid.pt','unused')
    changed = json.loads((data/'metadata.json').read_text())
    changed['model_spec']['std'][0] = 3
    (data/'metadata.json').write_text(json.dumps(changed))
    with pytest.raises(ValueError,match='mismatch'):
        evaluate(bundle,data,tmp_path/'invalid-eval')


def test_evs_cnn_joint_heads_checkpoint_and_export(tmp_path):
    from kart_e2e.export_onnx import export
    torch.set_num_threads(2)
    torch.manual_seed(42)
    routing = dict(schema='kart.section_heads.v1',frame='map',base_frame='base_link',closed=False,
                   points=[[0,0],[5,0],[10,0]],heads=['a','b'],
                   sections=[dict(id='s1',start_m=0.,end_m=5.,head='a'),
                             dict(id='s2',start_m=5.,end_m=10.,head='b')],
                   max_distance_m=1.,max_heading_rad=1.,ambiguity_distance_m=.1,max_jump_m=4.,
                   max_pose_age_s=.3,max_pose_skew_s=.05)
    model = EVSPilotNetPolicy(metadata()).configure_heads(routing)
    counts = torch.rand(2,20,120,160)
    model(counts).square().sum().backward()
    assert model.encoder.blocks[0].weight.grad.abs().sum() > 0
    assert all(head[0].weight.grad.abs().sum() > 0 for head in model.heads)
    path = tmp_path/'joint.pt'
    torch.save(dict(modality='evs',spec=model.spec,evs_metadata=model.evs_metadata,
                    mode=model.mode,runtime=model.runtime,routing=routing,state_dict=model.state_dict()),path)
    restored = load_checkpoint(path,'unused')
    torch.testing.assert_close(restored(counts),model(counts))
    report = export(restored,tmp_path/'joint-onnx')
    assert report['output_shape'] == [1,4] and report['routing'] == routing


def test_ui_evs_cnn_plan_needs_no_weights_or_dino_checkout(tmp_path):
    Learning = pytest.importorskip('kart_studio.e2e').Learning
    repo = Path(__file__).resolve().parents[4]
    records = tmp_path/'record'
    records.mkdir()
    service = Learning(repo,tmp_path,records,tmp_path/'.state')
    settings = service.settings()
    settings.update(python=sys.executable, device='cpu', encoder_repo=str(tmp_path/'missing'),
                    evs_weights=str(tmp_path/'missing.pt'))
    service.save_settings(settings)
    data = dataset(service.root/'datasets/events')
    (data/'artifact.json').write_text(json.dumps(dict(created=1,modality='evs')))
    body = dict(name='evs-cnn',modality='evs',architecture='simple_cnn',initialization='scratch',
                train=['events'],validation=['events'])
    plan = service.prepare('train',body)
    assert '--weights' not in plan['args'] and '--finetune' in plan['args']
    assert plan['provenance']['epochs'] == 100
    assert plan['provenance']['architecture'] == 'simple_cnn'
    assert plan['provenance']['modality'] == 'evs'
    assert not plan['provenance']['feature_cache_effective']
    with pytest.raises(ValueError, match='CNN'):
        service.prepare('train',dict(body,initialization='pretrained'))
