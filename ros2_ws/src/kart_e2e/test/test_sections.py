import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from PIL import Image
import pytest
import torch
from kart_e2e.sections import Router, digest
from kart_e2e.model import Policy, SimpleCNN
from kart_e2e.section_dataset import split


def config():
    return dict(schema='kart.section_heads.v1',frame='map',base_frame='base_link',closed=False,
                points=[[0,0],[5,0],[10,0],[15,0]],heads=['straight','curve'],
                sections=[dict(id='a',start_m=0.,end_m=5.,head='straight'),
                          dict(id='b',start_m=5.,end_m=10.,head='curve'),
                          dict(id='c',start_m=10.,end_m=15.,head='straight')],
                max_distance_m=1.,max_heading_rad=1.,ambiguity_distance_m=.1,max_jump_m=4.,
                max_pose_age_s=.3,max_pose_skew_s=.05)


def test_mapping_boundaries_reverse_direction_and_discontinuities():
    r=Router(config())
    assert r.select(4,0,0)['head_index']==0
    assert r.select(5,0,0)['section_id']=='b'
    assert r.select(6,0,np.pi) is None
    assert r.select(9,3,0) is None
    assert r.select(14,0,0) is None
    r.reset();assert r.select(14,0,0)['head_index']==0
    bad=config();bad['sections'][1]['start_m']=4
    with pytest.raises(ValueError,match='Overlapping'):Router(bad)
    bad=config();bad['sections'][2]['head']='missing'
    with pytest.raises(ValueError):Router(bad)


def test_joint_gradients_and_export_contract(tmp_path):
    from kart_e2e.export_onnx import export
    from kart_e2e.contract import model_contract
    torch.manual_seed(7)
    policy=Policy(SimpleCNN(),'steer_only',frozen=False,architecture='simple_cnn').configure_heads(config())
    image=torch.randn(4,3,128,224)
    output=policy(image).reshape(4,2,1)
    selected=output[torch.arange(4),torch.tensor([0,1,0,1])]
    selected.square().mean().backward()
    assert sum(float(p.grad.abs().sum()) for p in policy.encoder.parameters())>0
    assert all(sum(float(p.grad.abs().sum()) for p in h.parameters())>0 for h in policy.heads)
    report=export(policy,tmp_path/'export')
    assert report['output_shape']==[1,2]
    assert report['routing']['heads']==['straight','curve']
    assert model_contract(tmp_path/'export')[1]=='steer_only'


def make_data(tmp_path):
    source=tmp_path/'source';source.mkdir()
    rows=[];poses=[]
    cfg=config()
    for i,x in enumerate([1.,2.,4.,6.,7.,8.,10.,11.]):
        path=f'{i}.npy';np.save(source/path,np.full((12,16,3),50+i*10,dtype=np.uint8))
        stamp=1_000_000_000+i*100_000_000
        rows.append(dict(image=path,stamp_ns=stamp,steering=(x-7)/20,throttle=.1))
        poses.append(dict(stamp_ns=stamp,valid=True,x=x,y=0.,yaw=0.,frame='map',base_frame='base_link',routing_sha256=digest(cfg)))
    (source/'metadata.json').write_text(json.dumps(dict(modality='rgb',clock='header')))
    (source/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    trajectory=tmp_path/'poses.jsonl';trajectory.write_text(''.join(json.dumps(r)+'\n' for r in poses))
    return source,trajectory,cfg


def test_dataset_split_and_full_joint_training(tmp_path):
    source,trajectory,cfg=make_data(tmp_path)
    output=tmp_path/'labeled'
    report=split(source,trajectory,cfg,output)
    assert report['samples']==8
    assert set(report['sections'])=={'a','b','c'}
    assert len((output/'sections/b/samples.jsonl').read_text().splitlines())==3
    routing=tmp_path/'routing.json';routing.write_text(json.dumps(cfg))
    train=tmp_path/'training'
    result=subprocess.run([sys.executable,'-m','kart_e2e.train','--train',str(output),'--validation',str(output),
        '--repo','unused','--architecture','simple_cnn','--initialization','scratch','--section-routing',str(routing),
        '--mode','steer_only','--epochs','1','--batch-size','4','--num-workers','0','--device','cpu','--output',str(train)],
        capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    state=torch.load(train/'best.pt',weights_only=True)
    assert state['routing']==cfg
    assert set(state['metrics']['validation_heads'])==set(cfg['heads'])
    assert state['provenance']['head_sampling']=='equal_head_replacement'


def test_trajectory_clock_and_registration_rejected(tmp_path):
    source,trajectory,cfg=make_data(tmp_path)
    poses=trajectory.read_text().splitlines();poses[0]=json.dumps(dict(json.loads(poses[0]),routing_sha256='other'))
    trajectory.write_text('\n'.join(poses))
    with pytest.raises(ValueError,match='routing mismatch'):split(source,trajectory,cfg,tmp_path/'output')


def test_existing_map_sections_keep_explicit_head_mapping():
    from kart_e2e.section_dataset import bind_map_sections
    source={'lanes':[dict(id='lane',custom_speeds=[dict(start=2.,end=4.,speed=1.)])]}
    rows=bind_map_sections([dict(id='curve_1',map_section=0,head='curve')],source,'lane','customline')
    assert rows[0]['start_m']==2. and rows[0]['end_m']==4.
    assert rows[0]['head']=='curve'
    with pytest.raises(ValueError):bind_map_sections([dict(id='x',map_section=0,head='curve')],source,'lane','centerline')


def test_frozen_cache_and_ram_preserve_head_labels(tmp_path):
    from torch.utils.data import DataLoader
    from kart_e2e.train import Samples
    from kart_e2e.feature_cache import cached_samples
    from kart_e2e.ram_dataset import preload_datasets
    source,trajectory,cfg=make_data(tmp_path)
    output=tmp_path/'split';split(source,trajectory,cfg,output)
    data=Samples([output],'steer_only',routing=cfg)
    model=Policy(SimpleCNN(),'steer_only',frozen=True,architecture='simple_cnn').configure_heads(cfg)
    factory=lambda dataset,shuffle,persistent=True:DataLoader(dataset,batch_size=4,shuffle=shuffle)
    cached,_=cached_samples(data,model,tmp_path/'cache','test-weights',factory,'cpu')
    ram=preload_datasets([cached])[0]
    for i in range(len(data)):
        assert torch.equal(data[i][1],ram[i][1])
    assert set(int(ram[i][1][-1]) for i in range(len(ram)))=={0,1}


def test_filter_at_split_and_extract_preserve_teacher_controls(tmp_path):
    from kart_e2e.section_dataset import extract
    source,trajectory,cfg=make_data(tmp_path)
    curve=tmp_path/'curve'
    report=split(source,trajectory,cfg,curve,sections=['b'])
    assert report['samples']==3
    assert report['sections']=={'b':3}
    assert report['skipped']['section_filter']==5
    meta=json.loads((curve/'metadata.json').read_text())
    assert meta['selected_sections']==['b']
    assert meta['routing']['heads']==['straight','curve']
    all_data=tmp_path/'all';split(source,trajectory,cfg,all_data)
    straight=tmp_path/'straight'
    result=extract(all_data,straight,['a','c'])
    assert result['samples']==5
    original=[json.loads(r) for r in (all_data/'samples.jsonl').read_text().splitlines()]
    actual=[json.loads(r) for r in (straight/'samples.jsonl').read_text().splitlines()]
    by_stamp={r['stamp_ns']:r for r in original}
    for row in actual:
        expected=by_stamp[row['stamp_ns']]
        assert row['section_id'] in ('a','c')
        assert row['head_id']=='straight'
        assert (row['steering'],row['throttle'])==(expected['steering'],expected['throttle'])
        assert (straight/row['image']).read_bytes()==(all_data/expected['image']).read_bytes()
    with pytest.raises(ValueError,match='unknown'):extract(all_data,tmp_path/'bad',['typo'])
    assert not (tmp_path/'bad').exists()
    with pytest.raises(ValueError,match='unknown'):split(source,trajectory,cfg,tmp_path/'bad2',sections=['typo'])


def test_merge_filtered_runs_keeps_head_indices(tmp_path):
    from kart_e2e.section_dataset import extract
    from kart_e2e.train import Samples
    source,trajectory,cfg=make_data(tmp_path)
    all_data=tmp_path/'all';split(source,trajectory,cfg,all_data)
    curve,straight=tmp_path/'curve',tmp_path/'straight'
    extract(all_data,curve,['b']);extract(all_data,straight,['a','c'])
    merged=Samples([curve,straight],'steer_throttle',routing=cfg)
    assert len(merged)==8
    assert set(int(merged[i][1][-1]) for i in range(len(merged)))=={0,1}


def wildcard_config():
    cfg=config();cfg['heads'].append('stable');cfg['sections'].append(dict(id='*',head='stable'))
    return cfg


def test_wildcard_assignment_without_poses_and_joint_training(tmp_path):
    from kart_e2e.section_dataset import extract
    from kart_e2e.train import Samples
    source,trajectory,_=make_data(tmp_path)
    cfg=wildcard_config()
    # Specialists still require the registered trajectory for this exact routing.
    poses=[dict(json.loads(r),routing_sha256=digest(cfg)) for r in trajectory.read_text().splitlines()]
    trajectory.write_text(''.join(json.dumps(r)+'\n' for r in poses))
    specialized=tmp_path/'specialized'
    split(source,trajectory,cfg,specialized,sections=['*'])
    assert '*' not in json.loads((specialized/'metadata.json').read_text())['selected_sections']
    # Stable dataset needs neither pose file nor header-clock association.
    (source/'metadata.json').write_text(json.dumps(dict(modality='rgb',clock='bag',teacher_selection={'policy':'fixed_auto_only'})))
    stable=tmp_path/'stable'
    result=extract(source,stable,['*'],assign_routing=cfg)
    assert result['samples']==8 and result['sections']=={'*':8}
    rows=[json.loads(r) for r in (stable/'samples.jsonl').read_text().splitlines()]
    assert all(r['head_id']=='stable' and r['head_index']==2 and 'pose_stamp_ns' not in r for r in rows)
    meta=json.loads((stable/'metadata.json').read_text())
    assert meta['teacher_selection']=={'policy':'fixed_auto_only'}
    assert 'trajectory' not in meta
    data=Samples([specialized,stable],'steer_throttle',routing=cfg)
    assert len(data)==16 and set(int(data[i][1][-1]) for i in range(16))=={0,1,2}
    output=tmp_path/'training';routing=tmp_path/'routing.json';routing.write_text(json.dumps(cfg))
    args=[sys.executable,'-m','kart_e2e.train','--train',str(specialized),str(stable),
          '--validation',str(specialized),str(stable),'--repo','unused','--architecture','simple_cnn',
          '--initialization','scratch','--section-routing',str(routing),'--mode','steer_only',
          '--epochs','1','--batch-size','4','--num-workers','0','--device','cpu','--output',str(output)]
    run=subprocess.run(args,capture_output=True,text=True)
    assert run.returncode==0,run.stdout+run.stderr
    state=torch.load(output/'best.pt',weights_only=True)
    assert set(state['metrics']['validation_heads'])=={'straight','curve','stable'}


def test_wildcard_does_not_bypass_runtime_position_validation(tmp_path):
    from kart_e2e.section_dataset import extract,section_filter
    cfg=wildcard_config();router=Router(cfg)
    assert router.select(1,0,0)['head_id']=='straight'
    assert router.select(2,10,0) is None
    assert section_filter(cfg,['*'])=={'a','b','c','*'}
    bad=wildcard_config();bad['sections'].append(dict(id='*',head='stable'))
    with pytest.raises(ValueError,match='wildcard'):Router(bad)
    bad=wildcard_config();bad['sections'][-1]['start_m']=0
    with pytest.raises(ValueError,match='Wildcard'):Router(bad)
    source,_,_=make_data(tmp_path)
    with pytest.raises(ValueError,match='wildcard'):extract(source,tmp_path/'bad',['*'],assign_routing=config())
    with pytest.raises(ValueError):section_filter(cfg,['*','a'])


def test_extract_wildcard_preserves_existing_assignments(tmp_path):
    from kart_e2e.section_dataset import extract
    source,trajectory,cfg=make_data(tmp_path)
    labeled=tmp_path/'labeled';split(source,trajectory,cfg,labeled)
    dest=tmp_path/'all';extract(labeled,dest,['*'])
    assert len((dest/'samples.jsonl').read_text().splitlines())==8
    assert {json.loads(r)['head_id'] for r in (dest/'samples.jsonl').read_text().splitlines()}=={'straight','curve'}
