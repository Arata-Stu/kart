import json
import os
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch
import pytest
from kart_studio.e2e import Learning
from kart_studio.storage import atomic_json

REPO=Path(__file__).resolve().parents[3]


class RealJob:
    def check(self):pass
    def run(self,args,env=None):
        result=subprocess.run(args,env=env,capture_output=True,text=True)
        assert result.returncode==0,result.stdout+result.stderr


@pytest.fixture
def setup(tmp_path):
    records=tmp_path/'record';(records/'bag').mkdir(parents=True)
    (records/'bag/metadata.yaml').write_text('bag')
    learning=Learning(REPO,tmp_path,records,tmp_path/'.state')
    settings=learning.settings();settings.update(python=sys.executable,device='cpu',encoder_repo=str(tmp_path))
    learning.save_settings(settings)
    folder=tmp_path/'map/course';(folder/'cuvslam_map').mkdir(parents=True)
    (folder/'cuvslam_map/data.mdb').write_bytes(b'fixture')
    doc=dict(schema='kart.hdmap.v2',frame='map',lanes=[dict(id='lane',closed=False,left=[[0,1],[10,1]],
             right=[[0,-1],[10,-1]],custom_speeds=[dict(start=0,end=5,speed=1)],
             lines={'centerline':dict(points=[[0,0],[10,0]])})])
    atomic_json(folder/'map.json',doc)
    return learning,records,folder


def make_routing(learning):
    plan=learning.prepare('section-routing',dict(name='heads',map='course',lane='lane',line='centerline',
        sections=[dict(id='straight',head='straight',start_m=0,end_m=10),dict(id='*',head='stable')]))
    learning.execute(RealJob(),plan)
    return plan['key']


def make_dataset(learning,records):
    source=learning.root/'datasets/source';source.mkdir()
    atomic_json(source/'artifact.json',dict(created=1,samples=2,bag=str(records/'bag'),modality='rgb'))
    atomic_json(source/'metadata.json',dict(samples=2,modality='rgb',bag=str(records/'bag'),clock='header'))
    (source/'one.npy').write_bytes(b'image')
    (source/'samples.jsonl').write_text(''.join(json.dumps(dict(image='one.npy',stamp_ns=1_000_000_000+i,
                                               steering=.2,throttle=.1))+'\n' for i in range(2)))
    return 'source'


def test_ui_routing_wildcard_publication_and_assignment_without_vslam(setup):
    learning,records,_=setup
    routing=make_routing(learning)
    source=make_dataset(learning,records)
    plan=learning.prepare('section-dataset',dict(name='stable',routing=routing,dataset=source,operation='assign',sections=['*']))
    learning.execute(RealJob(),plan)
    catalog=learning.catalog()
    assert len(catalog['routings'])==1
    stable=next(d for d in catalog['datasets'] if d.get('section_assignment'))
    assert stable['section_counts']=={'*':2}
    assert stable['samples']==2
    with pytest.raises(ValueError,match='ROS/CUDA'):
        with patch('kart_studio.section_learning.shutil.which',return_value=None):
            learning.prepare('section-offline',dict(name='poses',routing=routing,map='course',bag='bag'))


def test_ui_training_infers_routing_and_checks_coverage(setup):
    learning,records,_=setup;routing=make_routing(learning);source=make_dataset(learning,records)
    route=learning.folder('routings',routing)
    cfg=json.loads((route/'routing.json').read_text())
    from kart_e2e.sections import digest
    for key,section in [('expert','straight'),('stable','*')]:
        folder=learning.root/'datasets'/key;folder.mkdir()
        atomic_json(folder/'artifact.json',dict(created=1,samples=2))
        atomic_json(folder/'metadata.json',dict(modality='rgb',samples=2,routing=cfg,routing_sha256=digest(cfg),section_counts={section:2}))
    body=dict(name='joint',train=['expert','stable'],validation=['expert','stable'],architecture='simple_cnn',initialization='scratch')
    plan=learning.prepare('train',body)
    assert '@ROUTING@' in plan['args'] and plan['routing']==cfg
    assert plan['provenance']['section_heads']==['straight','stable']
    with pytest.raises(ValueError,match='全head'):learning.prepare('train',dict(body,train=['expert']))
    with pytest.raises(ValueError,match='同じ割当'):learning.prepare('train',dict(body,train=['expert','source']))


def test_map_changes_paths_and_pose_bag_mismatch_rejected(setup):
    learning,records,folder=setup;routing=make_routing(learning);source=make_dataset(learning,records)
    with pytest.raises(ValueError):learning.prepare('section-dataset',dict(name='bad',routing=routing,dataset='../source',operation='assign',sections=['*']))
    with pytest.raises(ValueError):learning.prepare('section-routing',dict(name='bad',map='../course',lane='lane',sections=[]))
    info=learning.catalog()['routings'][0]
    trajectory=learning.root/'trajectories/poses';trajectory.mkdir()
    atomic_json(trajectory/'artifact.json',dict(created=1))
    atomic_json(trajectory/'metadata.json',dict(routing_sha256=info['routing_sha256'],bag='/different/bag'))
    (trajectory/'poses.jsonl').write_text('{}\n')
    with pytest.raises(ValueError,match='同じbag'):
        learning.prepare('section-dataset',dict(name='bad',routing=routing,dataset=source,trajectory='poses',operation='split',sections=['straight']))
    (folder/'map.json').write_text('{}')
    with patch('kart_studio.section_learning.shutil.which',return_value='/usr/bin/ros2'):
        with pytest.raises(ValueError,match='HDMap'):learning.prepare('section-offline',dict(name='bad',routing=routing,map='course',bag='bag'))


def test_ui_plans_execute_joint_cpu_training_and_export(setup):
    import numpy as np
    learning,records,_=setup;routing_id=make_routing(learning);source=make_dataset(learning,records)
    np.save(learning.folder('datasets',source)/'one.npy',np.full((12,16,3),100,dtype=np.uint8))
    route=learning.catalog()['routings'][0]
    trajectory=learning.root/'trajectories/poses';trajectory.mkdir()
    atomic_json(trajectory/'artifact.json',dict(created=1))
    atomic_json(trajectory/'metadata.json',dict(routing_sha256=route['routing_sha256'],bag=str(records/'bag'),valid_poses=2))
    rows=[dict(stamp_ns=1_000_000_000+i,valid=True,x=1.+i,y=0.,yaw=0.,frame='map',base_frame='base_link',routing_sha256=route['routing_sha256']) for i in range(2)]
    (trajectory/'poses.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    datasets=[]
    for operation,sections,key in [('split',['straight'],'expert'),('assign',['*'],'stable')]:
        plan=learning.prepare('section-dataset',dict(name=key,routing=routing_id,dataset=source,operation=operation,sections=sections,trajectory='poses'))
        learning.execute(RealJob(),plan);datasets.append(plan['key'])
    plan=learning.prepare('train',dict(name='joint-cpu',train=datasets,validation=datasets,
        architecture='simple_cnn',initialization='scratch',epochs=1,batch_size=2,num_workers=0,mode='steer_only'))
    learning.execute(RealJob(),plan)
    artifact=read(learning.folder('runs','joint-cpu')/'artifact.json')
    assert set(artifact['metrics']['validation_heads'])=={'straight','stable'}
    cfg=learning.settings();cfg['inference_python']=sys.executable;learning.save_settings(cfg)
    export=learning.prepare('export',dict(name='joint-onnx',run='joint-cpu'))
    learning.execute(RealJob(),export)
    assert learning.catalog()['models'][0]['routing']['heads']==['straight','stable']


def read(path):return json.loads(Path(path).read_text())


def test_offline_worker_requires_successful_playback_and_valid_poses(setup,tmp_path):
    from kart_e2e.section_workflow import run
    learning,records,folder=setup;routing=make_routing(learning)
    request=dict(action='offline',bag=str(records/'bag'),map_file=str(folder/'map.json'),map_dir=str(folder),
                 routing_file=str(learning.folder('routings',routing)/'routing.json'),tracking_mode='0')
    def player(args,check):
        paths=dict(arg.split(':=',1) for arg in args if ':=' in arg)
        Path(paths['completion_file']).write_text('{"returncode":0}')
        Path(paths['trajectory_output']).write_text('{"valid":true}\n{"valid":false}\n')
    with patch('kart_e2e.section_workflow.shutil.which',return_value='/usr/bin/ros2'),patch('kart_e2e.section_workflow.subprocess.run',side_effect=player):
        report=run(request,tmp_path/'good')
    assert report['valid_poses']==1 and report['total_poses']==2
    with patch('kart_e2e.section_workflow.shutil.which',return_value='/usr/bin/ros2'),patch('kart_e2e.section_workflow.subprocess.run'):
        with pytest.raises(ValueError,match='did not finish'):run(request,tmp_path/'incomplete')


def test_explicit_pipeline_rejects_wrong_training_and_export(setup):
    learning,records,_=setup
    source=make_dataset(learning,records)
    normal=dict(name='single-run',train=[source],validation=[source],architecture='simple_cnn',initialization='scratch')
    assert learning.prepare('train',dict(normal,pipeline='single'))['pipeline']=='single'
    with pytest.raises(ValueError,match='pipeline'):learning.prepare('train',dict(normal,pipeline='multi'))
    cfg=learning.settings();cfg['inference_python']=sys.executable;learning.save_settings(cfg)
    for key,heads in [('ordinary',[]),('joint',['curve','stable'])]:
        folder=learning.root/'runs'/key;folder.mkdir()
        atomic_json(folder/'artifact.json',dict(created=1,section_heads=heads,architecture='simple_cnn'))
        (folder/'best.pt').write_bytes(b'fixture')
    assert learning.prepare('export',dict(name='normal-export',run='ordinary',pipeline='single'))['pipeline']=='single'
    assert learning.prepare('export',dict(name='multi-export',run='joint',pipeline='multi'))['pipeline']=='multi'
    with pytest.raises(ValueError,match='pipeline'):
        learning.prepare('export',dict(name='wrong-export',run='joint',pipeline='single'))
    rows=learning.catalog()['runs']
    assert {r['name']:r['pipeline'] for r in rows}=={'ordinary':'single','joint':'multi'}
