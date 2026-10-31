import csv
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from kart_studio.coverage import Coverage
from kart_studio.storage import atomic_json

REPO=Path(__file__).resolve().parents[3]

@pytest.fixture
def service(tmp_path):
    service=Coverage(REPO,tmp_path,tmp_path/'record')
    for key in ('teacher','eval'):
        bag=service.records/key;bag.mkdir(parents=True)
        (bag/'data.mcap').write_bytes(b'test')
        (bag/'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths: [data.mcap]\n  starting_time: {nanoseconds_since_epoch: 1750000000000000001}\n  duration: {nanoseconds: 10000000000}\n')
    dataset=service.learning.root/'datasets'/'teacher';dataset.mkdir()
    atomic_json(dataset/'artifact.json',dict(bag=str(service.records/'teacher')))
    atomic_json(dataset/'metadata.json',{})
    (dataset/'samples.jsonl').write_text('')
    return service


def request():
    return dict(recordings=[dict(role='training',bag='teacher',dataset='teacher',mode='manual'),
                            dict(role='evaluation',bag='eval',mode='auto',event_seconds='2.25',before_s='1')])


def test_prepare_precision_and_resources(service):
    plan=service.prepare(request())
    assert plan['manifest']['recordings'][1]['events'][0]['stamp_ns']==1750000002250000001
    assert 'ros:92' in plan['resources']
    assert 'read:e2e:datasets:teacher' in plan['resources']

@pytest.mark.parametrize('change',[dict(bag='../outside'),dict(bag='/tmp'),dict(event_seconds='NaN'),dict(event_seconds='20'),dict(before_s='0'),dict(pose='../secret:0'),dict(command='shell')])
def test_reject_invalid_and_outside_paths(service,change):
    body=request();body['recordings'][1].update(change)
    with pytest.raises((ValueError,OSError)):service.prepare(body)


def test_dataset_binding_and_symlink(service,tmp_path):
    body=request();body['recordings'][0]['bag']='eval'
    with pytest.raises(ValueError,match='元bag'):service.prepare(body)
    path=service.records/'teacher/data.mcap';path.unlink();path.symlink_to(tmp_path/'secret')
    with pytest.raises(ValueError):service.prepare(request())


def test_worker_retains_failure(service):
    plan=service.prepare(request())
    def fail(args):raise RuntimeError('GPU unavailable')
    with pytest.raises(RuntimeError):service.run(SimpleNamespace(run=fail),plan)
    folder=service.folder(plan['key'])
    assert (folder/'manifest.json').exists()
    assert json.loads((folder/'ui_status.json').read_text())['status']=='failed'
    assert service.catalog()['results'][0]['status']=='failed'


def test_result_uses_exact_stamp_strings(service):
    folder=service.root/'coverage_test';(folder/'result').mkdir(parents=True)
    atomic_json(folder/'result/summary.json',dict(schema='kart.e2e_coverage.v1'))
    (folder/'result/evaluation_coverage.csv').write_text('recording,stamp_ns,steering,speed,training_curvature_speed,reason\neval,1750000000000000001,0.2,1.2,sparse,\n')
    result=service.result('coverage_test')
    assert result['timeline'][0]['stamp_ns']=='1750000000000000001'
    assert result['timeline'][0]['t']==0
    with pytest.raises(ValueError):service.result('../secret')


def test_success_dispatch_and_reuse(service):
    from kart_studio.server import Studio
    plan=service.prepare(request())
    def run(args):
        result=Path(args[-1]);result.mkdir()
        pose=result/'recording_1/official/run/poses/odom_poses.tum';pose.parent.mkdir(parents=True);pose.write_text('fixture')
        import hashlib
        atomic_json(result/'summary.json',dict(schema='kart.e2e_coverage.v1',provenance=[dict(
            bag=str(service.records/'teacher'),id='recording_1',pose=str(pose),
            bag_metadata_sha256=hashlib.sha256((service.records/'teacher/metadata.yaml').read_bytes()).hexdigest(),
            quality=dict(offline_pose=dict(pose_frame='camera_link')))]))
    result=service.run(SimpleNamespace(run=run,check=lambda:None),plan)
    assert result['coverage']==plan['key']
    catalog=service.catalog();assert catalog['results'][0]['status']=='complete'
    body=request();body['recordings'][0]['pose']=catalog['poses'][0]['id']
    assert service.prepare(body)['manifest']['recordings'][0]['pose_frame']=='camera_link'
    body['recordings'][1]['pose']=catalog['poses'][0]['id']
    with pytest.raises(ValueError,match='元bag'):service.prepare(body)
    calls=[]
    studio=SimpleNamespace(coverage=service,jobs=SimpleNamespace(start=lambda *args:calls.append(args) or dict(id='job'),list=lambda:[]))
    assert Studio.post(studio,'/api/coverage/run',request())==dict(id='job')
    assert calls[0][2].action=='coverage'
    assert Studio.get(studio,'/api/coverage',{})['results'][0]['status']=='complete'
