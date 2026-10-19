"""Feature job planning/publication and dated cleanup use the real service."""
import json
import sys
from pathlib import Path
import pytest
from kart_studio.e2e import Learning
from kart_studio import e2e_trash
from kart_studio.storage import atomic_json

REPO=Path(__file__).resolve().parents[3]

def test_features_plan_publication_and_trash(tmp_path):
    service=Learning(REPO,tmp_path,tmp_path/'record',tmp_path/'.state')
    settings=service.settings()
    service.save_settings(dict(settings,python=sys.executable,device='cpu',encoder_repo=str(tmp_path)))
    for kind,key in [('runs','run'),('datasets','data')]:
        folder=service.root/kind/key;folder.mkdir()
        atomic_json(folder/'artifact.json',dict(created=1,kind=kind))
    (service.root/'runs/run/best.pt').write_bytes(b'checkpoint')
    (service.root/'datasets/data/samples.jsonl').write_text('{}\n')
    plan=service.prepare('features',dict(name='inspect',run='run',dataset='data'))
    assert len(plan['key'].split('/'))==3
    assert 'e2e:runs:run' in plan['resources'] and 'e2e:datasets:data' in plan['resources']
    assert plan['python']==sys.executable and plan['args'][-1]=='12'
    class Job:
        def run(self,args,env):
            output=Path(args[args.index('--output')+1]);output.mkdir()
            frame={k:f'frames/0000/{k}.png' for k in ('rgb','pca','saliency')}
            for filename in frame.values():
                p=output/filename;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b'png')
            atomic_json(output/'report.json',dict(schema='kart.rgb_features.v1',frame_count=1,frames=[frame],mode='steer_only'))
        def check(self): pass
    result=service.execute(Job(),plan)
    assert service.catalog()['features'][0]['frame_count']==1
    moved=e2e_trash.move(service,'features',result['id'])
    assert not plan['target'].exists()
    e2e_trash.restore(service,moved['token'])
    assert plan['target'].is_dir()
    with pytest.raises(ValueError):
        service.prepare('features',dict(name='bad',run='run',dataset='data',max_frames=65))
