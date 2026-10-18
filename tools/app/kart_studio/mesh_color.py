"""Publish RGB mesh as a separate, cancellable artifact; preserve official output."""
import hashlib
import shutil
import sys
import time
import uuid
from .storage import read_json, atomic_json, within
from .dense import artifact


def validate(studio, key, body):
    import importlib.util
    missing=[m for m in ('trimesh','rtree','plyfile','cv2','scipy','rosbags') if importlib.util.find_spec(m) is None]
    if missing:
        raise ValueError('RGB着色の依存がありません: '+', '.join(missing)+
                         '。NotebookのDockerイメージを更新してください（tools/mesh_color/README.md参照）')
    doc=studio.maps.load(key)
    if body.get('revision')!=doc['revision']: raise ValueError('地図を開き直してください')
    folder=studio.maps.folder(key)
    mapping=read_json(folder/'mapping_result.json')
    if mapping.get('backend')!='nvidia.create_map_offline':
        raise ValueError('公式offline地図が必要です')
    artifact(studio.maps,key,'mesh_before_adjustment.ply')
    origin=read_json(within(folder,'job.json'))
    saved=body.get('bag') or origin['bag']
    from pathlib import Path
    if Path(saved).is_absolute():
        saved=str(Path(saved).resolve().relative_to(studio.records.resolve()))
    bag=within(studio.records,saved)
    if hashlib.sha256(within(bag,'metadata.yaml').read_bytes()).hexdigest()!=mapping.get('bag_metadata_sha256'):
        raise ValueError('地図生成時と同じbagを選んでください')
    run=within(folder,mapping.get('dense_output',mapping['official_output']))
    meta=within(run,'map_frames/rectified/frames_meta.json')
    base=origin['workflow']['overrides']['base_frame']
    return bag,meta,base


def build(studio,job,key,body,inputs):
    folder=studio.maps.folder(key)
    mapping=read_json(folder/'mapping_result.json')
    mesh=artifact(studio.maps,key,'mesh_before_adjustment.ply')
    parent=within(folder,'color_runs',exists=False);parent.mkdir(exist_ok=True)
    target=parent/uuid.uuid4().hex;target.mkdir()
    published=False
    try:
        bag,meta,base=inputs
        output=target/'mesh_rgb.ply'
        job.log('終了コード-9の場合はSIGKILLです。最後の処理段階・peak RSSとホストのOOMログを確認してください')
        job.run([sys.executable,str(studio.repo/'tools/mesh_color/run.py'),
                 '--bag',str(bag),'--mesh',str(mesh),'--metadata',str(meta),
                 '--base-frame',base,'--output',str(output)])
        job.check()
        doc=studio.maps.load(key)
        if doc['revision']!=body['revision']: raise ValueError('実行中に地図が更新されました')
        report=read_json(output.with_suffix('.ply.json'))
        mapping['mesh_color']=dict(output=str(output.relative_to(folder)),
            dense_source=mapping.get('dense_output',mapping['official_output']),report=report)
        doc.update(mapping=mapping,revision=doc['revision']+1,updated=time.time())
        atomic_json(folder/'mapping_result.json',mapping)
        published=True
        atomic_json(folder/'map.json',doc)
        return dict(map=key)
    finally:
        if not published: shutil.rmtree(target,ignore_errors=True)
