"""Offline E2E distribution audit; never publishes vehicle commands."""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import html
import json
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/vehicle_tuning'))
from analysis import analyze
from bag import read_bag
from offline_pose import attach_poses
from report import write_json, write_csv, table
from core import compare, settings


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve(base, value):
    path = Path(value).expanduser()
    return (base/path).resolve() if not path.is_absolute() else path.resolve()


def poses_for(entry, bag, base, work, workflow_path):
    if entry.get('poses'):
        if not entry.get('pose_frame'):
            raise ValueError('Existing poses require explicit pose_frame')
        return resolve(base,entry['poses']), entry['pose_frame']
    workflow = json.loads(workflow_path.read_text())
    if workflow.get('dense_reconstruction'):
        raise ValueError('Coverage workflow must disable dense reconstruction')
    job = dict(bag=str(bag), output=str(work), workflow=workflow)
    write_json(work/'job.json',job)
    env = dict(os.environ, ROS_DOMAIN_ID=str(workflow['ros_domain_id']))
    env['PYTHONPATH'] = str(ROOT/'ros2_ws/src/kart_mapping') + os.pathsep + env.get('PYTHONPATH','')
    command = [sys.executable, '-m', 'kart_mapping.build_map', '--job', str(work/'job.json')]
    print(f'Offline VSLAM: {entry["id"]}; log={work/"vslam.log"}', flush=True)
    with (work/'vslam.log').open('w') as log:
        subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    candidates = {p.resolve() for p in (work/'official').glob('*/poses/odom_poses.tum')}
    if len(candidates)!=1 or any(not p.is_relative_to(work.resolve()) for p in candidates):
        raise ValueError('Expected exactly one official odom_poses.tum inside work directory')
    return candidates.pop(), workflow['overrides']['base_frame']


def make_rows(data, cfg):
    summary, rows, _, _ = analyze(data,cfg,turning=True)
    result = []
    for row, odom in zip(rows,data['odom']):
        valid = not row['quality_reason'] and row['brake']==0
        speed = row['speed_mps']
        result.append(dict(stamp_ns=odom['header_ns'], steering=row['steering'], speed=speed,
            curvature=row['yaw_rate_rps']/speed if valid and speed>0 else None,
            mode=row['mode'], valid=valid, reason=row['quality_reason'] or ('brake' if row['brake'] else ''), artifact=None))
    return result, dict(quality=summary['quality'], offline_pose=summary['offline_pose'],
                       excluded=dict(Counter(r['reason'] for r in result if not r['valid'])))


def teacher_rows(dataset, motion, max_gap_s):
    dataset = dataset.resolve()
    samples = dataset/'samples.jsonl'
    metadata = dataset/'metadata.json'
    meta = json.loads(metadata.read_text())
    times = [r['stamp_ns'] for r in motion]
    result, last = [], None
    with samples.open() as stream:
        for line in stream:
            row = json.loads(line)
            stamp = row['stamp_ns']
            if type(stamp) is not int or stamp<=0 or (last is not None and stamp<=last):
                raise ValueError('Teacher stamps must be positive, strictly increasing integers')
            last=stamp
            steering=row['steering']
            if type(steering) not in (float,int) or not math.isfinite(steering) or abs(steering)>1:
                raise ValueError('Invalid teacher steering')
            i=bisect_left(times,stamp)
            choices=[j for j in (i-1,i) if 0<=j<len(times)]
            nearest=min(choices,key=lambda j:abs(times[j]-stamp)) if choices else None
            matched=nearest is not None and abs(times[nearest]-stamp)<=max_gap_s*1e9
            sample=dict(motion[nearest]) if matched else dict(speed=None,curvature=None,mode=None,valid=False,reason='pose_unmatched')
            artifact=row.get('image',row.get('counts'))
            if not artifact: raise ValueError('Teacher sample requires image or counts')
            artifact_path=(dataset/artifact).resolve()
            if not artifact_path.is_relative_to(dataset) or not artifact_path.is_file():
                raise ValueError('Missing or external teacher artifact')
            sample.update(stamp_ns=stamp,steering=steering,artifact=str(artifact_path))
            result.append(sample)
    if not result: raise ValueError('Empty teacher dataset')
    return result, dict(samples_sha256=sha(samples),metadata_sha256=sha(metadata),metadata=meta)


def run(manifest, output):
    spec=json.loads(manifest.read_text()); base=manifest.parent
    cfg=settings(spec.get('settings',{}))
    entries=spec['recordings']
    ids=[e['id'] for e in entries]
    if not entries or len(set(ids))!=len(ids) or any(not isinstance(i,str) or not i or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in i) for i in ids):
        raise ValueError('Unique simple recording IDs required')
    bags=[resolve(base,e['bag']) for e in entries]
    if len(set(bags))!=len(bags): raise ValueError('Same bag cannot count twice or leak between roles')
    if not any(e['role']=='training' for e in entries): raise ValueError('At least one training recording required')
    for e in entries:
        if e['role'] not in ('training','reference','evaluation') or e.get('mode') not in ('manual','auto'):
            raise ValueError('Explicit role and manual/auto mode required')
        if e['role']=='training' and not e.get('dataset'): raise ValueError('Training requires actual extracted dataset')
        if e.get('events') and e['role']!='evaluation': raise ValueError('Events belong to evaluation recordings')
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'input_manifest.json',spec)
    write_json(output/'status.json',dict(status='running'))
    try:
        records=[]; provenance=[]
        workflow=resolve(base,spec['workflow']) if spec.get('workflow') else ROOT/'ros2_ws/src/kart_bringup/config/mapping/workflow.json'
        for entry,bag in zip(entries,bags):
            work=output/entry['id']; work.mkdir()
            config=json.loads((ROOT/'tools/vehicle_tuning/config.json').read_text())
            config.update(clock='header',allowed_modes=[2 if entry['mode']=='manual' else 1],
                          max_yaw_rate_rps=2.,max_lateral_speed_mps=.5)
            data=read_bag(bag,config)
            pose,frame=poses_for(entry,bag,base,work,workflow)
            data=attach_poses(data,config,pose,frame)
            rows,quality=make_rows(data,config)
            evidence=dict(id=entry['id'],bag=str(bag),bag_metadata_sha256=sha(bag/'metadata.yaml'),
                          pose=str(pose),pose_sha256=sha(pose),config=config,quality=quality)
            if entry['role']=='training':
                dataset=resolve(base,entry['dataset'])
                rows,teacher=teacher_rows(dataset,rows,.05)
                # Location changes require the manifest's explicit bag binding; retain original provenance.
                evidence['teacher']=teacher
            for event in entry.get('events',[]):
                if not isinstance(event.get('name'),str) or not event['name'] or event['name']=='whole_recording':
                    raise ValueError('Event requires a nonempty unique name')
            names=[e['name'] for e in entry.get('events',[])]
            if len(names)!=len(set(names)): raise ValueError('Duplicate event names')
            record=dict(id=entry['id'],role=entry['role'],rows=rows,events=entry.get('events',[]))
            records.append(record); provenance.append(evidence)
            write_csv(work/'samples.csv',rows)
        result=compare(records,cfg)
        result.update(schema='kart.e2e_coverage.v1',provenance=provenance,manifest_sha256=sha(manifest))
        write_csv(output/'evaluation_coverage.csv',result.pop('evaluation_samples'))
        write_json(output/'summary.json',result)
        body='<h1>E2E データ分布と指定区間の比較</h1><p>unobserved: 未観測 / sparse: 少ない / observed_support: 設定閾値を満たす / unknown: 判定不能</p>'
        body+='<p>衝突原因や「学習データは十分だった」という判定ではありません。操舵指令と軌跡条件は別々に確認してください。</p>'
        body+='<h2>指定イベント（直前区間のみ）</h2>'+table(result['events'])
        body+='<h2>評価走行と基準データの比較（件数はサンプル数）</h2>'+table(result['comparisons'])
        body+='<h2>追加収集・確認の候補</h2>'+table(result['recommendations'])
        body+='<p><a href="evaluation_coverage.csv">評価の各時刻・条件・支持状態 CSV</a></p>'
        for name,dist in result['distributions'].items():
            body+='<details><summary>'+html.escape(name)+'</summary>'+table(dist)+'</details>'
        body+='<h2>分布bin・支持閾値</h2><pre>'+html.escape(json.dumps(cfg,indent=2))+'</pre>'
        body+='<p>binの値はfloor(value/bin幅)の整数インデックス。符号はROSの記録値を保持します。</p>'
        body+='<h2>解釈の範囲</h2><ul>'+''.join('<li>'+html.escape(s)+'</li>' for s in result['limitations'])+'</ul>'
        body+='<p><a href="summary.json">JSON・出典・品質情報</a></p>'
        (output/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>E2E coverage</title><style>body{font-family:system-ui;margin:2rem}table{border-collapse:collapse;display:block;overflow:auto}td,th{border:1px solid #bbb;padding:.4rem}details{margin:1rem 0}</style>'+body)
        write_json(output/'status.json',dict(status='complete'))
        return result
    except Exception as exc:
        write_json(output/'status.json',dict(status='failed',error=str(exc)))
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(args.manifest.resolve(),args.output.resolve())
    print(args.output.resolve()/'report.html')


if __name__=='__main__': main()
