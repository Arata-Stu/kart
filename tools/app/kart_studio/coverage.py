"""Confined Studio adapter for the offline E2E coverage pipeline."""
import csv
from decimal import Decimal, InvalidOperation
import hashlib
import os
from pathlib import Path
import sys
import time
import uuid
import yaml
from .storage import atomic_json, read_json, within, name
from .e2e import Learning
from .jobs import Cancelled


class Coverage:
    def __init__(self, repo, base, records):
        self.repo, self.base, self.records = repo, base, records
        self.root = base/'analysis/e2e_coverage'
        self.root.mkdir(parents=True, exist_ok=True)
        self.learning = Learning(repo, base, records, base/'.kart-studio')

    def folder(self, key):
        return within(self.root, name(key))

    def catalog(self):
        rows=[]; poses=[]
        for path in self.root.glob('*/request.json'):
            try:
                folder=self.folder(path.parent.name)
                request=read_json(within(folder,'request.json'))
                status=read_json(within(folder,'ui_status.json'))
                rows.append(dict(id=folder.name,created=request['created'],**status))
                summary=read_json(within(folder,'result/summary.json'))
                for i,p in enumerate(summary['provenance']):
                    relative=Path(p['pose']).relative_to(self.root.resolve())
                    pose=within(self.root,relative)
                    poses.append(dict(id=f'{folder.name}:{i}',bag=p['bag'],label=f'{folder.name} / {p["id"]}',
                                      metadata_sha256=p['bag_metadata_sha256']))
            except (ValueError,OSError,KeyError): continue
        return dict(results=sorted(rows,key=lambda r:r['created'],reverse=True),poses=poses)

    def prepare(self, body):
        if set(body)!={'recordings'} or not isinstance(body['recordings'],list) or not 1<=len(body['recordings'])<=30:
            raise ValueError('1〜30件の記録を指定してください')
        entries=[];resources=[];seen=set();compute=False
        for i,row in enumerate(body['recordings']):
            if set(row)-{'role','bag','dataset','mode','pose','event_seconds','before_s'}:
                raise ValueError('未対応の記録設定です')
            role=row.get('role');mode=row.get('mode')
            if role not in ('training','reference','evaluation') or mode not in ('manual','auto'):
                raise ValueError('記録の用途と走行モードを指定してください')
            bag=within(self.records,row.get('bag',''))
            meta_path=within(bag,'metadata.yaml')
            meta=yaml.safe_load(meta_path.read_text())['rosbag2_bagfile_information']
            if bag in seen: raise ValueError('同じbagは重複登録できません')
            seen.add(bag)
            if not meta.get('relative_file_paths'): raise ValueError('記録ファイルがありません')
            for relative in meta['relative_file_paths']: within(bag,relative)
            entry=dict(id=f'recording_{i+1}',role=role,mode=mode,bag=str(bag))
            resources.append('read:record:'+row['bag'])
            if role=='training':
                dataset=self.learning.folder('datasets',row.get('dataset',''))
                within(dataset,'samples.jsonl');within(dataset,'metadata.json')
                artifact=read_json(within(dataset,'artifact.json'))
                source=artifact.get('bag') or artifact.get('provenance',{}).get('bag')
                if source and Path(source).resolve()!=bag.resolve():
                    raise ValueError('教師datasetの元bagと選択bagが一致しません')
                entry['dataset']=str(dataset)
                resources.append('read:e2e:datasets:'+row['dataset'])
            if row.get('pose'):
                parts=row['pose'].split(':')
                if len(parts)!=2 or not parts[1].isdigit(): raise ValueError('poseの選択が不正です')
                source=self.folder(parts[0])
                provenance=read_json(within(source,'result/summary.json'))['provenance']
                index=int(parts[1])
                if index>=len(provenance): raise ValueError('poseが見つかりません')
                p=provenance[index]
                if p['bag']!=str(bag) or p['bag_metadata_sha256']!=hashlib.sha256(meta_path.read_bytes()).hexdigest():
                    raise ValueError('poseの元bagが一致しないか、記録が変更されています')
                pose=within(self.root,Path(p['pose']).relative_to(self.root.resolve()))
                entry.update(poses=str(pose),pose_frame=p['quality']['offline_pose']['pose_frame'])
                resources.append('read:coverage:'+parts[0])
            else: compute=True
            if row.get('event_seconds','')!='':
                if role!='evaluation': raise ValueError('イベントは評価走行だけに指定できます')
                try:
                    seconds=Decimal(str(row['event_seconds']));before=Decimal(str(row.get('before_s',3)))
                    if not seconds.is_finite() or not before.is_finite() or seconds<0 or not 0<before<=120:
                        raise ValueError('イベント時刻・直前秒数が範囲外です')
                    duration=int(meta['duration']['nanoseconds'])
                    if seconds*10**9>duration: raise ValueError('イベント時刻がbagの記録時間外です')
                    stamp=int(meta['starting_time']['nanoseconds_since_epoch'])+int(seconds*10**9)
                except (InvalidOperation,KeyError,TypeError) as exc:
                    raise ValueError('bagの開始時刻とイベント秒数を確認してください') from exc
                entry['events']=[dict(name='event',stamp_ns=stamp,before_s=float(before))]
            entries.append(entry)
        if not any(e['role']=='training' for e in entries): raise ValueError('教師データを1件以上指定してください')
        if compute:
            workflow=read_json(self.repo/'ros2_ws/src/kart_bringup/config/mapping/workflow.json')
            resources.append('ros:'+str(workflow['ros_domain_id']))
        key='coverage_'+uuid.uuid4().hex[:12]
        return dict(key=key,manifest=dict(recordings=entries),resources=resources+['coverage:'+key],body=body)

    def run(self, job, plan):
        folder=within(self.root,plan['key'],exists=False);folder.mkdir()
        atomic_json(folder/'request.json',dict(created=time.time(),selection=plan['body']))
        atomic_json(folder/'manifest.json',plan['manifest'])
        atomic_json(folder/'ui_status.json',dict(status='running'))
        try:
            job.run([os.environ.get('KART_COVERAGE_PYTHON',sys.executable),str(self.repo/'tools/e2e_coverage/cli.py'),
                     '--manifest',str(folder/'manifest.json'),'--output',str(folder/'result')])
            summary=read_json(within(folder,'result/summary.json'))
            if summary.get('schema')!='kart.e2e_coverage.v1': raise ValueError('解析結果のschemaが不正です')
            job.check()
            atomic_json(folder/'ui_status.json',dict(status='complete'))
            return dict(coverage=plan['key'])
        except BaseException as exc:
            atomic_json(folder/'ui_status.json',dict(status='cancelled' if isinstance(exc,Cancelled) else 'failed',message=str(exc)))
            raise

    def result(self, key):
        folder=self.folder(key)
        summary=read_json(within(folder,'result/summary.json'))
        timeline=[];origins={}
        with within(folder,'result/evaluation_coverage.csv').open() as stream:
            for row in csv.DictReader(stream):
                stamp=int(row['stamp_ns']);origin=origins.setdefault(row['recording'],stamp)
                timeline.append(dict(recording=row['recording'],t=(stamp-origin)/1e9,
                    stamp_ns=str(stamp),steering=float(row['steering']) if row['steering'] else None,
                    speed=float(row['speed']) if row['speed'] else None,
                    status=row['training_curvature_speed'],reason=row['reason']))
        stride=max(1,(len(timeline)+2999)//3000)
        return dict(summary=summary,timeline=timeline[::stride],timeline_stride=stride)
