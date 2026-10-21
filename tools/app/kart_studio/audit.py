"""Offline timing audit jobs and confined published reports; no ROS or remote execution."""
from datetime import datetime
import os
from pathlib import Path
import re
import shutil
import sys
import time
import uuid

from .storage import atomic_json, read_json, within

FILES = {'report.html', 'summary.json', 'timeline.csv', 'matches.csv', 'events.csv', 'metadata.jsonl'}
KEY = re.compile(r'[0-9]{4}-[0-9]{2}-[0-9]{2}/[0-9]{2}-[0-9]{2}-[0-9]{2}/audit_[a-f0-9]{8}')


class Audit:
    def __init__(self, repo, base, records):
        self.repo, self.records = repo, records
        self.root = base / 'analysis/pipeline'
        self.root.mkdir(parents=True, exist_ok=True)
        self.python = os.environ.get('KART_AUDIT_PYTHON', sys.executable)

    def folder(self, key):
        if not isinstance(key, str) or not KEY.fullmatch(key):
            raise ValueError('解析結果IDが不正です')
        folder = within(self.root, key)
        if read_json(within(folder, 'artifact.json')).get('schema') != 'kart.pipeline_audit.artifact.v1':
            raise ValueError('未完了または未対応の解析結果です')
        return folder

    def catalog(self):
        results = []
        for path in self.root.glob('*/*/*/artifact.json'):
            key = path.parent.relative_to(self.root).as_posix()
            try:
                folder = self.folder(key)
                within(folder, 'report.html')
                results.append({**read_json(folder / 'artifact.json'), 'id': key})
            except (OSError, ValueError):
                continue
        return dict(root=str(self.root), results=sorted(results, key=lambda r: r['created'], reverse=True))

    def prepare(self, body):
        if set(body) - {'bag', 'source_topic', 'age_spike_ms'}:
            raise ValueError('解析の設定項目が不正です')
        bag_key = body.get('bag')
        if not isinstance(bag_key, str) or not bag_key or any(p.startswith('.') for p in Path(bag_key).parts):
            raise ValueError('記録一覧からbagを選択してください')
        bag = within(self.records, bag_key)
        within(bag, 'metadata.yaml')
        # A bag's storage paths must not escape the selected recording.
        import yaml
        metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())
        paths = metadata['rosbag2_bagfile_information']['relative_file_paths']
        if not isinstance(paths, list) or not paths:
            raise ValueError('bagに記録ファイルがありません')
        for path in paths:
            within(bag, path)
        topic = body.get('source_topic', '/realsense/color/image_raw')
        if not isinstance(topic, str) or len(topic) > 256 or not re.fullmatch(r'/[A-Za-z0-9_/]+', topic):
            raise ValueError('画像／EVS metadataのtopic名が不正です')
        spike = body.get('age_spike_ms', 100.)
        if type(spike) not in (float, int) or not 1 <= spike <= 10000:
            raise ValueError('スパイク閾値は1..10000 msで指定してください')
        key = datetime.now().strftime('%Y-%m-%d/%H-%M-%S/') + 'audit_' + uuid.uuid4().hex[:8]
        return dict(bag=bag, bag_key=bag_key, source_topic=topic, age_spike_ms=float(spike), key=key)

    def run(self, job, plan):
        stage = self.root / ('.build-' + uuid.uuid4().hex)
        stage.mkdir()
        try:
            config = read_json(self.repo / 'tools/pipeline_audit/config.json')
            config['topics']['source'] = plan['source_topic']
            config['age_spike_ms'] = plan['age_spike_ms']
            atomic_json(stage / 'config.json', config)
            job.log('bagを読取専用で解析します。画像・Tensor本体は展開しません')
            job.run([self.python, str(self.repo / 'tools/pipeline_audit/cli.py'),
                     '--bag', str(plan['bag']), '--output', str(stage / 'result'),
                     '--config', str(stage / 'config.json')])
            summary = read_json(within(stage / 'result', 'summary.json'))
            if summary.get('schema') != 'kart.pipeline_audit.v1':
                raise ValueError('解析結果のschemaが不正です')
            for filename in FILES:
                within(stage / 'result', filename)
            artifact = dict(schema='kart.pipeline_audit.artifact.v1', created=time.time(), bag=plan['bag_key'],
                            source_topic=plan['source_topic'], age_spike_ms=plan['age_spike_ms'],
                            duration_s=summary['duration_s'], final=summary['qualified_latency_ms']['final'],
                            observed=summary['observed_latency_ms']['final'],
                            missing_topics=len(summary['missing_topics']))
            atomic_json(stage / 'result/artifact.json', artifact)
            target = within(self.root, plan['key'], exists=False)
            target.parent.mkdir(parents=True, exist_ok=True)
            job.check()
            if target.exists():
                raise ValueError('同じ解析結果が既に存在します')
            (stage / 'result').rename(target)
            return dict(audit=plan['key'])
        finally:
            shutil.rmtree(stage, ignore_errors=True)

    def artifact(self, key, filename):
        if filename not in FILES:
            raise ValueError('解析ファイル名が不正です')
        return within(self.folder(key), filename)
