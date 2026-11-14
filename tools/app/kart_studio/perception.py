"""Local RC-car detection projects, durable learning jobs and model artifacts."""
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import uuid
from urllib.parse import urlencode

from .mapping import local_records
from .storage import atomic_json, name, read_json, within
from .perception_dataset import DatasetStore
from .perception_selection_store import SelectionStore, image_signature


class Perception:
    def __init__(self, repo, base, records, state):
        self.repo, self.base, self.records = Path(repo).resolve(), Path(base).resolve(), Path(records).resolve()
        self.root = self.base / 'perception'
        self.settings_file = Path(state).resolve() / 'perception.json'
        self.tools_settings_file = Path(state).resolve() / 'perception_tools.json'
        self.datasets = DatasetStore(self.base, self.records)
        self.selections = SelectionStore(self.datasets)
        for kind in ('runs', 'models'):
            (self.root / kind).mkdir(parents=True, exist_ok=True)

    def settings(self):
        values = dict(python=os.environ.get('KART_PERCEPTION_PYTHON', str(self.repo / '.venv/perception/bin/python')),
                      weights=str(self.repo / 'weights/racing/yolov8s/yolov8s.pt'), device='auto')
        if self.settings_file.exists():
            values.update(read_json(self.settings_file))
        return values

    def save_settings(self, values):
        if set(values) != {'python', 'weights', 'device'} or not all(
                isinstance(v, str) and 0 < len(v) <= 4096 and '\x00' not in v for v in values.values()):
            raise ValueError('学習環境の設定項目が不正です')
        if not re.fullmatch(r'auto|cpu|cuda(?::[0-9]+)?|mps', values['device']):
            raise ValueError('deviceはauto / cpu / cuda[:番号] / mpsです')
        if not Path(values['weights']).is_absolute():
            raise ValueError('重みは絶対パスで指定してください')
        atomic_json(self.settings_file, values)
        return values

    def tools_settings(self):
        defaults = dict(model_root='/home/kart/workspaces/kart/weights/racing/yolov8s/finetuned',
                        container_model_root='/workspaces/weights/racing/yolov8s/finetuned',
                        encoder_repo=str(self.repo / 'python_ws/dinov3'),
                        encoder_weights=str(self.repo / 'weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth'))
        if self.tools_settings_file.exists():
            defaults.update(read_json(self.tools_settings_file))
        return defaults

    def save_tools_settings(self, values):
        if not isinstance(values, dict) or set(values) - set(self.tools_settings()) or not all(
                isinstance(value, str) and len(value) <= 4096 and '\x00' not in value for value in values.values()):
            raise ValueError('転送・画像選定の設定が不正です')
        updated = dict(self.tools_settings(), **values)
        atomic_json(self.tools_settings_file, updated)
        return updated

    def environment(self):
        settings = self.settings()
        executable = shutil.which(settings['python'])
        if not executable:
            return dict(ready=False, message='学習環境をセットアップするか、Pythonのパスを設定してください')
        code = ('import importlib.util,json;print(json.dumps({m:importlib.util.find_spec(m) is not None '
                'for m in ("ultralytics","rosbags","PIL","onnx")}))')
        try:
            probe = subprocess.run([executable, '-c', code], capture_output=True, text=True, timeout=8)
            modules = json.loads(probe.stdout) if probe.returncode == 0 else {}
        except (OSError, ValueError, subprocess.TimeoutExpired):
            modules = {}
        missing = [m for m in ('ultralytics', 'rosbags', 'PIL', 'onnx') if not modules.get(m)]
        weights = Path(settings['weights']).is_file()
        return dict(ready=not missing and weights, missing=missing, weights_available=weights,
                    message=('準備完了' if not missing and weights else
                             '不足: ' + ', '.join([*missing, *([] if weights else ['YOLOv8sの重み'])])))

    def extraction_python(self):
        # Extraction only needs bag/image readers. The Studio interpreter in the
        # kart container already provides them before the training venv exists.
        code = ('from rosbags.highlevel import AnyReader; '
                'from rosbags.typesys import Stores, get_typestore; from PIL import Image')
        for candidate in dict.fromkeys((self.settings()['python'], sys.executable)):
            executable = shutil.which(candidate)
            if not executable:
                continue
            try:
                probe = subprocess.run([executable, '-c', code], capture_output=True, text=True, timeout=8)
            except (OSError, subprocess.TimeoutExpired):
                continue
            if probe.returncode == 0:
                return executable
        raise ValueError('画像抽出用のPythonにrosbags・Pillowが必要です。'
                         '「3 学習・ONNX」→「Notebookの学習環境」→「学習環境をセットアップ」を実行するか、'
                         '同じ欄でこれらを導入済みのPythonを指定して「設定を保存」してください。'
                         '画像抽出自体にはYOLOの重みは不要です')

    def catalog(self):
        result = dict(settings=self.settings(), tools_settings=self.tools_settings(), environment=self.environment(),
                      projects=self.datasets.catalog(), records=[dict(row, key=row['id']) for row in local_records(self.records)])
        for project in result['projects']:
            project.update(frames=project['frames_count'], reviewed=project['reviewed_count'])
        for kind in ('runs', 'models'):
            rows = []
            for folder in (self.root / kind).iterdir():
                if folder.name.startswith('.') or folder.is_symlink() or not folder.is_dir():
                    continue
                try:
                    within(self.root / kind, folder.name)
                    info = read_json(folder / 'artifact.json')
                    if kind == 'models' and info.get('status') == 'succeeded':
                        info['downloads'] = [dict(name=filename, url='/perception-file?' + urlencode(dict(id=folder.name, file=filename)))
                                             for filename in ('opponent.onnx', 'report.json', 'artifact.json')]
                    if kind == 'runs':
                        metrics = self.metrics(folder.name)['metrics']
                        info['metrics'] = metrics[-1] if metrics else None
                    rows.append(dict(info, id=folder.name))
                except (ValueError, OSError):
                    continue
            result[kind] = sorted(rows, key=lambda v: v.get('created', 0), reverse=True)
        return result

    def create(self, body):
        title = body.get('name', '')
        if not isinstance(title, str) or not title.strip() or len(title) > 80:
            raise ValueError('プロジェクト名を1..80文字で指定してください')
        key = 'rc-car-' + datetime.now().strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:6]
        return self.datasets.create(key, title.strip())

    def annotate(self, body):
        from .perception_dataset import CLASS_NAMES
        if body.get('class_names') != CLASS_NAMES or not isinstance(body.get('class_ids'), list):
            raise ValueError('2クラスの確認画面を再読み込みしてから保存してください（rc_car / obstacle）')
        return self.datasets.save_frame(body.get('id'), body.get('frame'), body.get('revision'),
                                        body.get('frame_revision'), body.get('boxes'), body.get('reviewed', True),
                                        class_ids=body['class_ids'])

    @staticmethod
    def number(value, low, high, label, integer=False):
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high or (integer and int(value) != value):
            raise ValueError(f'{label}は{low}..{high}' + ('の整数です' if integer else 'です'))
        return int(value) if integer else float(value)

    def folder(self, kind, key):
        if kind not in ('runs', 'models'):
            raise ValueError('成果物の種類が不正です')
        return within(self.root / kind, name(key))

    def artifact(self, key, filename):
        if filename not in ('opponent.onnx', 'report.json', 'artifact.json'):
            raise ValueError('公開対象外の成果物です')
        folder = self.folder('models', key)
        if read_json(within(folder, 'artifact.json')).get('status') != 'succeeded':
            raise ValueError('モデルが未完成です')
        return within(folder, filename)

    def metrics(self, key):
        folder = self.folder('runs', key)
        path = within(folder, 'metrics.jsonl', exists=False)
        if not path.is_file():
            return dict(id=key, metrics=[])
        rows = []
        for line in path.read_text().splitlines()[-2000:]:
            try:
                row = json.loads(line)
                if isinstance(row, dict):
                    rows.append(row)
            except ValueError:
                continue  # The trainer may currently be writing the final line.
        return dict(id=key, metrics=rows)

    def prepare(self, action, body):
        if action not in ('setup', 'extract', 'prelabel', 'select', 'train', 'export'):
            raise ValueError('未対応の学習操作です')
        key = 'rc-' + datetime.now().strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8]
        plan = dict(action=action, key=key, repo=str(self.repo), base=str(self.base),
                    records=str(self.records), settings=self.settings(), created=time.time())
        if action == 'setup':
            plan['resources'] = ['perception:environment']
            return plan
        if action != 'extract' and not shutil.which(plan['settings']['python']):
            raise ValueError('先に「学習環境をセットアップ」を実行してください')
        if action in ('train', 'prelabel') and not Path(plan['settings']['weights']).is_file():
            raise ValueError('YOLOv8sの重みがありません。学習環境の重みパスを確認してください')
        if action in ('extract', 'prelabel', 'select', 'train'):
            project = self.datasets.load(body.get('id'))
            plan.update(project=project['id'], revision=project['revision'])
        if action == 'extract':
            bag_key = body.get('bag')
            if not isinstance(bag_key, str) or not bag_key or any(p.startswith('.') for p in Path(bag_key).parts):
                raise ValueError('記録一覧からbagを選んでください')
            bag = within(self.records, bag_key)
            within(bag, 'metadata.yaml')
            topic = body.get('topic', '/realsense/color/image_raw')
            if not isinstance(topic, str) or not re.fullmatch(r'/[A-Za-z0-9_/]+', topic):
                raise ValueError('画像topic名が不正です')
            split = body.get('split', 'train')
            if split not in ('train', 'val'):
                raise ValueError('学習用／検証用を選んでください')
            plan.update(bag=str(bag), bag_key=bag_key, split=split, topic=topic,
                        sample_hz=self.number(body.get('sample_hz', 2), .1, 30, '抽出Hz'),
                        max_frames=self.number(body.get('max_frames', 500), 1, 500, '最大画像数', True))
            plan['settings']['python'] = self.extraction_python()
            plan['resources'] = ['read:perception:environment', 'perception:project:' + project['id'], 'read:record:' + bag_key]
        elif action == 'prelabel':
            if len(project['frames']) > 5000:
                raise ValueError('候補生成は5000画像までです')
            plan['confidence'] = self.number(body.get('confidence', .1), .01, .95, '候補score')
            plan['resources'] = ['read:perception:environment', 'perception:compute', 'perception:project:' + project['id']]
        elif action == 'select':
            candidates = [frame for frame in project['frames'] if frame['split'] == 'train' and not frame['reviewed']]
            if not candidates:
                raise ValueError('選定する未確認のtrain画像がありません')
            method = body.get('method', 'yolo')
            if method not in ('yolo', 'dinov3'):
                raise ValueError('画像選定はyolo / dinov3を選んでください')
            maximum = self.number(body.get('max_images', 500), 1, 500, '処理画像上限', True)
            limit = self.number(body.get('limit', 100), 1, 500, '選定枚数', True)
            if limit > maximum:
                raise ValueError('選定枚数は処理画像上限以下にしてください')
            plan.update(method=method, max_images=maximum, limit=limit, images_signature=image_signature(project))
            if method == 'yolo':
                from .perception_selection import review_options
                plan.update(review_options(body))
            source = body.get('weights_source', 'official')
            if source == 'official':
                weights = Path(plan['settings']['weights'])
            else:
                run = self.folder('runs', source)
                if read_json(within(run, 'artifact.json')).get('status') != 'succeeded':
                    raise ValueError('完了した学習の重みを選んでください')
                weights = within(run, 'weights/best.pt')
            plan['weights'] = str(weights)
            if method == 'yolo' and not weights.is_file():
                raise ValueError('選定に使うYOLOの重みがありません')
            if method == 'dinov3':
                positives = [f for f in project['frames'] if f['split'] == 'train' and f['reviewed'] and f['boxes']]
                negatives = [f for f in project['frames'] if f['split'] == 'train' and f['reviewed'] and not f['boxes']]
                if not positives or not negatives:
                    raise ValueError('先にtrain画像の対象あり（RCカー / obstacle）・両方なしを、それぞれ1枚以上確認して保存してください')
                for field in ('encoder_repo', 'encoder_weights'):
                    value = body.get(field, self.tools_settings()[field])
                    if not isinstance(value, str) or not Path(value).expanduser().is_absolute():
                        raise ValueError('DINOv3のソース・重みは絶対パスを指定してください')
                    path = Path(value).expanduser().resolve()
                    if not (path.is_dir() if field == 'encoder_repo' else path.is_file()):
                        raise ValueError('DINOv3のソースまたは重みがありません。既存のローカル資産を指定してください')
                    plan[field] = str(path)
                self.save_tools_settings({field: plan[field] for field in ('encoder_repo', 'encoder_weights')})
            plan['resources'] = ['read:perception:environment', 'perception:compute', 'perception:project:' + project['id']]
        elif action == 'train':
            title = body.get('name') or key
            if not isinstance(title, str) or len(title) > 80:
                raise ValueError('学習名が不正です')
            plan.update(title=title, epochs=self.number(body.get('epochs', 50), 1, 1000, 'epoch', True),
                        batch=self.number(body.get('batch', 8), 1, 128, 'batch', True),
                        imgsz=self.number(body.get('imgsz', 640), 320, 1280, '画像サイズ', True),
                        lr0=self.number(body.get('lr0', .001), .000001, .1, '学習率'), seed=0, workers=2)
            if plan['imgsz'] % 32:
                raise ValueError('画像サイズは32の倍数で指定してください')
            plan['resources'] = ['read:perception:environment', 'perception:run:' + key, 'perception:compute']
        else:
            run = self.folder('runs', body.get('run'))
            info = read_json(within(run, 'artifact.json'))
            if info.get('status') != 'succeeded':
                raise ValueError('完了した学習を選んでください')
            within(run, 'weights/best.pt')
            title = body.get('name') or run.name
            if not isinstance(title, str) or len(title) > 80:
                raise ValueError('モデル名が不正です')
            plan.update(run=run.name, project=info.get('project'), title=title)
            plan['resources'] = ['read:perception:environment', 'read:perception:run:' + run.name, 'perception:model:' + key, 'perception:compute']
        return plan

    def execute(self, job, plan):
        if plan['repo'] != str(self.repo) or plan['base'] != str(self.base):
            raise ValueError('ジョブの保存先が変わりました')
        if plan['action'] == 'setup':
            job.run(['bash', str(self.repo / 'scripts/perception_setup.sh')])
            self.save_settings(dict(self.settings(), python=str(self.repo / '.venv/perception/bin/python')))
            return self.environment()
        request = self.root / ('.request-' + uuid.uuid4().hex + '.json')
        atomic_json(request, plan)
        env = dict(os.environ, PYTHONPATH=str(self.repo / 'tools/app') + os.pathsep + os.environ.get('PYTHONPATH', ''))
        try:
            job.run([plan['settings']['python'], '-m', 'kart_studio.perception_task', '--plan', str(request)], env=env)
            if plan['action'] in ('extract', 'prelabel', 'select'):
                return dict(project=plan['project'])
            return dict(id=plan['key'], kind='runs' if plan['action'] == 'train' else 'models')
        finally:
            request.unlink(missing_ok=True)
