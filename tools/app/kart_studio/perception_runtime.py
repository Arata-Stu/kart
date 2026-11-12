"""Lazy Ultralytics worker for reviewed RC-car datasets and model artifacts."""
import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import time

from .storage import atomic_json


REPO = Path(__file__).resolve().parents[3]
MAX_FRAMES = 5000
MAX_BOXES = 100
OFFICIAL_WEIGHTS_SHA256 = '1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a'


def _sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def _file(value, suffix=None):
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise ValueError('入力ファイルは絶対パスで指定してください')
    path = Path(value).resolve(strict=True)
    if not path.is_file() or not path.stat().st_size or (suffix and path.suffix.lower() != suffix):
        raise ValueError(f'入力ファイルが不正です: {path}')
    return path


def _base_weights(value):
    path = _file(value, '.pt')
    checksum = _sha(path)
    if checksum != OFFICIAL_WEIGHTS_SHA256:
        raise ValueError('初期重みはセットアップで取得した公式YOLOv8sを指定してください（SHA-256不一致）')
    return path, checksum


def _output(value, reserved):
    if not isinstance(value, str) or not Path(value).is_absolute() or Path(value).is_symlink():
        raise ValueError('出力先は独立した絶対パスのディレクトリで指定してください')
    path = Path(value).resolve()
    path.mkdir(parents=True, exist_ok=True)
    if any((path / name).exists() for name in reserved):
        raise ValueError('同じ出力先の学習・候補・モデル成果物は上書きできません')
    return path


def _number(value, lower, upper, label, integer=False):
    if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f'{label}は{lower}..{upper}で指定してください')
    if integer and int(value) != value:
        raise ValueError(f'{label}は整数で指定してください')
    return int(value) if integer else float(value)


def _device(value):
    if not isinstance(value, str) or not re.fullmatch(r'auto|cpu|cuda(?::[0-9]+)?|mps', value):
        raise ValueError('deviceはauto / cpu / cuda[:番号] / mpsです')
    import torch
    if value == 'auto':
        return '0' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    if value.startswith('cuda'):
        index = int(value.split(':')[1]) if ':' in value else 0
        if not torch.cuda.is_available() or index >= torch.cuda.device_count():
            raise ValueError('指定したCUDA deviceを使用できません')
        return str(index)
    if value == 'mps' and not torch.backends.mps.is_available():
        raise ValueError('この環境ではMPSを使用できません')
    return value


def _load_yolo(weights, output):
    # Jobs run in their selected interpreter; server imports never import torch.
    os.environ['YOLO_OFFLINE'] = 'true'
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    os.environ['YOLO_CONFIG_DIR'] = str(output / '.ultralytics')
    os.environ['MPLCONFIGDIR'] = str(output / '.matplotlib')
    os.environ['XDG_CACHE_HOME'] = str(output / '.cache')
    from ultralytics import YOLO, settings
    settings.update({name: False for name in ('sync', 'clearml', 'comet', 'dvc', 'mlflow',
                                             'raytune', 'tensorboard', 'wandb', 'vscode_msg', 'openvino_msg')})
    # check_det_dataset calls check_font even with plots=False. Supply a local
    # Matplotlib font so that this check cannot trigger an implicit download.
    import matplotlib
    from ultralytics.utils import USER_CONFIG_DIR
    font = Path(matplotlib.get_data_path()) / 'fonts/ttf/DejaVuSans.ttf'
    target = USER_CONFIG_DIR / 'Arial.ttf'
    if not target.exists():
        shutil.copyfile(font, target)
    return YOLO(str(weights), task='detect')


def _names(model):
    names = model.names
    return list(names.values()) if isinstance(names, dict) else list(names)


def _versions():
    versions = {}
    for name in ('ultralytics', 'torch', 'torchvision', 'onnx', 'Pillow'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            pass
    return versions


def _dataset(path):
    import yaml
    spec = yaml.safe_load(path.read_text())
    if not isinstance(spec, dict) or spec.get('nc') != 1 or spec.get('names') not in (
            ['rc_car'], {0: 'rc_car'}, {'0': 'rc_car'}):
        raise ValueError('rc_car 1クラスのreview済みdataset snapshotが必要です')
    if any(key in spec for key in ('download', 'validation', 'test', 'minival')):
        raise ValueError('学習snapshotにダウンロード指定・追加splitは使用できません')
    root = Path(spec.get('path', path.parent)).resolve()
    if root != path.parent.resolve():
        raise ValueError('datasetのpathはsnapshot自身を指す必要があります')
    image_sets = {}
    for split in ('train', 'val'):
        relative = spec.get(split)
        if not isinstance(relative, str) or Path(relative).is_absolute() or '..' in Path(relative).parts:
            raise ValueError('train/valはsnapshot内の画像ディレクトリを指定してください')
        folder = root / relative
        if not folder.is_dir() or not folder.resolve().is_relative_to(root):
            raise ValueError('train/val画像がありません')
        image_sets[split] = {p.resolve() for p in folder.iterdir() if p.suffix.lower() in ('.jpg', '.jpeg', '.png')}
        if not image_sets[split] or len(image_sets[split]) > MAX_FRAMES:
            raise ValueError('各splitには1..5000画像が必要です')
    if image_sets['train'] & image_sets['val']:
        raise ValueError('trainとvalに同じ画像を使用できません')
    metadata_path = root / 'snapshot.json'
    metadata = json.loads(metadata_path.read_text())
    if metadata.get('schema') != 'kart.perception.dataset.v1' or metadata.get('class_names') != ['rc_car']:
        raise ValueError('dataset snapshotのmetadataが不正です')
    bags = metadata.get('split_bag_ids', {})
    train, val = bags.get('train'), bags.get('val')
    if (not isinstance(train, list) or not isinstance(val, list) or not train or not val
            or not all(isinstance(x, str) and x for x in train + val) or set(train) & set(val)):
        raise ValueError('学習と検証には独立したbagを指定してください')
    frames = metadata.get('frames')
    if not isinstance(frames, list) or not frames or any(not isinstance(f, dict) or f.get('reviewed') is not True for f in frames):
        raise ValueError('未確認の画像・自動候補は学習に使用できません')
    return dict(snapshot=metadata, dataset_yaml_sha256=_sha(path), snapshot_sha256=_sha(metadata_path),
                images={k: len(v) for k, v in image_sets.items()})


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class Metrics:
    def __init__(self, path, epochs):
        self.path, self.epochs = Path(path), epochs
        self.started, self.seen = time.monotonic(), set()
        self.active_epoch = None

    def start_epoch(self, trainer):
        self.active_epoch = int(trainer.epoch) + 1

    def __call__(self, trainer):
        epoch = int(trainer.epoch) + 1
        # Ultralytics final_eval calls this hook again at epoch+1 for best.pt.
        # Log only an epoch that actually entered the training loop.
        if epoch != self.active_epoch or not 1 <= epoch <= self.epochs or epoch in self.seen:
            return
        self.seen.add(epoch)
        values = dict(getattr(trainer, 'metrics', {}) or {})
        losses = trainer.label_loss_items(trainer.tloss, prefix='train')
        train = {k.removeprefix('train/'): _finite(v) for k, v in losses.items()}
        loss_values = [v for v in train.values() if v is not None]
        val = {k.removeprefix('val/'): _finite(v) for k, v in values.items() if k.startswith('val/')}
        row = dict(epoch=epoch, total_epochs=self.epochs,
                   precision=_finite(values.get('metrics/precision(B)')),
                   recall=_finite(values.get('metrics/recall(B)')),
                   map50=_finite(values.get('metrics/mAP50(B)')),
                   map50_95=_finite(values.get('metrics/mAP50-95(B)')),
                   loss=sum(loss_values) if loss_values else None, train_loss=train, val_loss=val,
                   lr={k: _finite(v) for k, v in (getattr(trainer, 'lr', {}) or {}).items()},
                   elapsed_s=time.monotonic() - self.started, time=time.time())
        with self.path.open('a') as stream:
            stream.write(json.dumps(row, allow_nan=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        loss_text = '未取得' if row['loss'] is None else f"{row['loss']:.4f}"
        print(f"epoch {epoch}/{self.epochs}: loss={loss_text} mAP50={row['map50']} mAP50-95={row['map50_95']}", flush=True)


def train(plan):
    weights, source_sha = _base_weights(plan['weights'])
    dataset_yaml = _file(plan['dataset_yaml'], '.yaml')
    parameters = dict(epochs=_number(plan.get('epochs', 50), 1, 1000, 'epoch', True),
        batch=_number(plan.get('batch', 8), 1, 128, 'batch', True),
        imgsz=_number(plan.get('imgsz', 640), 320, 1280, '画像サイズ', True),
        lr0=_number(plan.get('lr0', .001), .000001, .1, '学習率'),
        seed=_number(plan.get('seed', 0), 0, 2**31 - 1, 'seed', True),
        workers=_number(plan.get('workers', 2), 0, 8, 'workers', True), optimizer='AdamW')
    if parameters['imgsz'] % 32:
        raise ValueError('画像サイズは32の倍数で指定してください')
    dataset = _dataset(dataset_yaml)
    output = _output(plan['output'], ('weights', 'metrics.jsonl', 'report.json', 'args.yaml', 'results.csv'))
    if output.is_relative_to(dataset_yaml.parent) or weights.is_relative_to(output):
        raise ValueError('学習出力は元の重み・dataset snapshotから独立したフォルダーにしてください')
    device = _device(plan.get('device', 'auto'))
    model = _load_yolo(weights, output)
    if len(_names(model)) != 80 or getattr(model, 'task', 'detect') != 'detect':
        raise ValueError('初期重みはCOCO80クラスの公式YOLOv8s Detectを指定してください')
    log = Metrics(output / 'metrics.jsonl', parameters['epochs'])
    model.add_callback('on_train_epoch_start', log.start_epoch)
    model.add_callback('on_fit_epoch_end', log)
    model.train(data=str(dataset_yaml), project=str(output.parent), name=output.name, exist_ok=True,
                device=device, amp=False, plots=False, save=True, val=True, cache=False,
                resume=False, pretrained=str(weights), **parameters)
    for name in ('best.pt', 'last.pt'):
        _file(str(output / 'weights' / name), '.pt')
    if _sha(weights) != source_sha:
        raise ValueError('学習中に初期重みが変更されました')
    report = dict(schema='kart.perception.train.v1', action='train', class_names=['rc_car'],
        source_weights=str(weights), source_weights_sha256=source_sha, dataset_yaml=str(dataset_yaml),
        dataset_yaml_sha256=dataset['dataset_yaml_sha256'], snapshot_sha256=dataset['snapshot_sha256'],
        annotation_revision=dataset['snapshot'].get('annotation_revision'),
        annotation_sha256=dataset['snapshot'].get('annotation_sha256'), split_bag_ids=dataset['snapshot']['split_bag_ids'],
        images=dataset['images'], parameters=dict(parameters, device=device, amp=False),
        completed_epochs=len(log.seen), versions=_versions(), finished=time.time())
    atomic_json(output / 'report.json', report)
    return report


def prelabel(plan):
    weights, source_sha = _base_weights(plan['weights'])
    confidence = _number(plan.get('confidence', .1), .01, .95, '候補score')
    frames = plan.get('frames')
    if not isinstance(frames, list) or len(frames) > MAX_FRAMES:
        raise ValueError('候補生成は最大5000画像です')
    from PIL import Image
    inputs = []
    ids = set()
    for frame in frames:
        identity = frame.get('id')
        if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', identity) or identity in ids:
            raise ValueError('候補画像IDが不正か重複しています')
        ids.add(identity)
        path = _file(frame['path'])
        if path.stat().st_size > 64 * 1024 * 1024:
            raise ValueError('候補画像が64 MiB上限を超えています')
        with Image.open(path) as image:
            width, height = image.size
            if min(width, height) < 1 or max(width, height) > 8192 or width * height > 16_777_216:
                raise ValueError('候補画像のサイズが上限を超えています')
            image.verify()
        inputs.append((identity, path, width, height))
    output = _output(plan['output'], ('suggestions.json', 'report.json'))
    suggestions = {}
    if inputs:
        device = _device(plan.get('device', 'auto'))
        model = _load_yolo(weights, output)
        if len(_names(model)) != 80:
            raise ValueError('候補生成にはCOCO80クラスの公式重みを使用してください')
        for index, (identity, path, width, height) in enumerate(inputs):
            prediction = model.predict(source=str(path), conf=confidence, device=device, imgsz=640,
                                       classes=None, max_det=MAX_BOXES, verbose=False, save=False)[0]
            if tuple(prediction.orig_shape) != (height, width):
                raise ValueError('推論結果と元画像のサイズが一致しません')
            boxes = []
            if prediction.boxes is not None:
                for row in prediction.boxes.xyxy.cpu().tolist()[:MAX_BOXES]:
                    if len(row) != 4 or not all(math.isfinite(v) for v in row):
                        continue
                    x1, y1, x2, y2 = [float(v) for v in row]
                    box = [max(0., min(width, x1)), max(0., min(height, y1)),
                           max(0., min(width, x2)), max(0., min(height, y2))]
                    if box[0] < box[2] and box[1] < box[3]:
                        boxes.append(box)
            suggestions[identity] = boxes
            print(f'候補 {index + 1}/{len(inputs)}: {identity} {len(boxes)}件（未確認）', flush=True)
    atomic_json(output / 'suggestions.json', suggestions)
    if _sha(weights) != source_sha:
        raise ValueError('候補生成中に初期重みが変更されました')
    report = dict(schema='kart.perception.prelabel.v1', action='prelabel', reviewed=False,
        automatic_ground_truth=False, source_weights=str(weights), source_weights_sha256=source_sha,
        confidence=confidence, images=len(inputs), proposals=sum(map(len, suggestions.values())),
        class_filter=None, note='元のCOCOクラスを問わないbbox候補です。rc_carかを手動確認してください。')
    atomic_json(output / 'report.json', report)
    return report


def export(plan):
    weights = _file(plan['weights'], '.pt')
    output = _output(plan['output'], ('opponent.onnx', 'report.json'))
    source_sha = _sha(weights)
    with tempfile.TemporaryDirectory(prefix='.onnx-', dir=output) as temporary:
        temporary = Path(temporary)
        copied = temporary / 'opponent.pt'
        shutil.copyfile(weights, copied)
        model = _load_yolo(copied, output)
        if len(_names(model)) != 1 or getattr(model, 'task', 'detect') != 'detect':
            raise ValueError('exportには学習済み1クラスDetectモデルが必要です')
        source = Path(model.export(format='onnx', imgsz=(480, 640), half=False, dynamic=False,
            nms=False, batch=1, opset=17, simplify=False, device='cpu')).resolve()
        if source.parent != temporary.resolve():
            raise ValueError('ONNX exportの出力先が不正です')
        spec = importlib.util.spec_from_file_location('_perception_prepare_onnx', REPO / 'scripts/prepare_racing_onnx.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        checked = temporary / 'checked.onnx'
        module.prepare(source, checked, num_classes=1)
        if _sha(weights) != source_sha:
            raise ValueError('export中に元checkpointが変更されました')
        report = dict(schema='kart.perception.export.v1', action='export', model_profile='opponent',
            num_classes=1, class_names=_names(model), source_weights=str(weights), source_weights_sha256=source_sha,
            onnx_sha256=_sha(checked), input=dict(name='images', shape=[1, 3, 480, 640], dtype='float32'),
            output=dict(name='output0', shape=[1, 5, 6300], dtype='float32'),
            versions=_versions(), finished=time.time())
        checked.replace(output / 'opponent.onnx')
    atomic_json(output / 'report.json', report)
    return report


def run(plan):
    if not isinstance(plan, dict) or plan.get('action') not in ('train', 'prelabel', 'export'):
        raise ValueError('未対応の学習runtime操作です')
    return {'train': train, 'prelabel': prelabel, 'export': export}[plan['action']](plan)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    args = parser.parse_args()
    if args.plan.stat().st_size > 8 * 1024 * 1024:
        parser.error('planが8 MiB上限を超えています')
    report = run(json.loads(args.plan.read_text()))
    print(json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
