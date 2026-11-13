"""Run dataset and Ultralytics work in the selected learning interpreter."""
import argparse
import os
from pathlib import Path
import tempfile
import time

from .perception_dataset import DatasetStore
from .storage import atomic_json, name, read_json, within
from .perception_selection_store import SelectionStore, image_signature
from .remote_agent import perception_contract, perception_manifest


class Progress:
    def log(self, message):
        print(message, flush=True)

    def check(self):
        pass  # The durable supervisor sends SIGINT to this whole process group.


def execute(plan):
    from .perception_runtime import run
    base, records = Path(plan['base']), Path(plan['records'])
    root = base / 'perception'
    datasets = DatasetStore(base, records)
    action = plan['action']
    settings = plan['settings']
    progress = Progress()
    if action == 'extract':
        datasets.extract(plan['project'], Path(plan['bag']), plan['topic'], split=plan['split'],
                         rate_hz=plan['sample_hz'], max_frames=plan['max_frames'],
                         expected_revision=plan['revision'], job=progress)
        return
    if action == 'prelabel':
        project = datasets.load(plan['project'])
        if project['revision'] != plan['revision']:
            raise ValueError('候補生成の開始前にannotationが変わりました。再取得して実行してください')
        frames = [dict(id=f['id'], path=str(datasets.image_path(project['id'], f['id'])))
                  for f in project['frames'] if not f['reviewed']]
        with tempfile.TemporaryDirectory(prefix='.prelabel-', dir=root) as temporary:
            run(dict(action='prelabel', output=temporary, weights=settings['weights'],
                     device=settings['device'], confidence=plan['confidence'], frames=frames))
            suggestions = read_json(Path(temporary) / 'suggestions.json')
            datasets.apply_suggestions(project['id'], plan['revision'], suggestions)
        return
    if action == 'select':
        from .perception_selection import run as select_images
        project = datasets.load(plan['project'])
        if project['revision'] != plan['revision'] or image_signature(project) != plan['images_signature']:
            raise ValueError('選定の開始前に画像やannotationが変わりました。再取得して実行してください')
        frames = [dict(frame, path=str(datasets.image_path(project['id'], frame['id']))) for frame in project['frames']]
        review = ({field: plan[field] for field in ('review_confidence', 'expected_max_detections', 'large_bbox_area_ratio')
                   if field in plan} if plan['method'] == 'yolo' else {})
        with tempfile.TemporaryDirectory(prefix='.selection-', dir=root) as temporary:
            selection = select_images(dict(method=plan['method'], action='select', output=temporary,
                weights=plan['weights'], device=settings['device'], max_images=plan['max_images'], limit=plan['limit'],
                encoder_repo=plan.get('encoder_repo'), encoder_weights=plan.get('encoder_weights'), frames=frames, **review))
            if not isinstance(selection, dict) or 'ranked' not in selection:
                selection = read_json(Path(temporary) / 'selection.json')
            SelectionStore(datasets).publish(project['id'], plan['revision'], plan['images_signature'], selection)
        return
    key = name(plan['key'])
    if action == 'train':
        output = within(root / 'runs', key, exists=False)
        if output.exists():
            raise ValueError('同じ学習出力が既にあります')
        output.mkdir()
        artifact = dict(schema=1, id=key, name=plan['title'], project=plan['project'],
                        created=plan['created'], status='running', **perception_contract(['rc_car', 'obstacle']),
                        project_revision=plan['revision'])
        atomic_json(output / 'artifact.json', artifact)
        try:
            snapshot = datasets.snapshot(plan['project'], plan['revision'], output / 'dataset', job=progress)
            atomic_json(output / 'request.json', plan)
            parameters = {k: plan[k] for k in ('epochs', 'batch', 'imgsz', 'lr0', 'seed', 'workers')}
            run(dict(action='train', output=str(output), weights=settings['weights'],
                     dataset_yaml=str(snapshot['dataset_yaml']), device=settings['device'], **parameters))
            within(output, 'weights/best.pt')
            within(output, 'report.json')
            artifact.update(status='succeeded', finished=time.time(), report=read_json(output / 'report.json'))
        except BaseException as exc:
            artifact.update(status='cancelled' if isinstance(exc, KeyboardInterrupt) else 'failed',
                            finished=time.time(), error=str(exc)[:2048])
            raise
        finally:
            atomic_json(output / 'artifact.json', artifact)
        return
    if action == 'export':
        source = within(root / 'runs', name(plan['run']))
        artifact = read_json(within(source, 'artifact.json'))
        if artifact['status'] != 'succeeded':
            raise ValueError('学習が完了していません')
        weights = within(source, 'weights/best.pt')
        target = within(root / 'models', key, exists=False)
        if target.exists():
            raise ValueError('同じモデル出力が既にあります')
        with tempfile.TemporaryDirectory(prefix='.export-', dir=target.parent) as temporary:
            stage = Path(temporary) / 'model'
            stage.mkdir()
            run(dict(action='export', output=str(stage), weights=str(weights)))
            within(stage, 'opponent.onnx')
            within(stage, 'report.json')
            report = read_json(stage / 'report.json')
            contract = perception_contract(report.get('class_names'))
            atomic_json(stage / 'artifact.json', dict(schema=1, id=key, name=plan['title'],
                run=source.name, project=plan['project'], created=plan['created'], status='succeeded',
                **contract,
                source_revision=artifact['project_revision']))
            # Validate the report, bindings, SHA and artifact before publication.
            perception_manifest(stage, transfer_required=False)
            os.rename(stage, target)
        return
    raise ValueError('未対応の学習タスクです')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    args = parser.parse_args()
    execute(read_json(args.plan))


if __name__ == '__main__':
    main()
