"""Append official offline stages without rerunning or changing VSLAM/HDMap."""
import os
import shutil
import time
import uuid

from kart_mapping.dense_outputs import collect, require_engine
from .storage import atomic_json, read_json, within


def source(maps, key, mapping, step):
    relative = mapping.get('depth_output') if step == 'occupancy' else None
    return within(maps.folder(key), relative or mapping['official_output'])


def status(maps, key):
    folder = maps.folder(key)
    manifest = folder / 'mapping_result.json'
    mapping = read_json(manifest) if manifest.is_file() else {}
    reason = ''
    try:
        run = source(maps, key, mapping, 'depth')
        rectified = within(run, 'map_frames/rectified')
        within(rectified, 'frames_meta.json')
        if not any(p.is_file() and p.suffix.lower() in ('.png', '.jpg', '.jpeg') for p in rectified.rglob('*')):
            raise ValueError('左右画像がありません')
    except (KeyError, ValueError, OSError) as exc:
        reason = '公式の画像・姿勢データがありません。bagからの地図作成が必要です。'
    depth_ready = False
    if not reason and ('depth' in mapping.get('steps', []) or mapping.get('depth_output')):
        try:
            depth = within(source(maps, key, mapping, 'occupancy'), 'map_frames/depth')
            depth_ready = any(p.is_file() and p.stat().st_size for p in depth.rglob('*'))
        except (ValueError, OSError):
            pass
    return dict(can_depth=not reason, can_occupancy=depth_ready,
                depth_ready=depth_ready, dense_ready=bool(mapping.get('dense')),
                dense_stale=bool(mapping.get('dense_stale')),
                reason=reason, resolution=mapping.get('fs_model_res') or 'low_res')


def validate(studio, key, body):
    step = body.get('step')
    resolution = body.get('resolution', 'low_res')
    if step not in ('depth', 'occupancy') or resolution not in ('low_res', 'high_res'):
        raise ValueError('工程またはモデル解像度が不正です')
    doc = studio.maps.load(key)
    if doc['revision'] != body.get('revision'):
        raise ValueError('地図が更新されています。開き直してください')
    current = status(studio.maps, key)
    if not current['can_' + step]:
        raise ValueError(current['reason'] or '先に深度推定を完了してください')
    if not shutil.which('ros2'):
        raise ValueError('CUDA対応Linuxのkartコンテナで実行してください')
    if step == 'depth':
        require_engine(resolution, os.environ.get('ISAAC_ROS_WS', str(studio.repo / 'ros2_ws')))
    return step, resolution


def build(studio, job, key, revision, step, resolution):
    folder = studio.maps.folder(key).resolve()
    doc = studio.maps.load(key)
    if doc['revision'] != revision:
        raise ValueError('地図が更新されています')
    mapping = read_json(folder / 'mapping_result.json')
    original = source(studio.maps, key, mapping, step)
    # Stable final path: metadata may contain absolute image paths. The manifest
    # is the publication point; unfinished directories are never advertised.
    parent = within(folder, 'dense_runs', exists=False)
    parent.mkdir(exist_ok=True)
    target = parent / uuid.uuid4().hex
    published = False

    def copy_file(src, dst):
        job.check()
        with open(src, 'rb') as reader, open(dst, 'wb') as writer:
            while chunk := reader.read(1024 * 1024):
                job.check()
                writer.write(chunk)
        return dst

    try:
        job.log('既存の画像・姿勢を保護する作業コピーを作成しています')
        for p in original.rglob('*'):
            if p.is_symlink() or not (p.is_file() or p.is_dir()):
                raise ValueError('公式中間成果物にリンク・特殊ファイルがあります')
        shutil.copytree(original, target, copy_function=copy_file)
        # Saved official metadata can refer to an earlier .build directory.
        # Rewrite only paths under map_frames; reject escaping suffixes.
        def relocate(value):
            if isinstance(value, dict):
                return {k: relocate(v) for k, v in value.items()}
            if isinstance(value, list):
                return [relocate(v) for v in value]
            if isinstance(value, str) and value.startswith('/') and '/map_frames/' in value:
                relative = 'map_frames/' + value.split('/map_frames/', 1)[1]
                return str(within(target, relative))
            return value
        for meta in (target / 'map_frames').rglob('*.json'):
            atomic_json(meta, relocate(read_json(meta)))
        # Never mistake copied outputs for newly generated outputs.
        if step == 'depth':
            shutil.rmtree(target / 'map_frames/depth', ignore_errors=True)
        else:
            from kart_mapping.dense_outputs import DENSE_FILES
            for filename in DENSE_FILES:
                (target / filename).unlink(missing_ok=True)
        env = dict(os.environ,
                   ISAAC_ROS_WS=os.environ.get('ISAAC_ROS_WS', str(studio.repo / 'ros2_ws')),
                   ROS_DOMAIN_ID=str(studio.mapping.defaults['ros_domain_id']),
                   ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        command = ['ros2', 'run', 'isaac_mapping_ros', 'create_map_offline.py',
                   '--map_dir=' + str(target), '--print_mode=all',
                   '--steps_to_run', step]
        if step == 'depth':
            require_engine(resolution, env['ISAAC_ROS_WS'])
            command += ['--depth_model=foundationstereo', '--fs_model_res=' + resolution]
        job.run(command, env=env)
        job.check()
        relative = str(target.relative_to(folder))
        if step == 'depth':
            depth = target / 'map_frames/depth'
            if not depth.is_dir() or not any(p.is_file() and p.stat().st_size for p in depth.rglob('*')):
                raise ValueError('深度画像が生成されませんでした')
            mapping.update(depth_output=relative, fs_model_res=resolution,
                           dense_stale=bool(mapping.get('dense')))
        else:
            mapping.update(dense=collect(target), dense_output=relative, dense_stale=False)
        mapping['steps'] = list(dict.fromkeys([*mapping.get('steps', []), step]))
        if studio.maps.load(key)['revision'] != revision:
            raise ValueError('実行中に地図が更新されました')
        doc.update(mapping=mapping, revision=revision + 1, updated=time.time())
        atomic_json(folder / 'mapping_result.json', mapping)
        published = True
        atomic_json(folder / 'map.json', doc)
        return dict(map=key)
    finally:
        if not published and target.exists():
            shutil.rmtree(target)
