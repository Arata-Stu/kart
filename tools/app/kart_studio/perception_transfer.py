"""Copy verified RC-car ONNX bundles over SSH/SCP; never build or run them."""
import json
from pathlib import Path
import re
import shlex
import shutil
import tempfile
import time
import uuid

from . import transfers
from .remote_agent import perception_manifest
from .storage import atomic_json, name, within


def _root(value):
    result = transfers.remote_path(value)
    if not result or not Path(result).is_absolute() or str(Path(result)) == '/':
        raise ValueError('モデル保存rootはルートdirectory以外の絶対パスを指定してください')
    return str(Path(result))


class PerceptionTransfer:
    def __init__(self, repo, root):
        self.repo, self.root = Path(repo).resolve(), Path(root).resolve()

    def _source(self, model_id):
        source = within(self.root, 'models/' + name(model_id))
        hashes = perception_manifest(source, transfer_required=False)
        artifact = json.loads((source / 'artifact.json').read_text())
        if artifact['id'] != model_id:
            raise ValueError('モデルIDと成果物metadataが一致しません')
        return source, hashes

    def prepare(self, body):
        model_id = name(body.get('id'))
        connection = transfers.profile(body.get('connection', {}))
        model_root = _root(body.get('model_root', ''))
        container = body.get('container_model_root', '')
        container_root = _root(container) if container else None
        _source, hashes = self._source(model_id)
        return dict(id=model_id, connection=connection, model_root=model_root,
                    container_model_root=container_root, source_hashes=hashes,
                    source_sha256=hashes['opponent.onnx'],
                    destination=name(model_id[:54] + '_' + uuid.uuid4().hex[:8]),
                    stage='.kart-upload-' + uuid.uuid4().hex)

    def push(self, job, plan):
        connection = transfers.profile(plan['connection'])
        root = _root(plan['model_root'])
        destination, stage = name(plan['destination']), plan['stage']
        if not isinstance(stage, str) or not re.fullmatch(r'\.kart-upload-[a-f0-9]{32}', stage):
            raise ValueError('転送の一時領域が不正です')
        source, source_hashes = self._source(plan['id'])
        if source_hashes != plan['source_hashes'] or source_hashes['opponent.onnx'] != plan['source_sha256']:
            raise ValueError('送信予約後にモデルが変更されました。送信し直してください')
        container_root = plan.get('container_model_root')
        if container_root:
            container_root = _root(container_root)
        host_path = root + '/' + destination
        container_path = container_root + '/' + destination if container_root else None
        command = ('bash scripts/racing_trt.sh --model opponent --model-root ' + shlex.quote(container_path)
                   if container_path else None)
        reserved = published = attempted = False
        with tempfile.TemporaryDirectory(prefix='kart-perception-') as temporary:
            bundle = Path(temporary) / 'bundle'
            bundle.mkdir()
            for filename in source_hashes:
                job.check()
                shutil.copyfile(within(source, filename), bundle / filename)
            atomic_json(bundle / 'transfer.json', dict(schema='kart.perception.transfer.v1',
                model=plan['id'], created=time.time(), source_sha256=plan['source_sha256'],
                payload_sha256=source_hashes, engine_included=False))
            hashes = perception_manifest(bundle)
            if {key: value for key, value in hashes.items() if key != 'transfer.json'} != source_hashes:
                raise ValueError('コピー中にモデルが変更されました。送信し直してください')
            job.check()
            try:
                transfers.request(connection, 'reserve-perception', root, stage)
                reserved = True
                job.log(f'ONNX送信先（SSHホスト上）: {host_path} / 一時領域: {stage}')
                transfers.run_transfer(job,
                    ['scp', *transfers.SSH_OPTIONS, '-P', str(connection['port']), '-r', str(bundle),
                     f"{connection['user']}@{connection['host']}:{root}/{stage}/"],
                    sum((bundle / filename).stat().st_size for filename in hashes),
                    lambda: transfers.request(connection, 'progress', root, stage)['bytes'])
                job.check()
                attempted = True
                result = transfers.request(connection, 'publish-perception', root, stage,
                                           destination=destination, hashes=hashes)
                if (result.get('published') is not True or result.get('model') != destination or
                        result.get('path') != host_path or result.get('hashes') != hashes):
                    raise ValueError('公開結果または送信後のSHA-256を確認できません')
                published = True
            except BaseException:
                if reserved and not published:
                    try:
                        transfers.request(connection, 'discard-perception', root, stage)
                    except Exception as exc:
                        job.log(f'一時領域の削除は未確認です: {root}/{stage}: {exc}')
                if attempted:
                    job.log(f'公開結果を確認してください: {host_path}（適用・ビルドは実行していません）')
                raise
        job.log('SHA-256検証後に新規モデルdirectoryへ公開しました。engineビルドは未実行です')
        return dict(model=plan['id'], published=destination, host_path=host_path, remote_path=host_path,
                    container_path=container_path, source_sha256=plan['source_sha256'], hashes=hashes,
                    build_command=command, engine_built=False)
