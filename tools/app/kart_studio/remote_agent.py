"""Fixed remote filesystem protocol executed via SSH, never a remote bringup."""

import hashlib
import json
import os
import re
import shutil
import sys
from pathlib import Path


def confined(root, relative):
    root = Path(root)
    if not root.is_absolute() or root.is_symlink():
        raise ValueError("root must be an absolute directory, not a symlink")
    if ".." in Path(relative).parts or Path(relative).is_absolute():
        raise ValueError("invalid relative path")
    path = root / relative
    for parent in [path, *path.parents]:
        if parent == root:
            break
        if parent.is_symlink():
            raise ValueError("symlink refused")
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError("path outside root")
    return path


def signature(path, *, ignore_raw_indexes=False):
    files = []
    for current, dirs, names in os.walk(path, followlinks=False):
        for entry in dirs + names:
            item = Path(current) / entry
            if item.is_symlink():
                raise ValueError("bag contains a symlink")
            if item.is_file():
                if (ignore_raw_indexes and item.name.lower().endswith('.raw.tmp_index')
                        and item.with_suffix('').is_file()):
                    continue
                stat = item.stat()
                files.append(
                    (str(item.relative_to(path)), stat.st_size, stat.st_mtime_ns)
                )
        if len(files) > 20000:
            raise ValueError("too many files")
    return dict(
        bytes=sum(f[1] for f in files),
        files=len(files),
        signature=hashlib.sha256(json.dumps(sorted(files)).encode()).hexdigest(),
    )


def model_manifest(path):
    expected = {"model.onnx", "metadata.json"}
    if {p.name for p in path.iterdir()} != expected:
        raise ValueError("model bundle must contain ONNX and metadata only")
    result = {}
    for filename in sorted(expected):
        target = path / filename
        if target.is_symlink() or not target.is_file():
            raise ValueError("model file missing or symlink")
        digest = hashlib.sha256()
        with target.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        result[filename] = digest.hexdigest()
    metadata = json.loads((path / "metadata.json").read_text())
    if metadata.get("onnx_sha256") != result["model.onnx"]:
        raise ValueError("model checksum mismatch")
    return result


def perception_contract(class_names):
    """Supported exported classes, in numeric class-ID order; stdlib only."""
    if class_names == ['rc_car']:
        profile = 'opponent'
    elif class_names == ['rc_car', 'obstacle']:
        profile = 'racing'
    else:
        raise ValueError('perception class names/order must be rc_car or rc_car, obstacle')
    return dict(model_profile=profile, num_classes=len(class_names), class_names=list(class_names))


def perception_manifest(path, transfer_required=True):
    """Verify the RC-car export contract using only the remote Python stdlib."""
    expected = {'opponent.onnx', 'report.json', 'artifact.json'}
    if transfer_required:
        expected.add('transfer.json')
    if path.is_symlink() or not path.is_dir():
        raise ValueError('perception bundle contains unexpected or missing files')
    entries = {item.name for item in path.iterdir()}
    caches = set() if transfer_required else {'.ultralytics', '.matplotlib'}
    if not expected <= entries or entries - expected - caches:
        raise ValueError('perception bundle contains unexpected or missing files')
    # The local export runtime owns these cache directories. They are not model
    # payloads and are never copied; reject links even inside a known cache.
    for directory in entries & caches:
        cache = path / directory
        if cache.is_symlink() or not cache.is_dir():
            raise ValueError('invalid perception runtime cache directory')
        signature(cache)
    hashes = {}
    for filename in sorted(expected):
        target = path / filename
        if target.is_symlink() or not target.is_file() or target.stat().st_size <= 0:
            raise ValueError('perception file missing, empty or symlink')
        if filename.endswith('.json') and target.stat().st_size > 4 * 1024 * 1024:
            raise ValueError('perception metadata is too large')
        digest = hashlib.sha256()
        with target.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        hashes[filename] = digest.hexdigest()
    report = json.loads((path / 'report.json').read_text())
    artifact = json.loads((path / 'artifact.json').read_text())
    if not isinstance(report, dict) or not isinstance(artifact, dict):
        raise ValueError('invalid perception metadata')
    contract = perception_contract(report.get('class_names'))
    if (report.get('schema') != 'kart.perception.export.v1' or report.get('model_profile') != contract['model_profile'] or
            type(report.get('num_classes')) is not int or
            report.get('num_classes') != contract['num_classes'] or
            report.get('onnx_sha256') != hashes['opponent.onnx']):
        raise ValueError('perception ONNX checksum or class contract mismatch')
    if (report.get('input') != dict(name='images', shape=[1, 3, 480, 640], dtype='float32') or
            report.get('output') != dict(name='output0', shape=[1, 4 + contract['num_classes'], 6300], dtype='float32')):
        raise ValueError('perception ONNX binding/shape contract mismatch')
    if any(type(value) is not int for binding in ('input', 'output') for value in report[binding]['shape']):
        raise ValueError('perception ONNX shape must contain integers')
    if (artifact.get('status') != 'succeeded' or artifact.get('model_profile') != contract['model_profile'] or
            type(artifact.get('num_classes')) is not int or
            artifact.get('num_classes') != contract['num_classes'] or artifact.get('class_names') != contract['class_names'] or
            not isinstance(artifact.get('id'), str) or
            not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', artifact['id'])):
        raise ValueError('perception export is incomplete or has invalid class metadata')
    if transfer_required:
        transfer = json.loads((path / 'transfer.json').read_text())
        payload = {key: value for key, value in hashes.items() if key != 'transfer.json'}
        if (not isinstance(transfer, dict) or transfer.get('schema') != 'kart.perception.transfer.v1' or
                transfer.get('model') != artifact['id'] or transfer.get('payload_sha256') != payload or
                transfer.get('source_sha256') != hashes['opponent.onnx']):
            raise ValueError('perception transfer manifest mismatch')
        # Existing one-class bundles predate these fields. They remain valid;
        # every new/two-class bundle must agree on the explicit class contract.
        for key, expected_value in contract.items():
            actual = transfer.get(key, expected_value if contract['model_profile'] == 'opponent' else None)
            if actual != expected_value or (key == 'num_classes' and type(actual) is not int):
                raise ValueError('perception transfer class/profile mismatch')
    return hashes


def execute(data):
    root, relative = data["root"], data.get("relative", "")
    path = confined(root, relative)
    action = data["action"]
    if action == "browse":
        if not path.is_dir():
            raise ValueError("record directory not found")
        result = []
        for item in sorted(path.iterdir()):
            if item.is_symlink() or not item.is_dir() or item.name.startswith("."):
                continue
            if len(result) >= 1000:
                raise ValueError("too many directories; select a narrower record root")
            result.append(
                dict(
                    name=item.name,
                    relative=str(item.relative_to(root)),
                    bag=(item / "metadata.yaml").is_file(),
                    modified=item.stat().st_mtime,
                )
            )
        return dict(relative=relative, entries=result)
    if action == "stat":
        if not (path / "metadata.yaml").is_file():
            raise ValueError("completed bag metadata.yaml not found")
        return signature(path)
    if action == "progress":
        if not re.fullmatch(r"\.kart-upload-[a-f0-9]{32}", relative):
            raise ValueError("invalid staging name")
        return signature(path)
    if action == "reserve":
        if not re.fullmatch(r"\.kart-upload-[a-f0-9]{32}", relative):
            raise ValueError("invalid staging name")
        if not Path(root).is_dir():
            raise ValueError("map root does not exist")
        path.mkdir()
        return dict(stage=relative)
    if action == 'reserve-perception':
        if not re.fullmatch(r'\.kart-upload-[a-f0-9]{32}', relative):
            raise ValueError('invalid staging name')
        Path(root).mkdir(parents=True, exist_ok=True)
        path = confined(root, relative)
        path.mkdir(mode=0o700)
        return dict(stage=relative)
    if action == 'discard-perception':
        if not re.fullmatch(r'\.kart-upload-[a-f0-9]{32}', relative):
            raise ValueError('invalid staging name')
        if path.exists():
            if not path.is_dir():
                raise ValueError('invalid staging directory')
            shutil.rmtree(path)
        return dict(discarded=relative)
    if action == 'publish-perception':
        if not re.fullmatch(r'\.kart-upload-[a-f0-9]{32}', relative):
            raise ValueError('invalid staging name')
        destination = data['destination']
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', destination):
            raise ValueError('invalid destination')
        target = confined(root, destination)
        source = confined(root, relative + '/bundle')
        if target.exists() or {item.name for item in path.iterdir()} != {'bundle'}:
            raise ValueError('destination exists or staging contains unexpected files')
        hashes = perception_manifest(source)
        if hashes != data['hashes']:
            raise ValueError('perception payload SHA-256 mismatch')
        os.rename(source, target)
        try:
            path.rmdir()
        except BaseException:
            # A leftover staging directory is not a completed publication.
            os.rename(target, source)
            raise
        return dict(path=str(target), model=destination, published=True, hashes=hashes)
    if action == "publish-model":
        if not re.fullmatch(r"\.kart-upload-[a-f0-9]{32}", relative):
            raise ValueError("invalid staging name")
        destination = data["destination"]
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,120}", destination):
            raise ValueError("invalid destination")
        target = confined(root, destination)
        source = confined(root, relative + "/bundle")
        if target.exists() or model_manifest(source) != data["hashes"]:
            raise ValueError("destination exists or model hashes differ")
        os.rename(source, target)
        path.rmdir()
        return dict(path=str(target), model=destination)
    if action == "publish":
        if not re.fullmatch(r"\.kart-upload-[a-f0-9]{32}", relative):
            raise ValueError("invalid staging name")
        destination = data["destination"]
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,120}", destination):
            raise ValueError("invalid destination")
        target = confined(root, destination)
        source = confined(root, relative + "/bundle")
        if target.exists() or not (source / "export" / "metadata.json").is_file():
            raise ValueError("destination exists or incomplete bundle")
        actual = signature(source)
        if actual["bytes"] != data["bytes"] or actual["files"] != data["files"]:
            raise ValueError("transferred file size/count mismatch")
        os.rename(source, target)
        path.rmdir()
        return dict(path=str(target))
    raise ValueError("unknown action")


if __name__ == "__main__":
    try:
        result = execute(json.load(sys.stdin))
        print(json.dumps(result))
    except Exception as exc:
        print(json.dumps(dict(error=str(exc))))
        sys.exit(1)
