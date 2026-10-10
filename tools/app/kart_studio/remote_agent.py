"""Fixed remote filesystem protocol executed via SSH, never a remote bringup."""

import hashlib
import json
import os
import re
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


def signature(path):
    files = []
    for current, dirs, names in os.walk(path, followlinks=False):
        for entry in dirs + names:
            item = Path(current) / entry
            if item.is_symlink():
                raise ValueError("bag contains a symlink")
            if item.is_file():
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
