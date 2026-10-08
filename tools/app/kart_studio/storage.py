"""Confined paths, atomic JSON and optimistic revisions."""

import hashlib
import json
import os
import re
import tempfile
from pathlib import Path


def name(value):
    if not isinstance(value, str) or not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", value
    ):
        raise ValueError("名前は英数字で始まる英数字・_・-（64文字以内）にしてください")
    return value


def within(root, relative, *, exists=True):
    root = Path(root).resolve()
    value = Path(str(relative))
    if value.is_absolute() or ".." in value.parts:
        raise ValueError("相対パスを指定してください")
    path = root / value
    for parent in [path, *path.parents]:
        if parent == root:
            break
        if parent.is_symlink():
            raise ValueError("シンボリックリンクは扱いません")
    if not path.resolve().is_relative_to(root):
        raise ValueError("指定範囲外のパスです")
    if exists and not path.exists():
        raise ValueError("ファイルが見つかりません")
    return path


def read_json(path):
    return json.loads(Path(path).read_text())


def atomic_json(path, data):
    path = Path(path)
    if path.is_symlink():
        raise ValueError("リンクへの保存はできません")
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix="." + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(
                data, stream, ensure_ascii=False, allow_nan=False, separators=(",", ":")
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        Path(tmp).unlink(missing_ok=True)


def fingerprint(data):
    return hashlib.sha256(
        json.dumps(data, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()[:20]
