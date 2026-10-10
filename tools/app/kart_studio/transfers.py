"""SSH browsing + SCP staging, with no vehicle runtime commands."""

import json
import os
import re
import shlex
import shutil
import subprocess
import tempfile
import uuid
from pathlib import Path

from .transfer_progress import run_transfer
from .remote_agent import signature
from .storage import atomic_json, name, within

SSH_OPTIONS = [
    "-o",
    "BatchMode=yes",
    "-o",
    "StrictHostKeyChecking=yes",
    "-o",
    "ConnectTimeout=8",
    "-o",
    "ServerAliveInterval=10",
    "-o",
    "ServerAliveCountMax=3",
]
AGENT = Path(__file__).with_name("remote_agent.py").read_text()


def profile(raw):
    user, host = str(raw.get("user", "")), str(raw.get("host", ""))
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]{0,63}", user):
        raise ValueError("SSHユーザー名が不正です")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.-]{0,252}", host):
        raise ValueError("ホスト名またはIPv4を指定してください")
    port = int(raw.get("port", 22))
    if not 1 <= port <= 65535:
        raise ValueError("SSH portが不正です")
    roots = {k: remote_path(raw.get(k, "")) for k in ("record_root", "map_root")}
    return dict(user=user, host=host, port=port, **roots)


def remote_path(value):
    value = str(value)
    if not re.fullmatch(r"/[A-Za-z0-9_./-]+", value) or ".." in Path(value).parts:
        raise ValueError(
            "リモートパスは絶対パス（英数字・/・_・.・-）を指定してください"
        )
    return value.rstrip("/")


def relative_path(value):
    value = str(value)
    if value and (
        not re.fullmatch(r"[A-Za-z0-9_./-]+", value)
        or value.startswith("/")
        or ".." in Path(value).parts
    ):
        raise ValueError("この名前の転送には未対応です（英数字・_・.・-を使用）")
    return value


def request(p, action, root, relative="", **extra):
    data = dict(action=action, root=root, relative=relative_path(relative), **extra)
    result = subprocess.run(
        [
            "ssh",
            *SSH_OPTIONS,
            "-p",
            str(p["port"]),
            f"{p['user']}@{p['host']}",
            "python3 -c " + shlex.quote(AGENT),
        ],
        input=json.dumps(data),
        text=True,
        capture_output=True,
        timeout=30,
    )
    try:
        response = json.loads(result.stdout)
    except ValueError:
        raise ValueError("SSH応答がありません: " + result.stderr[-1000:])
    if result.returncode or response.get("error"):
        raise ValueError(response.get("error") or result.stderr[-1000:])
    return response


class Transfers:
    def __init__(self, records, maps):
        self.records, self.maps = records, maps
        records.mkdir(parents=True, exist_ok=True)

    def browse(self, raw, relative):
        p = profile(raw)
        return request(p, "browse", p["record_root"], relative)

    def pull(self, job, raw, relative, key):
        p = profile(raw)
        relative = relative_path(relative)
        name(key)
        target = within(self.records, key, exists=False)
        if target.exists():
            raise ValueError("同名のローカル記録があります")
        before = request(p, "stat", p["record_root"], relative)
        stage = Path(tempfile.mkdtemp(prefix=".pull-", dir=self.records))
        job.log(f"受信予定 {before['bytes']:,} bytes / 一時保存 {stage}")
        try:
            source = f"{p['user']}@{p['host']}:{p['record_root']}/{relative}"
            run_transfer(
                job,
                [
                    "scp",
                    *SSH_OPTIONS,
                    "-P",
                    str(p["port"]),
                    "-r",
                    source,
                    str(stage / "bag"),
                ],
                before["bytes"],
                lambda: sum(
                    p.stat().st_size
                    for p in (stage / "bag").rglob("*")
                    if p.is_file() and not p.is_symlink()
                ),
            )
            after = request(p, "stat", p["record_root"], relative)
            if before != after:
                raise ValueError(
                    "転送中にbagが更新されました。録画停止後に再取得してください"
                )
            local = signature(stage / "bag")
            if local["bytes"] != before["bytes"] or local["files"] != before["files"]:
                raise ValueError("受信ファイルのサイズまたは件数が一致しません")
            if not (stage / "bag" / "metadata.yaml").is_file():
                raise ValueError("metadata.yamlがありません")
            job.check()
            os.rename(stage / "bag", target)
            return dict(record=key, bytes=before["bytes"])
        finally:
            shutil.rmtree(stage)

    def push(self, job, raw, key):
        p = profile(raw)
        exported = self.maps.export(key)
        source = self.maps.folder(key)
        stage_name = ".kart-upload-" + uuid.uuid4().hex
        destination = f"{key}_r{self.maps.load(key)['revision']}_{uuid.uuid4().hex[:8]}"
        with tempfile.TemporaryDirectory(prefix="kart-map-") as temp:
            bundle = Path(temp) / "bundle"
            bundle.mkdir()
            shutil.copytree(exported, bundle / "export")
            atomic_json(bundle / "map.json", self.maps.load(key))
            for filename in ("snapshot.json", "cloud.json"):
                shutil.copyfile(within(source, filename), bundle / filename)
            if (source / "cuvslam_map").is_dir():
                signature(source / "cuvslam_map")  # Refuse links before copy.
                shutil.copytree(source / "cuvslam_map", bundle / "cuvslam_map")
            expected = signature(bundle)
            job.check()
            request(p, "reserve", p["map_root"], stage_name)
            job.log(f"送信先: {p['map_root']}/{destination} / 一時領域: {stage_name}")
            run_transfer(
                job,
                [
                    "scp",
                    *SSH_OPTIONS,
                    "-P",
                    str(p["port"]),
                    "-r",
                    str(bundle),
                    f"{p['user']}@{p['host']}:{p['map_root']}/{stage_name}/",
                ],
                expected["bytes"],
                lambda: request(p, "progress", p["map_root"], stage_name)["bytes"],
            )
            job.check()
            return request(
                p,
                "publish",
                p["map_root"],
                stage_name,
                destination=destination,
                bytes=expected["bytes"],
                files=expected["files"],
            )
