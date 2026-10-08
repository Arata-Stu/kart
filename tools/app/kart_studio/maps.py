"""Map documents, immutable cloud source, generated revisions and exports."""

import os
import shutil
import time
import uuid

from . import footprint, lanes, obstacles
from .exports import export_map
from .lines import generate
from .snapshots import normalize
from .storage import atomic_json, fingerprint, name, read_json, within


class Maps:
    def __init__(self, root):
        self.root = root
        root.mkdir(parents=True, exist_ok=True)

    def folder(self, key):
        return within(self.root, name(key))

    def list(self):
        result = []
        for folder in sorted(self.root.iterdir()):
            if (
                folder.is_symlink()
                or folder.name.startswith(".")
                or not (folder / "map.json").is_file()
            ):
                continue
            try:
                d = self.load(folder.name)
                result.append(
                    dict(
                        id=folder.name,
                        title=d["title"],
                        revision=d["revision"],
                        lines=sorted(
                            {kind for lane in d["lanes"] for kind in lane["lines"]}
                        ),
                        lane_count=len(d["lanes"]),
                        point_count=d["point_count"],
                        updated=d["updated"],
                        has_vslam=(folder / "cuvslam_map").is_dir(),
                        snapshot_status=d.get("snapshot_status", "ready"),
                    )
                )
            except (ValueError, KeyError):
                continue
        return result

    def trash(self, key, revision):
        doc = self.load(key)
        if revision != doc["revision"]:
            raise ValueError(
                "地図が更新されています。再読み込みしてから削除してください"
            )
        root = within(self.root, ".trash", exists=False)
        root.mkdir(exist_ok=True)
        token = name(key) + "--" + uuid.uuid4().hex
        destination = within(root, token, exists=False)
        os.rename(self.folder(key), destination)
        return dict(token=token, title=doc["title"])

    def trashed(self):
        root = within(self.root, ".trash", exists=False)
        if not root.exists():
            return []
        result = []
        for folder in sorted(root.iterdir()):
            if folder.is_symlink() or not folder.is_dir():
                continue
            try:
                original, suffix = folder.name.rsplit("--", 1)
                name(original)
                if len(suffix) != 32 or any(
                    c not in "0123456789abcdef" for c in suffix
                ):
                    continue
                doc = read_json(within(folder, "map.json"))
                result.append(
                    dict(
                        token=folder.name,
                        id=original,
                        title=doc["title"],
                        revision=doc["revision"],
                    )
                )
            except (ValueError, KeyError, OSError):
                continue
        return result

    def restore(self, token):
        entry = next((e for e in self.trashed() if e["token"] == token), None)
        if entry is None:
            raise ValueError("ごみ箱内の地図が見つかりません")
        destination = within(self.root, entry["id"], exists=False)
        if destination.exists():
            raise ValueError("同名の地図があります。上書きせず復元を中止しました")
        os.rename(within(self.root, ".trash/" + token), destination)
        return dict(id=entry["id"])

    def load(self, key):
        return lanes.upgrade(read_json(within(self.folder(key), "map.json")))

    def initialize(self, folder, title):
        cloud = normalize(read_json(folder / "snapshot.json"))
        atomic_json(folder / "cloud.json", cloud)
        doc = dict(
            schema="kart.hdmap.v2",
            title=title,
            frame="map",
            lanes=[lanes.empty()],
            point_count=cloud["source_count"],
            updated=time.time(),
            revision=1,
        )
        atomic_json(folder / "map.json", doc)
        return doc

    def initialize_pending(self, folder, title):
        result = read_json(folder / "mapping_result.json")
        if result["status"] != "vslam_ready":
            raise ValueError("VSLAM地図が完成していません")
        doc = dict(
            schema="kart.hdmap.v2",
            title=title,
            frame="map",
            lanes=[lanes.empty()],
            point_count=0,
            updated=time.time(),
            revision=1,
            snapshot_status="pending",
            mapping=result,
        )
        atomic_json(folder / "map.json", doc)
        return doc

    def require_snapshot(self, key):
        if self.load(key).get("snapshot_status") == "pending":
            raise ValueError(
                "VSLAM地図は作成済みですが、HDMap用点群はまだ取得していません"
            )

    def import_snapshot(self, key, source):
        name(key)
        if (self.root / key).exists():
            raise ValueError("同名の地図があります")
        temp = self.root / (".import-" + uuid.uuid4().hex)
        temp.mkdir()
        try:
            # No arbitrary file read endpoint: only explicit JSON snapshot import.
            source = source.expanduser().resolve(strict=True)
            if source.suffix != ".json" or source.stat().st_size > 256 * 1024 * 1024:
                raise ValueError("256 MiB以下のsnapshot JSONを指定してください")
            shutil.copyfile(source, temp / "snapshot.json")
            doc = self.initialize(temp, key)
            os.rename(temp, self.root / key)
            return doc
        finally:
            if temp.exists():
                shutil.rmtree(temp)

    def save(self, key, body):
        self.require_snapshot(key)
        current = self.load(key)
        if body.get("revision") != current["revision"]:
            raise ValueError("別の保存で地図が更新されています。再読み込みしてください")
        if "lanes" not in body:
            raise ValueError(
                "lane形式が更新されています。ブラウザを再読み込みしてください"
            )
        regions = obstacles.validate(
            body.get("obstacles", current.get("obstacles", []))
        )
        if regions != current.get("obstacles", []):
            for lane in current["lanes"]:
                lane["lines"] = {}
        current.update(
            obstacles=regions,
            lanes=lanes.validate(body["lanes"], current["lanes"]),
            revision=current["revision"] + 1,
            updated=time.time(),
        )
        atomic_json(self.folder(key) / "map.json", current)
        return current

    def generate(self, key, kind, parameters, revision, check, lane_id=None):
        self.require_snapshot(key)
        document = self.load(key)
        if revision != document["revision"]:
            raise ValueError("編集内容を保存してから生成してください")
        lane = lanes.select(document, lane_id)
        result = generate(lane, kind, parameters, check)
        check()
        options = result["settings"]
        footprint.validate_line(
            result["points"],
            lane["closed"],
            options,
            obstacles=document.get("obstacles", []),
            yaws=[r[3] for r in result["profile"]] if "profile" in result else None,
            check=check,
        )
        result["source_fingerprint"] = fingerprint(
            {
                k: lane.get(k, [])
                for k in ("left", "right", "closed", "custom", "custom_speeds")
            }
        )
        lane["lines"][kind] = result
        document.update(revision=document["revision"] + 1, updated=time.time())
        atomic_json(self.folder(key) / "map.json", document)
        return dict(map=key, lane_id=lane["id"], revision=document["revision"])

    def export(self, key):
        self.require_snapshot(key)
        return export_map(self.folder(key), self.load(key))
