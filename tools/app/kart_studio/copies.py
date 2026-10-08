"""Independent map variants without replaying VSLAM."""

import os
import shutil
import stat
import time
import uuid

from .storage import atomic_json, fingerprint, name, within


def copy_map(maps, job, source, destination, revision):
    name(destination)
    folder = maps.folder(source)
    target = within(maps.root, destination, exists=False)
    if target.exists():
        raise ValueError("同名の地図があります。別の名前を指定してください")
    document = maps.load(source)
    if document["revision"] != revision:
        raise ValueError("元地図が更新されています。再読み込みしてください")
    # Both are required for an independently editable, transferable map.
    if document.get("snapshot_status") != "pending":
        within(folder, "snapshot.json")
        within(folder, "cloud.json")
    stage = maps.root / (".copy-" + uuid.uuid4().hex)
    stage.mkdir()

    def copy_entry(src, dst):
        job.check()
        mode = src.lstat().st_mode
        if stat.S_ISDIR(mode):
            dst.mkdir()
            for child in sorted(src.iterdir()):
                copy_entry(child, dst / child.name)
        elif stat.S_ISREG(mode):
            with src.open("rb") as reader, dst.open("xb") as writer:
                while chunk := reader.read(1024 * 1024):
                    job.check()
                    writer.write(chunk)
        else:
            raise ValueError("リンク・特殊ファイルを含む地図は複製できません")

    try:
        for entry in sorted(folder.iterdir()):
            # Exports are derived caches; rebuild against the new revision.
            if entry.name in ("map.json", "exports"):
                continue
            job.log("複製: " + entry.name)
            copy_entry(entry, stage / entry.name)
        document["copied_from"] = {
            "map": source,
            "revision": revision,
            "document_fingerprint": fingerprint(document),
            "timestamp": time.time(),
        }
        document.update(title=destination, revision=1, updated=time.time())
        atomic_json(stage / "map.json", document)
        if maps.load(source)["revision"] != revision:
            raise ValueError("複製中に元地図が更新されました")
        job.check()
        if target.exists():
            raise ValueError("同名の地図があります")
        os.rename(stage, target)
        return {"map": destination, "copied_from": source}
    finally:
        if stage.exists():
            shutil.rmtree(stage)
