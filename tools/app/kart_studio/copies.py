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
    stage = maps.root.resolve() / (".copy-" + uuid.uuid4().hex)
    stage.mkdir()
    links = []

    def copy_entry(src, dst):
        job.check()
        mode = src.lstat().st_mode
        if stat.S_ISLNK(mode):
            relative = src.relative_to(folder)
            try:
                resolved = src.resolve(strict=True)
            except (OSError, RuntimeError) as exc:
                raise ValueError(f"リンク先が存在しないか循環しています: {relative}") from exc
            if not resolved.is_relative_to(folder):
                raise ValueError(f"地図の外部を指すリンクは複製できません: {relative}")
            target_relative = resolved.relative_to(folder)
            if target_relative.parts and target_relative.parts[0] == "exports":
                raise ValueError(f"再生成するexportsを指すリンクは複製できません: {relative}")
            if resolved in src.parents:
                raise ValueError(f"親ディレクトリへ循環するリンクは複製できません: {relative}")
            if not (resolved.is_file() or resolved.is_dir()):
                raise ValueError(f"リンク先が特殊ファイルのため複製できません: {relative}")
            # Preserve aliases without traversing/duplicating their contents.
            # Absolute source links must never point back to the original map.
            dst.symlink_to(os.path.relpath(stage / target_relative, dst.parent),
                           target_is_directory=resolved.is_dir())
            links.append(dst)
        elif stat.S_ISDIR(mode):
            dst.mkdir()
            for child in sorted(src.iterdir()):
                copy_entry(child, dst / child.name)
        elif stat.S_ISREG(mode):
            with src.open("rb") as reader, dst.open("xb") as writer:
                while chunk := reader.read(1024 * 1024):
                    job.check()
                    writer.write(chunk)
        else:
            raise ValueError(f"特殊ファイルは複製できません: {src.relative_to(folder)}")

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
        for link in links:
            job.check()
            if not link.resolve(strict=True).is_relative_to(stage):
                raise ValueError(f"複製先のリンクが地図の外部を指しています: {link.relative_to(stage)}")
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
