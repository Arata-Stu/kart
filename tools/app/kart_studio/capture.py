"""Acquire a snapshot for pending maps without changing the saved VSLAM map."""

import hashlib
import os
import shutil
import time
import uuid

from .snapshots import normalize
from .storage import atomic_json, read_json, within


def validate(studio, key, body):
    doc = studio.maps.load(key)
    if doc.get("snapshot_status") != "pending":
        raise ValueError(
            "既に点群があります。HDMapの座標を保護するため再取得は行いません"
        )
    if doc["revision"] != body["revision"]:
        raise ValueError("地図が更新されています。再読み込みしてください")
    if not shutil.which("ros2"):
        raise ValueError("点群取得にはCUDA対応LinuxのROS環境が必要です")
    folder = studio.maps.folder(key)
    origin = read_json(within(folder, "job.json"))
    bag = within(studio.records, body["bag"])
    metadata = within(bag, "metadata.yaml")
    expected = doc["mapping"].get("bag_metadata_sha256")
    if not expected or hashlib.sha256(metadata.read_bytes()).hexdigest() != expected:
        raise ValueError("地図生成時と同じbagを指定してください（metadata不一致）")
    map_dir = within(folder, "cuvslam_map")
    if not any(p.is_file() and p.stat().st_size for p in map_dir.glob("*.mdb")):
        raise ValueError("cuVSLAM保存地図がありません")
    options = read_json(
        studio.repo / "ros2_ws/src/kart_bringup/config/mapping/capture.json"
    )
    return bag, origin["workflow"], options


def capture(studio, job, key, revision, bag, workflow, options):
    maps = studio.maps
    doc = maps.load(key)
    if doc["revision"] != revision or doc.get("snapshot_status") != "pending":
        raise ValueError("対象地図の状態が変わりました")
    folder = maps.folder(key)
    stage = maps.root / (".capture-" + uuid.uuid4().hex)
    stage.mkdir()
    try:
        source = within(folder, "cuvslam_map")
        if any(p.is_symlink() for p in source.rglob("*")):
            raise ValueError("VSLAM地図内のリンクは扱いません")
        job.log("元地図を保護するため作業用VSLAM地図を複製しています")

        def copy_file(src, dst):
            with open(src, "rb") as reader, open(dst, "wb") as writer:
                while chunk := reader.read(1024 * 1024):
                    job.check()
                    writer.write(chunk)
            return dst

        shutil.copytree(source, stage / "cuvslam_map", copy_function=copy_file)
        job_file = stage / "capture_job.json"
        atomic_json(
            job_file,
            {
                "operation": "capture",
                "map_id": key,
                "map_dir": str(stage / "cuvslam_map"),
                "bag": str(bag),
                "output": str(stage),
                "workflow": workflow,
                "capture": options,
            },
        )
        job.run(
            ["ros2", "run", "kart_mapping", "capture_snapshot", "--job", str(job_file)],
            env=dict(
                os.environ,
                ROS_DOMAIN_ID=str(workflow["ros_domain_id"]),
                ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
            ),
        )
        snapshot = read_json(stage / "snapshot.json")
        if snapshot.get("provenance", {}).get("localized_in_existing_map") is not True:
            raise ValueError("保存地図へのlocalize成功記録がありません")
        cloud = normalize(snapshot)
        if maps.load(key)["revision"] != revision:
            raise ValueError("点群取得中に地図が更新されました")
        job.check()
        # map.json is the commit record. Pending readers ignore partial source files.
        atomic_json(folder / "snapshot.json", snapshot)
        atomic_json(folder / "cloud.json", cloud)
        doc.update(
            snapshot_status="ready",
            point_count=cloud["source_count"],
            revision=revision + 1,
            updated=time.time(),
        )
        doc["mapping"]["snapshot_status"] = "ready"
        atomic_json(folder / "mapping_result.json", doc["mapping"])
        atomic_json(folder / "map.json", doc)
        return {"map": key, "revision": doc["revision"]}
    finally:
        shutil.rmtree(stage)
