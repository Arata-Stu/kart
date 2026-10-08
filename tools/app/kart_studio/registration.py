"""Preview and apply planar HDMap registration without touching VSLAM data."""

import copy
import math
import time
import uuid

from .geometry import number
from .storage import atomic_json, fingerprint, within


def preview(maps, target, body):
    source = body["source"]
    if source == target:
        raise ValueError("元地図と適用先は別の地図を指定してください")
    src, dst = maps.load(source), maps.load(target)
    if not any(
        len(lane["left"]) >= 2 and len(lane["right"]) >= 2 for lane in src["lanes"]
    ):
        raise ValueError("元地図に左右境界がありません")
    if body["revision"] != dst["revision"]:
        raise ValueError("適用先が更新されています。再読み込みしてください")
    transform = {
        k: number(body[k], -limit, limit, k)
        for k, limit in [("x", 1e5), ("y", 1e5), ("yaw", 360)]
    }
    angle = math.radians(transform["yaw"])
    c, s = math.cos(angle), math.sin(angle)

    def xy(p):
        return [
            c * p[0] - s * p[1] + transform["x"],
            s * p[0] + c * p[1] + transform["y"],
        ]

    result = copy.deepcopy(dst)
    result["lanes"] = copy.deepcopy(src["lanes"])
    result["obstacles"] = [
        dict(id=o["id"], polygon=[xy(p) for p in o["polygon"]])
        for o in src.get("obstacles", [])
    ]
    for lane in result["lanes"]:
        for key in ("left", "right", "custom"):
            lane[key] = [xy(p) for p in lane[key]]
        for line in lane["lines"].values():
            line["points"] = [xy(p) for p in line["points"]]
            for row in line.get("profile", []):
                row[1:3] = xy(row[1:3])
                row[3] = math.atan2(math.sin(row[3] + angle), math.cos(row[3] + angle))
            line["source_fingerprint"] = fingerprint(
                {
                    k: lane.get(k, [])
                    for k in ("left", "right", "closed", "custom", "custom_speeds")
                }
            )
    provenance = dict(
        source=source, source_revision=src["revision"], transform=transform
    )
    token = fingerprint(dict(source=src, target=dst, transform=transform))
    return dict(document=result, token=token, provenance=provenance)


def apply(maps, target, body):
    proposal = preview(maps, target, body)
    if body.get("token") != proposal["token"]:
        raise ValueError(
            "元地図・適用先・変換が変更されています。再プレビューしてください"
        )
    folder = maps.folder(target)
    backup = "registration-backups/" + uuid.uuid4().hex + ".json"
    atomic_json(within(folder, backup, exists=False), maps.load(target))
    result = proposal["document"]
    result.update(revision=result["revision"] + 1, updated=time.time())
    result["registration"] = dict(
        **proposal["provenance"], backup=backup, applied=time.time()
    )
    atomic_json(folder / "map.json", result)
    return result
