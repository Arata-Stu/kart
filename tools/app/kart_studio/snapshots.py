"""Import Kart snapshots and JetPilot reference snapshots without ROS imports."""

import base64
import math
import struct

from kart_mapping.frames import FinalFrames


def decode_cloud(cloud):
    raw = base64.b64decode(cloud["data"], validate=True)
    fields = {f["name"]: f for f in cloud["fields"]}
    formats = {7: "f", 8: "d"}
    unpack = []
    for key in ("x", "y", "z"):
        f = fields[key]
        if f["datatype"] not in formats or f.get("count", 1) != 1:
            raise ValueError("XYZはfloat32またはfloat64である必要があります")
        fmt = (">" if cloud.get("is_bigendian") else "<") + formats[f["datatype"]]
        unpack.append((int(f["offset"]), struct.Struct(fmt)))
    width, height, step, row = map(
        int, (cloud["width"], cloud["height"], cloud["point_step"], cloud["row_step"])
    )
    if (
        min(width, height, step, row) <= 0
        or width * height > 5_000_000
        or row < width * step
        or len(raw) < height * row
    ):
        raise ValueError("点群のサイズが不正です")
    if any(offset < 0 or offset + fmt.size > step for offset, fmt in unpack):
        raise ValueError("点群のfieldが不正です")
    result = []
    for y in range(height):
        for x in range(width):
            p = [
                fmt.unpack_from(raw, y * row + x * step + offset)[0]
                for offset, fmt in unpack
            ]
            if all(math.isfinite(v) for v in p):
                result.append(p)
    return result


def normalize(snapshot):
    if snapshot.get("schema") == "kart.snapshot.v1":
        if snapshot.get("frame") != "map":
            raise ValueError("Kart snapshotはmap座標が必要です")
        pts = snapshot["points"]
        trajectory = snapshot.get("trajectory", [])
        provenance = snapshot.get("provenance", {})
    else:
        cloud = snapshot.get("landmarks")
        if not cloud:
            raise ValueError("landmarks点群がありません")
        frames = FinalFrames()
        for child, tf in (
            snapshot.get("localization", {}).get("map_from_frame", {}).items()
        ):
            frames.update(
                dict(
                    parent="map",
                    child=child,
                    stamp_ns=str(tf.get("stamp_ns", "0")),
                    translation=[tf["translation"][k] for k in ("x", "y", "z")],
                    rotation=[tf["rotation"][k] for k in ("x", "y", "z", "w")],
                )
            )
        pts, tf = frames.convert(decode_cloud(cloud), cloud["header"]["frame_id"])
        path = snapshot.get("path") or {}
        raw = [
            [p["position"][k] for k in ("x", "y", "z")] for p in path.get("poses", [])
        ]
        trajectory, _ = (
            frames.convert(raw, path.get("frame_id", "map")) if raw else ([], None)
        )
        provenance = dict(
            source="JetPilot reference snapshot",
            applied_transform=tf,
            warning="旧snapshotにはTF時刻がない場合があります。最終時刻は元ログで確認してください。",
        )
    if not isinstance(pts, list) or not pts or len(pts) > 5_000_000:
        raise ValueError("点群は1〜500万点が必要です")
    for group in (pts, trajectory):
        for p in group:
            if len(p) != 3 or any(
                not math.isfinite(float(v)) or abs(float(v)) > 1e6 for v in p
            ):
                raise ValueError("不正なXYZ座標です")
    # The immutable source snapshot remains full-resolution.
    stride = max(1, math.ceil(len(pts) / 250_000))
    return dict(
        frame="map",
        points=pts[::stride],
        trajectory=trajectory,
        source_count=len(pts),
        display_stride=stride,
        provenance=provenance,
    )
