"""Offline lane-ID exports. No online planning or route selection."""

import csv
import os
import shutil
import uuid

from . import footprint
from .lines import settings
from .obstacles import validate
from .storage import atomic_json, fingerprint, within


def write_csv(path, kind, line):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        if kind == "centerline":
            writer.writerow(["x_m", "y_m", "width_right_m", "width_left_m"])
            writer.writerows(p + w for p, w in zip(line["points"], line["widths"]))
        else:
            writer.writerow(
                ["s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"]
            )
            writer.writerows(line["profile"])


def export_map(folder, doc):
    missing = [lane["id"] for lane in doc["lanes"] if "centerline" not in lane["lines"]]
    if missing:
        raise ValueError("centerlineを生成してください: " + ", ".join(missing))
    regions = validate(doc.get("obstacles", []))
    for lane in doc["lanes"]:
        for line in lane["lines"].values():
            options = settings(line.get("settings", {}))
            if line.get("settings", {}).get("collision_model") != footprint.MODEL:
                raise ValueError(
                    "旧ラインです。長方形車体の設定で各ラインを再生成してください"
                )
            corridor = footprint.corridor_for(
                lane["left"], lane["right"], lane["closed"], options
            )
            footprint.validate_line(
                line["points"],
                lane["closed"],
                options,
                corridor=corridor,
                obstacles=regions,
                yaws=[row[3] for row in line["profile"]] if "profile" in line else None,
            )
    root = within(folder, "exports", exists=False)
    root.mkdir(exist_ok=True)
    # Keep old v1 exports immutable even if a legacy map has not been saved yet.
    destination = within(root, f"revision-{doc['revision']}-v2", exists=False)
    if destination.exists():
        return destination
    temp = root / (".export-" + uuid.uuid4().hex)
    temp.mkdir()
    try:
        lanes, metadata = [], {}
        for lane in doc["lanes"]:
            relative = "lanes/" + lane["id"]
            directory = temp / relative
            directory.mkdir(parents=True)
            files = {}
            for kind, line in lane["lines"].items():
                write_csv(directory / (kind + ".csv"), kind, line)
                files[kind] = relative + "/" + kind + ".csv"
                if len(doc["lanes"]) == 1:
                    shutil.copyfile(directory / (kind + ".csv"), temp / (kind + ".csv"))
            lanes.append(
                dict(
                    id=lane["id"],
                    closed_loop=lane["closed"],
                    left_bound=lane["left"],
                    right_bound=lane["right"],
                    centerline=lane["lines"]["centerline"]["points"],
                    offline_lines=files,
                )
            )
            metadata[lane["id"]] = dict(
                closed=lane["closed"],
                custom_speeds=lane.get("custom_speeds", []),
                lines={
                    k: {
                        f: v
                        for f, v in line.items()
                        if f not in ("points", "profile", "widths")
                    }
                    for k, line in lane["lines"].items()
                },
            )
        atomic_json(
            temp / "hd_map.yaml",
            dict(
                schema="kart.hdmap.v2",
                frame_id="map",
                revision=doc["revision"],
                line_role="offline_reference",
                lanes=lanes,
                obstacles=regions,
            ),
        )
        atomic_json(
            temp / "metadata.json",
            dict(
                schema="kart.hdmap.v2",
                revision=doc["revision"],
                document_fingerprint=fingerprint(doc),
                frame="map",
                line_role="offline_reference",
                lanes=metadata,
                warning="オフライン参照ライン。障害物回避・オンライン計画は行わない。",
            ),
        )
        os.rename(temp, destination)
    finally:
        if temp.exists():
            shutil.rmtree(temp)
    return destination
