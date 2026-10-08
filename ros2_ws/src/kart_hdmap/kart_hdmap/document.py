"""Validate Studio v2 documents/exports without ROS or coordinate transforms."""

import csv
import json
import math
import re
from pathlib import Path

KINDS = ("centerline", "raceline", "customline")


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Non-finite map coordinate/profile")
    return value


def points(rows):
    result = []
    if not isinstance(rows, list) or len(rows) > 1_000_000:
        raise ValueError("Invalid point list")
    for row in rows:
        if len(row) != 2:
            raise ValueError("HDMap requires XY points in meters")
        result.append([finite(v) for v in row])
    return result


def read_document(path):
    text = path.read_text()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        import yaml

        try:
            return yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise ValueError("Invalid HDMap YAML") from exc


def csv_line(root, relative, kind):
    path = (root / relative).resolve(strict=True)
    if Path(relative).is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError("CSV must stay inside the export directory")
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        columns = (
            ["x_m", "y_m", "width_right_m", "width_left_m"]
            if kind == "centerline"
            else ["s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"]
        )
        if reader.fieldnames != columns:
            raise ValueError("Unexpected CSV columns: " + relative)
        rows = [[finite(row[c]) for c in columns] for row in reader]
    return {
        "points": [r[:2] if kind == "centerline" else r[1:3] for r in rows],
        **({"profile": rows} if kind != "centerline" else {}),
    }


def load(path, frame_id="map"):
    path = Path(path).expanduser().resolve(strict=True)
    doc = read_document(path)
    if not isinstance(doc, dict):
        raise TypeError("HDMap must be an object")
    if doc.get("schema") != "kart.hdmap.v2":
        raise ValueError("Use a Studio v2 map.json or exported hd_map.yaml")
    if doc.get("snapshot_status") == "pending":
        raise ValueError("HDMap is not ready: snapshot pending")
    frame = doc.get("frame_id", doc.get("frame"))
    if (
        not isinstance(frame, str)
        or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]*", frame)
        or frame != frame_id
    ):
        raise ValueError(
            "Map frame must match frame_id; relabeling coordinates is forbidden"
        )
    if doc.get("line_role", "offline_reference") != "offline_reference":
        raise ValueError("Only offline reference lines are supported")
    raw_lanes = doc.get("lanes")
    if not isinstance(raw_lanes, list) or not 1 <= len(raw_lanes) <= 64:
        raise ValueError("Expected 1..64 lanes")
    lanes, ids = [], set()
    exported = "frame_id" in doc
    for raw in raw_lanes:
        key = raw["id"]
        if (
            not isinstance(key, str)
            or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", key)
            or key.casefold() in ids
        ):
            raise ValueError("Invalid or duplicate lane ID")
        ids.add(key.casefold())
        closed = raw["closed_loop" if exported else "closed"]
        if not isinstance(closed, bool):
            raise TypeError("closed must be boolean")
        lane = {
            "id": key,
            "closed": closed,
            "left": points(raw["left_bound" if exported else "left"]),
            "right": points(raw["right_bound" if exported else "right"]),
            "lines": {},
        }
        data = (
            {
                k: csv_line(path.parent, v, k)
                for k, v in raw.get("offline_lines", {}).items()
                if k in KINDS
            }
            if exported
            else raw.get("lines", {})
        )
        if exported and "centerline" not in data:
            data["centerline"] = {"points": raw["centerline"]}
        for kind in KINDS:
            if kind not in data:
                continue
            line = {"points": points(data[kind]["points"])}
            if len(line["points"]) < (3 if closed else 2):
                raise ValueError("Generated line has too few points")
            if kind != "centerline":
                profile = [[finite(v) for v in row] for row in data[kind]["profile"]]
                if len(profile) != len(line["points"]) or any(
                    len(r) != 7 for r in profile
                ):
                    raise ValueError("Invalid speed profile shape")
                for i, (row, xy) in enumerate(zip(profile, line["points"])):
                    if (
                        row[0] < 0
                        or row[5] < 0
                        or (i and row[0] < profile[i - 1][0])
                        or any(abs(a - b) > 1e-6 for a, b in zip(row[1:3], xy))
                    ):
                        raise ValueError(
                            "Profile does not match line geometry/distances/speed"
                        )
                line["profile"] = profile
            lane["lines"][kind] = line
        if (
            exported
            and "centerline" in lane["lines"]
            and lane["lines"]["centerline"]["points"] != points(raw["centerline"])
        ):
            raise ValueError("Centerline CSV and HDMap disagree")
        lanes.append(lane)
    obstacles = []
    rows = doc.get("obstacles", [])
    if not isinstance(rows, list) or len(rows) > 100:
        raise ValueError("Invalid exclusion regions")
    seen = set()
    for row in rows:
        key = row["id"]
        if (
            not isinstance(key, str)
            or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", key)
            or key.casefold() in seen
        ):
            raise ValueError("Invalid exclusion region ID")
        seen.add(key.casefold())
        polygon = points(row["polygon"])
        if not 3 <= len(polygon) <= 100:
            raise ValueError("Exclusion region needs 3..100 vertices")
        obstacles.append(dict(id=key, polygon=polygon))
    return {
        "obstacles": obstacles,
        "schema": "kart.hdmap.visualization.v1",
        "frame_id": frame,
        "revision": doc.get("revision"),
        "line_role": "offline_reference",
        "lanes": lanes,
    }


def select(document, lane_id):
    if not lane_id:
        return None
    for lane in document["lanes"]:
        if lane["id"] == lane_id:
            return lane
    raise ValueError("Unknown lane ID: " + lane_id)


def path_samples(line, closed):
    xy = line["points"]
    profile = line.get("profile")
    result = []
    for i, p in enumerate(xy):
        a, b = (
            (p, xy[(i + 1) % len(xy)]) if closed or i + 1 < len(xy) else (xy[i - 1], p)
        )
        yaw = profile[i][3] if profile else math.atan2(b[1] - a[1], b[0] - a[0])
        result.append((p[0], p[1], math.sin(yaw / 2), math.cos(yaw / 2)))
    if closed and result and xy[-1] != xy[0]:
        result.append(result[0])
    return result
