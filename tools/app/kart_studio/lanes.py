"""ID-based offline lane documents and legacy single-lane migration."""

import copy

from .geometry import points, number
from .speed_sections import sections
from .storage import name

FIELDS = ("closed", "left", "right", "custom", "custom_speeds", "custom_speed_mode", "centerline_speed_mps", "lines")


def empty(lane_id="lane_001"):
    return dict(
        id=name(lane_id),
        closed=True,
        left=[],
        right=[],
        custom=[],
        custom_speeds=[],
        centerline_speed_mps=1.5,
        lines={},
    )


def upgrade(document):
    doc = copy.deepcopy(document)
    if "lanes" not in doc:
        lane = empty()
        for key in FIELDS:
            if key in doc:
                lane[key] = doc.pop(key)
        doc["lanes"] = [lane]
    for lane in doc["lanes"]:
        lane.setdefault("centerline_speed_mps", 1.5)
    doc["schema"] = "kart.hdmap.v2"
    return doc


def select(document, lane_id=None):
    lanes = document["lanes"]
    if lane_id is None and len(lanes) == 1:
        return lanes[0]
    for lane in lanes:
        if lane["id"] == lane_id:
            return lane
    raise ValueError("対象lane IDを指定してください")


def validate(raw, existing):
    if not isinstance(raw, list) or not 1 <= len(raw) <= 64:
        raise ValueError("laneは1〜64個定義してください")
    result, ids = [], set()
    previous = {lane["id"]: lane for lane in existing}
    for row in raw:
        lane_id = name(row["id"])
        if lane_id.casefold() in ids:
            raise ValueError("lane IDが重複しています: " + lane_id)
        ids.add(lane_id.casefold())
        closed = row.get("closed")
        if not isinstance(closed, bool):
            raise ValueError("closedはbooleanで指定してください")
        lane = {
            key: points(row.get(key, []), minimum=0, maximum=1000)
            for key in ("left", "right", "custom")
        }
        lane.update(id=lane_id, closed=closed)
        lane["custom_speeds"] = sections(
            row.get("custom_speeds", []), lane["custom"], closed
        )
        mode=row.get("custom_speed_mode", "profile")
        if mode not in ("profile", "step"): raise ValueError("未知のCustom速度方式です")
        lane["custom_speed_mode"]=mode
        old = previous.get(lane_id, empty(lane_id))
        speed = row.get("centerline_speed_mps", old.get("centerline_speed_mps", 1.5))
        if isinstance(speed, bool):
            raise ValueError("Center速度は0〜20 m/sで指定してください")
        lane["centerline_speed_mps"] = number(speed, 0, 20, "Center速度 m/s")
        # Generated results are server-owned; never trust client-provided lines.
        lane["lines"] = copy.deepcopy(old["lines"])
        if any(lane[k] != old[k] for k in ("left", "right", "closed")):
            lane["lines"] = {}
        elif lane["custom_speed_mode"] != old.get("custom_speed_mode", "profile") or any(lane[k] != old.get(k, []) for k in ("custom", "custom_speeds")):
            lane["lines"].pop("customline", None)
        result.append(lane)
    return result
