"""Static map-frame exclusion polygons; no obstacle avoidance or online planner."""

from .geometry import edges, inside, points, segment_distance, simple
from .storage import name


def validate(rows):
    if not isinstance(rows, list) or len(rows) > 100:
        raise ValueError("走行不可能領域は100個までです")
    result, ids = [], set()
    for row in rows:
        key = name(row["id"])
        if key.casefold() in ids:
            raise ValueError("走行不可能領域IDが重複しています")
        ids.add(key.casefold())
        polygon = points(row["polygon"], minimum=3, maximum=100)
        simple(polygon, True)
        area = sum(a[0] * b[1] - b[0] * a[1] for a, b in edges(polygon, True))
        if abs(area) < 1e-8:
            raise ValueError("走行不可能領域には面積が必要です")
        result.append(dict(id=key, polygon=polygon))
    return result


def check_line(line, closed, obstacles, clearance=0):
    for obstacle in validate(obstacles):
        polygon = obstacle["polygon"]
        for i, (a, b) in enumerate(edges(line, closed)):
            if (
                inside(a, polygon)
                or inside(b, polygon)
                or any(
                    segment_distance(a, b, c, d) <= max(clearance, 1e-8)
                    for c, d in edges(polygon, True)
                )
            ):
                raise ValueError(
                    f"走行不可能領域 {obstacle['id']} に区間{i + 1}が接触します（車幅・余裕を含む）。回避は自動生成しません"
                )
