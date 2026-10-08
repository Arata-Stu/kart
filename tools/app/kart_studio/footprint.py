"""Conservative planar swept rectangles, referenced to the rear axle."""

import math

from .geometry import Corridor, cross, edges, inside, segment_distance

MODEL = "rear_axle_rectangle_v1"


def headings(line, closed):
    result = []
    for i, p in enumerate(line):
        a = line[(i - 1) % len(line)] if closed or i else p
        b = line[(i + 1) % len(line)] if closed or i + 1 < len(line) else p
        result.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    return result


def rectangle(point, yaw, settings):
    margin = settings["margin"]
    rear = settings["rear_axle_to_rear"] + margin
    front = settings["vehicle_length"] - settings["rear_axle_to_rear"] + margin
    half = settings["vehicle_width"] / 2 + margin
    c, s = math.cos(yaw), math.sin(yaw)
    return [
        [point[0] + c * x - s * y, point[1] + s * x + c * y]
        for x, y in ((-rear, -half), (front, -half), (front, half), (-rear, half))
    ]


def hull(points):
    points = sorted(set(map(tuple, points)))

    def chain(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out

    return chain(points)[:-1] + chain(reversed(points))[:-1]


def swept(line, closed, settings, yaws=None):
    yaws = headings(line, closed) if yaws is None else yaws
    if len(yaws) != len(line) or any(not math.isfinite(y) for y in yaws):
        raise ValueError("車体姿勢の数または数値が不正です")
    radius = max(math.hypot(x, y) for x, y in rectangle([0, 0], 0, settings))
    for i, (a, b) in enumerate(edges(line, closed)):
        delta = math.remainder(yaws[(i + 1) % len(line)] - yaws[i], 2 * math.pi)
        count = max(1, math.ceil(abs(delta) / math.radians(10)))
        for n in range(count):
            t0, t1 = n / count, (n + 1) / count
            pa = [a[k] + (b[k] - a[k]) * t0 for k in (0, 1)]
            pb = [a[k] + (b[k] - a[k]) * t1 for k in (0, 1)]
            polygon = hull(
                rectangle(pa, yaws[i] + delta * t0, settings)
                + rectangle(pb, yaws[i] + delta * t1, settings)
            )
            # A rotating corner deviates from its linear chord by <= R*angle²/8.
            # Translation is linear. Inflating this hull therefore covers the interval.
            padding = radius * (delta / count) ** 2 / 8 + 1e-8
            yield i, polygon, padding


def bounds(polygon):
    return (
        min(p[0] for p in polygon),
        min(p[1] for p in polygon),
        max(p[0] for p in polygon),
        max(p[1] for p in polygon),
    )


def close_boxes(a, b, padding):
    return not (
        a[2] + padding < b[0]
        or b[2] + padding < a[0]
        or a[3] + padding < b[1]
        or b[3] + padding < a[1]
    )


def corridor_for(left, right, closed, settings):
    if closed:
        return Corridor(left, right, True)
    if math.dist(left[0], right[-1]) + math.dist(left[-1], right[0]) < math.dist(
        left[0], right[0]
    ) + math.dist(left[-1], right[-1]):
        right = list(reversed(right))
    # Open boundaries are road edges, not walls across entry/exit. Extend tangentially
    # only for the footprint test so the end axle position may reach the road end.
    extension = settings["vehicle_length"] + 2 * settings["margin"]

    def extend(line):
        result = [list(p) for p in line]
        for index, neighbor in ((0, 1), (-1, -2)):
            a, b = line[index], line[neighbor]
            length = math.dist(a, b)
            result[index] = [a[k] + extension * (a[k] - b[k]) / length for k in (0, 1)]
        return result

    return Corridor(extend(left), extend(right), False)


def validate_line(
    line, closed, settings, corridor=None, obstacles=(), yaws=None, check=lambda: None
):
    regions = [(row["id"], row["polygon"], bounds(row["polygon"])) for row in obstacles]
    boundary = (
        [(a, b, bounds([a, b])) for a, b in corridor.boundaries] if corridor else []
    )
    for index, polygon, padding in swept(line, closed, settings, yaws):
        check()
        box = bounds(polygon)
        segments = edges(polygon, True)
        if corridor:
            if any(not corridor.contains(p) for p in polygon):
                raise ValueError(f"長方形車体がコース境界外です（区間{index + 1}）")
            for a, b, edge_box in boundary:
                if close_boxes(box, edge_box, padding) and (
                    inside(a, polygon)
                    or inside(b, polygon)
                    or any(segment_distance(a, b, c, d) <= padding for c, d in segments)
                ):
                    raise ValueError(
                        f"長方形車体がコース境界に接触します（区間{index + 1}）"
                    )
        for key, region, region_box in regions:
            if close_boxes(box, region_box, padding) and (
                inside(polygon[0], region)
                or inside(region[0], polygon)
                or any(
                    segment_distance(a, b, c, d) <= padding
                    for a, b in segments
                    for c, d in edges(region, True)
                )
            ):
                raise ValueError(
                    f"長方形車体が走行不可能領域 {key} に接触します（区間{index + 1}）"
                )
