"""Customline speed limits anchored to distance along its control polyline."""

import math

from .geometry import edges, number


def sections(raw, control, closed):
    if not isinstance(raw, list) or len(raw) > 40:
        raise ValueError("速度区間は40個以内の配列で指定してください")
    total = sum(math.dist(a, b) for a, b in edges(control, closed))
    result = []
    for row in raw:
        start = number(row["start"], 0, total, "区間開始 (m)")
        end = number(row["end"], 0, total, "区間終了 (m)")
        speed = number(row["speed"], 0.1, 20, "区間速度 (m/s)")
        if abs(start - end) < 1e-6 or (not closed and start > end):
            raise ValueError(
                "区間の始終点を確認してください。始点を跨ぐ区間は閉路のみ指定できます"
            )
        result.append(dict(start=start, end=end, speed=speed))
    return result


def sample(control, closed, spacing, limits):
    es = edges(control, closed)
    cumulative = [0.0]
    for a, b in es:
        cumulative.append(cumulative[-1] + math.dist(a, b))
    total = cumulative[-1]
    count = max(3, min(600, math.ceil(total / spacing)))
    # Preserve corners and insert exact speed boundaries, even for very short zones.
    stations = set(cumulative + [total * i / count for i in range(count + 1)])
    stations.update(s[k] for s in limits for k in ("start", "end"))
    if closed:
        stations = {s for s in stations if s < total - 1e-8}
    result, index = [], 0
    for s in sorted(stations):
        if result and s - result[-1][0] < 1e-8:
            continue
        while index < len(es) - 1 and s > cumulative[index + 1]:
            index += 1
        a, b = es[index]
        t = (s - cumulative[index]) / (cumulative[index + 1] - cumulative[index])
        result.append((s, [a[k] + t * (b[k] - a[k]) for k in range(2)]))
    return [p for _, p in result]


def speed_cap(station, limits, default):
    cap = default
    for row in limits:
        a, b = row["start"], row["end"]
        hit = (
            a - 1e-7 <= station <= b + 1e-7
            if a < b
            else station >= a - 1e-7 or station <= b + 1e-7
        )
        if hit:
            cap = min(cap, row["speed"])
    return cap
