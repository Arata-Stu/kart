"""Bounded curvature reduction and acceleration-limited speed profiles."""

import math

from . import footprint
from .geometry import centerline, edges, number, points
from .speed_sections import sample, sections, speed_cap

DEFAULTS = dict(
    spacing=0.15,
    vehicle_width=0.19,
    vehicle_length=0.47,
    rear_axle_to_rear=0.1065,
    margin=0.05,
    max_speed=3.0,
    lateral_accel=2.5,
    accel=1.5,
    decel=2.5,
    curvature_limit=2.0,
)


def settings(raw):
    ranges = dict(
        spacing=(0.05, 1),
        vehicle_width=(0.05, 2),
        vehicle_length=(0.1, 5),
        rear_axle_to_rear=(0, 2),
        margin=(0, 1),
        max_speed=(0.1, 20),
        lateral_accel=(0.1, 15),
        accel=(0.1, 15),
        decel=(0.1, 15),
        curvature_limit=(0.01, 20),
    )
    p = {k: number(raw.get(k, v), *ranges[k], k) for k, v in DEFAULTS.items()}
    if p["rear_axle_to_rear"] >= p["vehicle_length"]:
        raise ValueError("後輪軸から後端までの距離は全長未満にしてください")
    p["collision_model"] = footprint.MODEL
    p["optimizer"] = raw.get("optimizer", "local")
    if p["optimizer"] not in ("local", "mincurv", "mincurv_iqp"):
        raise ValueError("未知の最適化方式です")
    return p


def curvature(a, b, c):
    ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
    return (
        2
        * ((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]))
        / max(ab * bc * ac, 1e-12)
    )


def profile(line, closed, p, limits=()):
    n = len(line)
    k = [
        curvature(line[(i - 1) % n], line[i], line[(i + 1) % n])
        if closed or 0 < i < n - 1
        else 0
        for i in range(n)
    ]
    v = [
        min(p["max_speed"], math.sqrt(p["lateral_accel"] / max(abs(c), 1e-8)))
        for c in k
    ]
    station = 0.0
    for i in range(n):
        v[i] = min(v[i], speed_cap(station, limits, p["max_speed"]))
        if i < n - 1:
            station += math.dist(line[i], line[i + 1])
    if not closed:
        v[0] = v[-1] = 0.0
    es = edges(line, closed)
    # Propagate squared-speed upper bounds around a closed loop until converged.
    for _ in range(n + 1 if closed else 2):
        old = v[:]
        for i, (a, b) in enumerate(es):
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * p["accel"] * math.dist(a, b)))
        for i in reversed(range(len(es))):
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * p["decel"] * math.dist(*es[i])))
        if max(abs(a - b) for a, b in zip(old, v)) < 1e-8:
            break
    s, rows = 0.0, []
    for i, point in enumerate(line):
        nxt = line[(i + 1) % n] if closed or i < n - 1 else line[i]
        prev = line[(i - 1) % n] if closed or i else line[i]
        heading = math.atan2(nxt[1] - prev[1], nxt[0] - prev[0])
        ds = math.dist(point, nxt)
        ax = (v[(i + 1) % n] ** 2 - v[i] ** 2) / (2 * ds) if ds else 0
        rows.append([s, *point, heading, k[i], v[i], ax])
        s += ds
    return rows


def generate(document, kind, raw, check=lambda: None):
    p = settings(raw)
    closed = document["closed"]
    center, widths, corridor = centerline(
        document["left"], document["right"], closed, p["spacing"]
    )
    clearance = p["vehicle_width"] / 2 + p["margin"]
    footprint_corridor = footprint.corridor_for(
        document["left"], document["right"], closed, p
    )
    if kind == "centerline":
        footprint.validate_line(
            center, closed, p, corridor=footprint_corridor, check=check
        )
        return dict(
            points=center, widths=widths, settings=p, method="paired_boundary_midpoints"
        )
    limits = []
    optimizer_info = None
    if kind == "customline":
        control = points(document.get("custom", []), 3 if closed else 2)
        limits = sections(document.get("custom_speeds", []), control, closed)
        line = sample(control, closed, p["spacing"], limits)
        method = "resampled_user_polyline"
    elif kind == "raceline" and p["optimizer"] != "local":
        from .optimizer import optimize

        corridor.validate(center, clearance)
        optimized = optimize(document, p, check)
        line = optimized["points"]
        optimizer_info = optimized["optimizer_info"]
        method = "tph_" + p["optimizer"]
    elif kind == "raceline":
        corridor.validate(center, clearance)
        line = [p[:] for p in center]
        n = len(line)
        normals = []
        for i in range(n):
            a, b = center[(i - 1) % n], center[(i + 1) % n]
            dx, dy = b[0] - a[0], b[1] - a[1]
            norm = max(math.hypot(dx, dy), 1e-9)
            normals.append([-dy / norm, dx / norm])
        offsets = [0.0] * n

        def local_cost(index):
            indexes = {(index - 1) % n, index, (index + 1) % n}
            return sum(
                curvature(line[(j - 1) % n], line[j], line[(j + 1) % n]) ** 2
                for j in indexes
                if closed or 0 < j < n - 1
            )

        for step in (0.08, 0.04, 0.02, 0.01):
            for _ in range(5):
                check()
                changed = False
                for i in range(n) if closed else range(1, n - 1):
                    original = line[i][:]
                    best = local_cost(i)
                    delta = 0
                    for change in (-step, step):
                        candidate = [
                            center[i][k] + (offsets[i] + change) * normals[i][k]
                            for k in range(2)
                        ]
                        if not corridor.segment_ok(
                            line[(i - 1) % n], candidate, clearance
                        ) or not corridor.segment_ok(
                            candidate, line[(i + 1) % n], clearance
                        ):
                            continue
                        line[i] = candidate
                        cost = local_cost(i)
                        if cost < best - 1e-9:
                            best, delta = cost, change
                    line[i] = [original[k] + delta * normals[i][k] for k in range(2)]
                    offsets[i] += delta
                    changed |= bool(delta)
                if not changed:
                    break
        method = "bounded_local_curvature_search_v1"
    else:
        raise ValueError("未対応のラインです")
    corridor.validate(line, clearance)
    rows = profile(line, closed, p, limits)
    footprint.validate_line(
        line,
        closed,
        p,
        corridor=footprint_corridor,
        yaws=[r[3] for r in rows],
        check=check,
    )
    if (
        kind == "raceline"
        and p["optimizer"] != "local"
        and max(abs(row[4]) for row in rows) > p["curvature_limit"] * 1.01
    ):
        raise ValueError(
            "補間後のラインが曲率上限を超えています。設定または境界を調整してください"
        )
    return dict(
        points=line,
        profile=rows,
        settings=p,
        method=method,
        speed_sections=limits,
        optimizer_info=optimizer_info,
    )
