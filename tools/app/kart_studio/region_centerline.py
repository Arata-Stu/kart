"""Experimental centerline from a raster corridor and two boundary distance fields.

No vertex pairing, occupancy reconstruction, or minimum-curvature optimization.
The zero contour of distance(left)-distance(right) is traced inside the mask.
Distances are to the original segments; the grid only discretizes the contour.
"""

import math

from .geometry import Corridor, distance, edges, points, resample

MAX_CELLS = 1_000_000
MAX_SAMPLES = 600
MAX_RELATIVE_IMBALANCE = 0.10


def _inside_grid(np, x, y, polygon, check):
    mask = np.zeros(x.shape, dtype=bool)
    for i, (a, b) in enumerate(edges(polygon, True)):
        if i % 16 == 0:
            check()
        if a[1] != b[1]:
            mask ^= ((a[1] > y) != (b[1] > y)) & (
                x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
            )
    return mask


def _distances(np, x, y, boundary, check):
    best = np.full(x.shape, np.inf)
    for i, (a, b) in enumerate(edges(boundary, True)):
        if i % 16 == 0:
            check()
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = np.clip(((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy), 0, 1)
        squared = (x - a[0] - t * dx) ** 2 + (y - a[1] - t * dy) ** 2
        np.minimum(best, squared, out=best)
    return np.sqrt(best)


def _trace(np, field, mask, origin, resolution, check):
    """Marching squares with shared edge/vertex identities, not a DFS point dump."""
    valid = mask[:-1, :-1] & mask[:-1, 1:] & mask[1:, 1:] & mask[1:, :-1]
    corners = (field[:-1, :-1], field[:-1, 1:], field[1:, 1:], field[1:, :-1])
    lower = np.minimum.reduce(corners)
    upper = np.maximum.reduce(corners)
    rows, cols = np.nonzero(valid & (lower <= 0) & (upper >= 0))
    locations, adjacency = {}, {}
    for count, (row, col) in enumerate(zip(rows.tolist(), cols.tolist())):
        if count % 256 == 0:
            check()
        vertices = ((row, col), (row, col + 1), (row + 1, col + 1), (row + 1, col))
        hits = []
        for index in range(4):
            a, b = vertices[index], vertices[(index + 1) % 4]
            fa, fb = float(field[a]), float(field[b])
            if (fa >= 0) == (fb >= 0):
                continue
            t = fa / (fa - fb)
            if t < 1e-10:
                key = ('vertex', a)
            elif t > 1 - 1e-10:
                key = ('vertex', b)
            else:
                key = ('edge', *sorted((a, b)))
            locations[key] = [origin[0] + (a[1] + t * (b[1] - a[1])) * resolution,
                              origin[1] + (a[0] + t * (b[0] - a[0])) * resolution]
            if key not in hits:
                hits.append(key)
        if len(hits) < 2:
            continue
        if len(hits) != 2:
            raise ValueError('領域方式の等距離線が曖昧です。格子解像度を細かくして再生成してください')
        a, b = hits
        adjacency.setdefault(a, set()).add(b)
        adjacency.setdefault(b, set()).add(a)
    if not adjacency or any(len(neighbors) != 2 for neighbors in adjacency.values()):
        raise ValueError('領域方式で閉じた中心線を抽出できません。狭い区間と格子解像度を確認してください')
    start = next(iter(adjacency))
    route, previous, current = [], None, start
    while True:
        if len(route) % 256 == 0:
            check()
        route.append(locations[current])
        candidates = adjacency[current] - ({previous} if previous is not None else set())
        following = min(candidates, key=str)
        previous, current = current, following
        if current == start:
            break
        if len(route) > len(adjacency):
            raise ValueError('領域方式の中心線の接続が不正です')
    if len(route) != len(adjacency):
        raise ValueError('領域方式で複数の中心線が得られました。境界・走行可能領域を確認してください')
    return route


def extract(left, right, closed, spacing, resolution, check=lambda: None):
    if not closed:
        raise ValueError('領域方式は現在閉路専用です。開路は従来の境界中点方式を使用してください')
    try:
        import numpy as np
    except ImportError as error:
        raise ValueError('領域方式にはNumPyが必要です。kart DockerまたはNumPy導入環境で実行してください') from error
    if not math.isfinite(resolution) or not 0.005 <= resolution <= 0.25:
        raise ValueError('領域方式の格子解像度は0.005〜0.25 mにしてください')
    check()
    left, right = points(left, 3), points(right, 3)
    corridor = Corridor(left, right, True)
    all_points = left + right
    minimum = [min(p[k] for p in all_points) for k in (0, 1)]
    maximum = [max(p[k] for p in all_points) for k in (0, 1)]
    # Fractional padding reduces zero contours exactly coinciding with grid vertices.
    origin = [v - 1.37 * resolution for v in minimum]
    nx, ny = [math.ceil((maximum[k] - origin[k]) / resolution) + 3 for k in (0, 1)]
    if nx * ny > MAX_CELLS:
        raise ValueError(f'領域方式の格子が大きすぎます（{nx}×{ny}、上限{MAX_CELLS}セル）。格子解像度を粗くしてください')
    x, y = np.meshgrid(origin[0] + np.arange(nx) * resolution,
                       origin[1] + np.arange(ny) * resolution)
    mask = _inside_grid(np, x, y, left, check) ^ _inside_grid(np, x, y, right, check)
    if not mask.any():
        raise ValueError('格子上に走行可能領域がありません。格子解像度と境界を確認してください')
    # Outside cells do not take part in contour tracing. Finite placeholders avoid
    # NaN arithmetic warnings in the vectorized cell candidate test.
    field = np.zeros(mask.shape)
    field[mask] = _distances(np, x[mask], y[mask], left, check) - _distances(np, x[mask], y[mask], right, check)
    route = _trace(np, field, mask, origin, resolution, check)
    area = lambda ps: sum(a[0] * b[1] - a[1] * b[0] for a, b in edges(ps, True))
    if area(route) * area(left) < 0:
        route.reverse()
    seam = min(range(len(route)), key=lambda i: math.dist(route[i], left[0]))
    route = route[seam:] + route[:seam]
    length = sum(math.dist(a, b) for a, b in edges(route, True))
    count = max(12, math.ceil(length / spacing))
    if count > MAX_SAMPLES:
        raise ValueError(f'中心線が{MAX_SAMPLES}点を超えます。点間隔を大きくしてください')
    route = resample(route, count, True)
    check()
    corridor.validate(route)
    widths = [[min(distance(p, a, b) for a, b in edges(boundary, True))
               for boundary in (right, left)] for p in route]
    imbalance = [abs(a - b) for a, b in widths]
    relative = [abs(a - b) / max(a + b, 1e-12) for a, b in widths]
    if max(imbalance) > 2 * resolution or max(relative) > MAX_RELATIVE_IMBALANCE:
        raise ValueError('再サンプル後の中心線が中央から外れています。点間隔・格子解像度を小さくしてください')
    info = dict(source='drawn_boundary_corridor', resolution_m=resolution, grid_shape=[ny, nx],
                max_distance_imbalance_m=max(imbalance),
                mean_distance_imbalance_m=sum(imbalance) / len(imbalance),
                max_relative_imbalance=max(relative),
                route_length_m=length)
    return route, widths, corridor, info
