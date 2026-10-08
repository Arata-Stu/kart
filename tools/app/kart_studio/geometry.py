"""Planar geometry: paired boundaries, corridor validation and sampled paths."""

import math


def number(value, low, high, label):
    result = float(value)
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f"{label}: {low}〜{high}で指定してください")
    return result


def points(values, minimum=2, maximum=1500):
    if not isinstance(values, list) or not minimum <= len(values) <= maximum:
        raise ValueError(f"点は{minimum}〜{maximum}個必要です")
    out = []
    for p in values:
        if not isinstance(p, (list, tuple)) or len(p) < 2:
            raise ValueError("XY座標が必要です")
        out.append([number(p[0], -1e5, 1e5, "X"), number(p[1], -1e5, 1e5, "Y")])
    if any(math.dist(a, b) < 1e-6 for a, b in zip(out, out[1:])):
        raise ValueError("連続した点が重なっています")
    return out


def edges(line, closed=False):
    return list(zip(line, line[1:] + (line[:1] if closed else [])))


def distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    t = max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den)) if den else 0
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def intersects(a, b, c, d):
    if cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0:
        return True
    return (
        min(distance(a, c, d), distance(b, c, d), distance(c, a, b), distance(d, a, b))
        < 1e-8
    )


def segment_distance(a, b, c, d):
    return (
        0
        if intersects(a, b, c, d)
        else min(
            distance(a, c, d), distance(b, c, d), distance(c, a, b), distance(d, a, b)
        )
    )


def simple(line, closed):
    es = edges(line, closed)
    for i, (a, b) in enumerate(es):
        if math.dist(a, b) < 1e-6:
            raise ValueError(
                "境界の始終点が重複しています。閉路では終点を重ねないでください"
            )
        for j in range(i + 2, len(es)):
            if closed and i == 0 and j == len(es) - 1:
                continue
            if intersects(a, b, *es[j]):
                raise ValueError("ラインが自己交差しています")


def inside(p, polygon):
    result = False
    for a, b in edges(polygon, True):
        if distance(p, a, b) < 1e-8:
            return True
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (
            b[1] - a[1]
        ) + a[0]:
            result = not result
    return result


def resample(line, count, closed):
    es = edges(line, closed)
    lengths = [math.dist(a, b) for a, b in es]
    total = sum(lengths)
    if total < 0.1:
        raise ValueError("ラインが短すぎます（最低0.1 m）")
    result, index, previous = [], 0, 0.0
    for i in range(count):
        target = total * i / (count if closed else count - 1)
        while index < len(es) - 1 and previous + lengths[index] < target:
            previous += lengths[index]
            index += 1
        t = (target - previous) / max(lengths[index], 1e-12)
        a, b = es[index]
        result.append([a[k] + t * (b[k] - a[k]) for k in range(2)])
    return result


class Corridor:
    def __init__(self, left, right, closed):
        self.left, self.right, self.closed = left, right, closed
        for line in (left, right):
            simple(line, closed)
        self.boundaries = edges(left, closed) + edges(right, closed)
        for a, b in edges(left, closed):
            if any(intersects(a, b, c, d) for c, d in edges(right, closed)):
                raise ValueError("左右の境界が交差しています")
        if closed:
            if inside(left[0], right) == inside(right[0], left):
                raise ValueError("閉路は内側と外側の境界を描いてください")
        else:
            self.polygon = left + list(reversed(right))
            simple(self.polygon, True)

    def contains(self, p):
        return (
            (inside(p, self.left) != inside(p, self.right))
            if self.closed
            else inside(p, self.polygon)
        )

    def segment_ok(self, a, b, clearance):
        if (
            not self.contains(a)
            or not self.contains(b)
            or not self.contains([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2])
        ):
            return False
        return all(
            segment_distance(a, b, c, d) >= max(clearance, 1e-7)
            for c, d in self.boundaries
        )

    def validate(self, line, clearance=0):
        simple(line, self.closed)
        for i, (a, b) in enumerate(edges(line, self.closed)):
            if not self.segment_ok(a, b, clearance):
                raise ValueError(
                    f"ラインの区間{i + 1}が境界の外、または車幅・余裕に近すぎます"
                )


def centerline(left, right, closed, spacing=0.15):
    minimum = 3 if closed else 2
    left, right = points(left, minimum), points(right, minimum)
    if closed:

        def area(ps):
            return sum(a[0] * b[1] - a[1] * b[0] for a, b in edges(ps, True))

        if area(left) * area(right) < 0:
            right = list(reversed(right))
    elif math.dist(left[0], right[-1]) + math.dist(left[-1], right[0]) < math.dist(
        left[0], right[0]
    ) + math.dist(left[-1], right[-1]):
        right = list(reversed(right))
    corridor = Corridor(left, right, closed)
    length = max(
        sum(math.dist(a, b) for a, b in edges(ps, closed)) for ps in (left, right)
    )
    count = max(12 if closed else 3, min(600, math.ceil(length / spacing)))
    left_samples, right_samples = (
        resample(left, count, closed),
        resample(right, count, closed),
    )
    if closed:
        # Match loop phase, so unequal starting vertices do not cross the track.
        shift = min(
            range(count),
            key=lambda k: sum(
                math.dist(left_samples[i], right_samples[(i + k) % count]) ** 2
                for i in range(0, count, max(1, count // 50))
            ),
        )
        right_samples = right_samples[shift:] + right_samples[:shift]
    line = [
        [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2]
        for a, b in zip(left_samples, right_samples)
    ]
    corridor.validate(line)
    widths = [
        [
            min(distance(p, a, b) for a, b in edges(right, closed)),
            min(distance(p, a, b) for a, b in edges(left, closed)),
        ]
        for p in line
    ]
    return line, widths, corridor
