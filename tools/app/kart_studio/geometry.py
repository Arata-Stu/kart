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


class PathValidationError(ValueError):
    """JSON-safe location of a rejected candidate; never a publishable line."""
    def __init__(self, message, a, b, index, **details):
        self.diagnostic = dict(segment=index + 1, points=[list(a), list(b)], **details)
        super().__init__(f"{message}（区間{index + 1}, "
                         f"XY=({a[0]:.3f}, {a[1]:.3f}) → ({b[0]:.3f}, {b[1]:.3f})）")


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
                nearest = min(self.boundaries, key=lambda edge: segment_distance(a, b, *edge))
                gap = segment_distance(a, b, *nearest)
                outside = any(not self.contains(p) for p in (a,b,[(a[0]+b[0])/2,(a[1]+b[1])/2]))
                reason = "生成線がコース境界の外です" if outside else "生成線と境界の距離が不足しています"
                raise PathValidationError(
                    f"{reason}: 境界まで{gap:.3f} m / 必要{clearance:.3f} m", a,b,i,
                    boundary=[list(p) for p in nearest], distance_m=gap,
                    required_clearance_m=clearance, reason="outside" if outside else "clearance")


def monotone_midpoints(left, right, closed, corridor):
    """Match local stations in order instead of equating whole-loop percentages.

    Only used when uniform pairing fails. Invalid connectors are excluded and
    the resulting complete path still passes the original corridor validator.
    """
    def visible(a,b):
        # Connector endpoints lie on the walls; validate only its interior.
        start=[a[k]+1e-4*(b[k]-a[k]) for k in range(2)]
        end=[b[k]+1e-4*(a[k]-b[k]) for k in range(2)]
        return corridor.segment_ok(start,end,0)
    n = len(left)
    if closed:
        # A nearby visible cross-section anchors the cyclic seam.
        candidates = sorted(range(n), key=lambda j: math.dist(left[0], right[j]))
        start = next((j for j in candidates if visible(left[0],right[j])), None)
        if start is None:raise ValueError("左右境界を結ぶ断面がありません。境界と始点を確認してください")
        right = right[start:] + right[:start]
        left, right = left + left[:1], right + right[:1]
    size = len(left)
    costs = [[math.dist(a,b)**2 for b in right] for a in left]
    # Each iteration excludes every blocked cross-section on the chosen path.
    for _ in range(8):
        previous = [math.inf]*size
        steps = [bytearray(size) for _ in range(size)]
        for i in range(size):
            current = [math.inf]*size
            for j in range(size):
                cost = costs[i][j]
                if i == j == 0:current[j] = cost;continue
                choices = [(previous[j-1]+2*cost,1) if i and j else (math.inf,1),
                           (previous[j]+cost,2) if i else (math.inf,2),
                           (current[j-1]+cost,3) if j else (math.inf,3)]
                current[j],steps[i][j] = min(choices)
            previous = current
        if not math.isfinite(previous[-1]):break
        pairs=[];i=j=size-1
        while True:
            pairs.append((i,j))
            if i == j == 0:break
            step=steps[i][j]
            if step in (1,2):i-=1
            if step in (1,3):j-=1
        pairs.reverse()
        blocked=[(i,j) for i,j in pairs if not visible(left[i],right[j])]
        if blocked:
            for i,j in blocked:costs[i][j]=math.inf
            continue
        line=[[(left[i][k]+right[j][k])/2 for k in range(2)] for i,j in pairs]
        if closed:line.pop()
        corridor.validate(line)
        return line
    raise ValueError("左右境界の対応付けができません。境界の近接・凹部・始点を確認してください")


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
    try:
        corridor.validate(line)
    except PathValidationError as original:
        try:
            line = monotone_midpoints(left_samples,right_samples,closed,corridor)
            length=sum(math.dist(a,b) for a,b in edges(line,closed))
            line=resample(line,max(12 if closed else 3,min(600,math.ceil(length/spacing))),closed)
            # Remove station-change kinks without bypassing any geometry/footprint check.
            line=[[(line[(i-1)%len(line)][k]+2*p[k]+line[(i+1)%len(line)][k])/4 for k in range(2)]
                  if closed or 0<i<len(line)-1 else p for i,p in enumerate(line)]
            corridor.validate(line)
        except ValueError as exc:
            # Keep the concrete failed location even if correspondence has no solution.
            if isinstance(exc,PathValidationError):raise
            raise original from exc
    widths = [
        [
            min(distance(p, a, b) for a, b in edges(right, closed)),
            min(distance(p, a, b) for a, b in edges(left, closed)),
        ]
        for p in line
    ]
    return line, widths, corridor
