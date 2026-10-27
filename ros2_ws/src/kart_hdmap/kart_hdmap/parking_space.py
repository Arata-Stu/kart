"""Exact polygon union of lane corridors and local parking approaches (ROS-free)."""
import math


def surface(doc, rows):
    from shapely.geometry import Polygon, LineString
    from shapely.ops import unary_union

    def poly(points):
        if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('boundの座標は有限のXYにしてください')
        p = Polygon(points)
        if not p.is_valid or p.area <= 1e-8:
            raise ValueError('boundから有効な走行領域を生成できません（交差・退化）')
        return p

    corridors = []
    for lane in doc.get('lanes', []):
        left = lane.get('left', lane.get('left_bound', []))
        right = lane.get('right', lane.get('right_bound', []))
        if lane.get('closed', lane.get('closed_loop', False)):
            a, b = poly(left), poly(right)
            outer, inner = (a, b) if a.area > b.area else (b, a)
            if not outer.contains(inner) or outer.boundary.intersects(inner.boundary):
                raise ValueError('閉じたleft/right boundは交差しない内外周にしてください')
            corridors.append(outer.difference(inner))
        else:
            if len(left) < 2 or len(right) < 2:
                raise ValueError('left/right boundには各2点以上必要です')
            if math.dist(left[0], right[-1])+math.dist(left[-1], right[0]) < math.dist(left[0], right[0])+math.dist(left[-1], right[-1]):
                right = list(reversed(right))
            corridors.append(poly(list(left)+list(reversed(right))))
    if not corridors:
        raise ValueError('駐車にはleft/right boundを定義してください')
    course = unary_union(corridors)
    additions = []
    for row in rows:
        approach, area = poly(row['maneuver']), poly(row['polygon'])
        if approach.intersection(course).area <= 1e-8:
            raise ValueError(row['id']+'の進入領域をコースに重ねて接続してください')
        gate = LineString(row['entrance'])
        # A full opening must be inside the local approach, not just touch its edge.
        if not approach.contains(gate) or approach.boundary.distance(gate) <= 1e-8:
            raise ValueError(row['id']+'の進入領域を入口の両側まで広げてください')
        additions.extend([approach, area])
    return unary_union([course]+additions)
