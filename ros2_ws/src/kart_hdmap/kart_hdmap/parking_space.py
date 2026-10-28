"""Exact polygon union of lane corridors and local parking approaches (ROS-free)."""
import math


def surface(doc, rows):
    try:
        from shapely.geometry import Polygon, LineString
        from shapely.ops import unary_union
    except (ImportError, AttributeError, OSError) as exc:
        raise ValueError(
            '駐車領域の計算環境に問題があります（Shapely / NumPyの読み込み失敗）。'
            '駐車枠の描き直しは不要です。Studio実行PythonへNumPy対応版Shapelyを導入し、'
            'Studioを再起動してください。手順: tools/app/README.md「Shapely / NumPyの互換性エラー」'
        ) from exc

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
        area = poly(row['polygon'])
        gate = LineString(row['entrance'])
        approach = poly(row['maneuver']) if row.get('maneuver') else None
        if approach is not None:
            if approach.intersection(course).area <= 1e-8:
                raise ValueError(row['id']+'の経路制限範囲をコースに重ねてください')
            if not approach.contains(gate) or approach.boundary.distance(gate) <= 1e-8:
                raise ValueError(row['id']+'の経路制限範囲を入口の両側まで広げてください')
        joined = unary_union([course, area])
        # A bay must connect through its entrance, not an unrelated side/corner.
        if not joined.contains(LineString([gate.interpolate(1e-6, normalized=True),gate.interpolate(1-1e-6, normalized=True)])):
            raise ValueError(row['id']+'の入口が通常コースに接続していません。boundと駐車枠の間の隙間を修正してください')
        additions.append(area)
    return unary_union([course]+additions)
