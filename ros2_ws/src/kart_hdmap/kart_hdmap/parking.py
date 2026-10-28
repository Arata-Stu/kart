"""HDMap parking attributes shared by Studio, export and the runtime planner."""
import math
from .parking_geometry import polygon, inside, point_segment, distance, edges, intersect


def validate(doc):
    rows=doc.get('parking_areas',[])
    if not isinstance(rows,list) or len(rows)>3:
        raise ValueError('駐車区画はP1・P2・P3の3区画までです')
    raw=doc.get('parking_drivable',[])
    drivable=polygon(raw) if raw else []
    result=[]; ids=set()
    for r in rows:
        key=r['id']
        if key not in ('P1','P2','P3') or key in ids:
            raise ValueError('駐車区画IDは重複のないP1/P2/P3にしてください')
        ids.add(key)
        area=polygon(r['polygon']); maneuver=polygon(r['maneuver']) if r.get('maneuver') else []
        goal=[float(v) for v in r['goal_pose']]
        entrance=[[float(v) for v in p] for p in r['entrance']]
        if len(goal)!=3 or not all(math.isfinite(v) for v in goal):
            raise ValueError('goal poseは有限のx・y・yawで指定してください')
        if len(entrance)!=2 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in entrance) or distance(*entrance)<1e-4:
            raise ValueError('入口は異なる2点で指定してください')
        if not any(all(point_segment(p,a,b)[0]<1e-6 for p in entrance) for a,b in edges(area)):
            raise ValueError('入口の2点は駐車区画の同じ辺上に置いてください')
        if not inside(goal,area) or (maneuver and not inside(goal,maneuver)):
            raise ValueError('goal poseは駐車区画・進入可能範囲の内側に置いてください')
        if drivable and not inside(goal,drivable):
            raise ValueError('駐車用の走行可能外周を設定し、goalを内側に置いてください')
        a,b=entrance
        inner=[[a[j]+(b[j]-a[j])*t for j in (0,1)] for t in (1e-6,1-1e-6)]
        for obstacle in doc.get('obstacles',[]):
            wall=polygon(obstacle['polygon'])
            if inside(goal,wall) or any(inside(p,wall) for p in entrance) or any(intersect(*inner,a,b) for a,b in edges(wall)):
                raise ValueError('goalまたは入口が壁・禁止領域に接触しています')
        item=dict(id=key,polygon=area,goal_pose=goal,entrance=entrance,maneuver=maneuver)
        result.append(item)
    if result and not drivable:
        from .parking_space import surface
        surface(doc, result)
    return dict(parking_areas=result,parking_drivable=drivable)


def warnings(doc):
    present={r['id'] for r in doc.get('parking_areas',[])}
    missing=[k for k in ('P1','P2','P3') if k not in present]
    return ['駐車areaを定義してください。未定義: '+', '.join(missing)+
            '。駐車を行わない地図なら、このまま書き出せます。'] if missing else []


def entrance_path(poses,zone):
    """One outside→inside crossing through the configured opening, never a side."""
    if 'entrance' not in zone: return True  # Legacy separate site compatibility.
    area=zone['area']; gate=zone['entrance']
    if not poses or inside(poses[0],area) or not inside(poses[-1],area): return False
    count=0
    for a,b in zip(poses,poses[1:]):
        before,after=inside(a,area),inside(b,area)
        if before==after: continue
        if before or not intersect(a,b,*gate): return False
        count+=1
    return count==1
