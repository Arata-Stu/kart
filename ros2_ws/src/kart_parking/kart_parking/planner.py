"""Directed loop + bounded forward-only state-lattice search in a parking approach."""
import heapq
import itertools
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from .geometry import World, distance, inside, polygon, point_segment, wrap


@dataclass(frozen=True)
class Settings:
    wheelbase_m: float = .257
    max_steering_rad: float = .3839724354
    front_m: float = .2435
    rear_m: float = .1065
    width_m: float = .19
    margin_m: float = .05
    sample_m: float = .025
    step_m: float = .15
    xy_resolution_m: float = .075
    yaw_resolution_rad: float = .12
    max_expansions: int = 12000
    search_timeout_s: float = 8.
    departure_radius_m: float = 4.
    min_approach_m: float = .5
    parking_speed_mps: float = .25
    deceleration_mps2: float = .4
    max_cross_track_m: float = .15
    goal_position_m: float = .08
    goal_yaw_rad: float = .15
    stopped_speed_mps: float = .03
    stopped_yaw_rate_rps: float = .05
    settle_s: float = 1.
    stall_s: float = 5.
    horizon_m: float = 4.

    def validate(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive')
        if (self.max_steering_rad >= 1.3 or self.sample_m > .05
                or self.step_m < 2*self.sample_m or self.horizon_m < 1.
                or type(self.max_expansions) is not int):
            raise ValueError('Invalid search geometry/limits')
        return self

    @property
    def curvature(self):
        return math.tan(self.max_steering_rad)/self.wheelbase_m


def load_site(path, frame, slot, c):
    from kart_hdmap.document import read_document
    from kart_hdmap.parking import validate
    doc = read_document(Path(path))
    derived = None
    if doc.get('schema') == 'kart.hdmap.v2':
        if doc.get('frame_id',doc.get('frame')) != frame:
            raise ValueError('HDMap parking frame mismatch')
        fields=validate(doc)
        row=next((r for r in fields['parking_areas'] if r['id']==slot),None)
        if row is None: raise ValueError('HDMapに駐車区画 '+slot+' を定義してください')
        if not fields['parking_drivable']:
            from kart_hdmap.parking_space import surface
            derived = surface(doc, [row])
        doc=dict(schema='kart.parking.v1',frame_id=frame,drivable=fields['parking_drivable'],
                 obstacles=[o['polygon'] for o in doc.get('obstacles',[])],
                 slots={slot:dict(area=row['polygon'],goal=row['goal_pose'],entrance=row['entrance'],
                                 maneuver=row['maneuver'],auto_forward=True)})
    if doc.get('schema') != 'kart.parking.v1' or doc.get('frame_id') != frame:
        raise ValueError('Parking schema/frame mismatch; coordinates must already be in map frame')
    zone = dict(doc['slots'][slot])
    zone['area'] = polygon(zone['area'])
    zone['maneuver'] = polygon(zone['maneuver']) if zone.get('maneuver') else None
    zone['goal'] = tuple(float(v) for v in zone['goal'])
    if len(zone['goal']) != 3 or not all(math.isfinite(v) for v in zone['goal']):
        raise ValueError('goal must be finite [x,y,yaw]')
    if not zone.get('auto_forward'):
        zone['forward_yaw'] = float(zone['forward_yaw'])
        if not math.isfinite(zone['forward_yaw']):
            raise ValueError('Invalid course forward_yaw')
    if derived is None:
        world = World(doc['drivable'],doc.get('obstacles',[]),c.front_m,c.rear_m,c.width_m,c.margin_m)
    else:
        from kart_hdmap.parking_geometry import SurfaceWorld
        world = SurfaceWorld(derived,doc.get('obstacles',[]),c.front_m,c.rear_m,c.width_m,c.margin_m)
    if not world.clear(zone['goal'],zone['area']) or not world.clear(zone['goal'],zone['maneuver']):
        raise ValueError('Goal vehicle footprint does not fit inside parking area with clearance')
    return world, zone


def directed(pose, zone):
    # World-forward movement alone does not forbid wrong-way travel. Constrain
    # longitudinal progress along the explicitly configured entrance direction.
    return math.cos(wrap(pose[2]-zone['forward_yaw'])) >= -1e-8


def curve(a, b, c, world, zone):
    """Cubic tangent connections; reject cusps, reverse progress and excess curvature."""
    length = distance(a,b)
    if length < .02:
        return None
    for scale in (.35, .6, 1.):
        h = length*scale
        p = [(a[0],a[1]), (a[0]+h*math.cos(a[2]),a[1]+h*math.sin(a[2])),
             (b[0]-h*math.cos(b[2]),b[1]-h*math.sin(b[2])), (b[0],b[1])]
        # Convex-hull derivative bound makes sample displacement <= sample_m.
        count = max(2, math.ceil(3*max(distance(u,v) for u,v in zip(p,p[1:]))/c.sample_m))
        samples=[]
        for i in range(count+1):
            t=i/count; u=1-t
            xy=[u**3*p[0][j]+3*u*u*t*p[1][j]+3*u*t*t*p[2][j]+t**3*p[3][j] for j in (0,1)]
            d=[3*u*u*(p[1][j]-p[0][j])+6*u*t*(p[2][j]-p[1][j])+3*t*t*(p[3][j]-p[2][j]) for j in (0,1)]
            dd=[6*u*(p[2][j]-2*p[1][j]+p[0][j])+6*t*(p[3][j]-2*p[2][j]+p[1][j]) for j in (0,1)]
            norm=math.hypot(*d)
            pose=(*xy,math.atan2(d[1],d[0]))
            if norm < .01 or abs(d[0]*dd[1]-d[1]*dd[0])/norm**3 > .95*c.curvature or not directed(pose,zone):
                break
            samples.append(pose)
        else:
            if world.swept(samples,zone['maneuver']):
                return samples
    return None


def arc(pose, k, length, sample):
    n=math.ceil(length/sample)
    result=[pose]
    x,y,yaw=pose
    for i in range(1,n+1):
        s=length*i/n
        if abs(k)<1e-8:
            result.append((x+s*math.cos(yaw),y+s*math.sin(yaw),yaw))
        else:
            result.append((x+(math.sin(yaw+k*s)-math.sin(yaw))/k,
                           y-(math.cos(yaw+k*s)-math.cos(yaw))/k,wrap(yaw+k*s)))
    return result


def connect(start, zone, world, c, deadline):
    goal=zone['goal']
    if not directed(start,zone) or not directed(goal,zone) or not world.clear(start,zone['maneuver']):
        return None
    direct=curve(start,goal,c,world,zone)
    if direct:
        return direct
    def key(p):
        return (round(p[0]/c.xy_resolution_m),round(p[1]/c.xy_resolution_m),round(wrap(p[2])/c.yaw_resolution_rad))
    serial=itertools.count()
    # Parent IDs preserve the exact continuous pose chain even if a lattice bin
    # is improved later. All primitives travel positive arc length.
    nodes=[(start,None,[])]
    queue=[(distance(start,goal),next(serial),0.,0)]
    best={key(start):0.}
    for _ in range(c.max_expansions):
        if not queue or time.monotonic()>deadline:
            return None
        _,_,cost,index=heapq.heappop(queue)
        pose,parent,samples=nodes[index]
        if cost>best[key(pose)]+1e-9:
            continue
        tail=curve(pose,goal,c,world,zone) if distance(pose,goal)<1.5 else None
        if tail:
            chunks=[tail[1:]]
            while index:
                _,parent,samples=nodes[index]
                chunks.append(samples[1:]); index=parent
            return [start]+[p for chunk in reversed(chunks) for p in chunk]
        for factor in (-1.,-.5,0.,.5,1.):
            points=arc(pose,factor*.95*c.curvature,c.step_m,c.sample_m)
            end=points[-1]
            if not all(directed(p,zone) for p in points) or not world.swept(points,zone['maneuver']):
                continue
            new=cost+c.step_m*(1+.05*abs(factor)); k=key(end)
            if new>=best.get(k,float('inf')):
                continue
            best[k]=new
            nodes.append((end,index,points))
            heapq.heappush(queue,(new+distance(end,goal),next(serial),new,len(nodes)-1))
    return None


def course_samples(points, speeds, closed, c):
    if not closed or len(points)<3 or len(points)!=len(speeds) or len(points)>20000:
        raise ValueError('Parking requires a directed closed reference line with speeds')
    if any(not math.isfinite(v) for p in points for v in p) or any(not math.isfinite(v) or v<0 for v in speeds):
        raise ValueError('Nonfinite reference or negative speed')
    result=[]
    for i,a in enumerate(points):
        b=points[(i+1)%len(points)]; length=distance(a,b)
        if length<1e-6:
            raise ValueError('Duplicate reference point')
        yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        for j in range(math.ceil(length/c.sample_m)):
            t=j/math.ceil(length/c.sample_m)
            result.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),yaw,
                           speeds[i]+t*(speeds[(i+1)%len(points)]-speeds[i])))
        if len(result)>100000:
            raise ValueError('Reference sampling exceeds limit')
    return result


@dataclass
class Prepared:
    course: list
    departure: int
    connector: list


def legal_entrance(path, zone):
    from kart_hdmap.parking import entrance_path
    return entrance_path(path,zone)


def course_forward(points, entrance):
    """Tangent of the selected directed reference segment nearest the entrance."""
    mid=tuple((a+b)/2 for a,b in zip(*entrance))
    nearest=min(range(len(points)),key=lambda i:point_segment(mid,points[i],points[(i+1)%len(points)])[0])
    a,b=points[nearest],points[(nearest+1)%len(points)]
    return math.atan2(b[1]-a[1],b[0]-a[0])


def prepare(points, speeds, closed, zone, world, c):
    course=course_samples(points,speeds,closed,c)
    if zone.get('auto_forward'):
        # Read direction from the selected reference line, never from goal yaw.
        zone=dict(zone,forward_yaw=course_forward(points,zone['entrance']))
    poses=[p[:3] for p in course]
    if sum(distance(a,b) for a,b in zip(poses,poses[1:]+poses[:1])) <= c.horizon_m+1.:
        raise ValueError('Course too short for non-overlapping route horizon')
    if not world.swept(poses+[poses[0]]):
        raise ValueError('Course footprint/walls incompatible with parking site')
    # Try separated departures near the bay; choose a valid forward connector.
    candidates=[i for i,p in enumerate(course) if .3<distance(p,zone['goal'])<c.departure_radius_m
                and math.cos(wrap(p[2]-zone['forward_yaw']))>.8
                and (zone['maneuver'] is None or inside(p,zone['maneuver']))]
    candidates.sort(key=lambda i:distance(course[i],zone['goal']))
    chosen=[]
    deadline=time.monotonic()+c.search_timeout_s
    for i in candidates:
        if any(distance(course[i],course[j])<.25 for j in chosen):
            continue
        chosen.append(i)
        # Direct connectors first avoid spending the whole budget on one bad exit.
        path=curve(course[i][:3],zone['goal'],c,world,zone)
        if path and legal_entrance(path,zone):
            return Prepared(course,i,path)
        if time.monotonic()>deadline:
            break
    for i in chosen:
        if time.monotonic()>deadline:
            break
        path=connect(course[i][:3],zone,world,c,min(deadline,time.monotonic()+2.))
        if path and legal_entrance(path,zone):
            return Prepared(course,i,path)
    raise ValueError('No forward collision-free parking path within search limits')


def activate(prepared, pose, c):
    if len(pose)!=3 or not all(math.isfinite(v) for v in pose):
        raise ValueError("Start pose must be finite [x,y,yaw]")
    course=prepared.course; n=len(course)
    nearest=min(range(n),key=lambda i:distance(pose,course[i]))
    if distance(pose,course[nearest])>c.max_cross_track_m or abs(wrap(pose[2]-course[nearest][2]))>.6:
        raise ValueError('Vehicle is not aligned with the directed course')
    steps=(prepared.departure-nearest)%n
    approach=sum(distance(course[(nearest+j)%n],course[(nearest+j+1)%n]) for j in range(steps))
    if approach<c.min_approach_m:
        steps+=n  # Passed or too close: only the next forward lap is legal.
    route=[course[(nearest+j)%n] for j in range(steps+1)]
    route[-1]=(*prepared.connector[0],min(route[-1][3],c.parking_speed_mps))
    route += [(*p,c.parking_speed_mps) for p in prepared.connector[1:]]
    speeds=[p[3] for p in route]; speeds[-1]=0.
    for i in range(len(route)-2,-1,-1):
        speeds[i]=min(speeds[i],math.sqrt(speeds[i+1]**2+2*c.deceleration_mps2*distance(route[i],route[i+1])))
    return [(*p[:3],v) for p,v in zip(route,speeds)]
