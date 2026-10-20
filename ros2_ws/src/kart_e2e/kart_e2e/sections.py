"""Shared, ROS-free section routing in a registered map frame (meters/radians)."""
import hashlib
import json
import math
import re
from pathlib import Path


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


class Router:
    def __init__(self, config):
        self.config = config
        if config.get('schema') != 'kart.section_heads.v1':
            raise ValueError('Expected kart.section_heads.v1')
        self.heads = config['heads']
        if (not self.heads or len(self.heads) > 64 or len(set(self.heads)) != len(self.heads)
                or any(not isinstance(x, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]{0,63}', x) for x in self.heads)):
            raise ValueError('Invalid heads')
        if not config.get('frame') or not config.get('base_frame') or type(config['closed']) is not bool:
            raise ValueError('Explicit frame/base_frame and closed required')
        self.points = config['points']
        if len(self.points) < (3 if config['closed'] else 2) or any(len(p)!=2 or not all(math.isfinite(x) for x in p) for p in self.points):
            raise ValueError('Invalid reference polyline')
        self.edges, station = [], 0.
        for a,b in zip(self.points, self.points[1:]+(self.points[:1] if config['closed'] else [])):
            dx,dy = b[0]-a[0],b[1]-a[1]
            length = math.hypot(dx,dy)
            if length <= 1e-8: raise ValueError('Duplicate reference vertices')
            self.edges.append((a,dx,dy,length,station))
            station += length
        self.length = station
        self.sections = config['sections']
        ids = set()
        intervals = []
        used = set()
        for row in self.sections:
            key,a,b,head = row['id'],row['start_m'],row['end_m'],row['head']
            if not isinstance(key,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',key) or key in ids or head not in self.heads:
                raise ValueError('Invalid section ID/head')
            ids.add(key);used.add(head)
            if not all(math.isfinite(x) for x in (a,b)) or not 0<=a<station or not 0<=b<=station or a==b or (a>b and not config['closed']):
                raise ValueError('Invalid section distances')
            intervals.extend([(a,b)] if a<b else [(a,station),(0,b)])
        intervals.sort()
        if any(b>c+1e-8 for (a,b),(c,d) in zip(intervals,intervals[1:])):
            raise ValueError('Overlapping sections')
        if used != set(self.heads): raise ValueError('Every head needs a section')
        for key in ('max_distance_m','max_heading_rad','ambiguity_distance_m','max_pose_age_s','max_pose_skew_s','max_jump_m'):
            if not math.isfinite(config[key]) or config[key]<=0: raise ValueError('Invalid '+key)
        if config['max_heading_rad']>math.pi: raise ValueError('Invalid heading tolerance')
        self.sha = digest(config)
        self.previous = None

    def reset(self):
        self.previous = None

    def select(self, x, y, yaw):
        if not all(math.isfinite(v) for v in (x,y,yaw)): raise ValueError('Nonfinite pose')
        candidates = []
        for a,dx,dy,length,start in self.edges:
            t = max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/length**2))
            dist = math.hypot(x-a[0]-t*dx,y-a[1]-t*dy)
            angle = abs(math.atan2(math.sin(yaw-math.atan2(dy,dx)),math.cos(yaw-math.atan2(dy,dx))))
            if dist<=self.config['max_distance_m'] and angle<=self.config['max_heading_rad']:
                s = start+t*length
                if self.config['closed']: s %= self.length
                candidates.append((dist,s))
        if not candidates: return None
        candidates.sort()
        distance,station = candidates[0]
        def delta(a,b):
            d=abs(a-b)
            return min(d,self.length-d) if self.config['closed'] else d
        # Nearby segments in a hairpin/crossing must not silently select another section.
        if any(d<=distance+self.config['ambiguity_distance_m'] and delta(s,station)>self.config['max_jump_m'] for d,s in candidates[1:]):
            return None
        if self.previous is not None and delta(station,self.previous)>self.config['max_jump_m']:
            return None
        self.previous=station
        for row in self.sections:
            a,b=row['start_m'],row['end_m']
            if (a<=station<b if a<b else station>=a or station<b) or (not self.config['closed'] and station==self.length==b):
                return dict(section_id=row['id'],head_id=row['head'],head_index=self.heads.index(row['head']),station_m=station,distance_m=distance)
        return None


def load(path):
    config=json.loads(Path(path).read_text())
    Router(config)
    return config
