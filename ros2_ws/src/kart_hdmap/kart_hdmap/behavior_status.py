"""Bounded, read-only behavior diagnostics; publication freshness != sensor freshness."""
import copy
import json
import math
import time


class BehaviorStatus:
    MAX_AGE_S = .5
    STATES = {'CLEAR','FOLLOW','OBSERVE','PASS_MOVING','AVOID_SLOW','AVOID_OBSTACLE',
              'COAST_PASS','HOLD_SIDE','MERGE','BLOCKED','FAULT'}

    def __init__(self):
        self.received = self.stamp = None
        self.status = None

    def capture(self, raw, now=None):
        self.received = time.monotonic() if now is None else now
        self.status,self.stamp = None,None
        try:
            if not isinstance(raw,str) or len(raw.encode()) > 128*1024:
                return
            value = json.loads(raw)
            if (not isinstance(value,dict) or value.get('mode') not in ('off','shadow','active')
                    or value.get('state') not in self.STATES or value.get('frame_id') != 'map'
                    or type(value.get('valid')) is not bool or not isinstance(value.get('reason'),str)
                    or type(value.get('target_speed')) not in (int,float)
                    or not math.isfinite(value['target_speed']) or not 0 <= value['target_speed'] <= 5):
                return
            published = value.get('published_at_ns')
            if not isinstance(published,str) or not published.isdecimal() or len(published)>30:
                return
            stamp = int(published)
            if stamp <= 0:
                return
            def path(points):
                return (isinstance(points,list) and len(points)<=300 and all(isinstance(p,list) and len(p)==2
                        and all(type(v) in (int,float) and math.isfinite(v) for v in p) for p in points))
            if not path(value.get('path',[])):
                return
            candidates=value.get('candidates',[])
            if (not isinstance(candidates,list) or len(candidates)>2 or any(not isinstance(row,dict)
                    or row.get('side') not in ('left','right') or not path(row.get('path',[])) for row in candidates)):
                return
            # Reject all nonfinite values, including optional nested diagnostics.
            json.dumps(value,allow_nan=False)
            value['reason']=value['reason'][:512]
            self.status,self.stamp=value,stamp
        except (ValueError,TypeError,OverflowError,RecursionError):
            pass

    def view(self, available, source_now_ns, now=None):
        now = time.monotonic() if now is None else now
        age = None if self.received is None else now-self.received
        source_age = None if self.stamp is None else (source_now_ns-self.stamp)/1e9
        fresh = bool(available and self.status is not None and age is not None and 0<=age<=self.MAX_AGE_S
                     and source_age is not None and -.05<=source_age<=self.MAX_AGE_S)
        return dict(available=bool(available),fresh=fresh,age_s=None if age is None else max(0.,age),
                    fresh_for_s=max(0.,min(self.MAX_AGE_S-age,self.MAX_AGE_S-source_age)) if fresh else 0.,
                    status=copy.deepcopy(self.status) if fresh else None)
