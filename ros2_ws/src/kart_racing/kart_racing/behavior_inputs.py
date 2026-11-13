"""Bounded, source-time observations for the experimental behavior router."""
from collections import deque
import math


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def fresh(stamp, received, now_ns, now, timeout):
    return (stamp is not None and stamp > 0 and received is not None
            and -.05 <= (now_ns - stamp) / 1e9 <= timeout and 0 <= now - received <= timeout)


def transform(point, tf):
    t, q = tf.translation, tf.rotation
    values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w, *point)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite_transform')
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if norm < 1e-8:
        raise ValueError('invalid_quaternion')
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    px, py, pz = point
    cx, cy, cz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return (px+w*cx+y*cz-z*cy+t.x, py+w*cy+z*cx-x*cz+t.y,
            pz+w*cz+x*cy-y*cx+t.z)


def yaw(tf):
    q = tf.rotation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('invalid_pose_quaternion')
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class Motion:
    def __init__(self):
        self.samples = deque(maxlen=32)

    def add(self, stamp, x, y):
        if self.samples and stamp <= self.samples[-1][0]:
            # Repeated camera stamps must not mature a stationary observation.
            if stamp < self.samples[-1][0]:
                self.samples.clear()
            else:
                return False
        if self.samples and (stamp-self.samples[-1][0])/1e9 > .4:
            self.samples.clear()
        self.samples.append((stamp, x, y))
        while len(self.samples) > 1 and (stamp-self.samples[0][0])/1e9 > .8:
            self.samples.popleft()
        return True

    def estimate(self, minimum=.35):
        if len(self.samples) < 3 or (self.samples[-1][0]-self.samples[0][0])/1e9 < minimum:
            return None
        times = [(row[0]-self.samples[-1][0])/1e9 for row in self.samples]
        mean = sum(times)/len(times)
        norm = sum((t-mean)**2 for t in times)
        if norm < 1e-8:
            return None
        velocity = [sum((t-mean)*row[col] for t, row in zip(times, self.samples))/norm for col in (1, 2)]
        if not all(math.isfinite(v) for v in velocity) or math.hypot(*velocity) > 20:
            return None
        return tuple(velocity)


class Observations:
    MAX_OBJECTS = 32

    def __init__(self):
        self.map_stamp = self.map_at = self.rc_stamp = self.rc_at = None
        self.map_valid, self.map_reason = False, 'waiting_map_points'
        self.rc_valid = False
        self.raw_rc = []
        self.records = {}
        self.serial = 0
        self.capacity_exceeded = False

    def _add(self, identity, kind, stamp, x, y, now):
        if identity not in self.records and len(self.records) >= self.MAX_OBJECTS:
            # Losing a retained object is not evidence that it left the course.
            self.capacity_exceeded = True
            self.map_valid,self.map_reason,self.rc_valid = False,'observation_capacity_exceeded_restart_required',False
            return None
        row = self.records.setdefault(identity, dict(id=identity, kind=kind, motion=Motion(), first_at=now))
        if row['motion'].add(stamp, x, y):
            row.update(x=float(x), y=float(y), observed_at=now, stamp=stamp)
        return row

    def map_points(self, msg, now):
        stamp = stamp_ns(msg.header)
        if self.map_stamp == stamp:
            # An invalid same-stamp clear event must replace a prior success.
            if not msg.valid:
                self.map_valid, self.map_reason = False, str(msg.reason)[:128]
            return
        if self.map_stamp is not None and stamp < self.map_stamp:
            self.records.clear()
        self.map_stamp, self.map_at = stamp, now
        if len(msg.objects) > self.MAX_OBJECTS:
            self.capacity_exceeded = True
        self.map_valid = bool(msg.valid and msg.header.frame_id == 'map' and not self.capacity_exceeded)
        self.map_reason, self.raw_rc = str(msg.reason)[:128], []
        if self.capacity_exceeded:
            self.map_reason = 'observation_capacity_exceeded_restart_required'
        if not self.map_valid:
            return
        used = set()
        for item in msg.objects:
            x, y = item.point.x, item.point.y
            if (item.class_name not in ('rc_car', 'obstacle') or not 0 <= item.score <= 1
                    or not all(math.isfinite(v) for v in (x, y, item.point.z, item.score))):
                self.map_valid, self.map_reason = False, 'invalid_map_object'
                continue
            if item.class_name == 'rc_car':
                self.raw_rc.append((x, y))
                continue
            candidates = [(math.hypot(row['x']-x, row['y']-y), identity) for identity, row in self.records.items()
                          if row['kind'] == 'obstacle' and identity not in used and 0 <= now-row['observed_at'] <= .4]
            distance, identity = min(candidates, default=(math.inf, ''))
            if distance > .35:
                self.serial += 1
                identity = f'obstacle:{self.serial}'
            used.add(identity)
            self._add(identity, 'obstacle', stamp, x, y, now)

    def rc_points(self, msg, tf, now):
        stamp = stamp_ns(msg.header)
        if self.rc_stamp == stamp:
            return
        self.rc_stamp, self.rc_at = stamp, now
        if len(msg.tracks) > self.MAX_OBJECTS:
            self.capacity_exceeded = True
            self.map_valid,self.map_reason = False,'observation_capacity_exceeded_restart_required'
        self.rc_valid = bool(msg.header.frame_id == 'base_link' and not self.capacity_exceeded)
        if not self.rc_valid:
            return
        for item in msg.tracks:
            if not item.track_id:
                self.rc_valid = False
                continue
            point = transform((item.point.x, item.point.y, item.point.z), tf)
            self._add('rc:'+str(item.track_id)[:128], 'rc_car', stamp, point[0], point[1], now)

    def objects(self, now, timeout, static_memory):
        result = []
        for identity, row in list(self.records.items()):
            if not 0 <= now-row['observed_at'] <= max(static_memory, timeout):
                del self.records[identity]
                continue
            velocity = row['motion'].estimate()
            speed = math.hypot(*velocity) if velocity is not None else 0.
            confirmed = velocity is not None
            if row['kind'] == 'obstacle':
                # The label alone does not prove an object remained stationary.
                confirmed = confirmed and speed <= .15
            result.append(dict(id=identity, kind=row['kind'], x=row['x'], y=row['y'], speed=speed,
                               vx=velocity[0] if velocity else 0., vy=velocity[1] if velocity else 0.,
                               observed_at=row['observed_at'], confirmed=confirmed))
        # Raw RC detections without a matching recent confirmed ByteTrack point
        # remain unknown objects. Detector-only operation cannot silently clear them.
        used=set()
        tracks=[row for row in result if row['kind']=='rc_car']
        for index, (x, y) in enumerate(self.raw_rc):
            matches=[(math.hypot(row['x']-x,row['y']-y),row['id']) for row in tracks
                     if row['id'] not in used
                     and 0<=now-row['observed_at']<=timeout and math.hypot(row['x']-x,row['y']-y)<.4]
            if matches:
                used.add(min(matches)[1])
            else:
                result.append(dict(id=f'untracked_rc:{index}', kind='rc_car', x=x, y=y, speed=0.,
                                   observed_at=self.map_at, confirmed=False))
        return result[:2*self.MAX_OBJECTS]
