"""Experimental reference router core for observed objects in known map geometry.

Swept assumed footprints and nominal constant-speed predictions are checked;
unobserved space is NOT proven free. This is not a collision monitor or a claim
about physical stopping, tracking accuracy, or actuator response.
"""
from bisect import bisect_right
from dataclasses import asdict, dataclass
import copy
import hashlib
import json
import math


def _finite(values):
    return all(type(value) in (int, float) and math.isfinite(value) for value in values)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Settings:
    speed_cap: float = .8
    vehicle_length: float = .35
    vehicle_width: float = .19
    rear_axle_to_rear: float = .1065
    uncertainty_margin: float = .12
    rc_length: float = .35
    rc_width: float = .19
    obstacle_length: float = .5
    obstacle_width: float = .4
    slow_speed: float = .35
    stationary_anchor_speed: float = .1
    headway: float = .7
    follow_min_gap: float = .4
    moving_merge_margin: float = .6
    static_merge_margin: float = .2
    moving_speed_advantage: float = .2
    moving_observation_s: float = .5
    object_timeout: float = .4
    static_memory_s: float = 5.
    min_transition: float = .8
    max_curvature: float = 1.2
    sample_spacing: float = .05
    lookahead_m: float = 20.
    preferred_side: str = 'left'
    deceleration: float = 1.
    prediction_horizon_s: float = 15.
    simple_pass_speed: float = 2.
    prediction_hold_s: float = 6.
    prediction_speed_uncertainty: float = .10
    prediction_acceleration: float = .05
    prediction_lateral_speed: float = .03
    prediction_uncertainty_limit: float = 1.5
    max_pass_heading_change: float = .25

    def __post_init__(self):
        bounds = {'speed_cap': (.05, 5), 'vehicle_length': (.1, 2), 'vehicle_width': (.05, 1),
                  'rear_axle_to_rear': (0, 1), 'uncertainty_margin': (.02, .5), 'rc_length': (.1, 2), 'rc_width': (.05, 1),
                  'obstacle_length': (.1, 3), 'obstacle_width': (.05, 2), 'slow_speed': (.05, 1.5),
                  'stationary_anchor_speed': (0, .3), 'headway': (.2, 3), 'follow_min_gap': (.1, 3),
                  'moving_merge_margin': (.2, 3), 'static_merge_margin': (.05, 1.5),
                  'moving_speed_advantage': (.05, 2), 'moving_observation_s': (.2, 5),
                  'object_timeout': (.1, 1), 'static_memory_s': (.5, 15), 'min_transition': (.8, 5),
                  'max_curvature': (.1, 3), 'sample_spacing': (.02, .1), 'lookahead_m': (3, 50),
                  'deceleration': (.1, 5), 'prediction_horizon_s': (2, 20),
                  'simple_pass_speed': (.1, 5), 'prediction_hold_s': (.5, 10),
                  'prediction_speed_uncertainty': (.01, 1), 'prediction_acceleration': (.01, 2),
                  'prediction_lateral_speed': (.01, .5), 'prediction_uncertainty_limit': (.1, 3),
                  'max_pass_heading_change': (.05, .6)}
        for key, (low, high) in bounds.items():
            value = getattr(self, key)
            if not _finite([value]) or not low <= value <= high:
                raise ValueError(f'{key} must be finite in [{low},{high}]')
        if (self.preferred_side not in ('left', 'right') or self.stationary_anchor_speed > self.slow_speed
                or self.static_merge_margin > self.moving_merge_margin or self.static_memory_s < self.object_timeout
                or self.rear_axle_to_rear >= self.vehicle_length or self.prediction_hold_s<self.object_timeout):
            raise ValueError('Invalid side, slow/static thresholds, merge margins, memory TTL or rear-axle body extents')


class _Reference:
    def __init__(self, points, speeds, closed):
        if type(closed) is not bool or not isinstance(points, (list, tuple)) or not 2 <= len(points) <= 5000:
            raise ValueError('Reference needs 2..5000 finite XY points and a closed bool')
        self.points = [tuple(point) for point in points]
        self.speeds = list(speeds)
        if (len(self.speeds) != len(self.points) or any(len(point) != 2 or not _finite(point) for point in self.points)
                or not _finite(self.speeds) or any(not 0 <= speed <= 20 for speed in self.speeds)):
            raise ValueError('Invalid reference coordinates or speeds')
        if closed and self.points[-1] == self.points[0]:
            self.points.pop()
            self.speeds.pop()
        if closed and len(self.points) < 3:
            raise ValueError('Closed reference needs three distinct points')
        self.closed = closed
        self.segments, self.starts = [], [0.]
        for index in range(len(self.points) if closed else len(self.points) - 1):
            a, b = self.points[index], self.points[(index + 1) % len(self.points)]
            length = math.dist(a, b)
            if not math.isfinite(length) or length < 1e-5:
                raise ValueError('Degenerate reference segment')
            unit = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
            self.segments.append((a, unit, length, min(self.speeds[index], self.speeds[(index + 1) % len(self.points)])))
            self.starts.append(self.starts[-1] + length)
        self.length = self.starts[-1]

    def at(self, station, offset=0.):
        s = station % self.length if self.closed else min(self.length, max(0., station))
        index = min(len(self.segments) - 1, bisect_right(self.starts, s) - 1)
        a, unit, _, speed = self.segments[index]
        along = s - self.starts[index]
        return (a[0] + unit[0] * along - unit[1] * offset,
                a[1] + unit[1] * along + unit[0] * offset, math.atan2(unit[1], unit[0]), speed)

    def project(self, xy, near=None):
        candidates = []
        for index, (a, unit, length, _) in enumerate(self.segments):
            dx, dy = xy[0] - a[0], xy[1] - a[1]
            along = max(0., min(length, dx * unit[0] + dy * unit[1]))
            ex, ey = dx - along * unit[0], dy - along * unit[1]
            station = self.starts[index] + along
            if self.closed and near is not None:
                station = near + (station - near + self.length / 2) % self.length - self.length / 2
            candidates.append((math.hypot(ex, ey), station, unit[0] * ey - unit[1] * ex))
        best = min(candidates)
        for distance, station, _ in candidates:
            difference = abs(station - best[1])
            if self.closed:
                difference = min(difference % self.length, self.length - difference % self.length)
            if distance < best[0] + .1 and difference > 1.:
                raise ValueError('Ambiguous reference projection')
        return best[1], best[2], best[0]


def _curvature(path):
    result = 0.
    for a, b, c in zip(path, path[1:], path[2:]):
        denominator = math.dist(a, b) * math.dist(b, c) * math.dist(a, c)
        if denominator < 1e-12:
            return math.inf
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        result = max(result, abs(2 * cross / denominator))
    return result


def _smooth(value):
    value = max(0., min(1., value))
    return value**3 * (10 + value * (-15 + 6 * value))


class Planner:
    MAX_OBJECTS = 64
    MAX_POINTS = 300

    def __init__(self, settings=None, model_factory=None):
        self.settings = settings or Settings()
        if not isinstance(self.settings, Settings):
            raise ValueError('Settings instance required')
        self.reference = self.world = None
        self.revision = self.signature = None
        self.settings_signature = asdict(self.settings)
        self.active = None
        self.last_path, self.last_speeds, self.last_candidates = [], [], []
        self.fault = None
        self.known, self.history = {}, {}
        self.ego_station = self.last_now = None
        self.route_number = 0
        self.started = False
        self.zones,self.models,self.model_stamps = [],{},{}
        self.model_factory = model_factory
        self.permission = None
        self.planning_mode = 'follow'

    def invalidate(self, reason):
        """External TF/source/clock invalidation; initial waiting cannot arm it."""
        if self.started:
            self._latch('external_input:' + str(reason)[:160])
        if self.fault:
            state, detail, path = self.fault
            return self._result(state, detail, path, target=None if self.active is None else self.active['target'],
                                side=None if self.active is None else self.active['side'])
        return self._result('OBSERVE', 'initial_waiting:' + str(reason)[:160])

    def set_reference(self, points, speeds, closed, map_document, lane_id, revision, line_type='centerline'):
        try:
            return self._set_reference(points, speeds, closed, map_document, lane_id, revision, line_type)
        except (ValueError, TypeError, KeyError, IndexError, ArithmeticError):
            if self.signature is not None:
                self._latch('invalid_reference_update')
            raise

    def _set_reference(self, points, speeds, closed, map_document, lane_id, revision, line_type):
        from kart_hdmap.parking_geometry import SurfaceWorld
        from kart_hdmap.parking_space import surface
        from shapely.geometry import Polygon
        from kart_hdmap.passing_zones import validate_passing_zones
        if self.signature is not None and asdict(self.settings) != self.settings_signature:
            self._latch('parameters_changed')
            return
        reference = _Reference(points, speeds, closed)
        if not isinstance(map_document, dict) or map_document.get('schema') not in ('kart.hdmap.v2', 'kart.hdmap.visualization.v1'):
            raise ValueError('Normalized HDMap or Studio v2 document required')
        if map_document.get('frame_id', map_document.get('frame')) != 'map':
            raise ValueError('Behavior geometry must already be in map frame')
        lane = next((lane for lane in map_document.get('lanes', []) if lane['id'] == lane_id), None)
        if lane is None or lane.get('closed') is not closed:
            raise ValueError('Reference lane/closed geometry mismatch')
        if (len(lane.get('left', [])) > 5000 or len(lane.get('right', [])) > 5000
                or len(map_document.get('obstacles', [])) > 100):
            raise ValueError('Map geometry exceeds bounded planner inputs')
        line=lane.get('lines',{}).get(line_type,{})
        zones=validate_passing_zones(line.get('passing_zones',[]),points,closed)
        if zones and line.get('points') != [list(point) for point in points]:
            raise ValueError('Passing permissions require the exact selected map line')
        lane = copy.deepcopy({key: lane[key] for key in ('id', 'closed', 'left', 'right')})
        document = dict(obstacles=copy.deepcopy(map_document.get('obstacles', [])))
        signature = hashlib.sha256(json.dumps(dict(points=points, speeds=speeds, closed=closed,
            lane=lane, obstacles=document.get('obstacles', []), revision=revision,
            map_revision=map_document.get('revision'),zones=zones,line_type=line_type), sort_keys=True, allow_nan=False).encode()).hexdigest()
        if self.signature is not None and signature != self.signature:
            self._latch('reference_geometry_or_revision_changed')
            return
        corridor = surface({'lanes': [lane]}, [])
        self.world = SurfaceWorld(corridor, [row['polygon'] for row in document.get('obstacles', [])],
                                  self.settings.vehicle_length - self.settings.rear_axle_to_rear,
                                  self.settings.rear_axle_to_rear,
                                  self.settings.vehicle_width, self.settings.uncertainty_margin)
        self.map_obstacles = [(row['id'], Polygon(row['polygon'])) for row in document.get('obstacles', [])]
        self.reference, self.revision, self.signature = reference, revision, signature
        self.zones=[]
        for zone in zones:
            start,end=reference.starts[zone['start_index']],reference.starts[zone['end_index']]
            if closed and end<=start:
                end+=reference.length
            self.zones.append(dict(zone,start_s=start,end_s=end))
        self.settings_signature = asdict(self.settings)

    def _permission(self, station, side=None):
        matches=[]
        for original in self.zones:
            zone=dict(original)
            if self.reference.closed:
                shift=math.floor((station-zone['start_s'])/self.reference.length)*self.reference.length
                zone['start_s']+=shift;zone['end_s']+=shift
            if (zone['start_s']-1e-8<=station<zone['end_s']-1e-8
                    and (side is None or zone['side'] in ('both',side))):
                matches.append(zone)
        return max(matches,key=lambda row:row['end_s'],default=None)

    def _model(self, identity):
        if identity not in self.models:
            if len(self.models)>=self.MAX_OBJECTS:
                removable=[key for key in self.models if not self.active or key!=self.active['target']['id']]
                oldest=min(removable,key=lambda key:self.model_stamps.get(key,-math.inf))
                self.models.pop(oldest,None)
                self.model_stamps.pop(oldest,None)
            if self.model_factory is None:
                from .opponent_model import OpponentModel
                self.model_factory=OpponentModel
            self.models[identity]=self.model_factory(self.reference.length)
        return self.models[identity]

    def _train(self, item):
        if item['kind']!='rc_car' or not item['confirmed'] or item['observed_at']<=self.model_stamps.get(item['id'],-math.inf):
            return
        model=self._model(item['id'])
        identity=int.from_bytes(hashlib.sha256(item['id'].encode()).digest()[:8],'big')
        model.observe(identity,item['observed_at'],item['s'],item['d'],item['speed'],
                      self.settings.uncertainty_margin**2,self.settings.prediction_speed_uncertainty**2)
        self.model_stamps[item['id']]=item['observed_at']

    def _uses_learned(self,item):
        return bool(item['kind']=='rc_car' and (item['speed']>self.settings.simple_pass_speed or
            (self.active and self.active['target']['id']==item['id']
             and self.active['target']['speed']>self.settings.simple_pass_speed)))

    def _trajectory(self, item, age):
        """Bounded GP propagation; predictions never become training samples."""
        model=self.models.get(item['id'])
        item['model_ready']=bool(model and model.available and model.ready())
        item['age_s']=age
        item['uncertainty']= (self.settings.prediction_speed_uncertainty*age
            +.5*self.settings.prediction_acceleration*age*age) if age>self.settings.object_timeout else 0.
        item['uncertainty']+=item.get('phase_uncertainty',0.)
        item['lateral_uncertainty']=self.settings.prediction_lateral_speed*age if age>self.settings.object_timeout else 0.
        if item['uncertainty']>self.settings.prediction_uncertainty_limit:
            raise ValueError('prediction_uncertainty_exceeded')
        if not item['model_ready'] or not self._uses_learned(item) or item['anchored']:
            return
        samples=[]
        station=item['s']
        for index in range(math.ceil(self.settings.prediction_horizon_s/.1)+1):
            prediction=model.predict(station)
            if not prediction['valid']:
                item['model_ready']=False
                item['model_reason']='learned_prediction_unavailable'
                return
            samples.append((station,prediction['d'],prediction['v'],prediction['d_sigma'],prediction['v_sigma']))
            station+=prediction['v']*.1
        item['_trajectory']=samples

    def _latch(self, reason, state='FAULT'):
        if self.fault is None:
            self.fault = (state, reason, [list(point) for point in self.last_path])

    def _path(self, start, end, offset=None):
        end = min(end, start + self.settings.lookahead_m)
        if not self.reference.closed:
            end = min(end, self.reference.length)
        if end - start < .05:
            raise ValueError('Reference end reached')
        count = min(self.MAX_POINTS, max(2, math.ceil((end - start) / self.settings.sample_spacing) + 1))
        stations = [start + (end - start) * index / (count - 1) for index in range(count)]
        samples = [self.reference.at(s, offset(s) if offset else 0.) for s in stations]
        return [[p[0], p[1]] for p in samples], [min(p[3], self.settings.speed_cap) for p in samples], stations

    def _objects(self, now, objects, ego_s):
        if not isinstance(objects, list) or len(objects) > self.MAX_OBJECTS:
            raise ValueError('Object array must contain at most 64 entries')
        seen, accepted = set(), []
        for raw in objects:
            if (not isinstance(raw, dict) or not isinstance(raw.get('id'), str) or not 0 < len(raw['id']) <= 128
                    or raw['id'] in seen or raw.get('kind') not in ('rc_car', 'obstacle')
                    or type(raw.get('confirmed')) is not bool
                    or not _finite([raw.get(key) for key in ('x', 'y', 'speed', 'observed_at')])
                    or not 0 <= raw['speed'] <= 20 or raw['observed_at'] > now + .05):
                raise ValueError('Invalid/duplicate object or observation timestamp')
            identity, kind = raw['id'], raw['kind']
            seen.add(identity)
            if self.active and self.active['target']['id']==identity and not raw['confirmed']:
                # Reacquisition must rebuild measured motion before it can move
                # the committed target or authorize an inward merge.
                continue
            length = raw.get('length', self.settings.rc_length if kind == 'rc_car' else self.settings.obstacle_length)
            width = raw.get('width', self.settings.rc_width if kind == 'rc_car' else self.settings.obstacle_width)
            if not _finite([length, width]) or not .05 <= min(length, width) or max(length, width) > 3:
                raise ValueError('Invalid assumed object footprint')
            if kind == 'obstacle' and raw['speed'] > self.settings.stationary_anchor_speed:
                raise ValueError('An obstacle cannot silently become a moving object')
            item = {key: raw[key] for key in ('id', 'kind', 'x', 'y', 'speed', 'observed_at', 'confirmed')}
            item.update(length=length, width=width,phase_uncertainty=0.)
            station, offset, _ = self.reference.project((item['x'], item['y']), ego_s)
            item.update(s=station, d=offset)
            measured_speed = item['speed']
            item['lateral_speed'] = 0.
            if 'vx' in raw or 'vy' in raw:
                if not _finite([raw.get('vx'), raw.get('vy')]):
                    raise ValueError('Both finite vx/vy are required')
                yaw = self.reference.at(station)[2]
                along = raw['vx'] * math.cos(yaw) + raw['vy'] * math.sin(yaw)
                lateral = -raw['vx'] * math.sin(yaw) + raw['vy'] * math.cos(yaw)
                if abs(math.hypot(raw['vx'], raw['vy']) - measured_speed) > max(.05, .15 * measured_speed):
                    raise ValueError('RC speed and world velocity disagree')
                if kind == 'rc_car' and (along < -.05 or abs(lateral) > .15 or
                                        (measured_speed > self.settings.slow_speed and along < .9 * measured_speed)):
                    raise ValueError('Opposing or crossing RC motion is outside same-direction behaviour')
                item['speed'], item['lateral_speed'] = max(0., along), lateral
            self._train(item)
            anchored = item['confirmed'] and (kind == 'obstacle' or item['speed'] <= self.settings.stationary_anchor_speed)
            previous = self.known.get(identity)
            if previous and previous['anchored']:
                if (not anchored or math.dist((item['x'], item['y']), (previous['x'], previous['y'])) > self.settings.uncertainty_margin
                        or (length, width, kind) != (previous['length'], previous['width'], previous['kind'])):
                    raise ValueError('Fixed-world object anchor changed')
                anchor_s, anchor_d, _ = self.reference.project((previous['x'], previous['y']), ego_s)
                item.update(x=previous['x'], y=previous['y'], s=anchor_s, d=anchor_d)
            committed=bool(self.active and self.active['target']['id']==identity)
            ttl = (self.settings.static_memory_s if anchored and previous and previous['anchored'] else
                   self.settings.prediction_hold_s if committed and previous and previous['confirmed'] else self.settings.object_timeout)
            if now - item['observed_at'] > ttl:
                raise ValueError('Object observation expired')
            item['anchored'] = anchored
            if anchored:
                item['anchor_motion_bound'] = max(measured_speed, previous.get('anchor_motion_bound', 0.) if previous else 0.)
                if item['anchor_motion_bound'] * max(0., now - item['observed_at']) > self.settings.uncertainty_margin:
                    raise ValueError('Fixed-world motion uncertainty expired')
                item['speed'] = 0.
            else:
                age=max(0.,now-item['observed_at'])
                model=self.models.get(identity)
                if self._uses_learned(item) and model and model.available and model.ready():
                    remaining=age
                    while remaining>1e-8:
                        prediction=model.predict(item['s'])
                        if not prediction['valid']:
                            raise ValueError('learned_prediction_unavailable')
                        step=min(.1,remaining)
                        item['s']+=prediction['v']*step
                        item['phase_uncertainty']+=2*prediction['v_sigma']*step
                        remaining-=step
                    prediction=model.predict(item['s'])
                    item['d']=prediction['d']
                else:
                    item['s'] += item['speed'] * age
                item['d'] += item['lateral_speed'] * max(0., now - item['observed_at'])
                p = self.reference.at(item['s'], item['d'])
                item['x'], item['y'] = p[:2]
            self._trajectory(item,max(0.,now-item['observed_at']))
            first, last = self.history.get(identity, (item['observed_at'], item['observed_at']))
            if not item['confirmed'] or (previous and not previous['confirmed']):
                first = item['observed_at']
            if item['observed_at'] < last or item['observed_at'] - last > self.settings.object_timeout:
                first = item['observed_at']
            if previous and not anchored:
                expected = previous['s'] + previous['speed'] * max(0., now - previous['updated_at'])
                if (abs(item['speed'] - previous['speed']) > .05 or abs(item['d'] - previous['d']) > self.settings.uncertainty_margin
                        or abs(item['s'] - expected) > self.settings.uncertainty_margin):
                    first = item['observed_at']
            self.history[identity] = (first, max(last, item['observed_at']))
            item['updated_at'] = now
            if identity not in self.known and len(self.known) >= self.MAX_OBJECTS:
                raise ValueError('Bounded object memory capacity exceeded')
            self.known[identity] = dict(item)
            accepted.append(item)
        for identity, item in list(self.known.items()):
            if identity in seen and any(obj['id'] == identity for obj in accepted):
                continue
            delta = item['s'] - ego_s
            relevant = -.5 <= delta <= self.settings.lookahead_m or (self.active and self.active['target']['id'] == identity)
            age = now - item['observed_at']
            if not item['confirmed'] and age>self.settings.object_timeout:
                # Debounce fleeting proposals; never-confirmed IDs do not
                # become permanent tracked obstacles merely by disappearing.
                self.known.pop(identity,None)
                self.history.pop(identity,None)
                continue
            if (item['anchored'] and age <= self.settings.static_memory_s
                    and item.get('anchor_motion_bound', 0.) * age <= self.settings.uncertainty_margin):
                accepted.append(dict(item))
            elif (self.active and self.active['target']['id']==identity and item['confirmed'] and not item['anchored']
                    and age<=self.settings.prediction_hold_s):
                predicted=dict(item)
                elapsed=max(0.,now-item['updated_at'])
                model=self.models.get(identity)
                if self._uses_learned(predicted) and model and model.available and model.ready():
                    remaining=elapsed
                    phase_uncertainty=predicted.get('phase_uncertainty',0.)
                    while remaining>1e-8:
                        prediction=model.predict(predicted['s'])
                        if not prediction['valid']:
                            raise ValueError('learned_prediction_unavailable')
                        step=min(.1,remaining)
                        predicted['s']+=prediction['v']*step
                        phase_uncertainty+=2*prediction['v_sigma']*step
                        remaining-=step
                    prediction=model.predict(predicted['s'])
                    if not prediction['valid']:
                        raise ValueError('learned_prediction_unavailable')
                    predicted['d']=prediction['d']+predicted.get('lateral_speed',0.)*age
                    predicted['phase_uncertainty']=phase_uncertainty
                else:
                    predicted['s']+=predicted['speed']*elapsed
                    predicted['d']+=predicted.get('lateral_speed',0.)*elapsed
                predicted['x'],predicted['y']=self.reference.at(predicted['s'],predicted['d'])[:2]
                predicted['updated_at']=now
                self._trajectory(predicted,age)
                self.known[identity]=dict(predicted)
                accepted.append(predicted)
            elif relevant:
                if not item['confirmed'] and age<=self.settings.object_timeout:
                    accepted.append(dict(item))
                else:
                    raise ValueError('Known target lost or fixed-world memory expired')
            else:
                self.known.pop(identity, None)
                self.history.pop(identity, None)
                # Learning memory is independent of the short-lived visible
                # object table; bounded eviction happens when a new model enters.
        # Registered exclusions have exact map polygons in addition to these
        # conservative route-relative boxes used only for choosing a manoeuvre.
        for identity, polygon in self.map_obstacles:
            centre = polygon.centroid
            station, offset, _ = self.reference.project((centre.x, centre.y), ego_s)
            yaw = self.reference.at(station)[2]
            u, v = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
            coordinates = list(polygon.exterior.coords)
            along = [x * u[0] + y * u[1] for x, y in coordinates]
            lateral = [x * v[0] + y * v[1] for x, y in coordinates]
            accepted.append(dict(id='map:' + identity, kind='obstacle', x=centre.x, y=centre.y,
                s=station, d=offset, speed=0., observed_at=now, confirmed=True, anchored=True,
                length=max(.05, max(along) - min(along)), width=max(.05, max(lateral) - min(lateral)),
                age_s=0.,uncertainty=0.,lateral_uncertainty=0.))
        return accepted

    def _object_pose(self, item, seconds):
        if item['anchored']:
            return item['x'], item['y'], self.reference.at(item['s'])[2]
        samples=item.get('_trajectory')
        if samples:
            index=min(len(samples)-2,int(seconds/.1))
            fraction=max(0.,min(1.,seconds/.1-index))
            a,b=samples[index],samples[index+1]
            return self.reference.at(a[0]+(b[0]-a[0])*fraction,a[1]+(b[1]-a[1])*fraction)[:3]
        return self.reference.at(item['s'] + item['speed'] * seconds,
                                 item['d'] + item.get('lateral_speed', 0.) * seconds)[:3]

    def _object_bounds(self,item,seconds):
        longitudinal=item.get('uncertainty',0.)
        lateral=item.get('lateral_uncertainty',0.)
        if item.get('_trajectory'):
            sample=item['_trajectory'][min(len(item['_trajectory'])-1,int(seconds/.1))]
            longitudinal+=2*sample[4]*seconds
            lateral+=2*sample[3]
        return longitudinal,lateral

    def _check(self, path, speeds, objects, ego=None):
        from kart_hdmap.parking_geometry import footprint
        from shapely.geometry import Polygon
        if len(path) < 2:
            return False, 'route_too_short', 0., 0.
        curvature = _curvature(path)
        if not math.isfinite(curvature) or curvature > self.settings.max_curvature + 1e-6:
            return False, 'curvature_limit', 0., curvature if math.isfinite(curvature) else 1e6
        poses = [(point[0], point[1], math.atan2(path[min(i + 1, len(path) - 1)][1] - path[max(0, i - 1)][1],
                                                       path[min(i + 1, len(path) - 1)][0] - path[max(0, i - 1)][0]))
                 for i, point in enumerate(path)]
        times = [0.]
        for i in range(1, len(path)):
            if min(speeds[i - 1], speeds[i]) <= 0:
                return False, 'zero_reference_speed', 0., curvature
            times.append(times[-1] + math.dist(path[i - 1], path[i]) / min(speeds[i - 1], speeds[i]))
        if times[-1] > self.settings.prediction_horizon_s and any(obj['speed'] > 0 for obj in objects):
            return False, 'prediction_horizon', 0., curvature
        if ego is not None:
            poses.insert(0, ego[:3])
            times.insert(0, 0.)
        minimum = 1e6
        for index, (a, b) in enumerate(zip(poses, poses[1:])):
            motion = math.dist(a[:2], b[:2]) + self.world.radius * abs(_wrap(b[2] - a[2]))
            extra = motion / 2
            for pose, seconds in ((a, times[index]), (b, times[index + 1])):
                body = Polygon(footprint(pose, self.settings.vehicle_length - self.settings.rear_axle_to_rear,
                                         self.settings.rear_axle_to_rear, self.settings.vehicle_width))
                clearance = self.world.surface.boundary.distance(body) - self.settings.uncertainty_margin - extra
                minimum = min(minimum, clearance)
                if not self.world.clear(pose, extra=extra):
                    return False, 'boundary_or_registered_obstacle', min(minimum, 0.), curvature
                for obj in objects:
                    first = self._object_pose(obj, times[index])
                    last = self._object_pose(obj, times[index + 1])
                    predicted = self._object_pose(obj, seconds)
                    radius = math.hypot(obj['length'] / 2, obj['width'] / 2)
                    uncertainty = (math.dist(first[:2], last[:2]) + radius * abs(_wrap(last[2] - first[2]))) / 2
                    along_bound,lateral_bound=self._object_bounds(obj,seconds)
                    shape = Polygon(footprint(predicted, obj['length'] / 2+along_bound,
                                              obj['length'] / 2+along_bound,obj['width']+2*lateral_bound))
                    clearance = body.distance(shape) - self.settings.uncertainty_margin - extra - uncertainty
                    minimum = min(minimum, clearance)
                    if body.intersects(shape) or clearance <= 0:
                        return False, 'object_collision:' + obj['id'], minimum, curvature
        return True, 'swept_assumed_geometry_clear', minimum, curvature

    def _blocking(self, objects, ego_s):
        return sorted([obj for obj in objects if -.1 <= obj['s'] - ego_s <= self.settings.lookahead_m
                       and abs(obj['d']) < (obj['width'] + self.settings.vehicle_width) / 2 + self.settings.uncertainty_margin],
                      key=lambda obj: (obj['s'], obj['id']))

    def _follow(self, state, reason, ego_s, ego, objects, target, cap):
        horizon = self.settings.lookahead_m if cap <= .01 else min(self.settings.lookahead_m,
                                                                 cap * self.settings.prediction_horizon_s * .8)
        path, speeds, _ = self._path(ego_s, ego_s + max(.1, horizon))
        speeds = [min(speed, cap) for speed in speeds]
        if cap > .01:
            valid, issue, _, _ = self._check(path, speeds, objects, ego)
            if not valid:
                return self._result('BLOCKED', 'follow_route_invalid:' + issue, path, target=target)
        elif _curvature(path) > self.settings.max_curvature or not self.world.clear(ego[:3]):
            return self._result('BLOCKED', 'stopping_route_geometry_invalid', path, target=target)
        return self._result(state, reason, path, speeds, target)

    def _merge_gap(self,target):
        moving=self._uses_learned(target)
        return (self.settings.rear_axle_to_rear+target['length']/2+self.settings.uncertainty_margin
                +(self.settings.moving_merge_margin+self.settings.headway*target['speed'] if moving
                  else self.settings.static_merge_margin)+target.get('uncertainty',0.))

    def _committed(self, now, ego_s, ego_d, ego, observed, target):
        active=self.active
        if self._permission(ego_s,active['side']) is None:
            raise ValueError('committed_permission_expired')
        self.permission=dict(zone_id=active['zone_id'],side=active['side'],
                             remaining_m=max(0.,active['zone_end_s']-ego_s))
        if active.get('merging'):
            if ego_s>=active['merge_end_s'] and abs(ego_d)<=.1:
                self.active=None
                self.last_candidates=[]
                return None
            path=active['path']
            nearest=min(range(len(path)),key=lambda i:math.dist(ego[:2],path[i]))
            if math.dist(ego[:2],path[nearest])>.35:
                raise ValueError('ego_left_adopted_route')
            start=max(0,nearest-1)
            path,speeds=path[start:],active['_speeds'][start:]
            valid,reason,_,_=self._check(path,speeds,observed,ego)
            if not valid:
                self._latch('adopted_route_invalid:'+reason,'BLOCKED')
                return self._result('BLOCKED','adopted_route_invalid:'+reason)
            return self._result('MERGE','merge_clearance_verified',path,speeds,target,active['side'])
        ramp=active['ramp']
        clear=(ego_s>=active['out_end_s'] and ego_s-target['s']>=self._merge_gap(target))
        if clear:
            end=ego_s+ramp+.3
            if end>active['zone_end_s']:
                return self._result('BLOCKED','permission_interval_too_short',target=target,side=active['side'])
            path,speeds,_=self._path(ego_s,end,lambda s:ego_d*(1-_smooth((s-ego_s)/ramp)))
            valid,reason,_,_=self._check(path,speeds,observed,ego)
            if valid:
                active.update(merging=True,path=path,_speeds=speeds,merge_start_s=ego_s,merge_end_s=ego_s+ramp)
                return self._result('MERGE','merge_clearance_verified',path,speeds,target,active['side'])
        # Publish the outward/parallel part only. A PP lookahead must never see
        # an inward merge before the live rear/front clearance gate succeeds.
        end=min(active['zone_end_s']-.3,ego_s+self.settings.lookahead_m,
                max(active['planned_end_s']+.5,ego_s+ego[3]**2/(2*self.settings.deceleration)+.5))
        if end-ego_s<.1:
            return self._result('BLOCKED','parallel_corridor_exhausted',target=target,side=active['side'])
        def outward(station):
            return active['start_d']+(active['offset']-active['start_d'])*_smooth((station-active['start_s'])/ramp)
        path,speeds,_=self._path(ego_s,end,outward)
        expected=outward(ego_s)
        if abs(ego_d-expected)>.35:
            raise ValueError('ego_left_adopted_route')
        headings=[self.reference.at(ego_s+(end-ego_s)*i/(len(path)-1))[2] for i in range(len(path))]
        if max(abs(_wrap(yaw-headings[0])) for yaw in headings)>self.settings.max_pass_heading_change:
            return self._result('BLOCKED','straight_section_required',target=target,side=active['side'])
        valid,reason,_,_=self._check(path,speeds,observed,ego)
        if not valid:
            self._latch('adopted_route_invalid:'+reason,'BLOCKED')
            return self._result('BLOCKED','adopted_route_invalid:'+reason)
        # A finite parallel corridor ends at zero, even when the merge remains
        # unavailable. Deceleration is a planning assumption, not brake proof.
        stations=[ego_s+(end-ego_s)*i/(len(path)-1) for i in range(len(path))]
        speeds=[min(speed,math.sqrt(max(0.,2*self.settings.deceleration*(end-s))))
                for speed,s in zip(speeds,stations)]
        active['path'],active['_speeds']=path,speeds
        missing=target.get('age_s',0.)>self.settings.object_timeout
        state='HOLD_SIDE' if ego_s>=active['planned_merge_start_s'] else 'COAST_PASS' if missing else active['state']
        reason='merge_clearance_pending' if state=='HOLD_SIDE' else 'committed_prediction' if missing else 'holding_committed_side'
        return self._result(state,reason,path,speeds,target,active['side'])

    def _candidate(self, side, ego_s, ego_d, target, blocking, objects, ego):
        zone=self._permission(ego_s,side)
        if zone is None:
            return dict(side=side,feasible=False,reason='passing_not_authorized',path=[],min_clearance_m=0.)
        sign = 1 if side == 'left' else -1
        candidates=[]
        for obj in blocking:
            lateral=[obj['d']]
            if obj.get('_trajectory'):
                lateral=[row[1]+sign*2*row[3] for row in obj['_trajectory']]
            edge=max(lateral) if sign>0 else min(lateral)
            candidates.append(edge+sign*((obj['width']+self.settings.vehicle_width)/2
                +self.settings.uncertainty_margin+.1+obj.get('lateral_uncertainty',0.)
                +(self.settings.prediction_lateral_speed*self.settings.prediction_hold_s if not obj['anchored'] else 0.)))
        offset = max(candidates) if sign > 0 else min(candidates)
        transition = max(self.settings.min_transition, math.sqrt(6 * max(abs(offset - ego_d), abs(offset)) / self.settings.max_curvature))
        cap = min(self.settings.speed_cap, self.reference.at(ego_s)[3])
        merge = transition
        for obj in blocking:
            moving = obj['kind'] == 'rc_car' and obj['speed'] > self.settings.simple_pass_speed
            margin = self.settings.moving_merge_margin + self.settings.headway * obj['speed'] if moving else self.settings.static_merge_margin
            clearance = self.settings.rear_axle_to_rear + obj['length'] / 2 + self.settings.uncertainty_margin + margin
            # Extra distance covers the lateral path's nominal travel time.
            if obj['speed']>0:
                clearance += self.settings.uncertainty_margin
            predicted_speed=max((row[2]+2*row[4] for row in obj.get('_trajectory',[])),default=obj['speed'])
            denominator = 1 - predicted_speed / cap if cap > 0 else -1
            if denominator <= .01:
                return dict(side=side, feasible=False, reason='insufficient_speed_advantage', path=[], min_clearance_m=0.)
            merge = max(merge, (obj['s'] - ego_s + clearance) / denominator)
        end = merge + transition
        data = dict(side=side, feasible=False, reason='route_too_long', path=[], min_clearance_m=0.,
                    offset=offset, out_end_s=ego_s + transition, merge_start_s=ego_s + merge,
                    merge_end_s=ego_s + end, merge_start=list(self.reference.at(ego_s + merge, offset)[:2]),
                    merge_end=list(self.reference.at(ego_s + end)[:2]), max_curvature=0.,zone_id=zone['id'],zone_end_s=zone['end_s'])
        if ego_s+end+.3>zone['end_s']:
            data['reason']='permission_interval_too_short'
            return data
        if end + .3 > self.settings.lookahead_m or (not self.reference.closed and ego_s + end + .3 > self.reference.length):
            return data
        def lateral(station):
            distance = station - ego_s
            if distance <= transition:
                return ego_d + (offset - ego_d) * _smooth(distance / transition)
            if distance <= merge:
                return offset
            return offset * (1 - _smooth((distance - merge) / transition))
        path, speeds, stations = self._path(ego_s, ego_s + end + .3, lateral)
        headings=[self.reference.at(station)[2] for station in stations]
        if max(abs(_wrap(heading-headings[0])) for heading in headings)>self.settings.max_pass_heading_change:
            data.update(reason='straight_section_required',path=path)
            return data
        elapsed = 0.
        for index in range(1, len(stations)):
            elapsed += math.dist(path[index - 1], path[index]) / max(.01, min(speeds[index - 1], speeds[index]))
            if stations[index] >= ego_s + merge:
                for obj in blocking:
                    if obj['kind'] == 'rc_car':
                        moving=obj['speed']>self.settings.simple_pass_speed
                        required = (self.settings.rear_axle_to_rear + obj['length'] / 2 + self.settings.uncertainty_margin
                                    +(self.settings.moving_merge_margin+self.settings.headway*obj['speed'] if moving else self.settings.static_merge_margin))
                        predicted_s=self.reference.project(self._object_pose(obj,elapsed)[:2],stations[index])[0]
                        along_bound,_=self._object_bounds(obj,elapsed)
                        if stations[index]-predicted_s<required+along_bound:
                            data.update(reason='moving_merge_clearance', path=path)
                            return data
                break
        valid, reason, clearance, curvature = self._check(path, speeds, objects, ego)
        data.update(feasible=valid, reason=reason, path=path, min_clearance_m=clearance, max_curvature=curvature,
                    _speeds=speeds, _stations=stations,out_end_s=ego_s+transition)
        return data

    def _result(self, state, reason, path=None, speeds=None, target=None, side=None):
        path = self.last_path if path is None else path
        speeds = [0.] * len(path) if speeds is None else speeds
        self.last_path, self.last_speeds = [list(point) for point in path], list(speeds)
        summary = None if target is None else {key: target[key] for key in ('id', 'kind', 'x', 'y', 'speed', 'length', 'width', 'confirmed')}
        geometry = {key: getattr(self.settings, key) for key in ('vehicle_length', 'vehicle_width', 'uncertainty_margin',
                    'rc_length', 'rc_width', 'obstacle_length', 'obstacle_width', 'static_merge_margin', 'moving_merge_margin', 'headway')}
        geometry.update(vehicle_length_m=self.settings.vehicle_length, vehicle_width_m=self.settings.vehicle_width,
                        rear_axle_to_rear_m=self.settings.rear_axle_to_rear,
                        rear_axle_to_front_m=self.settings.vehicle_length - self.settings.rear_axle_to_rear,
                        target_length_m=0. if target is None else target['length'],
                        target_width_m=0. if target is None else target['width'],
                        merge_clearance_m=(self.settings.moving_merge_margin + self.settings.headway * target['speed']
                            if target and self._uses_learned(target) else self.settings.static_merge_margin))
        model=self.models.get(target['id']) if target else None
        learning=dict(available=bool(model and model.available),ready=bool(model and model.available and model.ready()
                      and target.get('model_ready',True)),
                      coverage=model.coverage() if model and model.available else 0.,
                      reason=(target.get('model_reason') or getattr(model,'reason','no_rc_model')) if model else 'no_rc_model')
        observation=(dict(mode='predicted' if target.get('age_s',0.)>1e-6 else 'observed',
                          age_s=target.get('age_s',0.),uncertainty_m=target.get('uncertainty',0.)) if target else None)
        return dict(state=state, reason=reason, path=[list(point) for point in self.last_path], speeds=list(self.last_speeds), target=summary,
                    target_speed=self.last_speeds[0] if self.last_speeds else 0.,
                    min_route_speed=min(self.last_speeds, default=0.),
                    target_id=None if target is None else target['id'], side=side, route_revision=self.route_number,
                    reference_revision=self.revision, candidates=copy.deepcopy(self.last_candidates), geometry=geometry,
                    planning_mode=self.planning_mode,learning=learning,permission=copy.deepcopy(self.permission),
                    target_observation=observation,
                    experimental=True, unknown_space='Unobserved space is not proven free',
                    prediction_model='assumed footprints; nominal reference speed; target constant along-reference speed')

    def update(self, now, ego, objects, *, geometry_revision=None):
        if self.fault:
            state, reason, path = self.fault
            return self._result(state, reason, path, target=None if self.active is None else self.active['target'],
                                side=None if self.active is None else self.active['side'])
        try:
            if self.reference is None or self.world is None:
                raise ValueError('reference_missing')
            if (not _finite([now]) or not isinstance(ego, (tuple, list)) or len(ego) != 4 or not _finite(ego)
                    or not 0 <= ego[3] <= 20 or (self.last_now is not None and now < self.last_now)):
                raise ValueError('invalid_or_reversed_ego_time')
            if asdict(self.settings) != self.settings_signature:
                raise ValueError('parameters_changed')
            if geometry_revision is not None and geometry_revision != self.revision:
                raise ValueError('geometry_revision_changed')
            ego_s, ego_d, distance = self.reference.project(ego[:2], self.ego_station)
            if distance > 2.:
                raise ValueError('ego_outside_reference')
            self.ego_station, self.last_now = ego_s, now
            baseline_distance=min(self.settings.lookahead_m,max(.1,min(self.settings.speed_cap,
                self.reference.at(ego_s)[3])*self.settings.prediction_horizon_s*.8))
            if not self.last_path:
                self.last_path, self.last_speeds, _ = self._path(ego_s, ego_s + baseline_distance)
            observed = self._objects(now, objects, ego_s)
            self.started = True
            blocking = self._blocking(observed, ego_s)
            if self.active:
                active = self.active
                target = next((obj for obj in observed if obj['id'] == active['target']['id']), None)
                if target is None:
                    raise ValueError('active_target_lost')
                target_learned=self._uses_learned(target)
                if target_learned and not target.get('model_ready'):
                    raise ValueError('learned_prediction_invalidated')
                if target_learned:
                    prediction=self.models[target['id']].predict(target['s'])
                    if not prediction['valid']:
                        raise ValueError('learned_prediction_invalidated')
                    if target['speed']>prediction['v']+2*prediction['v_sigma']+self.settings.prediction_speed_uncertainty:
                        raise ValueError('active_target_accelerated')
                    if abs(target['d']-prediction['d'])>2*prediction['d_sigma']+self.settings.uncertainty_margin:
                        raise ValueError('active_target_left_prediction')
                elif target['speed'] > active['target']['speed'] + self.settings.moving_speed_advantage / 2:
                    raise ValueError('active_target_accelerated')
                if (target['kind'] == 'rc_car' and active['target']['speed'] <= self.settings.simple_pass_speed
                        and target['speed'] > self.settings.simple_pass_speed):
                    raise ValueError('simple_target_became_learned')
                if not target_learned and abs(target['d'] - active['target']['d']) > self.settings.uncertainty_margin:
                    raise ValueError('active_target_left_prediction')
                result=self._committed(now,ego_s,ego_d,ego,observed,target)
                return self.update(now,ego,objects) if result is None else result
            target = blocking[0] if blocking else None
            baseline, speeds, _ = self._path(ego_s, ego_s + baseline_distance)
            if target is None:
                self.permission,self.planning_mode=None,'follow'
                valid, reason, _, _ = self._check(baseline, speeds, observed, ego)
                if not valid:
                    return self._result('BLOCKED', reason, baseline)
                self.last_candidates = []
                return self._result('CLEAR', 'no_observed_blocking_object', baseline, speeds)
            gap = (target['s'] - ego_s - (self.settings.vehicle_length - self.settings.rear_axle_to_rear)
                   - target['length'] / 2 - self.settings.uncertainty_margin)
            follow_cap = max(0., min(self.settings.speed_cap, target['speed'],
                (gap - self.settings.follow_min_gap) / self.settings.headway,
                math.sqrt(max(0., 2 * self.settings.deceleration * (gap - self.settings.follow_min_gap)))))
            moving = target['kind'] == 'rc_car' and target['speed'] > self.settings.slow_speed
            learned=any(obj['kind']=='rc_car' and obj['speed']>self.settings.simple_pass_speed for obj in blocking)
            self.planning_mode='learned' if learned else 'simple'
            zone=self._permission(ego_s)
            self.permission=None if zone is None else dict(zone_id=zone['id'],side=zone['side'],remaining_m=zone['end_s']-ego_s)
            hold = self.history.get(target['id'], (now, now))
            if not target['confirmed']:
                return self._follow('OBSERVE', 'target_not_confirmed', ego_s, ego, observed, target, 0.)
            if zone is None:
                return self._follow('FOLLOW' if moving else 'BLOCKED','passing_not_authorized',ego_s,ego,observed,target,follow_cap)
            if learned and any(obj['kind']=='rc_car' and obj['speed']>self.settings.simple_pass_speed
                               and not obj.get('model_ready') for obj in blocking):
                return self._follow('FOLLOW','opponent_model_learning',ego_s,ego,observed,target,follow_cap)
            if moving and (self.settings.speed_cap - target['speed'] < self.settings.moving_speed_advantage - 1e-9
                           or hold[1] - hold[0] < self.settings.moving_observation_s):
                state = 'OBSERVE' if hold[1] - hold[0] < self.settings.moving_observation_s else 'FOLLOW'
                return self._follow(state, 'moving_observation_or_speed_advantage', ego_s, ego, observed, target, follow_cap)
            if abs(ego_d) > .25 or abs(_wrap(ego[2] - self.reference.at(ego_s)[2])) > .2:
                return self._result('BLOCKED', 'start_alignment_required', baseline, target=target)
            candidates = [self._candidate(side, ego_s, ego_d, target, blocking, observed, ego)
                          for side in ('left', 'right')]
            self.last_candidates = [{key: value for key, value in item.items() if not key.startswith('_')} for item in candidates]
            feasible = [item for item in candidates if item['feasible']]
            if not feasible:
                if moving:
                    return self._follow('FOLLOW', 'passing_opportunity_too_short', ego_s, ego, observed, target, follow_cap)
                return self._result('BLOCKED', 'no_swept_candidate', baseline, target=target)
            chosen = max(feasible, key=lambda item: (item['side'] == self.settings.preferred_side, item['min_clearance_m']))
            state = 'PASS_MOVING' if moving else 'AVOID_OBSTACLE' if target['kind'] == 'obstacle' else 'AVOID_SLOW'
            self.route_number += 1
            ramp=chosen['out_end_s']-ego_s
            self.active = dict(chosen,state=state,target=dict(target),index=0,start_s=ego_s,start_d=ego_d,ramp=ramp,
                               planned_merge_start_s=chosen['merge_start_s'],planned_end_s=chosen['merge_end_s'])
            return self._committed(now,ego_s,ego_d,ego,observed,target)
        except (ValueError, TypeError, KeyError, IndexError, ArithmeticError) as error:
            self._latch(str(error))
            return self.update(now, ego, objects)
