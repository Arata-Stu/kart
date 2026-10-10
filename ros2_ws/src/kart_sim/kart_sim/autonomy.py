"""Truth-based pure pursuit for repeatable simulated sensor collection."""
import math
import numpy as np


class WaypointController:
    def __init__(self, sim):
        cfg = sim.map.get('autodrive', {})
        points = np.asarray(cfg.get('waypoints', []), dtype=float)
        self.available = points.ndim == 2 and points.shape[1:] == (2,) and len(points) >= 3
        if 'autodrive' in sim.map and not self.available:
            raise ValueError('autodrive.waypoints requires at least three XY points')
        self.enabled = False
        self.error = ''
        self.laps = 0
        self.progress = 0.
        self.speed = float(cfg.get('speed_mps', .45))
        self.lookahead = float(cfg.get('lookahead_m', .35))
        if not math.isfinite(self.speed) or not .05 <= self.speed <= 1.5:
            raise ValueError('autodrive speed_mps must be 0.05..1.5')
        if not math.isfinite(self.lookahead) or not .15 <= self.lookahead <= 1:
            raise ValueError('autodrive lookahead_m must be 0.15..1')
        if not self.available:
            return
        if not np.isfinite(points).all():
            raise ValueError('Non-finite waypoints')
        # Dense polyline permits nearest-point projection without jumping to another lane.
        dense = []
        for a, b in zip(points, np.roll(points, -1, axis=0)):
            length = np.linalg.norm(b-a)
            if length < .001:
                raise ValueError('Repeated consecutive waypoint')
            dense.extend(a+(b-a)*t for t in np.linspace(0, 1, max(2, math.ceil(length/.025)), endpoint=False))
        self.path = np.asarray(dense)
        self.ds = np.linalg.norm(np.roll(self.path, -1, axis=0)-self.path, axis=1)
        self.arc = np.r_[0, np.cumsum(self.ds)]
        self.length = float(self.arc[-1])
        self.index = 0
        self.last_motion_time = 0.
        self.last_motion_position = None

    def start(self, sim):
        if not self.available:
            raise ValueError('This map has no autodrive.waypoints')
        pos = sim.state()[0][:2]
        self.index = int(np.argmin(np.linalg.norm(self.path-pos, axis=1)))
        self.progress = 0.
        self.laps = 0
        self.enabled = True
        self.error = ''
        self.last_motion_time = float(sim.data.time)
        self.last_motion_position = pos.copy()

    def stop(self):
        self.enabled = False

    def output(self, sim):
        if not self.enabled:
            return (0., 0., 1., 0.)
        pos, quat, linear, _ = sim.state()
        # Search only nearby forward samples: crossing/parallel lanes cannot steal progress.
        indices = (self.index+np.arange(min(45, len(self.path)))) % len(self.path)
        distances = np.linalg.norm(self.path[indices]-pos[:2], axis=1)
        offset = int(np.argmin(distances))
        for i in range(offset):
            self.progress += self.ds[(self.index+i) % len(self.path)]
        self.index = int(indices[offset])
        self.laps = int(self.progress/self.length)
        if distances[offset] > .65:
            self.error = 'Path deviation exceeded 0.65 m; stopped'
            self.stop()
            return (0., 0., 1., 0.)
        if np.linalg.norm(pos[:2]-self.last_motion_position) > .05:
            self.last_motion_position = pos[:2].copy()
            self.last_motion_time = float(sim.data.time)
        elif sim.data.time-self.last_motion_time > 8:
            self.error = 'Vehicle stalled for 8 sim seconds; stopped'
            self.stop()
            return (0., 0., 1., 0.)
        target = self.index
        distance = 0.
        while distance < self.lookahead:
            distance += self.ds[target]
            target = (target+1) % len(self.path)
        dx, dy = self.path[target]-pos[:2]
        w, x, y, z = quat
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        lateral = -math.sin(yaw)*dx+math.cos(yaw)*dy
        curvature = 2*lateral/max(dx*dx+dy*dy, .01)
        steering = np.clip(math.atan(sim.parameters.wheelbase*curvature)/sim.parameters.max_steer, -1, 1)
        target_speed = self.speed/max(1., abs(curvature)*.7)
        error = target_speed-float(linear[0])
        throttle = float(np.clip(.22+error*.8, 0, .65)) if error >= -.03 else 0.
        brake = float(np.clip(-error*.5, 0, .3)) if error < -.03 else 0.
        return (float(steering), throttle, brake, 0.)

    def status(self):
        return dict(available=self.available, enabled=self.enabled, laps=self.laps,
                    progress_m=round(self.progress, 2), speed_mps=self.speed, error=self.error)
