"""Optional, simulation-time sampled sensors; no ROS, logging or GUI dependency.

Frames own their arrays and remain valid after subsequent updates. Lengths are
metres. LiDAR coordinates: +X forward, +Y left, +Z up. Camera optical frame:
+X right, +Y down, +Z forward. Sensor poses in config are parent-body relative.
"""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


@dataclass(frozen=True)
class SensorConfig:
    name: str
    kind: str
    parent: str = 'chassis'
    pos: tuple = (0.18, 0, 0.20)
    # Mount frame uses vehicle convention; camera optical rotation is added below.
    quat: tuple = (1, 0, 0, 0)
    hz: float = 10
    width: int = 320
    height: int = 240
    fovy: float = 60
    depth: bool = False
    intrinsics: tuple | None = None  # fx, fy, cx, cy; same projection for render and CameraInfo.
    encoding: str = "rgb8"
    rays: int = 180
    channels: int = 1
    vertical_fov: float = 30
    min_range: float = 0.05
    max_range: float = 20

    def __post_init__(self):
        if self.kind not in ('camera', 'lidar'):
            raise ValueError('kind must be camera or lidar')
        if not self.name or not self.parent:
            raise ValueError('Sensor name and parent must not be empty')
        if self.encoding not in ('rgb8','mono8'): raise ValueError('Invalid encoding')
        if self.intrinsics is not None:
            if len(self.intrinsics)!=4 or not np.isfinite(self.intrinsics).all() or min(self.intrinsics[:2])<=0:
                raise ValueError('Invalid intrinsics')
        for value, size in ((self.pos, 3), (self.quat, 4)):
            if np.shape(value) != (size,) or not np.isfinite(value).all():
                raise ValueError('Invalid sensor pose')
        if not np.isclose(np.linalg.norm(self.quat), 1):
            raise ValueError('Sensor quaternion must have unit norm')
        for value in (self.hz, self.fovy, self.max_range, self.vertical_fov):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Rates, FOV and max range must be positive and finite')
        if not math.isfinite(self.min_range) or not 0 <= self.min_range < self.max_range:
            raise ValueError('Invalid range limits')
        if not 0 < self.fovy < 180 or self.vertical_fov >= 180:
            raise ValueError('FOV must be below 180 degrees')
        for value in (self.width, self.height, self.rays, self.channels):
            if type(value) is not int or value < 1:
                raise ValueError('Resolutions must be positive integers')


@dataclass(frozen=True)
class SensorFrame:
    time: float
    frame_id: str
    # Camera: rgb uint8 HxWx3, optional depth float32 HxW (optical Z).
    # LiDAR: ranges CxR, points CxRx3 in mount frame; no return = inf/NaN.
    values: dict = field(default_factory=dict)


def load_configs(enabled, path=None):
    path = Path(path) if path else Path(__file__).with_name('sensor_config.json')
    settings = json.loads(path.read_text())
    if len(enabled) != len(set(enabled)):
        raise ValueError('Duplicate sensor selection')
    return [SensorConfig(name=name, **settings[name]) for name in enabled]


def attach_sensors(root, configs):
    """Call before compiling MJCF. Group 5 is reserved for the host vehicle.

    LiDAR ignores the entire selected parent subtree, not just its root body.
    Renderers/viewers must enable group 5 to display the vehicle normally.
    Mounting poses are provisional; these do not add sensor mass or geometry.
    """
    for cfg in configs:
        parent = next((b for b in root.iter('body') if b.get('name') == cfg.parent), None)
        if parent is None:
            raise ValueError(f'Unknown sensor parent: {cfg.parent}')
        pose = dict(pos=' '.join(map(str, cfg.pos)), quat=' '.join(map(str, cfg.quat)))
        if cfg.kind == 'camera':
            mount = ET.SubElement(parent, 'body', name=cfg.name+'_mount', **pose)
            projection={'fovy':str(cfg.fovy)}
            if cfg.intrinsics is not None:
                fx,fy,cx,cy=cfg.intrinsics
                # MuJoCo frustum offset has the opposite sign to image principal point.
                projection={'resolution':f'{cfg.width} {cfg.height}', 'sensorsize':'1 1',
                    'focal':f'{fx/cfg.width} {fy/cfg.height}',
                    'principal':f'{(cfg.width/2-cx)/cfg.width} {(cy-cfg.height/2)/cfg.height}'}
            ET.SubElement(mount, 'camera', name=cfg.name, pos='0 0 0',
                          xyaxes='0 -1 0 0 0 1', **projection)
            visual = root.find('visual')
            if visual is None:
                visual = ET.SubElement(root, 'visual')
            glob = visual.find('global')
            if glob is None:
                glob = ET.SubElement(visual, 'global')
            for key, size in (('offwidth', cfg.width), ('offheight', cfg.height)):
                glob.set(key, str(max(int(glob.get(key, '0')), size)))
        else:
            ET.SubElement(parent, 'site', name=cfg.name, size='0.001', rgba='0 0 0 0', **pose)
            for geom in parent.iter('geom'):
                geom.set('group', '5')


class SensorSuite:
    """Use update(data) after stepping; returns only newly acquired frames.

    latest holds one frame per sensor. No unbounded queue or per-step rendering.
    Missed samples are skipped, not replayed. Call reset() on simulation reset.
    All methods should run on the same thread as the rendering context.
    """
    def __init__(self, model, configs):
        self.model = model
        self.configs = list(configs)
        if len({c.name for c in self.configs}) != len(self.configs):
            raise ValueError('Duplicate sensor names')
        self.latest = {}
        self._due = {c.name: 0.0 for c in self.configs}
        self._last_time = -1.0
        self._renderers = {}
        self._lidars = {}
        self._option = mujoco.MjvOption()
        self._option.geomgroup[5] = 1
        self._mask = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)
        for c in self.configs:
            if c.kind == 'camera':
                model.camera(c.name)  # Fail early for an unattached configuration.
                # MuJoCo scales clipping distances by scene extent. Large maps
                # otherwise clip away the road immediately below a low RC camera.
                model.vis.map.znear = min(model.vis.map.znear, 0.01/model.stat.extent)
                continue
            az = np.linspace(-np.pi, np.pi, c.rays, endpoint=False)
            el = np.deg2rad(np.linspace(-c.vertical_fov/2, c.vertical_fov/2, c.channels)) if c.channels > 1 else np.zeros(1)
            a, e = np.meshgrid(az, el)
            rays = np.stack([np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)], axis=-1).reshape(-1, 3)
            n = len(rays)
            self._lidars[c.name] = (model.site(c.name).id, rays, np.empty_like(rays),
                                    np.empty(n), np.empty(n, dtype=np.int32))

    def reset(self):
        self.latest.clear()
        self._due = {c.name: 0.0 for c in self.configs}
        self._last_time = -1.0

    def due(self, now):
        return any(now+1e-9 >= t for t in self._due.values())

    def update(self, data):
        now = float(data.time)
        if now < self._last_time:
            self.reset()
        self._last_time = now
        due = [c for c in self.configs if now + 1e-9 >= self._due[c.name]]
        if not due:
            return {}
        # mj_step leaves positional caches at the preceding integration state.
        mujoco.mj_fwdPosition(self.model, data)
        fresh = {}
        for c in due:
            values = self._camera(c, data) if c.kind == 'camera' else self._lidar(c, data)
            frame = SensorFrame(now, c.name, values)
            fresh[c.name] = self.latest[c.name] = frame
            self._due[c.name] = (math.floor((now+1e-9)*c.hz)+1)/c.hz
        return fresh

    def _camera(self, c, data):
        key = (c.width, c.height)
        if key not in self._renderers:
            self._renderers[key] = mujoco.Renderer(self.model, height=c.height, width=c.width)
        renderer = self._renderers[key]
        renderer.disable_depth_rendering()
        renderer.update_scene(data, camera=c.name, scene_option=self._option)
        rgb=renderer.render().copy()
        pixels=rgb
        if c.encoding=='mono8':
            pixels=((77*rgb[:,:,0].astype(np.uint16)+150*rgb[:,:,1].astype(np.uint16)+29*rgb[:,:,2].astype(np.uint16))>>8).astype(np.uint8)
        values = {'rgb': rgb, 'image':pixels, 'encoding':c.encoding}
        if c.depth:
            renderer.enable_depth_rendering()
            values['depth'] = renderer.render().copy()
            renderer.disable_depth_rendering()
        f = c.height/(2*math.tan(math.radians(c.fovy)/2))
        fx,fy,cx,cy=c.intrinsics or (f,f,c.width/2,c.height/2)
        values['K'] = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
        camera = self.model.camera(c.name).id
        values['position_world'] = data.cam_xpos[camera].copy()
        values['rotation_world'] = data.cam_xmat[camera].reshape(3, 3) * [1, -1, -1]
        return values

    def _lidar(self, c, data):
        site, rays, world, distances, ids = self._lidars[c.name]
        np.matmul(rays, data.site_xmat[site].reshape(3, 3).T, out=world)
        mujoco.mj_multiRay(self.model, data, data.site_xpos[site], world.ravel(),
                          self._mask, True, -1, ids, distances, None, len(rays), c.max_range)
        valid = (distances >= c.min_range) & (distances <= c.max_range) & (ids >= 0)
        ranges = np.where(valid, distances, np.inf)
        points = rays * np.where(valid, distances, np.nan)[:, None]
        return {'ranges': ranges.reshape(c.channels, c.rays),
                'points': points.reshape(c.channels, c.rays, 3),
                'position_world': data.site_xpos[site].copy(),
                'rotation_world': data.site_xmat[site].reshape(3, 3).copy()}

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
