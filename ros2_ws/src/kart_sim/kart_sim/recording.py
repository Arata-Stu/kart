"""ROS-free rosbag2/CDR writer for the same inputs as kart_mapping."""
from datetime import datetime
from pathlib import Path
import uuid
import numpy as np


class SimRecorder:
    def __init__(self, directory):
        self.directory = Path(directory).expanduser().resolve()
        self.writer = None
        self.path = ''
        self.error = ''
        self.messages = 0
        self.finalized = False

    def message(self, name, *args):
        return self.types[name](*args)

    def stamp(self, time):
        ns = round(time*1e9)
        return self.message('builtin_interfaces/msg/Time', ns//10**9, ns % 10**9)

    def header(self, time, frame):
        return self.message('std_msgs/msg/Header', self.stamp(time), frame)

    def vector(self, values):
        return self.message('geometry_msgs/msg/Vector3', *map(float, values))

    def quaternion(self, values):
        w, x, y, z = map(float, values)
        return self.message('geometry_msgs/msg/Quaternion', x, y, z, w)

    def write(self, topic, msg, time):
        if topic not in self.connections:
            qos = self.static_qos if topic == '/tf_static' else ()
            self.connections[topic] = self.writer.add_connection(
                topic, msg.__msgtype__, typestore=self.store, offered_qos_profiles=qos)
        self.writer.write(self.connections[topic], round(time*1e9),
                          self.store.serialize_cdr(msg, msg.__msgtype__))
        self.messages += 1

    def start(self, sim):
        if self.writer:
            return
        if not sim.sensor_configs:
            raise ValueError('Enable cameras (--sensors / camera_enabled) before recording')
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
        from rosbags.interfaces import Qos, QosHistory, QosReliability, QosDurability, QosLiveliness, QosTime
        self.store = get_typestore(Stores.ROS2_HUMBLE)
        self.types = self.store.types
        infinite = QosTime(2147483647, 4294967295)
        self.static_qos = [Qos(QosHistory.KEEP_LAST, 1, QosReliability.RELIABLE,
            QosDurability.TRANSIENT_LOCAL, infinite, infinite, QosLiveliness.AUTOMATIC, infinite, False)]
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = str(self.directory / (datetime.now().strftime('sim_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8]))
        self.writer = Writer(Path(self.path), version=9)
        self.connections = {}
        self.messages = 0
        self.error = ''
        self.finalized = False
        try:
            self.writer.open()
            now = float(sim.data.time)
            transforms = []
            # Never put truth map/odom frames in the mapping bag's /tf_static.
            for parent, child, xyz, quat in sim.rig.static_transforms():
                tf = self.message('geometry_msgs/msg/Transform', self.vector(xyz), self.quaternion(quat))
                transforms.append(self.message('geometry_msgs/msg/TransformStamped', self.header(now, parent), child, tf))
            self.write('/tf_static', self.message('tf2_msgs/msg/TFMessage', transforms), now)
        except Exception:
            self.stop()
            raise

    def append(self, sim, samples):
        if not self.writer:
            return
        for frames in samples['images']:
            for name, frame in frames.items():
                pixels = np.ascontiguousarray(frame.values['image'])
                height, width = pixels.shape[:2]
                header = self.header(frame.time, sim.rig.frames[name])
                image = self.message('sensor_msgs/msg/Image', header, height, width,
                    frame.values['encoding'], 0, width*(1 if pixels.ndim == 2 else 3), pixels.reshape(-1).view(np.uint8))
                suffix = 'image_rect_raw' if name.startswith('infra') else 'image_raw'
                self.write(f'/realsense/{name}/{suffix}', image, frame.time)
                roi = self.message('sensor_msgs/msg/RegionOfInterest', 0, 0, 0, 0, False)
                info = self.message('sensor_msgs/msg/CameraInfo', header, height, width, 'plumb_bob',
                    np.zeros(5), frame.values['K'].ravel(), np.eye(3).ravel(), sim.rig.projection(name).ravel(), 0, 0, roi)
                self.write(f'/realsense/{name}/camera_info', info, frame.time)
        for frame in samples['imu']:
            msg = self.message('sensor_msgs/msg/Imu', self.header(frame['time'], frame['frame_id']),
                self.quaternion(frame['orientation']), np.zeros(9), self.vector(frame['angular_velocity']),
                np.zeros(9), self.vector(frame['acceleration']), np.zeros(9))
            self.write('/realsense/imu', msg, frame['time'])
            self.write('/sim/ground_truth/imu', msg, frame['time'])
        now = float(sim.data.time)
        self.write('/clock', self.message('rosgraph_msgs/msg/Clock', self.stamp(now)), now)
        pos, quat, linear, angular = sim.state()
        pose = self.message('geometry_msgs/msg/Pose', self.message('geometry_msgs/msg/Point', *map(float, pos)), self.quaternion(quat))
        header = self.header(now, 'sim_world')
        self.write('/sim/ground_truth/pose', self.message('geometry_msgs/msg/PoseStamped', header, pose), now)
        twist = self.message('geometry_msgs/msg/Twist', self.vector(linear), self.vector(angular))
        odom = self.message('nav_msgs/msg/Odometry', header, 'base_link',
            self.message('geometry_msgs/msg/PoseWithCovariance', pose, np.zeros(36)),
            self.message('geometry_msgs/msg/TwistWithCovariance', twist, np.zeros(36)))
        self.write('/sim/odometry', odom, now)

    def stop(self):
        writer, self.writer = self.writer, None
        if writer:
            try:
                writer.close()
                self.finalized = (Path(self.path)/'metadata.yaml').is_file()
                if not self.finalized:
                    raise RuntimeError('Bag metadata was not finalized')
            except Exception as exc:
                self.error = 'Bag finalization failed: '+str(exc)
                raise

    def status(self):
        return dict(recording=self.writer is not None, finalized=self.finalized,
                    path=self.path, messages=self.messages, error=self.error)
