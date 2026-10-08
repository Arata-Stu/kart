"""Keep final cloud/path and timestamp-ordered map transforms, not early TF."""

import math

from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_msgs.msg import TFMessage

from .frames import FinalFrames, stamp_ns, transform_record


class Collector(Node):
    def __init__(self):
        super().__init__(
            "kart_map_collector",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.frames = FinalFrames()
        self.cloud = self.path = None
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2,
            "/visual_slam/vis/landmarks_cloud",
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Path,
            "/visual_slam/tracking/slam_path",
            self.on_path,
            qos_profile_sensor_data,
        )

    def on_tf(self, msg):
        for tf in msg.transforms:
            self.frames.update(transform_record(tf))

    def on_cloud(self, msg):
        if self.cloud is None or stamp_ns(msg.header.stamp) >= stamp_ns(
            self.cloud.header.stamp
        ):
            if msg.width * msg.height:
                self.cloud = msg

    def on_path(self, msg):
        if msg.poses and (
            self.path is None
            or stamp_ns(msg.header.stamp) >= stamp_ns(self.path.header.stamp)
        ):
            self.path = msg

    def snapshot(self, provenance):
        if self.cloud is None or self.path is None:
            raise ValueError("No final landmarks or SLAM path received")
        msg = self.cloud
        raw = [
            [float(p[0]), float(p[1]), float(p[2])]
            for p in read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        raw = [p for p in raw if all(math.isfinite(v) for v in p)]
        if not raw:
            raise ValueError("Final landmarks are empty")
        points, tf = self.frames.convert(raw, msg.header.frame_id)
        raw_path = [
            [p.pose.position.x, p.pose.position.y, p.pose.position.z]
            for p in self.path.poses
        ]
        trajectory, path_tf = self.frames.convert(raw_path, self.path.header.frame_id)
        provenance.update(
            cloud_frame=msg.header.frame_id,
            cloud_stamp_ns=str(stamp_ns(msg.header.stamp)),
            transform_policy="latest_timestamp_at_end",
            applied_transform=tf,
            final_transforms=self.frames.transforms,
            path_frame=self.path.header.frame_id,
            path_stamp_ns=str(stamp_ns(self.path.header.stamp)),
            path_transform=path_tf,
        )
        return dict(
            schema="kart.snapshot.v1",
            frame="map",
            points=points,
            trajectory=trajectory,
            raw_points=raw,
            raw_path=raw_path,
            provenance=provenance,
        )
