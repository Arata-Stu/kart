"""Validate recorded input types and camera extrinsics before starting CUDA."""

from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def inspect_bag(bag, workflow, parameters):
    bag = Path(bag)
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())[
        "rosbag2_bagfile_information"
    ]
    topics = {
        r["topic_metadata"]["name"]: r for r in metadata["topics_with_message_count"]
    }
    required = {
        workflow[k]: "sensor_msgs/msg/Image" for k in ("left_image", "right_image")
    }
    required.update(
        {workflow[k]: "sensor_msgs/msg/CameraInfo" for k in ("left_info", "right_info")}
    )
    required[workflow["tf_static"]] = "tf2_msgs/msg/TFMessage"
    if parameters["tracking_mode"] == 1:
        required[workflow["imu"]] = "sensor_msgs/msg/Imu"
    for topic, kind in required.items():
        entry = topics.get(topic)
        if (
            not entry
            or entry["topic_metadata"]["type"] != kind
            or entry["message_count"] < 1
        ):
            raise ValueError(f"Missing input {topic}: {kind}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=metadata["storage_identifier"]
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[workflow["tf_static"]]))
    graph = {}
    while reader.has_next():
        _, raw, _ = reader.read_next()
        for tf in deserialize_message(raw, TFMessage).transforms:
            parent, child = (
                tf.header.frame_id.lstrip("/"),
                tf.child_frame_id.lstrip("/"),
            )
            if parent in ("map", "odom") or child in ("map", "odom"):
                raise ValueError(
                    "Recorded /tf_static contains map/odom; isolate camera static extrinsics first"
                )
            graph.setdefault(parent, set()).add(child)
            graph.setdefault(child, set()).add(parent)
    seen, todo = set(), [parameters["base_frame"]]
    while todo:
        frame = todo.pop()
        if frame not in seen:
            seen.add(frame)
            todo.extend(graph.get(frame, []))
    frames = parameters["camera_optical_frames"][:]
    if parameters["tracking_mode"] == 1:
        frames.append(parameters["imu_frame"])
    for frame in frames:
        if frame not in seen:
            raise ValueError(
                f"Missing static TF from {parameters['base_frame']} to {frame}"
            )
    return list(required), metadata
