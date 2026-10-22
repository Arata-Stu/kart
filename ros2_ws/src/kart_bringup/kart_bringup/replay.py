"""Read-only offline playback contract. Filters competing TF/control publishers."""

import json
import math
from pathlib import Path

import yaml


def imu_topics(config, mode):
    """Required IMU topic for VIO; no input added for VO."""
    if str(mode) not in ('0','1'):raise ValueError('tracking_mode must be 0 (VO) or 1 (VIO)')
    if str(mode)=='0':return []
    return [json.loads((Path(config)/'localization/workflow.json').read_text())['imu']]


def validate_bag_shape(config, bag, shape):
    """Check the first recorded stereo Image and CameraInfo sizes, not live YAML."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image, CameraInfo
    except ImportError as error:
        raise ValueError("VGLのbag解像度検査にはROS環境（rosbag2_py・sensor_msgs）が必要です") from error
    if (not isinstance(shape, list) or len(shape) != 4 or shape[:2] != [1, 3]
            or any(type(v) is not int or v <= 0 for v in shape)):
        raise ValueError("VGLモデルの入力shapeが不正です")
    workflow = json.loads((Path(config) / "localization/workflow.json").read_text())
    topics = {workflow[k]: (Image, "sensor_msgs/msg/Image") for k in ("left_image", "right_image")}
    topics.update({workflow[k]: (CameraInfo, "sensor_msgs/msg/CameraInfo") for k in ("left_info", "right_info")})
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(bag).expanduser().resolve(strict=True))),
                rosbag2_py.ConverterOptions("", ""))
    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic, (_, kind) in topics.items():
        if available.get(topic) != kind:
            raise ValueError(f"Bag input missing or wrong type: {topic} (expected {kind})")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    sizes = {}
    while reader.has_next() and len(sizes) < len(topics):
        topic, raw, _ = reader.read_next()
        if topic in sizes:
            continue
        msg = deserialize_message(raw, topics[topic][0])
        actual = [msg.height, msg.width]
        if actual != shape[2:]:
            raise ValueError(f"VGLモデルとbagの解像度が一致しません: {topic}: "
                             f"bag={msg.width}x{msg.height}, model={shape[3]}x{shape[2]}")
        sizes[topic] = [msg.width, msg.height]
    if len(sizes) != len(topics):
        raise ValueError("Bagに左右画像・CameraInfoの有効なメッセージがありません")
    return sizes


def playback(config, bag, rate="", extra_topics=None):
    config, bag = Path(config), Path(bag).expanduser().resolve(strict=True)
    try:
        metadata = yaml.safe_load((bag / "metadata.yaml").read_text())[
            "rosbag2_bagfile_information"
        ]
        available = {
            "/" + row["topic_metadata"]["name"].lstrip("/")
            for row in metadata["topics_with_message_count"]
            if row["message_count"] > 0
        }
    except (KeyError, TypeError, yaml.YAMLError) as error:
        raise ValueError(f"Invalid rosbag metadata: {bag}") from error
    workflow = json.loads((config / "localization/workflow.json").read_text())
    settings = yaml.safe_load((config / "evaluation/replay.yaml").read_text())
    topics = [
        workflow[k] for k in ("left_image", "right_image", "left_info", "right_info")
    ] + settings["extra_topics"] + list(extra_topics or [])
    topics = list(dict.fromkeys(topics))
    if any(t in ('/tf', '/clock') for t in topics):
        raise ValueError('Do not replay competing dynamic TF or recorded clock')
    missing = set(topics) - available
    if missing:
        raise ValueError(
            "Bag lacks required recorded inputs: " + ", ".join(sorted(missing))
        )
    speed = float(settings["rate"] if rate == "" else rate)
    if not math.isfinite(speed) or not 0 < speed <= 4:
        raise ValueError("Playback rate must be in (0, 4]")
    return [
        "ros2",
        "bag",
        "play",
        str(bag),
        "--rate",
        str(speed),
        "--clock",
        str(settings["clock_hz"]),
        "--read-ahead-queue-size",
        str(settings["read_ahead_queue_size"]),
        "--start-paused",
        "--disable-keyboard-controls",
        "--qos-profile-overrides-path",
        str(config / "evaluation/qos.yaml"),
        "--topics",
        *topics,
    ]
