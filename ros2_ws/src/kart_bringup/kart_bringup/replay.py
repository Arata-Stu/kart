"""Read-only offline playback contract. Filters competing TF/control publishers."""

import json
import math
from pathlib import Path

import yaml


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
