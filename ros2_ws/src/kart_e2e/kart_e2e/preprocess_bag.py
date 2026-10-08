"""Two-pass rosbag extraction with bounded causal timestamp association."""

import argparse
import json
from pathlib import Path

from .data import decode_image, previous, valid_label


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--image-topic", default="/realsense/color/image_raw")
    parser.add_argument("--command-topic", default="/teleop/control_cmd")
    parser.add_argument("--mode-topic", default="/operation_mode/state")
    parser.add_argument("--clock", choices=["bag", "header"], default="bag")
    parser.add_argument("--max-skew-ms", type=float, default=100)
    args = parser.parse_args()
    if not 0 < args.max_skew_ms <= 1000:
        parser.error("max-skew-ms must be in (0, 1000]")
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    def records(topics):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=""),
            rosbag2_py.ConverterOptions("", ""),
        )
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        expected = {
            args.image_topic: "sensor_msgs/msg/Image",
            args.command_topic: "kart_interfaces/msg/ControlCommand",
            args.mode_topic: "kart_interfaces/msg/OperationModeState",
        }
        for topic in topics:
            if types.get(topic) != expected[topic]:
                raise ValueError(
                    f"Missing/wrong topic: {topic}, expected {expected[topic]}"
                )
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
        while reader.has_next():
            topic, raw, bag_stamp = reader.read_next()
            msg = deserialize_message(raw, get_message(types[topic]))
            stamp = (
                bag_stamp
                if args.clock == "bag"
                else msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            )
            if stamp > 0:
                yield topic, stamp, msg

    commands, modes = [], []
    for topic, stamp, msg in records([args.command_topic, args.mode_topic]):
        (commands if topic == args.command_topic else modes).append((stamp, msg))
    commands.sort(key=lambda v: v[0])
    modes.sort(key=lambda v: v[0])
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "images").mkdir()
    count, skipped = 0, 0
    with (args.output / "samples.jsonl").open("w") as stream:
        for _, stamp, msg in records([args.image_topic]):
            command = previous(commands, stamp, int(args.max_skew_ms * 1e6))
            mode = previous(modes, stamp, 300_000_000)
            if (
                command is None
                or mode is None
                or mode.mode != 2
                or not valid_label(command)
            ):
                skipped += 1
                continue
            name = f"images/{count:08d}.png"
            decode_image(msg).save(args.output / name)
            stream.write(
                json.dumps(
                    {
                        "image": name,
                        "stamp_ns": stamp,
                        "steering": command.steering,
                        "throttle": command.throttle,
                    }
                )
                + "\n"
            )
            count += 1
    (args.output / "metadata.json").write_text(
        json.dumps(
            {
                **vars(args),
                "bag": str(args.bag.resolve()),
                "output": str(args.output.resolve()),
                "samples": count,
                "skipped": skipped,
            },
            indent=2,
        )
    )
    if not count:
        raise RuntimeError(
            "No usable MANUAL samples; inspect topics, clock, mode and skew"
        )
    print(f"Exported {count} samples; skipped {skipped}")


if __name__ == "__main__":
    main()
