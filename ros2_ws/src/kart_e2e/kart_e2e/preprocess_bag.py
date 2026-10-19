"""Two-pass rosbag extraction with bounded causal timestamp association."""

import argparse
import json
from pathlib import Path

from .data import decode_image, save_rgb, previous, previous_record
from .teacher_selection import TeacherSelection, read_fixed_configs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--image-topic", default="/realsense/color/image_raw")
    parser.add_argument("--command-topic", default="/teleop/control_cmd")
    parser.add_argument("--mode-topic", default="/operation_mode/state")
    parser.add_argument("--fixed-config-topic", default="/teleop/fixed_config")
    parser.add_argument("--image-format", choices=["npy", "png"], default="npy",
                        help="RGB storage: uncompressed uint8 HWC NPY (default), or fast lossless PNG")
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
    selection = TeacherSelection(read_fixed_configs(args.bag, args.fixed_config_topic), modes)
    print(f"Teacher policy: {selection.metadata()['policy']}")
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "images").mkdir()
    count, skipped = 0, 0
    with (args.output / "samples.jsonl").open("w") as stream:
        for _, stamp, msg in records([args.image_topic]):
            record = previous_record(commands, stamp, int(args.max_skew_ms * 1e6))
            command = record[1] if record is not None else None
            mode = previous(modes, stamp, 300_000_000)
            source = selection.select(command, mode, stamp, record[0] if record else None)
            if source is None:
                skipped += 1
                continue
            name = f"images/{count:08d}.{args.image_format}"
            save_rgb(decode_image(msg), args.output / name, args.image_format)
            stream.write(
                json.dumps(
                    {
                        "image": name,
                        "teacher_source": source,
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
                "modality": "rgb",
                "image_dtype": "uint8",
                "image_layout": "HWC_RGB",
                "teacher_selection": selection.metadata(),
                "skipped": skipped,
            },
            indent=2,
        )
    )
    if not count:
        raise RuntimeError(
            "No usable human teacher samples; inspect topics, clock, mode and skew"
        )
    print(f"Exported {count} samples; skipped {skipped}")


if __name__ == "__main__":
    main()
