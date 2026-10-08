"""Replay a bag against an isolated copy of a saved map, with visualization."""

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path


def stop(process):
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def last_stereo_stamp(bag, workflow):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    from .frames import stamp_ns

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag)), rosbag2_py.ConverterOptions("", "")
    )
    topics = [workflow["left_image"], workflow["right_image"]]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    last = {}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        last[topic] = max(
            last.get(topic, -1), stamp_ns(deserialize_message(raw, Image).header.stamp)
        )
    if len(last) != 2 or min(last.values()) <= 0:
        raise ValueError("左右画像の有効な末尾stampがありません")
    return min(last.values())


def run(job_file):
    import rclpy
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from isaac_ros_visual_slam_interfaces.srv import LocalizeInMap

    from .bag_inputs import inspect_bag
    from .capture_collector import CaptureCollector

    job = json.loads(job_file.read_text())
    workflow, options = job["workflow"], job["capture"]
    bag, output = Path(job["bag"]), Path(job["output"])
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) != workflow["ros_domain_id"]:
        raise ValueError("ROS domain mismatch")
    config = Path(get_package_share_directory("kart_bringup")) / "config/mapping"
    parameters = yaml.safe_load((config / "capture_cuvslam.yaml").read_text())[
        "/**/visual_slam"
    ]["ros__parameters"]
    parameters.update(workflow["overrides"])
    parameters["load_map_folder_path"] = job["map_dir"]
    inputs, metadata = inspect_bag(bag, workflow, parameters)
    expected = last_stereo_stamp(bag, workflow)
    rclpy.init()
    node = CaptureCollector()
    launch = player = None
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        if any(
            n in ("visual_slam", "kart_mapping_container")
            for n in node.get_node_names()
        ):
            raise ValueError("隔離domainに既存VSLAMがあります")
        launch = subprocess.Popen(
            [
                "ros2",
                "launch",
                "kart_bringup",
                "mapping.launch.py",
                f"job_file:={job_file}",
            ]
        )
        client = node.create_client(LocalizeInMap, "/visual_slam/localize_in_map")
        deadline = time.monotonic() + options["ready_timeout_s"]
        while not (
            client.service_is_ready()
            and all(
                node.count_subscribers(workflow[k])
                for k in ("left_image", "right_image", "left_info", "right_info")
            )
        ):
            if launch.poll() is not None or time.monotonic() > deadline:
                raise ValueError("cuVSLAMの起動が完了しませんでした")
            rclpy.spin_once(node, timeout_sec=0.1)
        player = subprocess.Popen(
            [
                "ros2",
                "bag",
                "play",
                str(bag),
                "--clock",
                "--rate",
                str(options["replay_rate"]),
                "--qos-profile-overrides-path",
                str(config / "replay_qos.yaml"),
                "--topics",
                *inputs,
            ]
        )
        duration = metadata["duration"]["nanoseconds"] / 1e9
        deadline = (
            time.monotonic()
            + duration / options["replay_rate"] * 2
            + options["ready_timeout_s"]
        )
        future = None
        # Same mapping bag starts near the saved map origin; never claim this is global localization.
        while player.poll() is None:
            if launch.poll() is not None or time.monotonic() > deadline:
                raise ValueError("再生中にVSLAM終了またはタイムアウト")
            rclpy.spin_once(node, timeout_sec=0.1)
            if future is None and node.gate.stamp >= 0:
                request = LocalizeInMap.Request()
                request.map_folder_path = job["map_dir"]
                request.pose_hint.orientation.w = 1.0
                future = client.call_async(request)
            if future is not None and future.done() and not future.result().success:
                raise ValueError("保存地図へのlocalize要求が拒否されました")
        if player.returncode:
            raise ValueError("bag再生が失敗しました")
        deadline = time.monotonic() + options["drain_timeout_s"]
        tolerance = int(options["tail_tolerance_s"] * 1e9)
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise ValueError("点群収集中にVSLAMが終了しました")
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.gate.stamp >= expected - tolerance
                and time.monotonic() - node.last_update >= options["settle_s"]
            ):
                break
        if time.monotonic() - node.last_update < options["settle_s"]:
            raise ValueError("点群・TFの更新が制限時間内に落ち着きませんでした")
        if future is None or not future.done() or not future.result().success:
            raise ValueError("localize要求の受理を確認できませんでした")
        snapshot = node.final_snapshot(
            expected,
            tolerance,
            {
                "source": "saved_cuvslam_map_replay",
                "bag": str(bag),
                "map_id": job["map_id"],
                "workflow": workflow,
                "capture": options,
                "parameters": parameters,
            },
        )
        temporary = output / ".snapshot.tmp"
        temporary.write_text(json.dumps(snapshot, allow_nan=False))
        os.replace(temporary, output / "snapshot.json")
        print(f"Saved localized snapshot: {len(snapshot['points'])} points", flush=True)
    finally:
        stop(player)
        stop(launch)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", required=True)
    run(Path(parser.parse_args().job).resolve())


if __name__ == "__main__":
    main()
