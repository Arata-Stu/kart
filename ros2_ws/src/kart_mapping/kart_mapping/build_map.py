"""Run NVIDIA's offline mapping; visualization/snapshot acquisition is separate."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path


def run(job):
    workflow = job["workflow"]
    output, bag = Path(job["output"]), Path(job["bag"])
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) != workflow["ros_domain_id"]:
        raise ValueError("ROS_DOMAIN_ID must match workflow.ros_domain_id")
    if workflow["overrides"].get("tracking_mode", 0) != 0:
        raise ValueError("公式offline工程は今回Stereoのみ対応します")
    from .bag_inputs import inspect_bag

    parameters = dict(
        tracking_mode=0,
        base_frame=workflow["overrides"]["base_frame"],
        camera_optical_frames=workflow["overrides"]["camera_optical_frames"],
    )
    inspect_bag(bag, workflow, parameters)
    left_frame, right_frame = parameters["camera_optical_frames"]
    camera_config = output / "camera_topics.yaml"
    camera_config.write_text(
        json.dumps(
            {
                "stereo_cameras": [
                    {
                        "name": "realsense",
                        "left": workflow["left_image"],
                        "right": workflow["right_image"],
                        "left_camera_info": workflow["left_info"],
                        "right_camera_info": workflow["right_info"],
                        "left_frame_id": left_frame,
                        "right_frame_id": right_frame,
                    }
                ]
            }
        )
    )
    official = output / "official"
    official.mkdir()
    command = [
        "ros2",
        "run",
        "isaac_mapping_ros",
        "create_map_offline.py",
        f"--sensor_data_bag={bag}",
        f"--base_output_folder={official}",
        f"--camera_topic_config={camera_config}",
        f"--base_link_name={parameters['base_frame']}",
        "--use_raw_image=False",
        "--steps_to_run",
        "edex",
        "compute_poses",
    ]
    print("Official offline mapping: " + " ".join(command), flush=True)
    subprocess.run(command, check=True)
    outputs = list(official.glob("*/cuvslam_map"))
    if len(outputs) != 1 or not any(
        p.is_file() and p.stat().st_size > 0 for p in outputs[0].glob("*.mdb")
    ):
        raise ValueError("公式処理が非空のcuVSLAM .mdb地図を生成していません")
    shutil.move(str(outputs[0]), str(output / "cuvslam_map"))
    version = subprocess.run(
        ["dpkg-query", "-W", "-f=${Version}", "ros-lyrical-isaac-mapping-ros"],
        capture_output=True,
        text=True,
    )
    (output / "mapping_result.json").write_text(
        json.dumps(
            dict(
                backend="nvidia.create_map_offline",
                steps=["edex", "compute_poses"],
                status="vslam_ready",
                snapshot_status="pending",
                official_output=str(outputs[0].parent.relative_to(output)),
                command=command,
                apt_version=version.stdout.strip(),
                bag_metadata_sha256=hashlib.sha256(
                    (bag / "metadata.yaml").read_bytes()
                ).hexdigest(),
            ),
            ensure_ascii=False,
        )
    )
    print("VSLAM地図保存完了。HDMap用点群は未取得（別工程）", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", required=True)
    args = parser.parse_args()
    run(json.loads(Path(args.job).read_text()))


if __name__ == "__main__":
    main()
