"""Capture live ROS endpoint/parameter and host UDP metadata; no topic payloads."""

import argparse
import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import time

TOPICS = (
    "/realsense/color/image_raw", "/realsense/infra1/image_rect_raw",
    "/realsense/infra2/image_rect_raw", "/realsense/imu",
    "/visual_slam/tracking/odometry", "/e2e/tensor_input", "/e2e/tensor_output",
    "/e2e/resize/image", "/e2e/rgb/image", "/e2e/image_tensor",
    "/e2e/normalized_tensor", "/e2e/planar_tensor",
    "/auto/control_cmd", "/teleop/control_cmd", "/vehicle/control_cmd",
    "/operation_mode/state", "/planning/reference_line",
)
NODES = (
    "/realsense", "/visual_slam", "/e2e_resize", "/e2e_format",
    "/e2e_to_tensor", "/e2e_normalize", "/e2e_planar", "/e2e_reshape",
    "/e2e_tensor_rt", "/e2e_control_decoder", "/kart_rosbag_recorder",
)


def command(argv):
    started = time.monotonic()
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=10, check=False)
        return dict(argv=argv, returncode=result.returncode, stdout=result.stdout,
                    stderr=result.stderr, elapsed_s=time.monotonic() - started)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(argv=argv, returncode=None, error=str(exc),
                    elapsed_s=time.monotonic() - started)


def read(path):
    try:
        return Path(path).read_text()
    except OSError:
        return None


def udp_counters(text):
    """Linux network-namespace totals, not per-DDS or per-topic statistics."""
    rows = [line.split()[1:] for line in (text or "").splitlines() if line.startswith("Udp:")]
    if len(rows) != 2 or len(rows[0]) != len(rows[1]):
        return None
    try:
        return dict(zip(rows[0], map(int, rows[1])))
    except ValueError:
        return None


def counter_delta(before, after):
    if before is None or after is None or set(before) != set(after):
        return None
    if any(after[k] < before[k] for k in before):
        return None
    return {k: after[k] - before[k] for k in before}


def capture(topics, nodes, sample_seconds):
    before = udp_counters(read("/proc/net/snmp"))
    started = time.monotonic()
    time.sleep(sample_seconds)
    after = udp_counters(read("/proc/net/snmp"))
    report = dict(
        schema="kart.ros_transport_snapshot.v1",
        captured_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope="ROS metadata only; no payload subscriptions, parameter changes or control publications",
        limitations=["Not a latency benchmark or proof of race readiness",
                     "UDP totals cover the network namespace and may include other applications",
                     "CLI metadata discovery and parameter requests add some load",
                     "Intra-process/GPU buffer use cannot be established from endpoint QoS alone"],
        environment={k: os.environ.get(k) for k in (
            "ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI",
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_LOCALHOST_ONLY")},
        udp=dict(before=before, after=after, delta=counter_delta(before, after),
                 elapsed_s=time.monotonic() - started),
        kernel={k: read("/proc/sys/" + k.replace(".", "/")) for k in (
            "net.core.rmem_default", "net.core.rmem_max",
            "net.core.wmem_default", "net.core.wmem_max")},
        graph=command(["ros2", "node", "list"]),
        topic_list=command(["ros2", "topic", "list", "-t"]),
        topics={topic: command(["ros2", "topic", "info", "--verbose", topic]) for topic in topics},
        parameters={node: command(["ros2", "param", "dump", node]) for node in nodes},
        udp_sockets=command(["ss", "-u", "-a", "-n", "-m"]),
        installed_versions=command(["dpkg-query", "-W", "-f=${Package} ${Version}\n",
                                    "ros-*-rmw-cyclonedds-cpp", "ros-*-cyclonedds",
                                    "ros-*-isaac-ros-common", "ros-*-rosbag2-transport"]),
    )
    uri = os.environ.get("CYCLONEDDS_URI", "")
    if uri.startswith("file://"):
        report["dds_config"] = dict(path=uri[7:], text=read(uri[7:]))
    results = [report["graph"], report["topic_list"],
               *report["topics"].values(), *report["parameters"].values()]
    report["metadata_complete"] = all(r.get("returncode") == 0 for r in results)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="New JSON file; existing files are refused")
    parser.add_argument("--topic", action="append", help="Repeat to override default topics")
    parser.add_argument("--node", action="append", help="Repeat to override default parameter-dump nodes")
    parser.add_argument("--sample-seconds", type=float, default=5)
    args = parser.parse_args()
    if not math.isfinite(args.sample_seconds) or not 0 <= args.sample_seconds <= 60:
        parser.error("sample-seconds must be finite and between 0 and 60")
    if any(not name.startswith("/") or name.startswith("//")
           for name in [*(args.topic or TOPICS), *(args.node or NODES)]):
        parser.error("topic and node names must be absolute ROS names")
    # Reserve the destination before querying, without overwriting earlier evidence.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as output:
        report = capture(args.topic or TOPICS, args.node or NODES, args.sample_seconds)
        json.dump(report, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write("\n")
    print(f"Saved {args.output}; metadata_complete={report['metadata_complete']}")
    return 0 if report["metadata_complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
