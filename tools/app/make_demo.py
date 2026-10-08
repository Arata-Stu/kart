#!/usr/bin/env python3
"""Create an explicitly synthetic, isolated map for UI review. No ROS required."""

import argparse
import math
import sys
from pathlib import Path

APP = Path(__file__).resolve().parent
sys.path[:0] = [str(APP), str(APP.parent.parent / "ros2_ws/src/kart_mapping")]
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    args = parser.parse_args()
    root = args.data_root.resolve()
    maps = Maps(root / "map")
    if (maps.root / "demo_course").exists():
        raise SystemExit("demo_course already exists; use another data root")
    cloud = []
    for i in range(800):
        theta = i * math.tau / 800
        for radius in (4.0, 5.2):
            for z in (0.05, 0.3, 0.6, 1.2):
                cloud.append(
                    [radius * math.cos(theta), radius * 0.65 * math.sin(theta), z]
                )
    source = root / "synthetic_snapshot.json"
    atomic_json(
        source,
        dict(
            schema="kart.snapshot.v1",
            frame="map",
            points=cloud,
            provenance=dict(source="Synthetic demo, not measured data"),
        ),
    )
    maps.import_snapshot("demo_course", source)
    doc = maps.load("demo_course")
    for side, radius in [("left", 4.0), ("right", 5.2), ("custom", 4.6)]:
        doc["lanes"][0][side] = [
            [
                radius * math.cos(i * math.tau / 32),
                radius * 0.65 * math.sin(i * math.tau / 32),
            ]
            for i in range(32)
        ]
    doc = maps.save("demo_course", doc)
    for kind in ("centerline", "raceline", "customline"):
        maps.generate("demo_course", kind, {}, doc["revision"], lambda: None)
        doc = maps.load("demo_course")
    print(f"Synthetic demo ready: ./tools/app/start.sh --data-root {root}")


if __name__ == "__main__":
    main()
