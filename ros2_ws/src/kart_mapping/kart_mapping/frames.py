"""ROS-independent rigid transforms; apply the last TF only at finalization."""

import math


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform_record(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return dict(
        parent=msg.header.frame_id.lstrip("/"),
        child=msg.child_frame_id.lstrip("/"),
        stamp_ns=str(stamp_ns(msg.header.stamp)),
        translation=[t.x, t.y, t.z],
        rotation=[q.x, q.y, q.z, q.w],
    )


def rotate(p, q):
    norm = math.sqrt(sum(x * x for x in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Invalid TF quaternion")
    x, y, z, w = [v / norm for v in q]
    a, b, c = p
    tx, ty, tz = 2 * (y * c - z * b), 2 * (z * a - x * c), 2 * (x * b - y * a)
    return [
        a + w * tx + y * tz - z * ty,
        b + w * ty + z * tx - x * tz,
        c + w * tz + x * ty - y * tx,
    ]


def apply(p, tf):
    return [a + b for a, b in zip(rotate(p, tf["rotation"]), tf["translation"])]


class FinalFrames:
    def __init__(self, map_frame="map"):
        self.map_frame = map_frame
        self.transforms = {}

    def update(self, record):
        if record["parent"] != self.map_frame:
            return
        old = self.transforms.get(record["child"])
        if old is None or int(record["stamp_ns"]) >= int(old["stamp_ns"]):
            self.transforms[record["child"]] = record

    def convert(self, values, frame):
        frame = frame.lstrip("/")
        if frame == self.map_frame:
            return [list(p) for p in values], None
        tf = self.transforms.get(frame)
        if tf is None:
            raise ValueError(
                f"Missing final {self.map_frame} <- {frame} transform; refusing identity fallback"
            )
        return [apply(p, tf) for p in values], tf
