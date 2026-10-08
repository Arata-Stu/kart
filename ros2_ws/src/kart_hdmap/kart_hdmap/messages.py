"""ROS visualization messages; input XY stays unchanged in the map frame."""

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from .document import KINDS, path_samples

COLORS = {
    "left": (0.2, 0.6, 1.0),
    "right": (1.0, 0.5, 0.2),
    "centerline": (0.2, 0.8, 0.4),
    "raceline": (0.8, 0.3, 1.0),
    "customline": (1.0, 0.8, 0.1),
}


def path_message(line, closed, frame, stamp):
    msg = Path()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    if line:
        for x, y, qz, qw in path_samples(line, closed):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
            msg.poses.append(pose)
    return msg


def markers(document, selected, stamp, width, label_height):
    result = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    clear.header.frame_id, clear.header.stamp = document["frame_id"], stamp
    result.markers.append(clear)
    for lane in document["lanes"]:
        geometry = {"left": lane["left"], "right": lane["right"]}
        geometry.update(
            {k: lane["lines"][k]["points"] for k in KINDS if k in lane["lines"]}
        )
        for kind, pts in geometry.items():
            if not pts:
                continue
            m = Marker()
            m.header = clear.header
            m.ns, m.id = lane["id"] + "/" + kind, 0
            m.type, m.action = Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width * (1.5 if lane["id"] == selected else 1.0)
            m.color.r, m.color.g, m.color.b = COLORS[kind]
            m.color.a = 1.0
            vertices = pts + [pts[0]] if lane["closed"] and pts[-1] != pts[0] else pts
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in vertices]
            result.markers.append(m)
        anchor = next((p for p in geometry.values() if p), None)
        if anchor:
            m = Marker()
            m.header = clear.header
            m.ns, m.id = lane["id"] + "/label", 0
            m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.orientation.w = 1.0
            m.pose.position.x, m.pose.position.y = anchor[0]
            m.pose.position.z = label_height
            m.scale.z = label_height
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = lane["id"]
            result.markers.append(m)
    for obstacle in document.get("obstacles", []):
        m = Marker()
        m.header = clear.header
        m.ns, m.id = "obstacles/" + obstacle["id"] + "/outline", 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width * 2
        m.color.r, m.color.a = 1.0, 1.0
        vertices = obstacle["polygon"] + obstacle["polygon"][:1]
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in vertices]
        result.markers.append(m)
    return result


def reference_message(lane, kind, centerline_speed, frame, stamp):
    from kart_interfaces.msg import ReferenceLine

    msg = ReferenceLine()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.lane_id, msg.line_type = lane["id"] if lane else "", kind
    if lane and kind in lane["lines"]:
        line = lane["lines"][kind]
        msg.closed = lane["closed"]
        msg.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in line["points"]]
        msg.speeds = (
            [float(centerline_speed)] * len(msg.points)
            if kind == "centerline"
            else [float(row[5]) for row in line["profile"]]
        )
    return msg
