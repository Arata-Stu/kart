"""Run with generated ROS messages; skipped on non-ROS hosts."""

import unittest

try:
    from builtin_interfaces.msg import Time
except ImportError:
    ROS = False
else:
    from kart_hdmap.messages import markers, path_message

    ROS = True


@unittest.skipUnless(ROS, "ROS message packages unavailable")
class MessageTests(unittest.TestCase):
    def test_passing_zone_bands_labels_wrap_and_old_zones_clear(self):
        from kart_hdmap.passing_zones import geometry_revision
        points = [[0., 0.], [2., 0.], [2., 2.], [0., 2.]]
        line = dict(points=points, passing_zones=[dict(id='wrap', start_index=3, end_index=1,
                    side='right', geometry_revision=geometry_revision(points, True))])
        lane = dict(id='main', closed=True, left=[], right=[], lines=dict(raceline=line))
        document = dict(frame_id='map', lanes=[lane])
        rendered = markers(document, 'main', Time(sec=12), .04, .25)
        band = next(marker for marker in rendered.markers if marker.ns.endswith('/passing/wrap/band'))
        label = next(marker for marker in rendered.markers if marker.ns.endswith('/passing/wrap/label'))
        self.assertEqual([(point.x, point.y) for point in band.points], [tuple(points[index]) for index in (3, 0, 1)])
        self.assertTrue(all(point.z > 0 for point in band.points))
        self.assertGreater(band.scale.x, .04)
        self.assertIn('main/raceline: wrap [right]', label.text)
        self.assertEqual(band.header.frame_id, 'map')
        self.assertEqual(band.header.stamp.sec, 12)
        line['passing_zones'] = []
        cleared = markers(document, 'main', Time(sec=13), .04, .25)
        self.assertEqual(cleared.markers[0].action, 3)
        self.assertFalse(any('/passing/' in marker.ns for marker in cleared.markers))

    def test_frames_closure_and_stale_marker_clear(self):
        line = {"points": [[1.0, 2.0], [3.0, 2.0], [3.0, 4.0]]}
        lane = {
            "id": "main",
            "closed": True,
            "left": line["points"],
            "right": [],
            "lines": {"centerline": line},
        }
        doc = {"frame_id": "map", "lanes": [lane]}
        stamp = Time(sec=12)
        msg = markers(doc, "main", stamp, 0.04, 0.25)
        self.assertEqual(msg.markers[0].action, 3)
        for m in msg.markers:
            self.assertEqual(m.header.frame_id, "map")
            self.assertEqual(m.header.stamp.sec, 12)
        path = path_message(line, True, "map", stamp)
        self.assertEqual(len(path.poses), 4)
        self.assertEqual(path.poses[0].pose.position.x, 1.0)
        self.assertEqual(path.poses[0].pose.position.z, 0.0)
        self.assertEqual(path.poses[0].header.frame_id, "map")
        empty = path_message(None, False, "map", stamp)
        self.assertEqual(len(empty.poses), 0)

    def test_reference_speed_and_invalidation(self):
        from kart_hdmap.messages import reference_message

        line = {
            "points": [[0, 0], [1, 0], [2, 0]],
            "profile": [
                [0, 0, 0, 0, 0, 1.2, 0],
                [1, 1, 0, 0, 0, 0.4, 0],
                [2, 2, 0, 0, 0, 0, 0],
            ],
        }
        lane = {
            "id": "main",
            "closed": False,
            "lines": {"customline": line, "centerline": line},
        }
        ref = reference_message(lane, "customline", 0.5, "map", Time(sec=10))
        self.assertEqual(list(ref.speeds), [1.2, 0.4, 0.0])
        self.assertEqual(len(ref.points), 3)
        self.assertEqual(ref.header.frame_id, "map")
        self.assertEqual(
            list(reference_message(lane, "centerline", 0.5, "map", Time()).speeds),
            [0.5] * 3,
        )
        self.assertFalse(reference_message(lane, "raceline", 0.5, "map", Time()).points)
        self.assertFalse(
            reference_message(None, "centerline", 0.5, "map", Time()).points
        )
