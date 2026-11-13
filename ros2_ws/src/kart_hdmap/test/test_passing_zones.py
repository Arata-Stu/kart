"""Shared permission hashing and validation, without ROS or Studio imports."""
import copy
import math
import unittest

from kart_hdmap.passing_zones import geometry_revision, validate_passing_zones, zone_vertices


XY = [[0., 0.], [2., 0.], [2., 2.], [0., 2.]]


def zone(points=XY, closed=False, **changes):
    result = dict(id='straight-1', start_index=0, end_index=2, side='both',
                  geometry_revision=geometry_revision(points, closed))
    result.update(changes)
    return result


class PassingZoneTests(unittest.TestCase):
    def test_revision_normalizes_numbers_but_binds_vertex_order_and_closure(self):
        self.assertEqual(geometry_revision(XY, False), geometry_revision([[0, -0.0], [2, 0], [2, 2], [0, 2]], False))
        for points, closed in ((XY, True), (XY[::-1], False), ([[0., 0.], [2.1, 0.], *XY[2:]], False)):
            with self.subTest(points=points, closed=closed):
                self.assertNotEqual(geometry_revision(XY, False), geometry_revision(points, closed))

    def test_permissions_are_copied_and_empty_means_no_permission(self):
        rows = [zone(side='left')]
        before = copy.deepcopy(rows)
        normalized = validate_passing_zones(rows, XY, False)
        self.assertEqual(normalized, rows)
        normalized[0]['side'] = 'right'
        self.assertEqual(rows, before)
        self.assertEqual(validate_passing_zones([], XY, False), [])

    def test_closed_wrap_is_explicit_and_open_wrap_is_rejected(self):
        wrapped = zone(closed=True, start_index=3, end_index=1)
        self.assertEqual(validate_passing_zones([wrapped], XY, True), [wrapped])
        with self.assertRaisesRegex(ValueError, 'range'):
            validate_passing_zones([zone(start_index=3, end_index=1)], XY, False)

    def test_stale_geometry_is_not_silently_rebased(self):
        rows = [zone()]
        changed = copy.deepcopy(XY)
        changed[1][0] += .01
        with self.assertRaisesRegex(ValueError, 'revision mismatch'):
            validate_passing_zones(rows, changed, False)
        with self.assertRaisesRegex(ValueError, 'revision mismatch'):
            validate_passing_zones(rows, XY, True)

    def test_visualized_vertices_include_both_endpoints_and_only_the_authorized_arc(self):
        for closed, start, end, expected in ((False, 0, 2, XY[:3]),
                                             (True, 3, 1, [XY[3], XY[0], XY[1]])):
            with self.subTest(closed=closed):
                row = validate_passing_zones([zone(closed=closed, start_index=start, end_index=end)], XY, closed)[0]
                vertices = zone_vertices(XY, row)
                self.assertEqual(vertices, expected)
                vertices[0][0] += 10
                self.assertEqual(XY[0], [0., 0.])
                self.assertEqual(XY[3], [0., 2.])

    def test_waypoint_range_types_bounds_and_nonzero_length(self):
        for changes in ({'start_index': True}, {'end_index': 2.0}, {'start_index': -1},
                        {'end_index': len(XY)}, {'start_index': 1, 'end_index': 1}):
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, 'range'):
                validate_passing_zones([zone(**changes)], XY, False)

    def test_ids_sides_exact_fields_and_digest_are_checked(self):
        invalid = [zone(id=''), zone(id='../escape'), zone(side='left_or_right'),
                   zone(geometry_revision='0'*63), zone(geometry_revision='0'*64),
                   dict(zone(), arbitrary=True)]
        missing = zone()
        del missing['side']
        invalid.append(missing)
        for row in invalid:
            with self.subTest(row=row), self.assertRaises(ValueError):
                validate_passing_zones([row], XY, False)
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            validate_passing_zones([zone(id='A'), zone(id='a')], XY, False)

    def test_permission_count_is_bounded(self):
        rows = [zone(id=f'z{index}') for index in range(64)]
        self.assertEqual(len(validate_passing_zones(rows, XY, False)), 64)
        with self.assertRaisesRegex(ValueError, '64'):
            validate_passing_zones([*rows, zone(id='extra')], XY, False)
        for rows in (None, {}, 'all'):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                validate_passing_zones(rows, XY, False)

    def test_geometry_rejects_invalid_shape_nonfinite_and_boolean_coordinates(self):
        for points, closed in ((XY, 'false'), (XY[:1], False), (XY[:2], True),
                               ([[0, 0, 0], [1, 0, 0]], False),
                               ([[False, 0], [1, 0]], False), ([[math.nan, 0], [1, 0]], False),
                               ([[math.inf, 0], [1, 0]], False), ([['0', 0], [1, 0]], False)):
            with self.subTest(points=points, closed=closed), self.assertRaises(ValueError):
                geometry_revision(points, closed)


if __name__ == '__main__':
    unittest.main()
