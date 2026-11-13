"""Experimental routing semantics; real swept geometry, no ROS or GP fixtures."""
from dataclasses import replace
import json
import math
import unittest

from kart_racing.behavior import Planner, Settings
from kart_hdmap.passing_zones import geometry_revision


def document(left=2., right=-2., obstacles=None):
    points=[[-5.,0.],[30.,0.]]
    line=dict(points=points,passing_zones=[dict(id='straight',start_index=0,end_index=1,side='both',
                                               geometry_revision=geometry_revision(points,False))])
    return dict(schema='kart.hdmap.visualization.v1', frame_id='map', revision=1,
                lanes=[dict(id='main', closed=False, left=[[-5., left], [30., left]],
                            right=[[-5., right], [30., right]],lines=dict(centerline=line))], obstacles=obstacles or [])


def object_(identity='target', kind='obstacle', x=3., y=0., speed=0., now=0., confirmed=True, **values):
    return dict(id=identity, kind=kind, x=x, y=y, speed=speed, observed_at=now, confirmed=confirmed, **values)


class BehaviorTests(unittest.TestCase):
    def planner(self, doc=None, settings=None, speed=1.):
        planner = Planner(settings)
        planner.set_reference([[-5., 0.], [30., 0.]], [speed, speed], False, doc or document(), 'main', 'r1')
        return planner

    def assert_stop(self, result):
        self.assertTrue(result['path'])
        self.assertEqual(result['speeds'], [0.] * len(result['path']))
        self.assertEqual(result['target_speed'], 0.)
        json.dumps(result, allow_nan=False)

    def moving_pass(self):
        planner = self.planner()
        for t in (0., .2, .4, .6):
            result = planner.update(t, (.8 * t, 0., 0., .8), [object_(kind='rc_car', x=2 + .4 * t, speed=.4, now=t)])
        self.assertEqual(result['state'], 'PASS_MOVING')
        return planner, result

    def test_clear_is_bounded_capped_and_disclaims_unobserved_space(self):
        planner = self.planner(settings=Settings(sample_spacing=.02, lookahead_m=14.), speed=.6)
        result = planner.update(0., (0., 0., 0., .5), [])
        self.assertEqual(result['state'], 'CLEAR')
        self.assertLessEqual(len(result['path']), 300)
        self.assertEqual(len(result['path']), len(result['speeds']))
        self.assertLessEqual(max(result['speeds']), .6)
        self.assertTrue(result['experimental'])
        self.assertIn('not proven free', result['unknown_space'])
        json.dumps(result, allow_nan=False)

    def test_static_and_slow_targets_avoid_immediately_without_gp(self):
        for kind, speed, state in (('obstacle', 0., 'AVOID_OBSTACLE'), ('rc_car', 0., 'AVOID_SLOW'),
                                   ('rc_car', .2, 'AVOID_SLOW')):
            planner = self.planner()
            result = planner.update(0., (0., 0., 0., .8), [object_(kind=kind, speed=speed)])
            with self.subTest(kind=kind, speed=speed):
                self.assertEqual(result['state'], state)
                self.assertEqual(result['route_revision'], 1)
                self.assertEqual(result['geometry']['merge_clearance_m'], .2)
                self.assertTrue(all(candidate['max_curvature'] <= 1.2 + 1e-6
                                    for candidate in result['candidates'] if candidate['feasible']))
                self.assertGreater(max(point[1] for point in result['path']), .3)

    def test_moving_rc_follows_then_passes_after_stable_speed_advantage(self):
        planner = self.planner()
        for t in (0., .2, .4):
            result = planner.update(t, (.8*t, 0., 0., .8), [object_(kind='rc_car', x=2+.4*t, speed=.4, now=t)])
            self.assertEqual(result['state'], 'OBSERVE')
            self.assertLessEqual(max(result['speeds']), .4)
        result = planner.update(.6, (.48, 0., 0., .8), [object_(kind='rc_car', x=2.24, speed=.4, now=.6)])
        self.assertEqual(result['state'], 'PASS_MOVING')
        self.assertAlmostEqual(result['geometry']['merge_clearance_m'], .2)
        self.assertTrue(result['candidates'][0]['merge_start'])
        self.assertTrue(result['candidates'][0]['merge_end'])

    def test_insufficient_speed_advantage_continues_following_with_headway_cap(self):
        planner = self.planner()
        for t in (0., .2, .4, .6):
            result = planner.update(t, (.8*t, 0., 0., .8), [object_(kind='rc_car', x=3+.7*t, speed=.7, now=t)])
        self.assertEqual(result['state'], 'FOLLOW')
        self.assertGreater(max(result['speeds']), 0.)
        self.assertLessEqual(max(result['speeds']), .7)
        close = self.planner().update(0., (0., 0., 0., .8), [object_(kind='rc_car', x=.7, speed=.7)])
        self.assertEqual(max(close['speeds']), 0.)

    def test_unstable_or_unconfirmed_moving_observations_do_not_arm_passing(self):
        planner = self.planner()
        for index, t in enumerate((0., .2, .4, .6, .8)):
            speed = .4 if index % 2 else .5
            result = planner.update(t, (.4*t, 0., 0., .8), [object_(kind='rc_car', x=2+.4*t, speed=speed, now=t)])
            self.assertEqual(result['state'], 'OBSERVE')
        result = self.planner().update(0., (0., 0., 0., .8), [object_(confirmed=False)])
        self.assertEqual(result['state'], 'OBSERVE')
        self.assert_stop(result)

    def test_fleeting_unconfirmed_detection_is_debounced_without_a_permanent_fault(self):
        planner=self.planner()
        first=planner.update(0.,(0.,0.,0.,0.),[object_('untracked_rc:0',kind='rc_car',confirmed=False)])
        self.assertEqual(first['state'],'OBSERVE')
        missing=planner.update(.1,(0.,0.,0.,0.),[])
        self.assertEqual(missing['state'],'OBSERVE')
        self.assert_stop(missing)
        expired=planner.update(.41,(0.,0.,0.,0.),[])
        self.assertEqual(expired['state'],'CLEAR')
        self.assertIsNone(planner.fault)

    def test_opposite_side_fallback_and_both_sides_blocked(self):
        result = self.planner(document(left=.45)).update(0., (0., 0., 0., .8), [object_()])
        self.assertEqual(result['state'], 'AVOID_OBSTACLE')
        self.assertEqual(result['side'], 'right')
        self.assertFalse(next(c for c in result['candidates'] if c['side'] == 'left')['feasible'])
        blocked = self.planner(document(left=.45, right=-.45)).update(0., (0., 0., 0., .8), [object_()])
        self.assertEqual(blocked['state'], 'BLOCKED')
        self.assert_stop(blocked)

    def test_all_blocking_objects_extend_return_and_registered_geometry_is_checked(self):
        planner = self.planner()
        result = planner.update(0., (0., 0., 0., .8), [object_(), object_('second', x=4.2)])
        self.assertEqual(result['state'], 'AVOID_OBSTACLE')
        selected = next(c for c in result['candidates'] if c['side'] == result['side'])
        self.assertGreater(selected['merge_start_s'] - 5., 4.2 + .2)
        registered = dict(id='wall', polygon=[[2.75, -.2], [3.25, -.2], [3.25, .2], [2.75, .2]])
        result = self.planner(document(obstacles=[registered])).update(0., (0., 0., 0., .8), [])
        self.assertEqual(result['state'], 'AVOID_OBSTACLE')
        self.assertEqual(result['target_id'], 'map:wall')

    def test_full_length_footprint_catches_front_corner_not_only_centre(self):
        wide = self.planner()
        result = wide.update(0., (0., 0., 0., .8), [object_()])
        selected = next(c for c in result['candidates'] if c['side'] == result['side'])
        path, speeds = selected['path'], [.8] * len(selected['path'])
        # A thin exclusion intersects the front of the first ego footprint while
        # its centre and the next centre samples remain outside the polygon.
        registered = dict(id='front-corner', polygon=[[.14, .04], [.18, .04], [.18, .08], [.14, .08]])
        narrow = self.planner(document(obstacles=[registered]))
        valid, _, _, _ = narrow._check(path, speeds, [], (0., 0., 0., .8))
        self.assertFalse(valid)

    def test_rear_axle_origin_uses_asymmetric_vehicle_body(self):
        # .31m is outside the centred half-length + .12 uncertainty envelope,
        # but inside the measured-policy front .2435 + .12 envelope.
        registered = dict(id='axle-front', polygon=[[.31, .04], [.33, .04], [.33, .08], [.31, .08]])
        planner = self.planner(document(obstacles=[registered]))
        self.assertAlmostEqual(planner.world.front, .2435)
        self.assertAlmostEqual(planner.world.rear, .1065)
        self.assertFalse(planner.world.clear((0., 0., 0.)))
        with self.assertRaises(ValueError):
            Settings(vehicle_length=.2, rear_axle_to_rear=.2)

    def test_adopted_route_is_persistent_and_object_memory_allows_camera_loss(self):
        planner = self.planner()
        first = planner.update(0., (0., 0., 0., .8), [object_()])
        revision, merge = first['route_revision'], planner.active['merge_end_s']
        second = planner.update(.1, (.1, 0., 0., .8), [])
        self.assertEqual(second['state'], 'AVOID_OBSTACLE')
        self.assertEqual(second['route_revision'], revision)
        self.assertEqual(planner.active['merge_end_s'], merge)
        self.assertGreater(max(p[1] for p in second['path']), .3)

    def test_static_camera_disappearance_can_finish_merge_then_return_to_baseline(self):
        planner = self.planner()
        first = planner.update(0., (0., 0., 0., .8), [object_()])
        elapsed, previous = 0., first['path'][0]
        states = []
        result=first
        for _ in range(150):
            point=result['path'][min(3,len(result['path'])-1)]
            elapsed += math.dist(previous, point) / .8
            yaw = math.atan2(point[1]-previous[1], point[0]-previous[0])
            observed = [object_(now=elapsed)] if point[0] < 3. else []
            result = planner.update(elapsed, (point[0], point[1], yaw, .8), observed)
            states.append(result['state'])
            previous = point
            if planner.active is None:
                break
        self.assertIn('MERGE', states)
        self.assertEqual(states[-1], 'CLEAR')
        self.assertTrue(all(abs(point[1]) < 1e-9 for point in result['path']))

    def test_moving_target_loss_is_predicted_but_acceleration_stops_on_adopted_route(self):
        for scenario in ('loss', 'acceleration'):
            planner, first = self.moving_pass()
            observed = [] if scenario == 'loss' else [object_(kind='rc_car', x=2.3, speed=.65, now=.7)]
            result = planner.update(.7, (.56, 0., 0., .8), observed)
            with self.subTest(scenario=scenario):
                if scenario=='loss':
                    self.assertIn(result['state'],('PASS_MOVING','COAST_PASS','HOLD_SIDE'))
                    self.assertGreater(result['target_speed'],0.)
                    continue
                self.assertEqual(result['state'], 'FAULT')
                self.assertEqual(result['path'], first['path'])
                self.assert_stop(result)
                self.assertGreater(max(p[1] for p in result['path']), .3)

    def test_speed_flicker_inside_simple_threshold_keeps_the_committed_side(self):
        planner = self.planner()
        first = planner.update(0., (0., 0., 0., .8), [object_(kind='rc_car', speed=.34)])
        self.assertEqual(first['state'], 'AVOID_SLOW')
        result = planner.update(.1, (.05, 0., 0., .8), [object_(kind='rc_car', x=3.036, speed=.36, now=.1)])
        self.assertEqual(result['state'],'AVOID_SLOW')
        self.assertEqual(result['side'],first['side'])
        self.assertGreater(result['target_speed'],0.)

    def test_static_memory_expiration_is_unknown_not_clear(self):
        planner = self.planner()
        first = planner.update(0., (0., 0., 0., .8), [object_()])
        result = planner.update(5.01, (.1, 0., 0., .8), [])
        self.assertEqual(result['state'], 'FAULT')
        self.assertEqual(result['path'], first['path'])
        self.assert_stop(result)
        self.assertEqual(planner.update(5.1, (.1, 0., 0., .8), [object_(now=5.1)])['state'], 'FAULT')

    def test_new_obstacle_invalidates_adopted_side_without_baseline_fallback(self):
        planner = self.planner()
        first = planner.update(0., (0., 0., 0., .8), [object_()])
        result = planner.update(.1, (.05, 0., 0., .8), [object_(now=.1), object_('new', x=2.5, y=.5, now=.1)])
        self.assertEqual(result['state'], 'BLOCKED')
        self.assertEqual(result['path'], first['path'])
        self.assert_stop(result)

    def test_reference_geometry_revision_and_parameter_changes_latch_held_path(self):
        for change in ('revision', 'geometry', 'parameters'):
            planner = self.planner()
            first = planner.update(0., (0., 0., 0., .8), [object_()])
            if change == 'parameters':
                planner.settings = replace(planner.settings, speed_cap=.7)
                planner.set_reference([[-5., 0.], [30., 0.]], [1., 1.], False, document(), 'main', 'r1')
            elif change == 'geometry':
                planner.set_reference([[-5., 0.], [30., 0.]], [1., 1.], False, document(left=1.9), 'main', 'r1')
            result = planner.update(.1, (.1, 0., 0., .8), [object_(now=.1)], geometry_revision='r2' if change == 'revision' else 'r1')
            with self.subTest(change=change):
                self.assertEqual(result['state'], 'FAULT')
                self.assertEqual(result['path'], first['path'])
                self.assert_stop(result)

    def test_external_invalidation_returns_result_and_initial_waiting_does_not_latch(self):
        planner = self.planner()
        waiting = planner.invalidate('waiting_for_tf')
        self.assertEqual(waiting['state'], 'OBSERVE')
        self.assertIsNone(planner.fault)
        first = planner.update(0., (0., 0., 0., .8), [object_()])
        failed = planner.invalidate('detections_timeout')
        self.assertEqual(failed['state'], 'FAULT')
        self.assertEqual(failed['path'], first['path'])
        self.assert_stop(failed)

    def test_invalid_stale_duplicate_inputs_and_clock_reversal_never_clear(self):
        cases = ([object_(now=-2.)], [object_(), object_()], [object_(x=float('nan'))],
                 [object_(kind='unknown')], [object_(confirmed=1)], [object_()] * 65)
        for objects in cases:
            with self.subTest(objects=objects):
                planner = self.planner()
                result = planner.update(0., (0., 0., 0., .8), objects)
                self.assertEqual(result['state'], 'FAULT')
                self.assert_stop(result)
        planner = self.planner()
        planner.update(1., (0., 0., 0., .8), [])
        self.assertEqual(planner.update(.9, (0., 0., 0., .8), [])['state'], 'FAULT')

    def test_output_mutation_cannot_modify_held_route(self):
        planner = self.planner()
        result = planner.update(0., (0., 0., 0., .8), [object_()])
        first = [point[:] for point in result['path']]
        result['path'][0][0] = float('nan')
        result['candidates'][0]['path'][0][0] = float('nan')
        failed = planner.invalidate('test')
        self.assertEqual(failed['path'], first)
        json.dumps(failed, allow_nan=False)

    def test_settings_constraints_reject_unbounded_or_inconsistent_values(self):
        for values in (dict(speed_cap=float('nan')), dict(max_curvature=0.), dict(min_transition=.1),
                       dict(vehicle_width=True), dict(preferred_side='centre'),
                       dict(static_merge_margin=1., moving_merge_margin=.5), dict(static_memory_s=.5, object_timeout=1.)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Settings(**values)

    def test_opposing_crossing_or_inconsistent_world_velocity_is_unknown(self):
        for vx, vy, speed in ((-.5, 0., .5), (.4, .2, math.hypot(.4, .2)), (.4, 0., 1.)):
            result = self.planner().update(0., (0., 0., 0., .8),
                [object_(kind='rc_car', speed=speed, vx=vx, vy=vy)])
            with self.subTest(vx=vx, vy=vy):
                self.assertEqual(result['state'], 'FAULT')
                self.assert_stop(result)
        result = self.planner().update(0., (0., 0., 0., .8), [object_(kind='rc_car', speed=.4, vx=.4, vy=0.)])
        self.assertEqual(result['state'], 'OBSERVE')

    def test_sharp_baseline_curvature_is_blocked_even_without_detections(self):
        planner = Planner()
        doc=document(left=10.,right=-10.)
        doc['lanes'][0]['lines']['centerline'].pop('passing_zones')
        planner.set_reference([[-5., 0.], [2., 0.], [2., 10.]], [1.] * 3, False,
                              doc, 'main', 'r1')
        result = planner.update(0., (0., 0., 0., .8), [])
        self.assertEqual(result['state'], 'BLOCKED')
        self.assertEqual(result['reason'], 'curvature_limit')
        self.assert_stop(result)

    def test_closed_lane_keeps_inner_hole_as_nondrivable(self):
        def circle(radius):
            return [[radius * math.cos(i * 2 * math.pi / 120), radius * math.sin(i * 2 * math.pi / 120)]
                    for i in range(120)]
        for inner, expected in ((3., 'CLEAR'), (4.9, 'BLOCKED')):
            doc = dict(schema='kart.hdmap.visualization.v1', frame_id='map', obstacles=[],
                       lanes=[dict(id='ring', closed=True, left=circle(inner), right=circle(7.))])
            planner = Planner()
            planner.set_reference(circle(5.), [1.] * 120, True, doc, 'ring', 'ring1')
            result = planner.update(0., (5., 0., math.pi/2, .8), [])
            with self.subTest(inner=inner):
                self.assertEqual(result['state'], expected)
                if expected == 'BLOCKED':
                    self.assert_stop(result)


if __name__ == '__main__':
    unittest.main()
