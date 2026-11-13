"""Passing permissions and measured/learned coasting through the real planner."""
import hashlib
import math
import os
import unittest

from kart_hdmap.passing_zones import geometry_revision
from kart_racing.behavior import Planner, Settings
from kart_racing.opponent_model import LIBRARY_ENV, OpponentModel


class Model:
    """Inject only the prediction boundary; all route/geometry checks stay real."""
    def __init__(self, ready=False, speed=2.1, available=True):
        self.available, self.reason = available, '' if available else 'native_library_unavailable'
        self.is_ready, self.speed = ready, speed
        self.observations, self.queries = [], []

    def observe(self, *measurement):
        self.observations.append(measurement)
        return self.available

    def ready(self):
        return self.is_ready and self.available

    def coverage(self):
        return .95 if self.ready() else .3 if self.available else 0.

    def predict(self, station):
        self.queries.append(station)
        return dict(d=0., v=self.speed, d_sigma=.01, v_sigma=.01, valid=self.ready())

    def clear(self):
        self.is_ready = False


def open_map(*, start=0, end=35, side='both', zones=True):
    points = [[float(x), 0.] for x in range(-5, 31)]
    permissions = [dict(id='straight', start_index=start, end_index=end, side=side,
                        geometry_revision=geometry_revision(points, False))] if zones else []
    line = dict(points=points, passing_zones=permissions)
    document = dict(schema='kart.hdmap.visualization.v1', frame_id='map', revision=1, obstacles=[],
                    lanes=[dict(id='main', closed=False, left=[[-5., 3.], [30., 3.]],
                                right=[[-5., -3.], [30., -3.]], lines=dict(centerline=line))])
    return points, document


def target(now, *, identity='target', x=3., speed=0., kind='obstacle', confirmed=True, **values):
    return dict(id=identity, kind=kind, x=x, y=0., speed=speed, observed_at=now, confirmed=confirmed, **values)


def summary(result):
    return dict(state=result['state'], reason=result['reason'],
                candidates=[(item['side'], item['feasible'], item['reason']) for item in result.get('candidates', [])])


def point_at_distance(path, distance):
    for a, b in zip(path, path[1:]):
        length = math.dist(a, b)
        if distance <= length:
            factor = max(0., min(1., distance / length))
            return (a[0] + (b[0] - a[0]) * factor, a[1] + (b[1] - a[1]) * factor,
                    math.atan2(b[1] - a[1], b[0] - a[0]), .8)
        distance -= length
    a, b = path[-2:]
    return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0]), .8


def ideal_step(result, ego, dt):
    """Advance along the current published polyline using its local speed cap."""
    path, speeds = result['path'], result['speeds']
    closest = None
    for index, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        denominator = dx * dx + dy * dy
        fraction = max(0., min(1., ((ego[0]-a[0])*dx + (ego[1]-a[1])*dy) / denominator))
        projected = (a[0]+fraction*dx, a[1]+fraction*dy)
        value = (math.dist(ego[:2], projected), index, fraction, projected)
        if closest is None or value[:2] < closest[:2]:
            closest = value
    distance, index, fraction, projected = closest
    if distance > .05:
        raise AssertionError(f'Published route jumped {distance:.3f}m from ideal ego')
    speed = min(speeds[index], speeds[index+1])
    remaining = speed * dt
    for segment in range(index, len(path)-1):
        a = projected if segment == index else path[segment]
        b = path[segment+1]
        length = math.dist(a, b)
        if remaining <= length:
            along = remaining / length if length else 0.
            return (a[0]+along*(b[0]-a[0]), a[1]+along*(b[1]-a[1]),
                    math.atan2(b[1]-a[1], b[0]-a[0]), speed)
        remaining -= length
    a, b = path[-2:]
    return b[0], b[1], math.atan2(b[1]-a[1], b[0]-a[0]), speed


class PermissionTests(unittest.TestCase):
    def planner(self, *, settings=None, start=0, end=35, side='both', zones=True, model=None):
        points, document = open_map(start=start, end=end, side=side, zones=zones)
        factory = (lambda _length: model) if model is not None else None
        planner = Planner(settings, model_factory=factory)
        planner.set_reference(points, [5.] * len(points), False, document, 'main', 'revision1', line_type='centerline')
        return planner

    def assert_stopped(self, result):
        self.assertEqual(result['speeds'], [0.] * len(result['path']))
        self.assertEqual(result['target_speed'], 0.)

    def simple_pass(self, planner=None):
        planner = planner or self.planner(model=Model(available=False))
        for t in (0., .2, .4, .6):
            result = planner.update(t, (.8*t, 0., 0., .8),
                                    [target(t, x=2+.4*t, speed=.4, kind='rc_car')])
        self.assertEqual(result['state'], 'PASS_MOVING', summary(result))
        return planner, result

    def fast_pass(self, model, *, threshold=2., speed=2.1):
        settings = Settings(speed_cap=5., lookahead_m=20., sample_spacing=.1, simple_pass_speed=threshold)
        planner = self.planner(settings=settings, model=model)
        for step in (0., .2, .4, .6):
            result = planner.update(100.+step, (5.*step, 0., 0., 5.),
                                    [target(100.+step, x=4+speed*step, speed=speed, kind='rc_car')])
        return planner, result

    def test_missing_permission_and_outside_zone_cannot_start_a_pass(self):
        for options in (dict(zones=False), dict(start=10, end=25)):
            planner = self.planner(**options)
            result = planner.update(0., (0., 0., 0., .8), [target(0.)])
            with self.subTest(options=options):
                self.assertNotIn(result['state'], ('AVOID_OBSTACLE', 'AVOID_SLOW', 'PASS_MOVING', 'MERGE'))
                self.assertIsNone(planner.active)
                self.assert_stopped(result)

    def test_authorized_side_overrides_preferred_side(self):
        planner = self.planner(side='right')
        result = planner.update(0., (0., 0., 0., .8), [target(0.)])
        self.assertEqual(result['state'], 'AVOID_OBSTACLE')
        self.assertEqual(result['side'], 'right')
        left = next(candidate for candidate in result['candidates'] if candidate['side'] == 'left')
        self.assertFalse(left['feasible'])

    def test_permission_must_cover_full_return_not_only_target_position(self):
        # Ego x=0 and target x=3 are allowed, but the interval ends at x=4,
        # before the rear axle can finish the return transition.
        planner = self.planner(start=0, end=9)
        result = planner.update(0., (0., 0., 0., .8), [target(0.)])
        self.assertIsNone(planner.active)
        self.assertNotEqual(result['state'], 'AVOID_OBSTACLE')
        self.assert_stopped(result)

    def test_stale_geometry_permission_or_selected_line_mismatch_is_rejected(self):
        points, document = open_map()
        document['lanes'][0]['lines']['centerline']['passing_zones'][0]['geometry_revision'] = '0' * 64
        with self.assertRaises(ValueError):
            Planner().set_reference(points, [1.] * len(points), False, document, 'main', 'r1')
        points, document = open_map()
        document['lanes'][0]['lines']['centerline']['points'][0] = [-4.9, 0.]
        with self.assertRaises(ValueError):
            Planner().set_reference(points, [1.] * len(points), False, document, 'main', 'r1')

    def test_closed_wrap_permission_spans_seam_on_same_straight(self):
        points = [[0., 0.], [5., 0.], [10., 0.], [15., 0.], [20., 0.], [20., 10.],
                  [-20., 10.], [-20., 0.], [-15., 0.], [-10., 0.], [-5., 0.]]
        zone = dict(id='seam', start_index=9, end_index=2, side='both',
                    geometry_revision=geometry_revision(points, True))
        document = dict(schema='kart.hdmap.visualization.v1', frame_id='map', obstacles=[],
            lanes=[dict(id='ring', closed=True,
                left=[[-18., 2.], [18., 2.], [18., 8.], [-18., 8.]],
                right=[[-22., -2.], [22., -2.], [22., 12.], [-22., 12.]],
                lines=dict(centerline=dict(points=points, passing_zones=[zone])))])
        planner = Planner()
        planner.set_reference(points, [1.] * len(points), True, document, 'ring', 'ring-revision')
        result = planner.update(0., (-3., 0., 0., .8), [target(0., x=0.)])
        self.assertEqual(result['state'], 'AVOID_OBSTACLE', summary(result))
        self.assertGreater(planner.active['merge_end_s'], planner.reference.length)
        self.assertLess(planner.active['merge_end_s'], planner._permission(planner.ego_station)['end_s'])

    def test_fast_target_requires_ready_gp_while_threshold_target_needs_no_lap(self):
        for available in (False, True):
            model = Model(ready=False, available=available)
            planner, result = self.fast_pass(model)
            with self.subTest(available=available):
                self.assertIsNone(planner.active)
                self.assertNotEqual(result['state'], 'PASS_MOVING')
                self.assertNotEqual(result['state'], 'FAULT', 'An immature historical model is normal following')
        planner, result = self.fast_pass(Model(ready=False, available=False), speed=2.)
        self.assertEqual(result['state'], 'PASS_MOVING', summary(result))

    def test_ready_gp_is_used_for_fast_predicted_occupancy(self):
        model = Model(ready=True)
        planner, result = self.fast_pass(model)
        self.assertEqual(result['state'], 'PASS_MOVING', summary(result))
        self.assertTrue(model.queries, 'Ready alone cannot replace actual GP prediction calls')
        self.assertEqual(len(model.observations), 4)
        self.assertTrue(planner.known['target']['model_ready'])

    def test_committed_target_can_coast_without_training_on_predictions(self):
        model = Model(available=False)
        planner, first = self.simple_pass(self.planner(model=model))
        observations = list(model.observations)
        for age in (.1, .5):
            ego = point_at_distance(first['path'], .8 * age)
            result = planner.update(.6 + age, ego, [])
            self.assertIn(result['state'], ('PASS_MOVING', 'COAST_PASS', 'MERGE'), summary(result))
            self.assertEqual(result['route_revision'], first['route_revision'])
            self.assertEqual(model.observations, observations, 'A forecast must not become a measurement')

    def test_missing_target_timeout_stops_on_held_route(self):
        planner, first = self.simple_pass()
        result = planner.update(.6 + planner.settings.prediction_hold_s + .01,
                                point_at_distance(first['path'], .5), [])
        self.assertIn(result['state'], ('FAULT', 'BLOCKED'))
        self.assert_stopped(result)
        self.assertGreater(max(point[1] for point in result['path']), .3)
        self.assertEqual(result['path'], first['path'])

    def test_tentative_candidate_retires_on_healthy_zero_but_source_invalid_stops(self):
        planner = self.planner(model=Model(available=False))
        result = planner.update(0., (0., 0., 0., .8), [target(0., kind='rc_car', speed=.4, confirmed=False)])
        self.assertEqual(result['state'], 'OBSERVE')
        result = planner.update(.1, (0., 0., 0., .8), [])
        self.assertEqual(result['state'], 'OBSERVE')
        self.assert_stopped(result)
        result = planner.update(.41, (0., 0., 0., .8), [])
        self.assertEqual(result['state'], 'CLEAR')
        self.assertNotIn('target', planner.known)
        invalid = self.planner(model=Model(available=False))
        invalid.update(0., (0., 0., 0., .8), [target(0., kind='rc_car', speed=.4, confirmed=False)])
        stopped = invalid.invalidate('projection_invalid')
        self.assertEqual(stopped['state'], 'FAULT')
        self.assert_stopped(stopped)

    def test_confirmed_target_loss_before_commit_does_not_retire_as_false_proposal(self):
        planner = self.planner(model=Model(available=False))
        first = planner.update(0., (0., 0., 0., .8), [target(0., kind='rc_car', speed=.4)])
        self.assertEqual(first['state'], 'OBSERVE')
        self.assertIsNone(planner.active)
        result = planner.update(.5, (0., 0., 0., .8), [])
        self.assertEqual(result['state'], 'FAULT')
        self.assert_stopped(result)

    def test_merge_requires_actual_longitudinal_clearance(self):
        planner, first = self.simple_pass()
        merge_station = planner.active['merge_start_s']
        point = min(first['path'], key=lambda xy: abs(planner.reference.project(xy)[0] - merge_station))
        # Current measurement says the target is still alongside, despite the
        # route station already reaching its planned merge start.
        result = planner.update(.7, (point[0], point[1], 0., .8),
                                [target(.7, x=point[0] - .1, speed=.4, kind='rc_car')])
        self.assertNotEqual(result['state'], 'MERGE')
        self.assertNotEqual(result['state'], 'CLEAR')
        # Holding/continuing the verified parallel corridor is permitted. Its
        # published PP lookahead must not expose an unverified inward return.
        future = [xy for xy in result['path'] if xy[0] >= point[0] + .1]
        self.assertTrue(future)
        self.assertGreater(min(xy[1] for xy in future), .3)

    def test_initial_pp_route_excludes_inward_merge_until_gap_is_verified(self):
        planner, result = self.simple_pass()
        outward = [point for point in result['path']
                   if planner.reference.project(point, planner.ego_station)[0] > planner.active['out_end_s'] + .1]
        self.assertTrue(outward)
        sign = 1 if result['side'] == 'left' else -1
        self.assertGreater(min(sign * point[1] for point in outward), .3)

    def test_ready_gp_coasting_uses_predictions_without_new_measurements(self):
        model = Model(ready=True)
        planner, first = self.fast_pass(model)
        self.assertEqual(first['state'], 'PASS_MOVING', summary(first))
        count = len(model.observations)
        queries = len(model.queries)
        for age in (.1, .5):
            pose = point_at_distance(first['path'], 5. * age)
            result = planner.update(100.6 + age, (*pose[:3], 5.), [])
            self.assertIn(result['state'], ('PASS_MOVING', 'COAST_PASS', 'MERGE'), summary(result))
            self.assertEqual(len(model.observations), count)
            self.assertEqual(result['target_observation']['mode'], 'predicted')
        self.assertGreater(len(model.queries), queries)

    def test_missing_branch_integrates_current_gp_speed_and_phase_uncertainty(self):
        model = Model(ready=True)
        planner, first = self.fast_pass(model)
        self.assertEqual(first['state'], 'PASS_MOVING', summary(first))
        start = planner.known['target']['s']
        observations = len(model.observations)
        def variable_profile(station):
            model.queries.append(station)
            return dict(d=0., v=2.1 if station <= start + .04 else 3.2,
                        d_sigma=.01, v_sigma=.2, valid=True)
        # Exercise a future section whose learned speed differs from the last
        # scalar measurement. The predictor boundary is injected; current phase
        # propagation and its uncertainty still run inside the actual core.
        model.predict = variable_profile
        pose = point_at_distance(first['path'], 2.5)
        result = planner.update(101.1, (*pose[:3], 5.), [])
        predicted = planner.known['target']
        self.assertAlmostEqual(predicted['s'], start + .1 * 2.1 + .4 * 3.2, places=6)
        self.assertAlmostEqual(predicted['phase_uncertainty'], 2 * .2 * .5, places=6)
        self.assertGreaterEqual(predicted['uncertainty'], .2)
        self.assertGreaterEqual(result['target_observation']['uncertainty_m'], .2)
        self.assertEqual(len(model.observations), observations)

    def test_unconfirmed_reappearance_cannot_authorize_merge_or_refresh_source_age(self):
        model = Model(available=False)
        planner, first = self.simple_pass(self.planner(model=model))
        observed = len(model.observations)
        pose = point_at_distance(first['path'], .8 * 2.3)
        # This would manufacture a large rear clearance if the unconfirmed
        # reappearance replaced the last confirmed state.
        result = planner.update(2.9, pose,
                                [target(2.9, x=-3., speed=.4, kind='rc_car', confirmed=False)])
        self.assertNotIn(result['state'], ('MERGE', 'CLEAR'))
        self.assertTrue(planner.known['target']['confirmed'])
        self.assertAlmostEqual(planner.known['target']['observed_at'], .6)
        self.assertGreater(planner.known['target']['x'], 2.)
        self.assertEqual(result['target_observation']['mode'], 'predicted')
        self.assertEqual(len(model.observations), observed)

    def test_learned_expected_acceleration_continues_but_profile_outlier_stops(self):
        for measured, expected in ((2.4, 2.4), (2.8, 2.1)):
            model = Model(ready=True)
            planner, first = self.fast_pass(model)
            model.speed = expected
            pose = point_at_distance(first['path'], .5)
            result = planner.update(100.7, (*pose[:3], 5.),
                                    [target(100.7, x=5.5, speed=measured, kind='rc_car')])
            with self.subTest(measured=measured, expected=expected):
                if measured == expected:
                    self.assertIn(result['state'], ('PASS_MOVING', 'COAST_PASS', 'HOLD_SIDE', 'MERGE'), summary(result))
                    self.assertEqual(result['route_revision'], first['route_revision'])
                else:
                    self.assertEqual(result['state'], 'FAULT')
                    self.assert_stopped(result)

    def test_gp_cache_survives_short_term_track_pruning_and_reuses_same_model(self):
        models = []
        def factory(_length):
            model = Model(ready=True)
            models.append(model)
            return model
        points, document = open_map()
        planner = Planner(model_factory=factory)
        planner.set_reference(points, [1.] * len(points), False, document, 'main', 'r1')
        first = planner.update(0., (0., 0., 0., .8), [target(0., x=-2., speed=.4, kind='rc_car')])
        self.assertEqual(first['state'], 'CLEAR')
        historical = planner.models['target']
        planner.update(.5, (.4, 0., 0., .8), [])
        self.assertNotIn('target', planner.known)
        self.assertIs(planner.models['target'], historical)
        planner.update(.6, (.48, 0., 0., .8), [target(.6, x=-1.76, speed=.4, kind='rc_car')])
        self.assertEqual(len(models), 1)
        self.assertIs(planner.models['target'], historical)
        self.assertEqual(len(historical.observations), 2)

    def test_gp_cache_evicts_old_inactive_models_while_retaining_committed_target(self):
        models = []
        def factory(_length):
            model = Model(available=False)
            models.append(model)
            return model
        points, document = open_map()
        planner = Planner(model_factory=factory)
        planner.set_reference(points, [1.] * len(points), False, document, 'main', 'r1')
        for index in range(planner.MAX_OBJECTS + 2):
            now = index * .01
            planner.update(now, (0., 0., 0., .8),
                           [target(now, identity=f'old-{index}', x=-3., speed=.4, kind='rc_car')])
        self.assertLessEqual(len(planner.models), planner.MAX_OBJECTS)
        self.assertNotIn('old-0', planner.models)
        self.assertIn(f'old-{planner.MAX_OBJECTS+1}', planner.models)
        start = 2.
        for step in (0., .2, .4, .6):
            first = planner.update(start+step, (.8*step, 0., 0., .8),
                                   [target(start+step, x=2+.4*step, speed=.4, kind='rc_car')])
        self.assertEqual(first['state'], 'PASS_MOVING', summary(first))
        committed = planner.models['target']
        for index in range(planner.MAX_OBJECTS + 2):
            elapsed = (index+1)*.01
            now = start+.6+elapsed
            pose = point_at_distance(first['path'], .8*elapsed)
            result = planner.update(now, pose,
                [target(now, x=2.24+.4*elapsed, speed=.4, kind='rc_car'),
                 target(now, identity=f'new-{index}', x=-3., speed=.4, kind='rc_car')])
            self.assertNotEqual(result['state'], 'FAULT', summary(result))
            self.assertLessEqual(len(planner.models), planner.MAX_OBJECTS)
            self.assertIs(planner.models['target'], committed)

    def test_simple_moving_rc_fov_loss_coast_merge_and_clear_to_completion(self):
        points = [[float(x), 0.] for x in range(-5, 81)]
        zone = dict(id='long-straight', start_index=0, end_index=len(points)-1, side='both',
                    geometry_revision=geometry_revision(points, False))
        document = dict(schema='kart.hdmap.visualization.v1', frame_id='map', revision=1, obstacles=[],
            lanes=[dict(id='main', closed=False, left=[[-10., 3.], [100., 3.]],
                        right=[[-10., -3.], [100., -3.]],
                        lines=dict(centerline=dict(points=points, passing_zones=[zone])))])
        model = Model(available=False)
        planner = Planner(Settings(speed_cap=3., lookahead_m=20.), model_factory=lambda _length: model)
        planner.set_reference(points, [3.] * len(points), False, document, 'main', 'e2e1')
        ego, result = (0., 0., 0., 0.), None
        lost, loss_samples, loss_at = False, None, None
        states = []
        merged = completed = False
        for index in range(600):
            now = index * .05
            opponent_x = 5. + 1.2 * now
            if result is not None:
                ego = ideal_step(result, ego, .05)
            if planner.active is not None and opponent_x - ego[0] < .6:
                if not lost:
                    loss_at, loss_samples = now, len(model.observations)
                lost = True
            observations = [] if lost else [target(now, x=opponent_x, speed=1.2, kind='rc_car', vx=1.2, vy=0.)]
            result = planner.update(now, ego, observations)
            states.append(result['state'])
            diagnostic = dict(t=now, ego=ego, opponent_x=opponent_x, lost=lost,
                              loss_at=loss_at, result=summary(result), observation=result.get('target_observation'))
            self.assertNotIn(result['state'], ('FAULT', 'BLOCKED'), diagnostic)
            self.assertLessEqual(len(result['path']), 300)
            self.assertLessEqual(max(result['speeds']), 3.)
            if lost:
                self.assertEqual(len(model.observations), loss_samples)
            if result['state'] == 'MERGE' and not merged:
                required = planner.settings.rear_axle_to_rear + planner.settings.rc_length/2 + planner.settings.uncertainty_margin + planner.settings.static_merge_margin
                self.assertGreaterEqual(ego[0] - opponent_x, required - 1e-3, diagnostic)
                merged = True
            if merged and result['state'] == 'CLEAR':
                completed = True
                break
        self.assertTrue(lost, dict(states=states[-10:], ego=ego))
        self.assertTrue(any(state in ('COAST_PASS', 'HOLD_SIDE') for state in states), states)
        self.assertTrue(merged, states)
        self.assertTrue(completed, dict(states=states[-10:], ego=ego, result=summary(result)))
        self.assertIsNone(planner.active)
        self.assertLess(abs(ego[1]), .1)
        self.assertTrue(all(abs(point[1]) < 1e-9 for point in result['path']))

    def test_revoking_permission_holds_adopted_route_with_zero_speed(self):
        planner, first = self.simple_pass()
        points, document = open_map(zones=False)
        planner.set_reference(points, [5.] * len(points), False, document, 'main', 'revision1')
        result = planner.update(.7, (.5, 0., 0., .8), [])
        self.assertEqual(result['state'], 'FAULT')
        self.assert_stopped(result)
        self.assertEqual(result['path'], first['path'])

    def test_short_static_merge_begins_only_after_real_rear_clearance(self):
        planner = self.planner()
        first = planner.update(0., (0., 0., 0., .8), [target(0.)])
        self.assertEqual(first['state'], 'AVOID_OBSTACLE')
        merged = None
        for index in range(1, 45):
            distance = index * .15
            elapsed = distance / .8
            pose = point_at_distance(first['path'], distance)
            result = planner.update(elapsed, pose, [target(elapsed)])
            if result['state'] == 'MERGE':
                merged = result
                required = planner.settings.rear_axle_to_rear + .5/2 + planner.settings.uncertainty_margin + planner.settings.static_merge_margin
                self.assertGreaterEqual(pose[0] - 3., required - 1e-6)
                self.assertGreater(result['path'][0][1], .3)
                self.assertAlmostEqual(result['path'][-1][1], 0.)
                break
            self.assertEqual(result['state'], 'AVOID_OBSTACLE', summary(result))
        self.assertIsNotNone(merged, 'A short static merge must be possible once actual clearance is reached')


class NativeBehaviorTests(unittest.TestCase):
    def test_actual_native_gp_profile_survives_gap_and_enters_fast_planner(self):
        model = OpponentModel(35.)
        self.addCleanup(model.close)
        if not model.available and not os.environ.get(LIBRARY_ENV):
            self.skipTest('Install native GP or set ' + LIBRARY_ENV)
        self.assertTrue(model.available, model.reason)
        identity = int.from_bytes(hashlib.sha256(b'target').digest()[:8], 'big')
        # Actual C++ GP, one measured course traversal; phase ends at x=4 (s=9).
        for index in range(351):
            self.assertTrue(model.observe(identity, index * .05, 9. - 36.75 + index * .105,
                                          0., 2.1, .0001, .0001))
        self.assertTrue(model.ready())
        queries = []
        predict = model.predict
        def counted(station):
            queries.append(station)
            return predict(station)
        model.predict = counted
        helper = PermissionTests()
        planner, result = helper.fast_pass(model)
        self.assertTrue(model.ready(), 'Same-identity long gap must retain learned profile')
        self.assertTrue(queries)
        self.assertEqual(result['state'], 'PASS_MOVING', summary(result))
        self.assertTrue(planner.known['target']['model_ready'])


if __name__ == '__main__':
    unittest.main()
