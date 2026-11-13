"""Real C++ GP through an in-process ABI, with no ROS / Python GP replacement."""
from concurrent.futures import ThreadPoolExecutor
import ctypes
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from kart_racing.opponent_model import LIBRARY_ENV, OpponentModel, _Prediction


class OpponentModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = Path(__file__).resolve().parents[1]
        configured = os.environ.get(LIBRARY_ENV)
        cls.temporary = tempfile.TemporaryDirectory(prefix='kart-opponent-native-')
        cls.addClassCleanup(cls.temporary.cleanup)
        if configured:
            cls.library = Path(configured).resolve(strict=True)
        else:
            compiler = shutil.which('c++')
            if compiler is None:
                raise RuntimeError('Portable native GP tests require a C++17 compiler')
            cls.library = Path(cls.temporary.name) / ('libkart_racing_opponent_model.dylib' if sys.platform == 'darwin'
                                                     else 'libkart_racing_opponent_model.so')
            subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-fPIC',
                            '-dynamiclib' if sys.platform == 'darwin' else '-shared',
                            '-I', str(cls.package / 'include'), str(cls.package / 'src/opponent_model_c.cpp'),
                            '-o', str(cls.library)], check=True, capture_output=True, text=True)

    def model(self, length=40.):
        model = OpponentModel(length, self.library)
        self.addCleanup(model.close)
        self.assertTrue(model.available, model.reason)
        return model

    def train(self, model, identity=1):
        for index in range(421):
            self.assertTrue(model.observe(identity, index * .05, index * .1, .2, 2., .001, .01))
        self.assertTrue(model.ready())
        self.assertGreaterEqual(model.coverage(), .9)

    def test_real_gp_ready_periodic_mean_and_uncertainty(self):
        model = self.model()
        self.assertFalse(model.ready())
        self.assertEqual(model.coverage(), 0.)
        self.assertFalse(model.predict(0.)['valid'])
        self.train(model)
        for station in (39.9, .1, 40.1, -39.9):
            prediction = model.predict(station)
            self.assertTrue(prediction['valid'])
            self.assertAlmostEqual(prediction['d'], .2, delta=.03)
            self.assertAlmostEqual(prediction['v'], 2., delta=.03)
            self.assertGreaterEqual(prediction['d_sigma'], .05)
            self.assertGreaterEqual(prediction['v_sigma'], .05)

    def test_mature_profile_survives_long_gap_without_synthetic_observations(self):
        model = self.model()
        self.train(model)
        coverage = model.coverage()
        before = model.predict(5.)
        # Arbitrarily many reads do not insert synthetic measurements or change
        # sigma; elapsed-time uncertainty belongs to the live-state estimator.
        for _ in range(20):
            self.assertEqual(model.predict(5.), before)
        self.assertTrue(model.observe(1, 23., 46., .2, 2., .001, .01))
        self.assertTrue(model.ready())
        self.assertEqual(model.coverage(), coverage)

    def test_immature_gap_keeps_bins_without_counting_unseen_lap_progress(self):
        model = self.model()
        for index in range(101):
            self.assertTrue(model.observe(1, index * .05, index * .1, .2, 2., .001, .01))
        coverage = model.coverage()
        self.assertFalse(model.ready())
        self.assertTrue(model.observe(1, 25., 50., .2, 2., .001, .01))
        self.assertGreaterEqual(model.coverage(), coverage)
        self.assertFalse(model.ready(), 'Unobserved progress must not mature a partial model')

    def test_identity_clock_progress_and_profile_change_invalidate_memory(self):
        for change in ('identity', 'clock', 'progress', 'profile'):
            model = self.model()
            self.train(model)
            if change == 'identity':
                self.assertTrue(model.observe(2, 21.1, 42.2, .2, 2., .001, .01))
            elif change == 'clock':
                self.assertFalse(model.observe(1, 20., 42.2, .2, 2., .001, .01))
            elif change == 'progress':
                self.assertTrue(model.observe(1, 21.1, 1000., .2, 2., .001, .01))
            else:
                self.assertTrue(model.observe(1, 21.1, 42.2, 2., 2., .001, .01))
            with self.subTest(change=change):
                self.assertFalse(model.ready())
                self.assertLess(model.coverage(), .1)

    def test_clear_and_close_are_explicit_idempotent_and_fail_closed(self):
        model = self.model()
        self.train(model)
        self.assertTrue(model.clear())
        self.assertFalse(model.ready())
        self.assertEqual(model.coverage(), 0.)
        model.close()
        model.close()
        self.assertFalse(model.available)
        self.assertFalse(model.observe(1, 0., 0., 0., 0., 0., 0.))
        self.assertFalse(model.ready())
        self.assertFalse(model.predict(0.)['valid'])
        self.assertFalse(model.clear())

    def test_environment_library_override_and_unavailable_library(self):
        with patch.dict(os.environ, {LIBRARY_ENV: str(self.library)}):
            model = OpponentModel(40.)
            self.addCleanup(model.close)
            self.assertTrue(model.available, model.reason)
        with patch.dict(os.environ, {LIBRARY_ENV: str(self.library)}):
            missing = OpponentModel(40., '/definitely-missing/kart-gp.so')
            self.addCleanup(missing.close)
            self.assertFalse(missing.available, 'Explicit missing path cannot fall back to the env library')
        self.assertEqual(missing.reason, 'native_library_unavailable')
        self.assertNotIn('/definitely-missing', missing.reason)
        self.assertIn('definitely-missing', missing.detail)
        self.assertFalse(missing.ready())
        self.assertEqual(missing.coverage(), 0.)
        self.assertFalse(missing.predict(3.)['valid'])
        self.assertFalse(missing.observe(1, 0., 0., 0., 0., 0., 0.))

    def test_invalid_input_and_u64_mapping_are_bounded(self):
        model = self.model()
        for identity in (-1, 2**64, True, 'car-id'):
            self.assertFalse(model.observe(identity, 0., 0., 0., 0., 0., 0.))
        self.assertTrue(model.observe(2**64 - 1, 0., 0., 0., 0., 0., 0.))
        for values in ((1., 0., math.nan, 2., 0., 0.), (1., 0., 0., -1., 0., 0.),
                       (1., 0., 0., 2., -.1, 0.), (1., 0., 0., 2., 0., 5.)):
            self.assertFalse(model.observe(2**64 - 1, *values))
        self.assertFalse(model.predict(math.nan)['valid'])
        for length in (True, 2., 20001., math.inf):
            with self.assertRaises(ValueError):
                OpponentModel(length, self.library)

    def test_abi_handles_nulls_without_exception_or_uninitialised_output(self):
        model = self.model()
        library = model._library
        prediction = _Prediction(5., 5., 5., 5., 1)
        self.assertEqual(library.kart_opponent_model_predict(None, 0., ctypes.byref(prediction)), -1)
        self.assertEqual((prediction.d, prediction.v, prediction.d_sigma, prediction.v_sigma, prediction.valid),
                         (0., 0., 1., 1., 0))
        self.assertEqual(library.kart_opponent_model_ready(None), 0)
        self.assertEqual(library.kart_opponent_model_clear(None), 0)
        self.assertEqual(library.kart_opponent_model_create(0.), None)
        library.kart_opponent_model_destroy(None)

    def test_queries_are_in_process_and_thread_serialized(self):
        model = self.model()
        self.train(model)
        with patch('subprocess.run', side_effect=AssertionError('No runtime subprocess permitted')):
            with ThreadPoolExecutor(max_workers=4) as executor:
                predictions = list(executor.map(model.predict, [index * .1 for index in range(200)]))
            self.assertTrue(all(value['valid'] for value in predictions))
            self.assertTrue(model.ready())

    def test_adapter_import_and_use_need_no_ros_or_numpy(self):
        code = ('import sys; from kart_racing.opponent_model import OpponentModel; '
                f'm=OpponentModel(40., {str(self.library)!r}); assert m.available; '
                'assert not any(x in sys.modules for x in ("rclpy","numpy","torch")); m.close()')
        subprocess.run([sys.executable, '-c', code], check=True)


if __name__ == '__main__':
    unittest.main()
