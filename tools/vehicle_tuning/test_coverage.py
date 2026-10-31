"""Distribution evidence remains independent of constant-input fitting."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from data_coverage import diagnose
from analysis import analyze, validate_config
from test_tuning import CONFIG, synthetic
from report import analysis_report
from cli import main


class CoverageTests(unittest.TestCase):
    def rows(self, values, dt=.1):
        return [dict(t=i*dt, throttle=u, brake=0, mode=2, reason='', speed_mps=1.) for i,u in enumerate(values)]

    def test_ramps_are_counted_without_plateaus(self):
        rows = self.rows([.1+i*.005 for i in range(50)])
        result = diagnose(rows, [], CONFIG)
        self.assertEqual(result['valid_samples'], 50)
        self.assertGreater(result['transitions']['increase'], 3)
        self.assertEqual(result['transitions']['decrease'], 0)
        self.assertGreater(sum(b['seconds'] for b in result['throttle_bins']), 0)

    def test_gap_and_invalid_data_do_not_create_transitions(self):
        rows = self.rows([.1,.1,.9,.9])
        rows[2]['t']=5; rows[3]['t']=5.1
        result = diagnose(rows, [], CONFIG)
        self.assertEqual(result['transitions']['increase'], 0)
        self.assertLess(sum(b['seconds'] for b in result['throttle_bins']), .21)
        rows[2]['reason']='bridge_not_ready'; rows[3]['reason']='steering'
        result = diagnose(rows, [], CONFIG)
        self.assertEqual(result['observed_max_throttle'], .1)

    def test_target_does_not_invent_observations(self):
        result = diagnose(self.rows([.2]*30), [], dict(CONFIG, coverage_max_throttle=.5))
        self.assertEqual(result['observed_max_throttle'], .2)
        self.assertEqual(result['throttle_bins'][-1]['status'], 'unobserved')
        self.assertTrue(any(r['code']=='throttle_unobserved' for r in result['recommendations']))
        default = diagnose(self.rows([.2]*30), [], CONFIG)
        self.assertIsNone(default['requested_max_throttle'])
        self.assertLess(len(default['throttle_bins']), len(result['throttle_bins']))

    def test_sample_frequency_does_not_replace_repetitions(self):
        result = diagnose(self.rows([.2]*1000, .01), [], CONFIG)
        bucket = next(b for b in result['throttle_bins'] if b['throttle_min']==.2)
        self.assertEqual(bucket['visits_over_0_3s'], 1)
        self.assertEqual(bucket['status'], 'sparse')

    def test_empty_and_braking_data(self):
        rows = self.rows([.5]*10)
        for row in rows: row['brake']=.1
        result = diagnose(rows, [], CONFIG)
        self.assertEqual(result['valid_samples'], 0)
        self.assertIsNone(result['observed_max_throttle'])
        self.assertEqual(result['throttle_bins'], [])
        self.assertTrue(any(r['code']=='no_valid_data' for r in result['recommendations']))

    def test_config_and_artifacts(self):
        for value in (0, -1, 1.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                validate_config(dict(CONFIG, coverage_max_throttle=value))
        data = synthetic()
        original = copy.deepcopy(data)
        summary, rows, segments, bins = analyze(data, CONFIG)
        self.assertGreater(summary['coverage']['valid_samples'], summary['selected_samples'])
        self.assertEqual(data, original)
        with tempfile.TemporaryDirectory() as tmp:
            analysis_report(Path(tmp), summary, rows, segments, bins)
            saved = json.loads((Path(tmp)/'coverage.json').read_text())
            self.assertEqual(saved, summary['coverage'])
            self.assertIn('データ分布', (Path(tmp)/'report.html').read_text())
            self.assertTrue((Path(tmp)/'coverage_speed_throttle.csv').is_file())

    def test_cli_reports_ramp_coverage_even_without_fit_segments(self):
        data = synthetic()
        for i, command in enumerate(data['command']):
            command['throttle'] = .1 + (i % 30) * .01
            command['brake'] = 0.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root/'input.json'
            source.write_text(json.dumps(data))
            output = root/'out'
            result = main(['analyze', '--normalized-json', str(source), '--output', str(output),
                           '--coverage-max-throttle', '.6'])
            self.assertEqual(result, 2)
            summary = json.loads((output/'summary.json').read_text())
            self.assertEqual(summary['selected_samples'], 0)
            self.assertGreater(summary['coverage']['valid_samples'], 0)
            self.assertEqual(summary['coverage']['requested_max_throttle'], .6)
            self.assertTrue((output/'coverage.json').is_file())
