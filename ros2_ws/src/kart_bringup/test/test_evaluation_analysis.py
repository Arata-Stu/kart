"""Portable numerical checks; these do not validate ROS recording."""
from pathlib import Path
import math
import json
from unittest.mock import patch
from types import SimpleNamespace
from kart_bringup import evaluation_analysis as analysis
import tempfile
import unittest

from kart_bringup.evaluation_analysis import compare_reference, reference_rows, summarize, SLAM
from kart_bringup.evaluation_recording import recording_plan, start_recording

CONFIG = Path(__file__).parents[1] / 'config'


def pose(t, x=0., yaw=0.):
    return dict(t=t,receipt=t+1000,parent='map',child='camera_link',x=x,y=0.,z=0.,yaw=yaw)


class EvaluationTests(unittest.TestCase):
    def test_recording_is_opt_in_and_never_overwrites(self):
        self.assertIsNone(recording_plan(CONFIG))
        with self.assertRaises(ValueError): recording_plan(CONFIG, 'false', '/tmp/result')
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(ValueError): recording_plan(CONFIG,'true',root)
            output = str(Path(root)/'new')
            plan = recording_plan(CONFIG,'true',output)
            self.assertFalse(Path(output).exists())
            self.assertIn('/visual_slam/vis/slam_odometry',plan['topics'])
            self.assertNotIn('/realsense/infra1/image_rect_raw',plan['topics'])

    def test_lost_durations_do_not_bridge_missing_diagnostics(self):
        rows=[dict(t=t,vo=v) for t,v in [(1,'OK'),(1.1,'Lost'),(1.2,'Lost'),(2,'Lost'),(2.1,'OK')]]
        report=summarize({SLAM:[pose(1),pose(1.1),pose(2)]},rows,.25)
        self.assertEqual(report['tracking']['observed_lost_runs'],2)
        self.assertAlmostEqual(report['tracking']['observed_lost_s'],.2)
        self.assertAlmostEqual(report['tracking']['unclassified_interval_s'],.8)
        self.assertEqual(report['poses'][SLAM]['header_gap_count'],1)
        self.assertIsNone(summarize({SLAM:[]},[],.25)['tracking']['trailing_state'])

    def test_reference_interpolation_wraparound_and_coverage(self):
        refs=[pose(1,0,math.radians(179)),pose(1.1,2,math.radians(-179))]
        result, rows=compare_reference([pose(1.05,1,math.pi),pose(2)],refs,'map','camera_link',.2)
        self.assertEqual(result['matched'],1)
        self.assertEqual(result['unmatched'],1)
        self.assertAlmostEqual(rows[0]['xy_m'],0)
        self.assertAlmostEqual(rows[0]['yaw_deg'],0)
        result,_=compare_reference([pose(1.05)],refs,'map','camera_link',.05)
        self.assertEqual(result['matched'],0)
        with self.assertRaises(ValueError): compare_reference([pose(1)],refs,'map','base_link',.2)
        with self.assertRaises(ValueError): compare_reference([pose(1.1),pose(1)],refs,'map','camera_link',.2)

    def test_recorder_callbacks_capture_native_error_and_stop_on_exit(self):
        factory=lambda **kwargs: SimpleNamespace(**kwargs)
        with tempfile.TemporaryDirectory() as directory, patch.dict('sys.modules', {
            'launch.actions':SimpleNamespace(ExecuteProcess=factory, RegisterEventHandler=lambda handler:handler, EmitEvent=factory),
            'launch.event_handlers':SimpleNamespace(OnProcessExit=factory, OnProcessIO=factory),
            'launch.events':SimpleNamespace(Shutdown=factory),
        }):
            root=Path(directory)/'run'
            plan=recording_plan(CONFIG,'true',str(root))
            io, exit_handler, recorder=start_recording(CONFIG,plan,{'source_bag':'/original'})
            io.on_stderr(SimpleNamespace(text=b'Failed to reindex pose graph'))
            self.assertIn('reindex',(root/'process_output.log').read_text())
            self.assertEqual(len(exit_handler.on_exit(SimpleNamespace(returncode=1),SimpleNamespace(is_shutdown=False))),1)
            self.assertEqual(exit_handler.on_exit(SimpleNamespace(returncode=0),SimpleNamespace(is_shutdown=True)),[])
            self.assertNotIn('--use-sim-time',recorder.cmd)
            self.assertEqual(json.loads((root/'manifest.json').read_text())['source_bag'],'/original')
            self.assertTrue((root/'config/localization/cuvslam.yaml').exists())

    def test_report_exports_metrics_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'bag').mkdir()
            (root/'bag/metadata.yaml').write_text('test fixture')
            (root/'process_output.log').write_text('Failed to reindex pose graph')
            data=({SLAM:[pose(1),pose(1.1)],analysis.VO:[]},[],[],[],{SLAM:2})
            with patch.object(analysis,'read_bag',return_value=data), patch('sys.argv',['analyze',str(root)]), patch('builtins.print'):
                analysis.main()
                with self.assertRaises(SystemExit): analysis.main()
            report=json.loads((root/'analysis/summary.json').read_text())
            self.assertEqual(report['native_reindex_error_count'],1)
            self.assertIsNone(report['tracking']['trailing_state'])
            self.assertTrue(report['warnings'])
            self.assertIn('plotly.js',(root/'analysis/report.html').read_text())
            self.assertEqual(len(json.loads((root/'analysis/samples.json').read_text())['poses'][SLAM]),2)

    def test_reference_rejects_nonfinite_and_repeated_stamps(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'pose.tum'
            for text in ('1 nan 0 0 0 0 0 1\n','1 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n'):
                path.write_text(text)
                with self.assertRaises(ValueError): reference_rows(path)


if __name__ == '__main__': unittest.main()
