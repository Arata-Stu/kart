import csv
import runpy
from pathlib import Path
import tempfile
import unittest

REPORT = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/e2e_live_latency_report'))['summarize']

class ReportTest(unittest.TestCase):
    def test_first_only_and_invalid(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'run.csv'
            with path.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['active','first_output','source_age_ms','prediction_to_output_ms','log_dropped_total'])
                writer.writerows([[1,1,10,2,0],[1,0,999,999,0],[0,0,'','',0],[1,1,'',4,3]])
            result = REPORT(path)
            self.assertEqual(result['first_output_rows'], 2)
            self.assertEqual(result['repeated_active_rows'], 1)
            self.assertEqual(result['source_age_ms']['count'], 1)
            self.assertEqual(result['source_age_ms']['p99'], 10)
            self.assertEqual(result['prediction_to_output_ms']['mean'], 3)
            self.assertEqual(result['log_dropped_observed'], 3)
    def test_empty(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'run.csv'
            path.write_text('active,first_output,source_age_ms,prediction_to_output_ms,log_dropped_total\n')
            result=REPORT(path)
            self.assertIsNone(result['source_age_ms']['p50'])
            self.assertEqual(result['first_output_rows'],0)

if __name__ == '__main__':
    unittest.main()
