import unittest
from unittest.mock import Mock
from kart_studio.transfer_progress import run_transfer


class TransferProgressTests(unittest.TestCase):
    def test_success_reports_payload_and_verification_phase(self):
        job = Mock()
        run_transfer(job, ["scp", "source", "destination"], 1048576, lambda: 0)
        self.assertIn("0.0%", job.log.call_args_list[0].args[0])
        self.assertIn("100.0%", job.log.call_args_list[-1].args[0])
        self.assertIn("整合性確認中", job.log.call_args_list[-1].args[0])
        self.assertIn("MiB/s", job.log.call_args_list[-1].args[0])

    def test_failure_does_not_report_complete(self):
        job = Mock()
        job.run.side_effect = ValueError("scp failed")
        with self.assertRaisesRegex(ValueError, "scp failed"):
            run_transfer(job, [], 100, lambda: 20)
        self.assertEqual(job.log.call_count, 1)

    def test_live_sampling_reports_partial_progress(self):
        import time

        job = Mock()
        job.run.side_effect = lambda args: time.sleep(2.1)
        run_transfer(job, [], 1000, lambda: 400)
        self.assertTrue(any("40.0%" in call.args[0] for call in job.log.call_args_list))
