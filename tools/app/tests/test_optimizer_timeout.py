"""Execution budgets are separate from the numerical convergence criteria."""
import os
import unittest
from unittest.mock import Mock, patch

from kart_studio.optimizer import optimize


class TimeoutTests(unittest.TestCase):
    def process(self):
        process = Mock()
        process.poll.return_value = None
        return process

    def test_default_allows_long_compute_but_cancellation_still_works(self):
        process = self.process()
        check = Mock(side_effect=[None, RuntimeError("cancelled")])
        with patch.dict(os.environ, {}, clear=True), \
             patch("kart_studio.optimizer.subprocess.Popen", return_value=process), \
             patch("kart_studio.optimizer.time.monotonic", return_value=10**9), \
             patch("kart_studio.optimizer.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "cancelled"):
                optimize(dict(closed=True), {}, check)
        self.assertEqual(check.call_count, 2)
        process.terminate.assert_called_once()
        process.wait.assert_called_once()

    def test_explicit_budget_reports_interruption(self):
        process = self.process()
        with patch.dict(os.environ, {"KART_RACELINE_TIMEOUT_SECONDS": "1800"}), \
             patch("kart_studio.optimizer.subprocess.Popen", return_value=process), \
             patch("kart_studio.optimizer.time.monotonic", side_effect=[0, 1801]):
            with self.assertRaisesRegex(ValueError, "時間上限（1800秒）に達したため中断"):
                optimize(dict(closed=True), {}, Mock())
        process.terminate.assert_called_once()

    def test_invalid_budgets_do_not_start_process(self):
        for value in ("-1", "nan", "inf", "invalid"):
            with self.subTest(value=value), \
                 patch.dict(os.environ, {"KART_RACELINE_TIMEOUT_SECONDS": value}), \
                 patch("kart_studio.optimizer.subprocess.Popen") as start:
                with self.assertRaisesRegex(ValueError, "0以上の有限"):
                    optimize(dict(closed=True), {}, Mock())
                start.assert_not_called()
