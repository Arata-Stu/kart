"""IQP convergence guards without native numerical dependencies."""
import unittest
import os
from types import SimpleNamespace
from unittest.mock import Mock

from kart_studio.iqp import bounded_iqp


class IqpTests(unittest.TestCase):
    def helper(self, errors):
        solve = Mock(side_effect=[([0], error) for error in errors])
        tph = SimpleNamespace(opt_min_curv=SimpleNamespace(opt_min_curv=solve))

        def handler(**args):
            while True:
                result = tph.opt_min_curv.opt_min_curv(reftrack=[[0, 0, 1, 1]])
                if result[1] <= args["curv_error_allowed"]:
                    return result

        tph.iqp_handler = SimpleNamespace(iqp_handler=handler)
        return tph, solve

    def test_nonconvergence_is_bounded_and_restores_solver(self):
        tph, solve = self.helper([.5] * 3)
        with self.assertRaisesRegex(ValueError, "3回で収束しません") as caught:
            bounded_iqp(tph, dict(curv_error_allowed=.01), max_iterations=3)
        self.assertEqual(solve.call_count, 3)
        self.assertIs(tph.opt_min_curv.opt_min_curv, solve)
        self.assertEqual(caught.exception.diagnostic["reason"], "iqp_nonconvergence")

    def test_convergence_and_history(self):
        tph, solve = self.helper([.5, .1, .005])
        history = []
        result = bounded_iqp(tph, dict(curv_error_allowed=.01), history=history)
        self.assertEqual(result[1], .005)
        self.assertEqual(len(history[0]), 3)
        self.assertIs(tph.opt_min_curv.opt_min_curv, solve)

    def test_native_error_restores_solver(self):
        tph, solve = self.helper([])
        solve.side_effect = ValueError("constraints are inconsistent")
        with self.assertRaisesRegex(ValueError, "constraints are inconsistent"):
            bounded_iqp(tph, dict(curv_error_allowed=.01))
        self.assertIs(tph.opt_min_curv.opt_min_curv, solve)

    @unittest.skipUnless(os.environ.get("KART_TEST_HELPER"), "Requires installed helper")
    def test_real_helper_nonconvergence_is_reported(self):
        import json
        from pathlib import Path
        from kart_studio.lines import generate, settings
        from kart_studio.optimizer_worker import run

        doc = json.loads((Path(__file__).parent / "fixtures/e522_boundaries.json").read_text())
        doc["lines"] = {"centerline": generate(doc, "centerline", dict(spacing=.1))}
        with self.assertRaisesRegex(ValueError, "20回で収束しません") as caught:
            run(doc, settings(dict(optimizer="mincurv_iqp", curvature_limit=2.5)))
        self.assertEqual(len(caught.exception.diagnostic["iterations"]), 20)
