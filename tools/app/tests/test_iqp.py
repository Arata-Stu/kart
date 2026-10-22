"""IQP monitoring without native numerical dependencies."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from kart_studio.iqp import monitored_iqp


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

    def test_more_than_twenty_iterations_are_allowed(self):
        tph, solve = self.helper([.5] * 25 + [.005])
        result = monitored_iqp(tph, dict(curv_error_allowed=.01))
        self.assertEqual(result[1], .005)
        self.assertEqual(solve.call_count, 26)
        self.assertIs(tph.opt_min_curv.opt_min_curv, solve)

    def test_convergence_and_history(self):
        tph, solve = self.helper([.5, .1, .005])
        history = []
        result = monitored_iqp(tph, dict(curv_error_allowed=.01), history=history)
        self.assertEqual(result[1], .005)
        self.assertEqual(len(history[0]), 3)
        self.assertIs(tph.opt_min_curv.opt_min_curv, solve)

    def test_native_error_restores_solver(self):
        tph, solve = self.helper([])
        solve.side_effect = ValueError("constraints are inconsistent")
        with self.assertRaisesRegex(ValueError, "constraints are inconsistent"):
            monitored_iqp(tph, dict(curv_error_allowed=.01))
        self.assertIs(tph.opt_min_curv.opt_min_curv, solve)
