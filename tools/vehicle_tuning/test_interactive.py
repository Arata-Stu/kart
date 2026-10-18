import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('interactive', Path(__file__).resolve().parents[2] / 'scripts/lib/vehicle_tuning_control.py')
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


class InteractiveTest(unittest.TestCase):
    def test_complete_atomic_condition(self):
        current = dict(fixed_axes='both', steering=0., throttle=.1, brake=0., duration_s=0., max_throttle=.2)
        for choice, axes, answers in [('1','throttle',['.15','0','3']), ('2','steering',['-.2','2']), ('3','both',['.1','.12','0','4'])]:
            values = control.trial_values(current, choice, lambda _: answers.pop(0))
            self.assertEqual(values['fixed_axes'], axes)
            self.assertEqual(set(values), set(control.FIELDS))
            self.assertEqual(current['throttle'], .1)

    def test_invalid_input_never_sent(self):
        current = dict(fixed_axes='both', steering=0., throttle=.1, brake=0., duration_s=0., max_throttle=.2)
        for throttle in ['nan', 'inf', '.3', '-.1']:
            answers = [throttle, '0', '0']
            with self.assertRaises(ValueError):
                control.trial_values(current, '1', lambda _: answers.pop(0))
        answers = ['.1', '.1', '0']
        with self.assertRaises(ValueError):
            control.trial_values(current, '1', lambda _: answers.pop(0))
