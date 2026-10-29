"""ESC trial CLI/TUI routing without launching ROS or connecting to a vehicle."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
from unittest.mock import patch
import pytest

spec = importlib.util.spec_from_file_location('esc_bringup', Path(__file__).parents[1]/'lib/bringup.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def run(*args):
    out=io.StringIO()
    with patch.object(sys,'argv',['bringup.py',*args]),contextlib.redirect_stdout(out):
        module.main()
    return out.getvalue()


def test_dry_run_needs_no_map_camera_or_ros():
    with patch.object(module.os,'execvp',side_effect=AssertionError('must not launch')):
        result=run('--mode','esc-trial','--dry-run','--device','/dev/test device')
    assert "esc_trial.launch.py 'device:=/dev/test device'" in result
    assert 'mission.launch.py' not in result


def test_tui_only_selects_purpose():
    prompts=[]
    def choose(title,values,label=str,default=0):
        prompts.append(title)
        assert title=='用途'
        assert 'Joy不要' in label('esc-trial')
        return 'esc-trial'
    with patch.object(sys.stdin,'isatty',return_value=True),patch.object(module,'choose',side_effect=choose):
        assert 'esc_trial.launch.py' in run('--dry-run')
    assert prompts==['用途']


@pytest.mark.parametrize('options', [('--no-bridge',),('--map-dir','/map'),('--infra-profile','0'),
                                     ('--tuning',),('--evs',),('--auto-fixed-throttle','0.1'),('--bag','/bag')])
def test_unrelated_options_rejected(options):
    with pytest.raises(ValueError):run('--mode','esc-trial','--dry-run',*options)


def test_cli_executes_only_dedicated_launch():
    with patch.object(module.shutil,'which',return_value='/usr/bin/ros2'),patch.object(module.os,'execvp') as execute:
        run('--mode','esc-trial')
    execute.assert_called_once_with('ros2',['ros2','launch','kart_bringup','esc_trial.launch.py'])
