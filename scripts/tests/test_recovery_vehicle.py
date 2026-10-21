"""Portable Python recovery -> C++ HostSession/ESC sequence check; no ROS/ESC."""
from pathlib import Path
import shutil
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/kart_recovery'))
from kart_recovery.core import Command, Observation, Recovery


def test_recovery_brake_demand_reaches_real_vehicle_sequence(tmp_path):
    compiler=shutil.which('c++')
    if compiler is None:
        pytest.skip('C++ compiler unavailable')
    probe=tmp_path/'vehicle_stream'
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Wpedantic',
                    '-I'+str(ROOT/'ros2_ws/src/kart_vehicle/include'),
                    '-I'+str(ROOT/'ros2_ws/src/kart_system/include'),
                    str(ROOT/'scripts/tests/helpers/vehicle_command_stream.cpp'),'-o',str(probe)],
                   check=True,capture_output=True,text=True)
    recovery=Recovery()
    now=20.
    recovery.step(now,3,Command(),Observation())
    records=[]
    brake_ticks=0
    seen_active=False
    for _ in range(1000):
        now+=.02
        moving=recovery.state=='reversing' or (recovery.state=='post_reverse_braking' and brake_ticks<20)
        if recovery.state=='post_reverse_braking':brake_ticks+=1
        output=recovery.step(now,1,Command(steering=.3,throttle=.1),Observation(True,15 if moving else 0,0))
        assert recovery.state!='fault', recovery.reason
        # ROS producer 50 Hz, actual vehicle output 100 Hz with held latest demand.
        for delta in (0.,.01):records.append((now+delta,recovery.state,output))
        seen_active |= recovery.active
        if seen_active and recovery.state=='driving':break
    else:pytest.fail('Recovery did not finish')
    payload=''.join(f'{t:.9f} {c.steering} {c.throttle} {c.brake} {c.reverse}\n' for t,_,c in records)
    result=subprocess.run([str(probe)],input=payload,text=True,capture_output=True,check=True)
    frames=[line.split() for line in result.stdout.splitlines()]
    assert len(frames)==len(records)
    indices=[i for i,(_,state,_) in enumerate(records) if state=='post_reverse_braking']
    first=indices[0]
    assert records[first-1][2].reverse>0  # No intervening neutral from recovery.
    assert int(frames[first-1][1])>0
    assert all(records[i][2].brake==recovery.p.brake_command for i in indices)
    phases=[frames[i][3] for i in indices]
    assert phases[0]=='brake_entry'
    assert set(phases)=={'brake_entry','braking'}
    switched=False
    for i in indices:
        forward,reverse,brake,phase,flags=frames[i]
        assert int(flags)==3 and int(reverse)==0
        if phase=='brake_entry':
            assert not switched and int(forward)==80 and int(brake)==0
        else:
            switched=True
            assert int(forward)==0 and int(brake)==100
    resume=[i for i,(_,state,_) in enumerate(records) if state=='resume_neutral']
    assert resume
    assert all([int(x) for x in frames[i][:3]]==[0,0,0] for i in resume)
