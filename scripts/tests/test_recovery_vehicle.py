"""Portable Python recovery -> C++ HostSession/ESC model check; no ROS/ESC."""
from pathlib import Path
import shutil
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/kart_recovery'))
from kart_recovery.core import Command, Observation, Recovery


def test_recovery_pulse_and_brake_reach_vehicle_model(tmp_path):
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
        moving=recovery.state in ('reversing','post_reverse_pulse') or (recovery.state=='post_reverse_braking' and brake_ticks<20)
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
    pulse=[i for i,(_,state,_) in enumerate(records) if state=='post_reverse_pulse']
    indices=[i for i,(_,state,_) in enumerate(records) if state=='post_reverse_braking']
    assert pulse == list(range(pulse[0],pulse[-1]+1))
    assert records[pulse[0]-1][2].reverse>0
    assert int(frames[pulse[0]-1][1])>0
    assert indices[0]==pulse[-1]+1
    duration=records[indices[0]][0]-records[pulse[0]][0]
    assert recovery.p.post_reverse_pulse_s <= duration < recovery.p.post_reverse_pulse_s+.021
    for i in pulse:
        assert records[i][2].throttle==recovery.p.post_reverse_pulse_throttle
        assert frames[i]==['80','0','0','forward','3']
    for i in indices:
        assert records[i][2].brake==recovery.p.brake_command
        assert frames[i]==['0','0','100','brake','3']
    resume=[i for i,(_,state,_) in enumerate(records) if state=='resume_neutral']
    assert resume
    assert all([int(x) for x in frames[i][:3]]==[0,0,0] for i in resume)
