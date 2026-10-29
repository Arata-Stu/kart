"""Portable sequence and ROS adapter tests; no actual ROS, network or vehicle."""
import importlib
import math
from pathlib import Path
import queue
import sys
import threading
import types
import uuid

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kart_control_trials.sequence import Sequence, validate_steps


def drive(model, start, end, ready=True, healthy=True, heartbeat=True):
    for i in range(round((end-start)*100)+1):
        now=start+i*.01
        if heartbeat:
            model.heartbeat(model.run_id,now)
        model.tick(now,ready,healthy)


def test_complete_preserves_neutral_and_never_automatically_restarts():
    s=Sequence();s.start('run',validate_steps([
        {'output':.3,'seconds':.1},{'output':-.3,'seconds':.1},
        {'output':0,'seconds':.1},{'output':-.3,'seconds':.1}]),1)
    seen=[]
    for i in range(100):
        now=1+i*.01;s.heartbeat('run',now);s.tick(now,True,True)
        if not seen or seen[-1]!=s.output:seen.append(s.output)
    assert seen==[0,.3,-.3,0,-.3,0]
    assert s.phase=='complete'
    drive(s,2,3)
    assert s.output==0 and s.phase=='complete'


@pytest.mark.parametrize('rows', [[],[{'output':True,'seconds':1}],
    [{'output':math.nan,'seconds':1}],[{'output':.31,'seconds':1}],
    [{'output':0,'seconds':0}], [{'output':0,'seconds':20}]*4])
def test_validation(rows):
    with pytest.raises(ValueError): validate_steps(rows)


@pytest.mark.parametrize('fault',['lease','health','late','backward','stop','arming'])
def test_faults_neutralize_and_latch(fault):
    s=Sequence();s.start('run',[{'output':.3,'seconds':10}],1)
    drive(s,1,1.4,ready=fault!='arming')
    if fault=='lease':
        # Foreign viewers must not renew an executing run's lease.
        for i in range(401):
            t=1.41+i*.01;s.heartbeat('other',t);s.tick(t,True,True)
    elif fault=='health':s.tick(1.41,True,False)
    elif fault=='late':s.tick(1.6,True,True)
    elif fault=='backward':s.tick(1.3,True,True)
    elif fault=='stop':s.stop()
    else:drive(s,1.41,3.1,ready=False)
    assert not s.active and s.output==0
    s.heartbeat('run',6);s.tick(6,True,True)
    assert not s.active and s.output==0


@pytest.fixture
def adapter(monkeypatch):
    # Import adapter with message stubs to exercise queue/STOP ordering.
    class Message:
        def __init__(self,**kw):
            self.header=types.SimpleNamespace(stamp=None,frame_id='')
            self.throttle=self.reverse=self.brake=0.
            self.__dict__.update(kw)
    for name,attrs in {
        'rclpy':{},'rclpy.node':{'Node':object},
        'rcl_interfaces.msg':dict(ParameterDescriptor=Message,SetParametersResult=Message),
        'diagnostic_msgs.msg':dict(DiagnosticArray=Message),
        'std_msgs.msg':dict(Int32MultiArray=Message,UInt8=Message,String=Message),
        'kart_interfaces.msg':dict(ControlCommand=Message,OperationModeRequest=Message,OperationModeState=Message),
    }.items():
        module=types.ModuleType(name);module.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,module)
    sys.modules.pop('kart_control_trials.node',None)
    module=importlib.import_module('kart_control_trials.node')
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    node=module.TrialNode.__new__(module.TrialNode)
    node.last_sequence=('idle',-1,0.,'');node.model=Sequence();node.requests=queue.Queue(16);node.stop_event=threading.Event()
    node.generation=0;node.stream='session';node.used_ids=set();node.mode=3;node.path=0
    node.limit=.3;node.lease_s=3.;node.auto_sent=False;node.diag={};node.last_stop=node.last_sample=10.
    node.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:None))
    node.healthy=lambda now:True
    node.snapshot=lambda body,now:dict(phase=node.model.phase)
    node.events=[];node.event=lambda *args:node.events.append(args)
    node.sent=[];node.request_mode=lambda m:node.sent.append(m)
    node.outputs=[];node.command=types.SimpleNamespace(publish=node.outputs.append)
    yield node
    sys.modules.pop('kart_control_trials.node',None)


def enqueue(node, action='/start', **extra):
    reply=queue.Queue(1)
    body=dict(run_id=str(uuid.uuid4()),stream='session',generation=0,
              steps=[{'output':.3,'seconds':1}],**extra)
    node.requests.put((action,body,reply,11.))
    return body,reply


def test_stop_cancels_every_queued_start_and_rejects_delayed_start(adapter):
    node=adapter
    replies=[enqueue(node)[1] for _ in range(8)]
    node.stop_event.set();node.tick()
    assert all('error' in r.get_nowait() for r in replies)
    assert node.generation==1 and node.sent[0]==3
    assert node.outputs[-1].throttle==node.outputs[-1].reverse==0
    _,reply=enqueue(node);node.tick()
    assert 'error' in reply.get_nowait() and not node.model.active


def test_start_is_one_shot_and_negative_uses_reverse_field(adapter):
    node=adapter;body,reply=enqueue(node);node.tick()
    assert reply.get_nowait()['phase']=='arming'
    node.model.stop();reply=queue.Queue(1);node.requests.put(('/start',body,reply,11.));node.tick()
    assert 'already been used' in reply.get_nowait()['error']
    node.model.start('x',[{'output':-.3,'seconds':1}],9.5)
    node.model.phase='running';node.model.index=0;node.model.stage_at=node.model.last_tick=10.
    node.model.output=-.3;node.mode=1;node.path=2;node.diag={'host_phase':'3'}
    node.tick()
    assert node.outputs[-1].reverse==.3 and node.outputs[-1].throttle==node.outputs[-1].brake==0
