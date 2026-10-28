import queue
import time
from types import SimpleNamespace as NS
from concurrent.futures import Future
from kart_hdmap.parking_bridge import ParkingBridge
from kart_hdmap.tuning import StopGate
from kart_hdmap.tuning_bridge import TuningBridge


def bridge():
    gate=StopGate();gate.update_mode(1);gate.update_speed(.5)
    b=ParkingBridge.__new__(ParkingBridge)
    b.bridge=NS(identity='map',revision=lambda:'r',gate=gate)
    b.enabled=True;b.pending=None;b.received=time.monotonic();b.data={'state':'ready'}
    b.type=NS(Request=lambda:object());calls=[];future=Future()
    b.clients={k:NS(service_is_ready=lambda:True,call_async=lambda r:(calls.append(r),future)[1]) for k in ('start','reset')}
    return b,calls,future


def test_start_once_completion_and_map_guard():
    b,calls,f=bridge();reply=queue.Queue();body=dict(map_id='map',expected_revision='r')
    b.begin('/parking_start',dict(body,map_id='wrong'),reply,time.monotonic()+3)
    assert 'error' in reply.get();assert not calls
    b.begin('/parking_start',body,reply,time.monotonic()+3);assert len(calls)==1
    b.begin('/parking_start',body,reply,time.monotonic()+3);assert 'error' in reply.get();assert len(calls)==1
    f.set_result(NS(success=True,message='armed'));b.tick();assert reply.get()['success']
    b.tick();assert len(calls)==1


def test_timeout_and_reset_guards():
    b,calls,f=bridge();reply=queue.Queue();body=dict(map_id='map',expected_revision='r')
    b.begin('/parking_reset',body,reply,time.monotonic()+3);assert 'error' in reply.get();assert not calls
    b.begin('/parking_start',body,reply,time.monotonic()+3)
    b.pending['deadline']=0;b.tick();assert '結果が不明' in reply.get()['error'];assert len(calls)==1
    b.bridge.gate.update_mode(2)
    b.begin('/parking_start',body,reply,time.monotonic()+3);assert 'error' in reply.get();assert len(calls)==1


def test_parking_composition_blocks_map_mutations():
    b=TuningBridge.__new__(TuningBridge);b.node=NS(settings={'tuning_parking_enabled':True})
    assert '駐車構成' in b.issue()
