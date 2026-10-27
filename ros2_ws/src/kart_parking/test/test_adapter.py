"""Execute production callback bodies with data doubles, without importing ROS.
This verifies callback state transitions; it is not a ROS transport/executor test.
"""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace as NS


def method(name):
    source=Path(__file__).resolve().parents[1]/'kart_parking/node.py'
    tree=ast.parse(source.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ParkingNode')
    node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    scope={'copy':copy}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),scope)
    return scope[name]


def reference(stamp, speed=1.):
    return NS(header=NS(frame_id='map',stamp=stamp),lane_id='main',line_type='centerline',closed=True,
              points=[NS(x=0.,y=0.,z=0.),NS(x=1.,y=0.,z=0.),NS(x=1.,y=1.,z=0.)],speeds=[speed]*3)


def test_source_heartbeat_updates_stamp_without_replanning():
    node=NS(inputs={},signature=None,base=None,generation=0,prepared=None,mission=None)
    node.receive=lambda key,m:node.inputs.update({key:m})
    callback=method('line')
    callback(node,reference(1))
    prepared=object();node.prepared=prepared
    callback(node,reference(2))
    assert node.base.header.stamp==2
    assert node.generation==1 and node.prepared is prepared
    assert node.inputs['line'].header.stamp==2


def test_changed_course_invalidates_active_mission():
    node=NS(signature=None,generation=0,prepared=None,mission=None)
    node.receive=lambda *a:None
    callback=method('line');callback(node,reference(1))
    faults=[];node.mission=NS(fault=faults.append)
    callback(node,reference(2,.4))
    assert faults==['course_changed'] and node.locked
    assert node.prepared is None and node.generation==2
