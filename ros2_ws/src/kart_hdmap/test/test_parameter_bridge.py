"""Real transaction state machine with injected service futures, not ROS integration."""
from concurrent.futures import Future
import queue
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from kart_hdmap.parameter_bridge import ParameterBridge
from kart_hdmap.parameter_policy import TARGETS, catalog, changes_for


def fields(key='tracking'):
    return [dict(name=name,value=1.,type=3,read_only=name in ('wheelbase_m','max_steering_rad'),label=name,description='',constraints='')
            for name in TARGETS[key][1]]


class PolicyTests(unittest.TestCase):
    def test_type_readonly_revision_and_node_boundaries(self):
        current = catalog('tracking',fields())
        self.assertEqual(changes_for(current,{'lookahead_m':2},current['revision']),{'lookahead_m':2.})
        settings={'lookahead_low_speed_m':.6,'lookahead_high_speed_m':2.}
        self.assertEqual(changes_for(current,settings,current['revision']),settings)
        mode_fields=fields()
        for row in mode_fields:
            if row['name']=='lookahead_mode': row.update(type=4,value='fixed')
            if row['name']=='lateral_controller': row.update(type=4,value='pure_pursuit')
        mode_catalog=catalog('tracking',mode_fields)
        self.assertEqual(changes_for(mode_catalog,{'lateral_controller':'mpc'},mode_catalog['revision']), {'lateral_controller':'mpc'})
        with self.assertRaises(ValueError):
            changes_for(mode_catalog,{'lateral_controller':'invalid'},mode_catalog['revision'])
        self.assertEqual(changes_for(mode_catalog,{'lookahead_mode':'speed_interpolated'},mode_catalog['revision']),
                         {'lookahead_mode':'speed_interpolated'})
        with self.assertRaises(ValueError):
            changes_for(mode_catalog,{'lookahead_mode':'arbitrary'},mode_catalog['revision'])
        for changes, revision in (({'lookahead_m':0.8},'stale'),({'arbitrary':1.},current['revision']),
                                  ({'wheelbase_m':.2},current['revision']),({'lookahead_m':float('nan')},current['revision']),
                                  ({'lookahead_m':True},current['revision'])):
            with self.subTest(changes=changes), self.assertRaises(ValueError): changes_for(current,changes,revision)
        with self.assertRaises(ValueError):catalog('/operation_mode_manager',[])


class BridgeTests(unittest.TestCase):
    def test_vehicle_and_recovery_keep_stationary_gate(self):
        manager = ParameterBridge.__new__(ParameterBridge)
        manager.bridge = NS(issue=lambda:'', gate=NS(issue=lambda:'STOP required'))
        for key in ('tracking', 'speed', 'map'):
            self.assertEqual(manager.issue(key), '')
        for key in ('vehicle', 'recovery'):
            self.assertEqual(manager.issue(key), 'STOP required')

    def setUp(self):
        self.values={row['name']:row['value'] for row in fields()}
        self.ready=True
        self.writes=[]
        self.hold_set=False
        self.clients=[]
        test=self
        class Service:
            Request=NS
        class Client:
            def __init__(self,index): self.index=index
            def service_is_ready(self): return test.ready
            def call_async(self,request):
                future=Future()
                if self.index==0:
                    future.set_result(NS(values=[NS(type=4 if isinstance(test.values[name],str) else 3,double_value=test.values[name] if not isinstance(test.values[name],str) else 0.,integer_value=0,bool_value=False,string_value=test.values[name] if isinstance(test.values[name],str) else '') for name in request.names]))
                elif self.index==1:
                    future.set_result(NS(descriptors=[NS(read_only=name in ('wheelbase_m','max_steering_rad'),description='',additional_constraints='') for name in request.names]))
                else:
                    test.writes.append(request)
                    for parameter in request.parameters:test.values[parameter.name]=parameter.value
                    if not test.hold_set:future.set_result(NS(result=NS(successful=True,reason='')))
                return future
        self.clients=tuple(Client(i) for i in range(3))
        self.bridge=NS(identity='map-a',issue=lambda:'',node=NS())
        self.manager=ParameterBridge.__new__(ParameterBridge)
        self.manager.bridge=self.bridge
        self.manager.types=(Service,Service,Service)
        self.manager.pending=None
        self.manager.clients_for=lambda key:self.clients
        class Parameter:
            def __init__(self,name,value):self.name,self.value=name,value
            def to_parameter_msg(self):return self
        self.module_patch=patch.dict(sys.modules,{'rclpy.parameter':NS(Parameter=Parameter)})
        self.module_patch.start();self.addCleanup(self.module_patch.stop)

    def begin(self,action,**body):
        result=queue.Queue()
        self.manager.begin(action,{'map_id':'map-a','target':'tracking',**body},result,100.)
        return result

    def test_read_apply_and_readback_without_blocking_executor(self):
        with patch('kart_hdmap.parameter_bridge.time.monotonic',return_value=10.):
            reply=self.begin('/parameters_get');self.manager.tick();current=reply.get_nowait()
            self.assertEqual(current['fields'][0]['value'],1.)
            reply=self.begin('/parameters_set',expected_revision=current['revision'],changes={'lookahead_m':.8})
            self.manager.tick()  # dispatch only
            self.assertTrue(reply.empty())
            self.manager.tick()  # readback only
            self.assertTrue(reply.empty())
            self.manager.tick()
            result=reply.get_nowait()
            self.assertTrue(result['applied'] and result['readback_matches'])
            self.assertEqual(next(f['value'] for f in result['fields'] if f['name']=='lookahead_m'),.8)
            self.assertEqual(len(self.writes),1)

    def test_unavailable_stale_and_stop_rejections_do_not_write(self):
        with patch('kart_hdmap.parameter_bridge.time.monotonic',return_value=10.):
            self.ready=False
            reply=self.begin('/parameters_get')
            self.assertFalse(reply.get_nowait()['available'])
            self.ready=True
            reply=self.begin('/parameters_set',expected_revision='stale',changes={'lookahead_m':.8});self.manager.tick()
            self.assertIn('error',reply.get_nowait())
            self.bridge.issue=lambda:'STOP required'
            reply=self.begin('/parameters_set',expected_revision='x',changes={'lookahead_m':.8})
            self.assertIn('error',reply.get_nowait())
            self.assertEqual(self.writes,[])

    def test_busy_request_does_not_discard_pending_and_no_retry_after_timeout(self):
        with patch('kart_hdmap.parameter_bridge.time.monotonic',return_value=10.):
            reply=self.begin('/parameters_get')
            pending=self.manager.pending
            other=self.begin('/parameters_get')
            self.assertIn('error',other.get_nowait())
            self.assertIs(self.manager.pending,pending)
            self.manager.tick();current=reply.get_nowait()
            self.hold_set=True
            reply=self.begin('/parameters_set',expected_revision=current['revision'],changes={'lookahead_m':.8})
            self.manager.tick()
        with patch('kart_hdmap.parameter_bridge.time.monotonic',return_value=101.):
            self.manager.tick()
            self.assertIn('自動再送',reply.get_nowait()['error'])
            self.assertEqual(len(self.writes),1)
            self.assertIsNone(self.manager.pending)

    def test_map_geometry_revision_is_part_of_parameter_version(self):
        self.values={'line_type':'centerline','centerline_speed_mps':.3}
        self.bridge.node.document={'lanes':[{'id':'main','lines':{'centerline':{},'customline':{}}}]}
        self.bridge.node.selected='main'
        version=['map-r1']
        self.bridge.revision=lambda:version[0]
        with patch('kart_hdmap.parameter_bridge.time.monotonic',return_value=10.):
            reply=queue.Queue()
            self.manager.begin('/parameters_get',{'map_id':'map-a','target':'map'},reply,100.)
            self.manager.tick();current=reply.get_nowait()
            self.assertEqual(current['line_choices'],['centerline','customline'])
            version[0]='map-r2'
            reply=queue.Queue()
            self.manager.begin('/parameters_set',{'map_id':'map-a','target':'map',
                               'expected_revision':current['revision'],'changes':{'line_type':'customline'}},reply,100.)
            self.manager.tick()
            self.assertIn('error',reply.get_nowait())
            self.assertEqual(self.writes,[])
