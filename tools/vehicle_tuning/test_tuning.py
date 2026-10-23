"""Physical synthetic trajectories, rejection behavior and real rosbag2 adapter tests."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).parent))
from analysis import analyze, prepare, simulate, static_transform
from bag import read_bag
from cli import main

CONFIG=json.loads(Path(__file__).with_name('config.json').read_text())


def synthetic():
    """Known independent plant a=4*u-2*b-.8*v-.05, finite-duration coast/brake runs."""
    data={k:[] for k in CONFIG['topics']};data.update(tf_static=[],source='synthetic, not vehicle measurement')
    data['inventory']={v:{'type':'fixture','count':1} for v in CONFIG['topics'].values()}
    t=0.; v=.2; origin=1_750_000_000_000_000_000
    pattern=[]
    for u in (.2,.3,.4,.25,.35,.3):
        pattern.extend([(u,0.,6.),(0.,0.,1.),(u,0.,3.),(0.,.1,1.)])
    for throttle,brake,duration in pattern:
        for _ in range(round(duration/.02)):
            ns=origin+round(t*1e9); base={'bag_ns':ns,'header_ns':ns}
            data['odom'].append({**base,'frame':'odom','child':'base_link','linear':[v,0.,0.],'angular':[0.,0.,0.]})
            data['command'].append({**base,'steering':0.,'throttle':throttle,'brake':brake,'reverse':0.})
            data['mode'].append({**base,'mode':2})
            data['path'].append({**base,'header_ns':None,'value':2})
            data['vslam'].append({**base,'level':0,'message':'ok','values':{'vo_status':'OK'}})
            data['bridge'].append({**base,'level':0,'message':'ok','values':{
                'host_active':'true','esc_brake_rejected':'false','esc_command_state':'brake' if brake else 'forward' if throttle else 'neutral'}})
            data['imu'].append({**base,'frame':'imu','linear':[4*throttle-2*brake-.8*v-.05,0.,9.81],'angular':[0.,0.,0.]})
            v=max(0.,v+.02*(4*throttle-2*brake-.8*v-.05)); t+=.02
    return data


class AnalysisTests(unittest.TestCase):
    def test_explicit_neutral_subtypes_and_legacy_bag_compatibility(self):
        for subtype in ('neutral','neutral_initial','neutral_after_forward','neutral_after_brake'):
            data=synthetic()
            for row in data['bridge']:
                if row['values']['esc_command_state']=='neutral':row['values']['esc_command_state']=subtype
            _,rows,_,_=analyze(data,CONFIG)
            assert not any('bridge_not_ready' in row['reason'] for row in rows)
        data=synthetic()
        for row in data['bridge']:row['values']['esc_command_state']='neutral_after_reverse'
        _,rows,_,_=analyze(data,CONFIG)
        assert all('bridge_not_ready' in row['reason'] for row in rows)

    @classmethod
    def setUpClass(cls): cls.data=synthetic()

    def test_recovers_plant_with_whole_segment_validation(self):
        summary,rows,segments,bins=analyze(self.data,CONFIG)
        model=summary['model']
        self.assertTrue(model['usable'],model)
        self.assertAlmostEqual(model['A'],4,delta=.12)
        self.assertAlmostEqual(model['B'],2,delta=.12)
        self.assertAlmostEqual(model['C'],.8,delta=.03)
        self.assertAlmostEqual(model['D'],.05,delta=.03)
        self.assertLess(model['validation_speed_rollout_rmse_mps'],.05)
        self.assertFalse(set(model['train_segment_ids']) & set(model['validation_segment_ids']))
        self.assertIsNotNone(summary['kff_candidate'])
        self.assertEqual({b['kind'] for b in bins},{'accel','coast','brake'})
        self.assertTrue(all(r['acceleration_mps2'] is None for r in rows if r['reason']))

    def test_small_command_header_backstep_preserves_order_and_raw_stamp(self):
        cfg = dict(CONFIG, clock='header')
        for backstep_ns in (2_175_000, 5_000_000):
            data = {k: copy.deepcopy(v[:5]) for k, v in self.data.items() if isinstance(v, list)}
            previous_ns = data['command'][1]['header_ns']
            raw_ns = previous_ns - backstep_ns
            data['command'][2]['header_ns'] = raw_ns
            original = copy.deepcopy(data['command'])
            streams, quality, _ = prepare(data, cfg)
            times, rows = streams['command']
            self.assertEqual(times[1], times[2])
            self.assertEqual(rows[2]['header_ns'], raw_ns)
            self.assertEqual(data['command'], original)
            self.assertEqual(quality['command']['clamped_header_count'], 1)
            self.assertAlmostEqual(quality['command']['max_header_clamp_s'], backstep_ns / 1e9)

    def test_header_backstep_tolerance_does_not_hide_other_clock_failures(self):
        cfg = dict(CONFIG, clock='header')
        for key, field, backstep_ns in (
            ('command', 'header_ns', 5_000_001),
            ('mode', 'header_ns', 1),
            ('odom', 'header_ns', 1),
            ('command', 'bag_ns', 1),
        ):
            data = {k: copy.deepcopy(v[:5]) for k, v in self.data.items() if isinstance(v, list)}
            data[key][2][field] = data[key][1][field] - backstep_ns
            with self.assertRaisesRegex(ValueError, 'clock goes backwards'):
                prepare(data, cfg)
        data = {k: copy.deepcopy(v[:5]) for k, v in self.data.items() if isinstance(v, list)}
        high_water = data['command'][1]['header_ns']
        data['command'][2]['header_ns'] = high_water - 3_000_000
        data['command'][3]['header_ns'] = high_water - 6_000_000
        with self.assertRaisesRegex(ValueError, 'selected clock goes backwards'):
            prepare(data, cfg)

    def test_static_rotation_and_lever_arm(self):
        # Sensor axes rotated 90 deg, one metre ahead. v_child=(2,1,0) in body;
        # body omega z=1, so rear velocity=(2,0,0).
        transform=[{'parent':'base_link','child':'sensor','xyz':[1,0,0],
                    'q':[0,0,2**-.5,2**-.5]}]
        tf=static_transform(transform,'base_link','sensor')
        velocity=tf[:3,:3]@np.array([1,-2,0])-np.cross(np.array([0,0,1]),tf[:3,3])
        np.testing.assert_allclose(velocity,[2,0,0],atol=1e-9)
        with self.assertRaisesRegex(ValueError,'Missing static TF'):
            static_transform([], 'base_link','camera_optical')
        with self.assertRaisesRegex(ValueError,'changed'):
            static_transform(transform+[{**transform[0],'xyz':[2,0,0]}],'base_link','sensor')

    def test_clock_and_frame_failures(self):
        for mutate,pattern in (
            (lambda d:d['odom'][30].update(header_ns=1),'clock mismatch'),
            (lambda d:d['odom'][30].update(bag_ns=d['odom'][29]['bag_ns']),'duplicate'),
            (lambda d:d['odom'][30].update(child='camera'),'frame IDs'),
        ):
            data=copy.deepcopy(self.data);mutate(data)
            with self.assertRaisesRegex(ValueError,pattern): analyze(data,CONFIG)

    def test_rejects_stale_failed_turn_reverse_and_nonhost(self):
        data=copy.deepcopy(self.data)
        data['command'][50]['header_ns']-=500_000_000
        data['vslam'][100]['values']['vo_status']='LOST'
        data['odom'][150]['angular'][2]=.8
        data['command'][200].update(throttle=0.,reverse=.1)
        data['path'][250]['value']=1
        data['bridge'][300]['values']['esc_brake_rejected']='true'
        summary,rows,_,_=analyze(data,CONFIG)
        for idx,reason in ((50,'stale_source_command'),(100,'vslam_not_ok'),(150,'turn_or_lateral_motion'),
                           (200,'reverse'),(250,'not_host_path'),(300,'bridge_not_ready')):
            self.assertIn(reason,rows[idx]['reason'])
            self.assertIsNone(rows[idx]['acceleration_mps2'])

    def test_no_future_command_matching(self):
        data=copy.deepcopy(self.data)
        data['command']=data['command'][10:]
        _,rows,_,_=analyze(data,CONFIG)
        self.assertTrue(all('missing_command' in r['reason'] for r in rows[:10]))

    def test_changing_trim_rejected(self):
        data=copy.deepcopy(self.data)
        data['trim']=[{'steering':0.,'throttle':0.},{'steering':0.,'throttle':.02}]
        with self.assertRaisesRegex(ValueError,'trim changed'): analyze(data,CONFIG)

    def test_auto_trial_cli_requires_explicit_mode(self):
        data=copy.deepcopy(self.data)
        for row in data['mode']: row['mode']=1
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'auto.json';source.write_text(json.dumps(data))
            self.assertEqual(main(['analyze','--normalized-json',str(source),'--output',str(root/'manual')]),2)
            self.assertEqual(main(['analyze','--mode','auto','--normalized-json',str(source),'--output',str(root/'auto')]),0)
            summary=json.loads((root/'auto/summary.json').read_text())
            self.assertEqual(summary['config']['allowed_modes'],[1])
            self.assertGreater(summary['selected_samples'],0)

    def test_gap_not_smoothed_and_no_model_from_constant_speed(self):
        data=copy.deepcopy(self.data)
        del data['odom'][100:120]
        _,rows,_,_=analyze(data,CONFIG)
        self.assertIn('odometry_gap',rows[100]['reason'])
        self.assertIsNone(rows[100]['acceleration_mps2'])
        for key in CONFIG['topics']:
            data[key]=data[key][:40]
        summary,_,_,_=analyze(data,CONFIG)
        self.assertFalse(summary['model']['usable'])

    def test_braking_model_required_and_output_saturation(self):
        model=analyze(self.data,CONFIG)[0]['model']
        settings=dict(kp=.1,ki=.03,kd=0.,kff=.21,integral_limit=.2,derivative_tau_s=.1,throttle_limit=.4,brake_limit=.1)
        rows=simulate(model,settings,[.7,1.2,.4],3.,.15)
        self.assertTrue(all(0<=r['throttle']<=.4 and 0<=r['brake']<=.1 and not(r['throttle'] and r['brake']) for r in rows))
        with self.assertRaisesRegex(ValueError,'Braking model'):
            simulate({**model,'braking_identified':False},settings,[.5],1.,.1)

    def test_cli_roundtrip_and_no_overwrite_or_live_config_edit(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'data.json';source.write_text(json.dumps(self.data))
            output=root/'analysis'
            self.assertEqual(main(['analyze','--normalized-json',str(source),'--output',str(output)]),0)
            self.assertTrue((output/'report.html').exists())
            self.assertEqual(main(['analyze','--normalized-json',str(output/'normalized.json'),'--output',str(root/'repeat')]),0)
            base=Path(__file__).resolve().parents[2]/'ros2_ws/src/kart_bringup/config/control/speed_controller.yaml'
            before=base.read_bytes()
            self.assertEqual(main(['tune','--analysis',str(output),'--output',str(root/'tune'),
                '--kp','.1','--kff','.21','--targets','.4,.8,.4','--delays','.1,.2']),0)
            self.assertEqual(base.read_bytes(),before)
            with self.assertRaises(SystemExit): main(['analyze','--normalized-json',str(source),'--output',str(output)])


class BagTests(unittest.TestCase):
    def test_full_bag_to_model(self):
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores,get_typestore,get_types_from_msg
        from bag import TYPES, REQUIRED
        store=get_typestore(Stores.LATEST)
        root=Path(__file__).resolve().parents[2]
        for name in ('ControlCommand','OperationModeState'):
            store.register(get_types_from_msg((root/f'ros2_ws/src/kart_interfaces/msg/{name}.msg').read_text(),'kart_interfaces/msg/'+name))
        types=store.types
        def make(name,*args): return types[name](*args)
        def vector(values): return make('geometry_msgs/msg/Vector3',*values)
        pose=make('geometry_msgs/msg/PoseWithCovariance',make('geometry_msgs/msg/Pose',
            make('geometry_msgs/msg/Point',0.,0.,0.),make('geometry_msgs/msg/Quaternion',0.,0.,0.,1.)),np.zeros(36))
        data=synthetic()
        with tempfile.TemporaryDirectory() as folder:
            bag=Path(folder)/'bag'
            with Writer(bag,version=9) as writer:
                connections={key:writer.add_connection(CONFIG['topics'][key],TYPES[key],typestore=store) for key in REQUIRED}
                for i in range(len(data['odom'])):
                    for key in REQUIRED:
                        row=data[key][i];ns=row['bag_ns'];sec,nsec=divmod(ns,10**9)
                        header=make('std_msgs/msg/Header',make('builtin_interfaces/msg/Time',sec,nsec),'odom' if key=='odom' else '')
                        if key=='odom':
                            msg=make(TYPES[key],header,'base_link',pose,make('geometry_msgs/msg/TwistWithCovariance',
                                make('geometry_msgs/msg/Twist',vector(row['linear']),vector(row['angular'])),np.zeros(36)))
                        elif key=='command': msg=make(TYPES[key],header,row['steering'],row['throttle'],row['brake'],row['reverse'])
                        elif key=='mode': msg=make(TYPES[key],header,2,'test')
                        elif key=='path': msg=make(TYPES[key],2)
                        else:
                            status=make('diagnostic_msgs/msg/DiagnosticStatus',0,key,'ok','visual_slam' if key=='vslam' else 'JPBB-01',
                                [make('diagnostic_msgs/msg/KeyValue',k,v) for k,v in row['values'].items()])
                            msg=make(TYPES[key],header,[status])
                        writer.write(connections[key],ns,store.serialize_cdr(msg,TYPES[key]))
            result=read_bag(bag,CONFIG)
            self.assertFalse(result['missing_required'])
            summary,_,_,_=analyze(result,CONFIG)
            self.assertTrue(summary['model']['usable'],summary['model'])
            self.assertAlmostEqual(summary['model']['A'],4.,delta=.12)

    def test_real_sqlite_bag_custom_messages_and_headerless_path(self):
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores,get_typestore,get_types_from_msg
        store=get_typestore(Stores.LATEST)
        root=Path(__file__).resolve().parents[2]
        for name in ('ControlCommand','OperationModeState'):
            store.register(get_types_from_msg((root/f'ros2_ws/src/kart_interfaces/msg/{name}.msg').read_text(),'kart_interfaces/msg/'+name))
        def default(typename):
            cls=store.types[typename]
            # Explicit construction is clearer for the few messages in this adapter test.
            return cls
        Time=default('builtin_interfaces/msg/Time');Header=default('std_msgs/msg/Header')
        header=Header(Time(1750000000,0),'base_link')
        msg=default('kart_interfaces/msg/ControlCommand')(header,0.,.2,0.,0.)
        path=default('std_msgs/msg/UInt8')(2)
        with tempfile.TemporaryDirectory() as folder:
            bag=Path(folder)/'bag'
            with Writer(bag,version=9) as writer:
                for topic,value,typ in (('/vehicle/control_cmd',msg,'kart_interfaces/msg/ControlCommand'),('/vehicle/active_path',path,'std_msgs/msg/UInt8')):
                    conn=writer.add_connection(topic,typ,typestore=store)
                    writer.write(conn,1750000000000000000,store.serialize_cdr(value,typ))
            data=read_bag(bag,CONFIG)
            self.assertAlmostEqual(data['command'][0]['throttle'],.2)
            self.assertEqual(data['path'][0]['header_ns'],None)
            self.assertIn('/visual_slam/tracking/odometry',data['missing_required'])


if __name__=='__main__': unittest.main()
