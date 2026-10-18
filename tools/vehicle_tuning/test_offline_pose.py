import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from offline_pose import attach_poses
from turning import identify
from cli import main
from test_tuning import synthetic, CONFIG


class OfflineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'odom_poses.tum'
        self.cfg = copy.deepcopy(CONFIG)
        self.cfg.update(clock='header', max_yaw_rate_rps=2., max_speed_mps=2.)
        self.data = synthetic()
        for k in CONFIG['topics']:
            self.data[k] = self.data[k][:250]
        self.data['odom'] = []; self.data['vslam'] = []
        self.data['tf_static'] = [dict(parent='base_link',child='camera_link',
            xyz=[.23,.0475,.03],q=[0.,0.,0.,1.])]
        origin = self.data['command'][0]['header_ns']
        lines = []
        for i in range(250):
            t = i*.02; yaw = .3*t
            # Rear axle v=0.6, radius=2 m; offset camera follows a different circle.
            rear = np.array([2*math.sin(yaw),2*(1-math.cos(yaw)),0.])
            rot = np.array([[math.cos(yaw),-math.sin(yaw),0],
                            [math.sin(yaw),math.cos(yaw),0],[0,0,1]])
            pos = rear + rot @ np.array([.23,.0475,.03])
            ns=origin+i*20000000
            lines.append(f'{ns//10**9}.{ns%10**9:09d} '+ ' '.join(map(str,pos))+
                         f' 0 0 {math.sin(yaw/2)} {math.cos(yaw/2)}')
            self.data['command'][i]['steering']=.2
        self.path.write_text('\n'.join(lines))

    def test_circle_lever_arm_and_turn_report(self):
        data=attach_poses(self.data,self.cfg,self.path,'camera_link')
        od=data['odom'][100]
        self.assertAlmostEqual(od['linear'][0],.6,delta=.001)
        self.assertAlmostEqual(od['linear'][1],0,delta=.001)
        self.assertAlmostEqual(od['angular'][2],.3,delta=.001)
        summary,rows,trials=identify(data,self.cfg,.257)
        self.assertGreater(summary['selected_samples'],50)
        self.assertAlmostEqual(trials[0]['curvature_per_m'],.5,delta=.002)
        self.assertFalse(summary['model']['usable'])  # One trial cannot validate a model.
        self.assertTrue(summary['offline_pose'])
        fixture=Path(self.tmp.name)/'bag.json'
        fixture.write_text(json.dumps(self.data))
        out=Path(self.tmp.name)/'out'
        self.assertEqual(main(['turn','--normalized-json',str(fixture),'--offline-poses',str(self.path),
            '--pose-frame','camera_link','--wheelbase','.257','--output',str(out)]),0)
        self.assertTrue((out/'report.html').is_file())
        self.assertTrue((out/'turning_segments.csv').is_file())

    def test_side_model_recovers_known_angle_with_heldout_trials(self):
        data=synthetic()
        cfg=copy.deepcopy(self.cfg)
        # Repeated left/right plateaus include the same command range in validation.
        pattern=[.1,.2,.3,.1,.2,.3,-.1,-.2,-.3,-.1,-.2,-.3]
        count=75*len(pattern)
        for key in CONFIG['topics']:
            data[key]=data[key][:count]
        for i in range(count):
            u=pattern[i//75]
            data['command'][i].update(steering=u,throttle=.2,brake=0.)
            data['odom'][i].update(linear=[.6,0.,0.],angular=[0.,0.,.6*math.tan(.7*u)/.257])
            data['bridge'][i]['values']['esc_command_state']='forward'
        summary,_,_=identify(data,cfg,.257)
        self.assertTrue(summary['model']['usable'],summary['model'])
        for fit in summary['model']['sides'].values():
            self.assertAlmostEqual(fit['slope_rad'],.7,places=6)
            self.assertFalse(set(fit['train_segment_ids']) & set(fit['validation_segment_ids']))

    def test_missing_tf_and_no_time_overlap(self):
        self.data['tf_static']=[]
        with self.assertRaisesRegex(ValueError,'Missing static TF'):
            attach_poses(self.data,self.cfg,self.path,'camera_link')
        for r in self.data['command']: r['header_ns']+=10**12
        with self.assertRaisesRegex(ValueError,'overlap'):
            attach_poses(self.data,self.cfg,self.path,'base_link')

    def test_duplicate_and_optimized_rejected(self):
        self.path.write_text(self.path.read_text().splitlines()[0]+'\n'+self.path.read_text())
        with self.assertRaisesRegex(ValueError,'non-monotonic'):
            attach_poses(self.data,self.cfg,self.path,'camera_link')
        with self.assertRaisesRegex(ValueError,'odom_poses'):
            attach_poses(self.data,self.cfg,self.path.with_name('slam_poses.tum'),'camera_link')

    def test_jump_gap_and_online_gate(self):
        lines=self.path.read_text().splitlines()
        fields=lines[100].split();fields[1]='100';lines[100]=' '.join(fields)
        del lines[150:165]
        self.path.write_text('\n'.join(lines))
        data=attach_poses(self.data,self.cfg,self.path,'camera_link')
        self.assertIn('pose_jump',{r['pose_reason'] for r in data['odom']})
        self.assertIn('pose_gap',{r['pose_reason'] for r in data['odom']})
        _,rows,_=identify(data,self.cfg,.257)
        self.assertTrue(all(r['curvature_per_m'] is None for r in rows if r['reason']))
        data.pop('offline_pose')
        with self.assertRaisesRegex(ValueError,'vslam'):
            identify(data,self.cfg,.257)


if __name__=='__main__':
    unittest.main()
