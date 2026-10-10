"""Calibration, cadence and ideal inertial measurements without GL."""
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import mujoco
from kart_sim.core import Simulation, CAMERA_XYZ
from kart_sim.rig import D455Rig, rotation, OPTICAL_QUAT
from kart_sim.sensors import SensorSuite

PACKAGE=Path(__file__).resolve().parents[1]
MAP=PACKAGE.parents[2]/'maps/sim/indoor_empty.json'

class RigTests(unittest.TestCase):
    def make(self,**kwargs):
        sim=Simulation(MAP,PACKAGE/'assets',**kwargs)
        self.addCleanup(sim.close)
        return sim

    def test_mount_and_stereo_baseline(self):
        sim=self.make(camera=True)
        left=sim.data.cam_xpos[sim.model.camera('infra1').id]
        right=sim.data.cam_xpos[sim.model.camera('infra2').id]
        R=sim.data.xmat[sim.chassis_id].reshape(3,3)
        np.testing.assert_allclose(left,sim.state()[0]+R@CAMERA_XYZ)
        np.testing.assert_allclose(R.T@(right-left),[0,-.095,0],atol=1e-9)
        for name in ('infra1','infra2','color'):
            cid=sim.model.camera(name).id
            cfg=next(c for c in sim.sensor_configs if c.name==name)
            intrinsic=sim.model.cam_intrinsic[cid]
            actual=intrinsic[:2]/sim.model.cam_sensorsize[cid]*[424,240]
            np.testing.assert_allclose(actual,cfg.intrinsics[:2],rtol=1e-6)
        self.assertLess(sim.rig.projection('infra2')[0,3],0)
        self.assertEqual(sim.rig.projection('infra1')[0,3],0)
        for _ in range(200): sim.step()
        # Approved nominal lens height is floor-relative; base_link is on the rear axle.
        self.assertAlmostEqual(sim.data.cam_xpos[sim.model.camera('infra1').id][2],.133,delta=.001)
        plate=sim.model.geom('TT02Frame_collision')
        self.assertAlmostEqual(sim.data.geom_xpos[plate.id][2]+plate.size[2],.070,delta=.001)

    def test_camera_and_tf_rotations_agree(self):
        sim=self.make(camera=True)
        fixed={child:(parent,np.array(xyz),rotation(quat)) for parent,child,xyz,quat in sim.rig.static_transforms()}
        def pose(frame):
            if frame=='base_link':
                return sim.state()[0],sim.data.xmat[sim.chassis_id].reshape(3,3)
            parent,xyz,R=fixed[frame]; position,parent_R=pose(parent)
            return position+parent_R@xyz,parent_R@R
        for name,frame in sim.rig.frames.items():
            position,R=pose(frame); cid=sim.model.camera(name).id
            np.testing.assert_allclose(position,sim.data.cam_xpos[cid],atol=1e-9)
            np.testing.assert_allclose(R,sim.data.cam_xmat[cid].reshape(3,3)*[1,-1,-1],atol=1e-9)
        pos,R=pose('camera_gyro_optical_frame')
        np.testing.assert_allclose(pos,sim.imu_state()['position_world'])
        np.testing.assert_allclose(R,rotation(sim.imu_state()['orientation']))

    def test_sampling_30_60_90_and_independent_rgb_rate(self):
        def fake_camera(suite,c,data):
            return {'encoding':c.encoding,'K':np.eye(3)}
        for rate,rgb_rate in ((30.,90.),(60.,60.),(90.,30.)):
            sim=self.make(camera=True,stereo_hz=rate,rgb_hz=rgb_rate)
            stamps={name:[] for name in ('infra1','infra2','color')}; imu=[]
            with patch.object(SensorSuite,'_camera',fake_camera):
                for _ in range(100):
                    samples=sim.step(steps=10,collect_sensors=True)
                    imu.extend(f['time'] for f in samples['imu'])
                    for frames in samples['images']:
                        for name,frame in frames.items(): stamps[name].append(frame.time)
            np.testing.assert_allclose(stamps['infra1'],stamps['infra2'],atol=0)
            self.assertLessEqual(abs(len(stamps['infra1'])-rate),1)
            self.assertLessEqual(abs(len(stamps['color'])-rgb_rate),1)
            self.assertLessEqual(abs(len(imu)-200),1)
            self.assertLessEqual(np.max(np.abs(np.diff(stamps['infra1'])[1:]-1/rate)),.0010001)
            self.assertTrue(np.all(np.diff(imu)>0))
            sim.reset()
            self.assertEqual(sim.sensors.latest,{})
            self.assertEqual(sim._imu_due,0)

    def test_stationary_imu_gravity_and_freefall(self):
        sim=self.make()
        for _ in range(200): sim.step()
        imu=sim.imu_state()
        self.assertAlmostEqual(np.linalg.norm(imu['acceleration']),9.81,places=2)
        self.assertLess(imu['acceleration'][1],-9.7)  # Optical +Y points down.
        self.assertLess(np.linalg.norm(imu['angular_velocity']),.001)
        sim.data.qpos[2]+=1.; sim.data.qvel[:]=0
        mujoco.mj_forward(sim.model,sim.data)
        np.testing.assert_allclose(sim.imu_state()['acceleration'],0,atol=1e-4)

    def test_rotating_imu_truth(self):
        sim=self.make(); sim.data.qpos[2]+=1
        sim.data.qvel[3:6]=[0,0,1.2]
        mujoco.mj_forward(sim.model,sim.data)
        np.testing.assert_allclose(sim.imu_state()['angular_velocity'],[0,-1.2,0],atol=1e-9)

    def test_rate_validation(self):
        for kwargs in ({'stereo_hz':29.},{'rgb_hz':91.},{'imu_hz':float('nan')}):
            with self.assertRaises(ValueError): D455Rig(PACKAGE/'assets/d455.json',**kwargs)

if __name__=='__main__': unittest.main()
