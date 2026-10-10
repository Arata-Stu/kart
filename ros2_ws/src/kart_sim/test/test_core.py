import json
import math
from pathlib import Path
import tempfile
import unittest
import mujoco
import numpy as np
from kart_sim.core import Simulation
from kart_sim.world import load_map

PACKAGE=Path(__file__).resolve().parents[1]
REPO=PACKAGE.parents[2]
MAPS=REPO/'maps/sim'

class SimulationTests(unittest.TestCase):
    def make(self,name='indoor_empty',**kwargs):
        sim=Simulation(MAPS/(name+'.json'),PACKAGE/'assets',**kwargs)
        self.addCleanup(sim.close)
        return sim

    def test_maps_compile_and_settle(self):
        for name in ('indoor_empty','minicar_2026'):
            sim=self.make(name)
            for _ in range(200): sim.step()
            self.assertTrue(np.isfinite(sim.data.qpos).all())
            self.assertFalse(any(w.number for w in sim.data.warning))
            self.assertAlmostEqual(sim.state()[0][2],sim.parameters.tire_radius,places=3)
            self.assertIn('room_wall_0',[sim.model.geom(i).name for i in range(sim.model.ngeom)])

    def test_spawn_and_rear_axle(self):
        sim=self.make('minicar_2026')
        np.testing.assert_allclose(sim.state()[0][:2],sim.map['spawn'][:2],atol=1e-10)
        self.assertAlmostEqual(sim.base_offset[0],-.257/2)
        sim.step(throttle=.4,steps=1000); sim.reset()
        np.testing.assert_allclose(sim.state()[0][:2],sim.map['spawn'][:2],atol=1e-10)
        self.assertEqual(sim.data.time,0)

    def test_forward_reverse_and_left(self):
        for throttle,reverse,steering,sign in ((.5,0,0,1),(0,.5,0,-1),(.5,0,.5,1)):
            sim=self.make()
            for _ in range(200): sim.step(throttle=throttle,reverse=reverse,steering=steering)
            self.assertGreater(sign*sim.state()[0][0],.1)
            if steering: self.assertGreater(sim.state()[0][1],.05)
            self.assertFalse(any(w.number for w in sim.data.warning))

    def test_brake_dissipates_without_reverse(self):
        sim=self.make()
        for _ in range(200): sim.step(throttle=.7)
        before=sim.state()[2][0]
        for _ in range(100): sim.step(brake=.8)
        after=sim.state()[2][0]
        self.assertGreater(before,.1)
        self.assertLess(abs(after),before*.5)
        self.assertGreater(after,-.03)

    def test_invalid_commands(self):
        sim=self.make()
        for args in ({'throttle':float('nan')},{'steering':2},{'throttle':.2,'brake':.2},{'reverse':-1}):
            with self.assertRaises(ValueError): sim.step(**args)

    def test_rear_axle_twist_matches_position_derivative(self):
        sim=self.make()
        for _ in range(100): sim.step(throttle=.6,steering=.5)
        pos,_,linear,_=sim.state()
        R=sim.data.xmat[sim.chassis_id].reshape(3,3).copy()
        sim.step(throttle=.6,steering=.5,steps=1)
        numerical=(sim.state()[0]-pos)/sim.model.opt.timestep
        np.testing.assert_allclose(R@linear,numerical,atol=.015)

    def test_wall_contact(self):
        sim=self.make()
        sim.data.qpos[:2]=[5.5,0]
        mujoco.mj_forward(sim.model,sim.data)
        for _ in range(500): sim.step(throttle=1)
        self.assertLess(sim.state()[0][0],6)
        self.assertFalse(any(w.number for w in sim.data.warning))

    def test_camera_pose_matches_kart(self):
        from kart_sim.core import CAMERA_XYZ
        sim=self.make(camera=True)
        base,_,_,_=sim.state()
        R=sim.data.xmat[sim.chassis_id].reshape(3,3)
        np.testing.assert_allclose(sim.data.cam_xpos[sim.model.camera('infra1').id],base+R@CAMERA_XYZ)
        # Optical +Z points forward; +X right; +Y down.
        optical=sim.data.cam_xmat[0].reshape(3,3)*[1,-1,-1]
        np.testing.assert_allclose(optical,[[0,0,1],[-1,0,0],[0,-1,0]],atol=1e-8)

    def test_tunnel_and_parking_entrances_are_open(self):
        sim=self.make('minicar_2026')
        geom_id=np.array([-1],dtype=np.int32)
        distance=mujoco.mj_ray(sim.model,sim.data,np.array([8.45,.7,.05]),
                               np.array([1.,0.,0.]),None,1,-1,geom_id)
        self.assertGreater(distance,1.0)  # Clear crossing of former x=8.75 curtain.
        for p in sim.map['parking']:
            origin=np.array([p['x']+p['length']/2,.7,.05])
            distance=mujoco.mj_ray(sim.model,sim.data,origin,np.array([0.,-1.,0.]),None,1,-1,geom_id)
            self.assertGreater(distance,1.15)  # Open course-facing mouth.
            self.assertLess(distance,1.25)  # Stops at the bay's rear wall.
            self.assertTrue(sim.model.geom(int(geom_id[0])).name.startswith('parking_wall_'))

    def test_ramp_surface_heights_on_edges_and_corners(self):
        sim=self.make('minicar_2026')
        a,b,c,d=sim.map['ramp']['bounds']; h=sim.map['ramp']['height']; width=.05
        geom_id=np.array([-1],dtype=np.int32)
        samples=[(a+.025,(b+d)/2,h/2),(c-.025,(b+d)/2,h/2),
                 ((a+c)/2,b+.025,h/2),((a+c)/2,d-.025,h/2),
                 ((a+c)/2,(b+d)/2,h)]
        for x,y,sx,sy in ((a,b,1,1),(c,b,-1,1),(c,d,-1,-1),(a,d,1,-1)):
            for dx,dy in ((.0125,.025),(.025,.0125),(.025,.025)):
                samples.append((x+sx*dx,y+sy*dy,h*min(dx,dy)/width))
        for x,y,expected in samples:
            distance=mujoco.mj_ray(sim.model,sim.data,np.array([x,y,.1]),
                                   np.array([0.,0.,-1.]),None,1,-1,geom_id)
            self.assertAlmostEqual(.1-distance,expected,places=6)

    def test_vehicle_crosses_all_four_ramp_edges(self):
        # Cross both slopes and the plateau, approaching from each direction.
        for x,y,yaw,axis,target,sign in ((5.6,2.65,0,0,7.65,1),
                (7.9,2.65,math.pi,0,5.9,-1),
                (6.75,2.0,math.pi/2,1,3.3,1),
                (6.75,3.5,-math.pi/2,1,2.15,-1)):
            sim=self.make('minicar_2026')
            sim.data.qpos[:2]=[x+.1285*math.cos(yaw),y+.1285*math.sin(yaw)]
            sim.data.qpos[3:7]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
            mujoco.mj_forward(sim.model,sim.data)
            for _ in range(1000):
                sim.step(throttle=.7)
                if sign*(sim.state()[0][axis]-target)>0: break
            self.assertGreater(sign*(sim.state()[0][axis]-target),0)
            self.assertFalse(any(w.number for w in sim.data.warning))

    def test_invalid_map_rejected(self):
        doc=load_map(MAPS/'indoor_empty.json')
        doc['room']['height']=float('nan')
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'bad.json'; p.write_text(json.dumps(doc))
            with self.assertRaises(ValueError): load_map(p)

class CommandGateTests(unittest.TestCase):
    def test_mode_and_dual_clock_expiry(self):
        from kart_sim.safety import CommandGate
        gate=CommandGate()
        gate.accept_command((0,.5,0,0),1,1,10)
        self.assertEqual(gate.output(1,10),(0,0,0,0))
        gate.accept_mode(1,1,1,10)
        self.assertEqual(gate.output(1,10),(0,0,0,0))
        gate.accept_command((0,.5,0,0),1,1,10)
        self.assertEqual(gate.output(1.1,10.1),(0,.5,0,0))
        self.assertEqual(gate.output(1.21,10.1),(0,0,0,0))
        self.assertEqual(gate.output(1.1,10.21),(0,0,0,0))
        gate.accept_mode(3,1.1,1.1,10.1)
        self.assertEqual(gate.output(1.1,10.1),(0,0,0,0))

    def test_pause_reset_and_bad_stamps(self):
        from kart_sim.safety import CommandGate
        gate=CommandGate(require_mode=False)
        self.assertTrue(gate.accept_command((0,.5,0,0),1+1e-10,1,10))
        self.assertEqual(gate.output(1,10),(0,.5,0,0))
        self.assertFalse(gate.accept_command((0,.5,0,0),2,1,10))
        self.assertFalse(gate.accept_command((0,.5,0,0),0,1,10))
        self.assertFalse(gate.accept_command((0,.5,.5,0),1,1,10))
        gate.accept_command((0,.5,0,0),1,1,10); gate.invalidate_command()
        self.assertEqual(gate.output(1,10),(0,0,0,0))
        gate.accept_command((0,.5,0,0),1,1,10); gate.reset()
        self.assertEqual(gate.output(0,10),(0,0,0,0))

if __name__=='__main__': unittest.main()
