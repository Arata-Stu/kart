"""Manual GL smoke: run with PYTHONPATH=ros2_ws/src/kart_sim; requires display/GL.

Not collected by pytest because the portable suite must work without GL.
"""
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from kart_sim.core import Simulation
from kart_sim.sensors import SensorConfig, SensorSuite, attach_sensors
root=Path(__file__).resolve().parents[4]
assets=Path(__file__).resolve().parents[1]/'assets'
s=Simulation(root/'maps/sim/minicar_2026.json',assets,camera=True)
t0=time.monotonic(); samples=s.step(steps=100,collect_sensors=True)
for name,frame in s.sensors.latest.items():
    im=frame.values['image']; assert im.shape==((240,424) if name.startswith('infra') else (240,424,3))
    assert im.dtype==np.uint8 and np.ptp(im)>30
assert np.mean(np.abs(s.sensors.latest['infra1'].values['image'].astype(float)-s.sensors.latest['infra2'].values['image']))>1
assert samples['images'][0]['infra1'].time==samples['images'][0]['infra2'].time
assert not np.shares_memory(s.sensors.latest['infra1'].values['rgb'],s.sensors.latest['color'].values['rgb'])
print('100ms sim elapsed wall',round(time.monotonic()-t0,3),'sample counts',{n:sum(n in x for x in samples['images']) for n in s.rig.offsets})
for _ in range(200): s.step()
with mujoco.Renderer(s.model,height=240,width=424) as renderer:
    renderer.enable_segmentation_rendering()
    for name in s.rig.offsets:
        renderer.update_scene(s.data,camera=name,scene_option=s.sensors._option)
        seg=renderer.render()
        ids=seg[seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM),0]
        # Vehicle rendering remains enabled. Correct placement must remove self-occlusion.
        assert np.all(s.model.geom_bodyid[ids]==0),f'{name}: vehicle is visible'
print('Settled 133mm lens height: no vehicle pixels in left/right/RGB')
s.close()
# Off-centre pinhole projection: emitting red sphere, optical point (x,y,z)=(.2,.1,2).
root=ET.fromstring('<mujoco><visual><headlight ambient="0 0 0" diffuse="0 0 0" specular="0 0 0"/></visual><asset><material name="red" rgba="1 0 0 1" emission="1"/></asset><worldbody><body name="chassis"/><geom type="sphere" pos="2 -.2 -.1" size=".015" material="red"/></worldbody></mujoco>')
cfg=SensorConfig('projection','camera',pos=(0,0,0),width=424,height=240,intrinsics=(220,215,200,110))
right=SensorConfig('right','camera',pos=(0,-.095,0),width=424,height=240,intrinsics=(220,215,200,110))
attach_sensors(root,[cfg,right]); model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode')); data=mujoco.MjData(model); mujoco.mj_forward(model,data)
with SensorSuite(model,[cfg,right]) as sensors:
    frames=sensors.update(data)
    im=frames['projection'].values['image']; y,x=np.where((im[:,:,0]>200)&(im[:,:,1]<20))
    assert len(x)>0
    print('projection expected',222,120.75,'observed',x.mean(),y.mean())
    assert abs(x.mean()-222)<1.2 and abs(y.mean()-120.75)<1.2
    im=frames['right'].values['image']; ry,rx=np.where((im[:,:,0]>200)&(im[:,:,1]<20))
    assert len(rx)>0
    print('disparity expected',220*.095/2,'observed',x.mean()-rx.mean())
    assert abs(x.mean()-rx.mean()-220*.095/2)<1.2
print('render checks passed')
