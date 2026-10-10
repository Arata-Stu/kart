"""MuJoCo vehicle and world, independently usable without ROS."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .cad import vehicle_xml
from .vehicle import Parameters, control
from .world import load_map, add_world
from .sensors import SensorSuite, attach_sensors
from .rig import D455Rig, OPTICAL_QUAT, rotation, quaternion, rpy_quaternion

CAMERA_XYZ = (.23385, .04750, .10000)  # ~133 mm above floor with nominal 33 mm tires.

class Simulation:
    def __init__(self, map_file, asset_root, camera=False, max_steer=.45, wheel_torque=.025,
                 rig_file=None, stereo_hz=60., rgb_hz=30., imu_hz=200., imu_enabled=True):
        if not 0 < max_steer <= .6 or not math.isfinite(wheel_torque) or wheel_torque <= 0:
            raise ValueError('Invalid actuator calibration')
        self.parameters=Parameters(max_steer=max_steer,wheel_torque=wheel_torque)
        self.map=load_map(map_file)
        root=vehicle_xml(asset_root,self.parameters)
        add_world(root,self.map)
        chassis=root.find("./worldbody/body[@name='chassis']")
        rear=chassis.find("body[@name='knuckle_rl']")
        self.base_offset=np.array([-.5*self.parameters.wheelbase,0,float(rear.get('pos').split()[2])])
        self.rig=D455Rig(rig_file or Path(asset_root)/'d455.json',stereo_hz,rgb_hz,imu_hz)
        self.sensor_configs=self.rig.camera_configs(self.base_offset) if camera else []
        attach_sensors(root,self.sensor_configs)
        self.imu_enabled=imu_enabled
        if imu_enabled:
            rig=self.rig
            pos=self.base_offset+np.array(rig.settings['mount_xyz'])+rig.mount_rotation@rig.settings['imu_xyz']
            quat=quaternion(rig.mount_rotation@rotation(rpy_quaternion(rig.settings['imu_rpy']))@rotation(OPTICAL_QUAT))
            ET.SubElement(chassis,'site',name='d455_imu',pos=' '.join(map(str,pos)),
                          quat=' '.join(map(str,quat)),size='.001',rgba='0 0 0 0')
            sensor=ET.SubElement(root,'sensor')
            ET.SubElement(sensor,'accelerometer',name='d455_accel',site='d455_imu')
            ET.SubElement(sensor,'gyro',name='d455_gyro',site='d455_imu')
        self.model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        self.data=mujoco.MjData(self.model)
        self.chassis_id=self.model.body('chassis').id
        self.sensors=SensorSuite(self.model,self.sensor_configs)
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model,self.data)
        x,y,yaw=self.map['spawn']
        rotation=np.array([[math.cos(yaw),-math.sin(yaw),0],
                           [math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        # spawn denotes rear axle XY, not chassis centre.
        self.data.qpos[:2]=np.array([x,y])-(rotation@self.base_offset)[:2]
        self.data.qpos[3:7]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
        self.sensors.reset()
        self._imu_due=0.0
        mujoco.mj_forward(self.model,self.data)

    def step(self, steering=0, throttle=0, brake=0, reverse=0, steps=10, collect_sensors=False):
        values=np.array([steering,throttle,brake,reverse],float)
        if not np.isfinite(values).all() or abs(steering)>1 or np.any(values[1:]<0) or np.any(values[1:]>1) or np.count_nonzero(values[1:]>0)>1:
            raise ValueError('Invalid normalized ControlCommand')
        if not isinstance(steps,int) or steps<1: raise ValueError('Invalid step count')
        samples={"images":[],"imu":[]}
        for _ in range(steps):
            control(self.model,self.data,throttle-reverse,steering,self.parameters)
            if brake:
                # Simplified dissipative torque opposing each wheel's angular velocity.
                # Never reinterpret braking as reverse; not a hardware ESC model.
                for wheel in ('fl','fr','rl','rr'):
                    speed=self.data.qvel[self.model.joint('spin_'+wheel).dofadr[0]]
                    self.data.ctrl[self.model.actuator('drive_'+wheel).id]=-brake*math.tanh(speed/.5)
            mujoco.mj_step(self.model,self.data)
            if collect_sensors:
                now=float(self.data.time)
                imu_due=self.imu_enabled and now+1e-9>=self._imu_due
                if imu_due or self.sensors.due(now):
                    mujoco.mj_forward(self.model,self.data)
                    frames=self.sensors.update(self.data)
                    if frames: samples['images'].append(frames)
                    if imu_due:
                        samples['imu'].append(self.imu_state())
                        self._imu_due=(math.floor((now+1e-9)*self.rig.imu_hz)+1)/self.rig.imu_hz
        mujoco.mj_forward(self.model,self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError('Non-finite physics state')
        return samples

    def state(self):
        R=self.data.xmat[self.chassis_id].reshape(3,3)
        position=self.data.xpos[self.chassis_id]+R@self.base_offset
        quat=self.data.xquat[self.chassis_id].copy()  # wxyz
        velocity=np.empty(6)
        # mj_objectVelocity uses the body inertial COM, not the body-frame origin.
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,self.chassis_id,velocity,0)
        omega=velocity[:3]
        linear=velocity[3:]+np.cross(omega,position-self.data.xipos[self.chassis_id])
        return position.copy(),quat,R.T@linear,R.T@omega

    def imu_state(self):
        if not self.imu_enabled: raise ValueError('IMU is disabled')
        site=self.model.site('d455_imu').id
        R=self.data.site_xmat[site].reshape(3,3)
        return {'time':float(self.data.time),'frame_id':'camera_gyro_optical_frame',
                'acceleration':self.data.sensor('d455_accel').data.copy(),
                'angular_velocity':self.data.sensor('d455_gyro').data.copy(),
                'orientation':quaternion(R),'position_world':self.data.site_xpos[site].copy()}

    def close(self):
        self.sensors.close()
