"""Nominal D455 calibration and frame definitions. No ROS dependency."""
import json
import math
from pathlib import Path
import numpy as np
import mujoco
from .sensors import SensorConfig

OPTICAL_QUAT = (.5, -.5, .5, -.5)  # wxyz; optical right/down/forward -> mount forward/left/up

def rotation(quat):
    result=np.empty(9)
    mujoco.mju_quat2Mat(result,np.asarray(quat,float))
    return result.reshape(3,3)

def quaternion(matrix):
    result=np.empty(4)
    mujoco.mju_mat2Quat(result,np.ascontiguousarray(matrix).ravel())
    return result

def rpy_quaternion(rpy):
    roll,pitch,yaw=rpy
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return np.array([cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy])

class D455Rig:
    def __init__(self,path,stereo_hz=60.,rgb_hz=30.,imu_hz=200.):
        self.settings=json.loads(Path(path).read_text())
        p=self.settings
        for name,rate,low,high in [('stereo_hz',stereo_hz,30,90),('rgb_hz',rgb_hz,30,90),('imu_hz',imu_hz,30,1000)]:
            if not math.isfinite(rate) or not low<=rate<=high: raise ValueError(f'Invalid {name}')
        self.stereo_hz=float(stereo_hz); self.rgb_hz=float(rgb_hz); self.imu_hz=float(imu_hz)
        if (p['width'],p['height'])!=(424,240): raise ValueError('D455 profile must be 424x240')
        if not math.isfinite(p['baseline_m']) or p['baseline_m']<=0: raise ValueError('Invalid baseline')
        for name in ('mount_xyz','mount_rpy','rgb_xyz','imu_xyz','rgb_rpy','imu_rpy'):
            v=np.asarray(p[name],float)
            if v.shape!=(3,) or not np.isfinite(v).all(): raise ValueError('Invalid '+name)
        self.mount_quat=rpy_quaternion(p['mount_rpy'])
        self.mount_rotation=rotation(self.mount_quat)
        self.offsets={'infra1':np.zeros(3),'infra2':np.array([0,-p['baseline_m'],0]),'color':np.array(p['rgb_xyz'])}
        self.frames={'infra1':'camera_infra1_optical_frame','infra2':'camera_infra2_optical_frame','color':'camera_color_optical_frame'}
        self.K={}
        for name in self.offsets:
            key='stereo' if name.startswith('infra') else 'rgb'
            if key+'_intrinsics' in p:
                fx,fy,cx,cy=p[key+'_intrinsics']
            else:
                horizontal,vertical=p[key+'_fov_deg']
                if not 0<horizontal<180 or not 0<vertical<180: raise ValueError('Invalid FOV')
                fx=424/(2*math.tan(math.radians(horizontal)/2))
                fy=240/(2*math.tan(math.radians(vertical)/2))
                cx,cy=212.,120.
            if not np.isfinite([fx,fy,cx,cy]).all() or min(fx,fy)<=0: raise ValueError('Invalid intrinsics')
            self.K[name]=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],float)

    def camera_configs(self,base_offset):
        p=self.settings
        return [SensorConfig(name,'camera',pos=tuple(base_offset+np.array(p['mount_xyz'])+self.mount_rotation@offset),
                quat=tuple(quaternion(self.mount_rotation@rotation(rpy_quaternion(p['rgb_rpy']))) if name=='color' else self.mount_quat),hz=self.stereo_hz if name.startswith('infra') else self.rgb_hz,
                width=424,height=240,intrinsics=tuple(self.K[name][[0,1,0,1],[0,1,2,2]]),
                encoding='mono8' if name.startswith('infra') else 'rgb8') for name,offset in self.offsets.items()]

    def projection(self,name):
        P=np.c_[self.K[name],np.zeros(3)]
        if name=='infra2': P[0,3]=-self.K[name][0,0]*self.settings['baseline_m']
        return P

    def static_transforms(self):
        result=[('base_link','camera_link',self.settings['mount_xyz'],self.mount_quat)]
        for name,offset in self.offsets.items():
            frame='camera_'+name+'_frame'
            result += [('camera_link',frame,offset,rpy_quaternion(self.settings['rgb_rpy']) if name=='color' else (1,0,0,0)),(frame,self.frames[name],(0,0,0),OPTICAL_QUAT)]
        result += [('camera_link','camera_gyro_frame',self.settings['imu_xyz'],rpy_quaternion(self.settings['imu_rpy'])),
                   ('camera_gyro_frame','camera_gyro_optical_frame',(0,0,0),OPTICAL_QUAT),
                   ('camera_link','camera_accel_frame',self.settings['imu_xyz'],rpy_quaternion(self.settings['imu_rpy'])),
                   ('camera_accel_frame','camera_accel_optical_frame',(0,0,0),OPTICAL_QUAT)]
        return result
