"""ROS bridge. Simulation owns its clock; actuator input always expires."""
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_srvs.srv import Trigger, SetBool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from kart_interfaces.msg import ControlCommand, OperationModeState
from .core import Simulation
from .safety import CommandGate

def stamp(seconds):
    ns=round(seconds*1e9)
    return Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)

def transform(parent,child,xyz,quat,when):
    msg=TransformStamped(); msg.header.frame_id=parent; msg.child_frame_id=child; msg.header.stamp=when
    msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,xyz)
    msg.transform.rotation.w,msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z=map(float,quat)
    return msg

class SimNode(Node):
    def __init__(self):
        super().__init__('kart_sim')
        defaults={'map_file':'','asset_dir':'','camera_enabled':False,'viewer_enabled':False,
            'require_mode':True,'command_timeout_s':.2,'mode_timeout_s':.5,
            'step_count':10,'max_steering_rad':.45,'wheel_torque_nm':.025,
            'camera_rig_file':'','stereo_hz':60.0,'rgb_hz':30.0,'imu_hz':200.0,
            'imu_enabled':True,'publish_truth_tf':True}
        for name,value in defaults.items():
            self.declare_parameter(name,value,ParameterDescriptor(read_only=True))
        p={name:self.get_parameter(name).value for name in defaults}
        if not p['map_file']: raise ValueError('map_file is required')
        if p['command_timeout_s']<=0 or p['mode_timeout_s']<=0 or not 1<=p['step_count']<=100:
            raise ValueError('Invalid timeout / step_count')
        self.p=p
        assets=p['asset_dir'] or str(Path(get_package_share_directory('kart_sim'))/'assets')
        self.sim=Simulation(p['map_file'],assets,p['camera_enabled'],p['max_steering_rad'],p['wheel_torque_nm'],
            rig_file=p['camera_rig_file'] or None,stereo_hz=p['stereo_hz'],rgb_hz=p['rgb_hz'],
            imu_hz=p['imu_hz'],imu_enabled=p['imu_enabled'])
        self.gate=CommandGate(p["command_timeout_s"],p["mode_timeout_s"],p["require_mode"])
        self.paused=False
        self.clock_pub=self.create_publisher(ClockMessage,'/clock',10)
        self.odom_pub=self.create_publisher(Odometry,'sim/odometry',qos_profile_sensor_data)
        self.pose_pub=self.create_publisher(PoseStamped,'sim/ground_truth/pose',qos_profile_sensor_data)
        self.tf=TransformBroadcaster(self); self.static_tf=StaticTransformBroadcaster(self)
        static=[transform('base_link','rear_axle',[0,0,0],[1,0,0,0],stamp(0))]
        if p['publish_truth_tf']:
            static += [transform('sim_world','map',[0,0,0],[1,0,0,0],stamp(0)),
                       transform('map','odom',[0,0,0],[1,0,0,0],stamp(0))]
        self.image_pubs={}; self.info_pubs={}
        if p['camera_enabled'] or p['imu_enabled']:
            static += [transform(parent,child,xyz,quat,stamp(0))
                       for parent,child,xyz,quat in self.sim.rig.static_transforms()]
        if p['camera_enabled']:
            for name in self.sim.rig.offsets:
                suffix='image_rect_raw' if name.startswith('infra') else 'image_raw'
                self.image_pubs[name]=self.create_publisher(Image,f'realsense/{name}/{suffix}',qos_profile_sensor_data)
                self.info_pubs[name]=self.create_publisher(CameraInfo,f'realsense/{name}/camera_info',qos_profile_sensor_data)
        if p['imu_enabled']:
            self.imu_pub=self.create_publisher(Imu,'realsense/imu',qos_profile_sensor_data)
            self.truth_imu_pub=self.create_publisher(Imu,'sim/ground_truth/imu',qos_profile_sensor_data)
        self.static_tf.sendTransform(static)
        self.create_subscription(ControlCommand,'vehicle/control_cmd',self.on_command,1)
        self.create_subscription(OperationModeState,'operation_mode/state',self.on_mode,1)
        self.create_service(Trigger,'sim/reset',self.reset)
        self.create_service(SetBool,'sim/pause',self.pause)
        self.viewer=None
        if p['viewer_enabled']:
            from mujoco import viewer
            self.viewer=viewer.launch_passive(self.sim.model,self.sim.data)
            self.viewer.opt.geomgroup[4]=0; self.viewer.opt.geomgroup[5]=1
            a,b,c,d=self.sim.map['room']['bounds']
            self.viewer.cam.lookat[:]=[(a+c)/2,(b+d)/2,0]
            self.viewer.cam.distance=max(c-a,d-b); self.viewer.cam.elevation=-70
        period=self.sim.model.opt.timestep*p['step_count']
        self.timer=self.create_timer(period,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_state()
        self.get_logger().info(f"Map: {p['map_file']}; assumptions: {self.sim.map.get('assumptions',[])}")

    def on_command(self,msg):
        source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.gate.accept_command((msg.steering,msg.throttle,msg.brake,msg.reverse),
                                 source,self.sim.data.time,time.monotonic())

    def on_mode(self,msg):
        source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.gate.accept_mode(msg.mode,source,self.sim.data.time,time.monotonic())

    def tick(self):
        if self.paused: return
        samples=self.sim.step(*self.gate.output(self.sim.data.time,time.monotonic()),steps=self.p['step_count'],collect_sensors=True)
        self.publish_state()
        for frames in samples['images']:
            for name,frame in frames.items(): self.publish_camera(name,frame)
        for frame in samples['imu']: self.publish_imu(frame)
        if self.viewer is not None and self.viewer.is_running(): self.viewer.sync()

    def publish_state(self):
        when=stamp(self.sim.data.time)
        clock=ClockMessage(); clock.clock=when; self.clock_pub.publish(clock)
        position,quat,linear,angular=self.sim.state()
        msg=Odometry(); msg.header.stamp=when; msg.header.frame_id='sim_world'; msg.child_frame_id='base_link'
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,position)
        msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z=map(float,quat)
        msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z=map(float,linear)
        msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z=map(float,angular)
        self.odom_pub.publish(msg)
        pose=PoseStamped(); pose.header=msg.header; pose.pose=msg.pose.pose; self.pose_pub.publish(pose)
        if self.p['publish_truth_tf']:
            self.tf.sendTransform(transform('odom','base_link',position,quat,when))

    def publish_camera(self,name,frame):
        pixels=np.ascontiguousarray(frame.values['image'])
        image=Image(); image.header.stamp=stamp(frame.time); image.header.frame_id=self.sim.rig.frames[name]
        image.height,image.width=pixels.shape[:2]; image.encoding=frame.values['encoding']
        image.step=image.width*(1 if image.encoding=='mono8' else 3)
        image.data=pixels.tobytes(); self.image_pubs[name].publish(image)
        info=CameraInfo(); info.header=image.header; info.height=image.height; info.width=image.width
        info.k=frame.values['K'].ravel().tolist(); info.r=np.eye(3).ravel().tolist()
        info.p=self.sim.rig.projection(name).ravel().tolist(); info.distortion_model='plumb_bob'; info.d=[0.]*5
        self.info_pubs[name].publish(info)

    def publish_imu(self,frame):
        msg=Imu(); msg.header.stamp=stamp(frame['time']); msg.header.frame_id=frame['frame_id']
        msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z=map(float,frame['orientation'])
        msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=map(float,frame['angular_velocity'])
        msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=map(float,frame['acceleration'])
        # Exact simulator truth; no sensor noise or bias. Orientation is also provided.
        self.imu_pub.publish(msg); self.truth_imu_pub.publish(msg)

    def reset(self,request,response):
        self.sim.reset(); self.gate.reset()
        self.publish_state(); response.success=True; response.message='Reset to spawn; commands and mode invalidated; clock rewound.'
        return response

    def pause(self,request,response):
        self.paused=request.data; self.gate.invalidate_command()
        response.success=True; response.message='Paused' if self.paused else 'Resumed; requires fresh command'
        return response

    def destroy_node(self):
        if self.viewer is not None: self.viewer.close()
        self.sim.close(); return super().destroy_node()

def main():
    rclpy.init(); node=None
    try:
        node=SimNode(); rclpy.spin(node)
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
