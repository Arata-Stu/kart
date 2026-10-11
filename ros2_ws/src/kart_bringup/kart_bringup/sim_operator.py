"""ROS sim pipeline supervisor. Fixed argv only; never runs HTTP-supplied shell text."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import socket
import time
from .mission import discover, hdmap_choices


def inventory(map_root, model_root):
    return {'hdmaps': [str(p) for p in discover(map_root,'hdmap')],
            'vslam_maps': [str(p) for p in discover(map_root,'vslam')],
            'vgl_bundles': [str(p) for p in discover(map_root,'bundle')],
            'models': sorted({str(p) for root in (model_root,map_root) for p in discover(root,'models')}),
            'lanes': {str(p):[list(x) for x in hdmap_choices(p)] for p in discover(map_root,'hdmap')}}


def validate_selection(value, assets):
    keys={'map_file','map_dir','model_dir','enable_vgl','lane_id','line_type'}
    if not isinstance(value,dict) or set(value)!=keys or any(not isinstance(v,str) for v in value.values()):
        raise ValueError('Expected map_file/map_dir/model_dir/enable_vgl/lane_id/line_type strings')
    if value['enable_vgl'] not in ('true','false'): raise ValueError('Invalid enable_vgl')
    paired=value['enable_vgl']=='true'
    if value['map_file'] not in assets['hdmaps'] or value['map_dir'] not in assets['vgl_bundles' if paired else 'vslam_maps']:
        raise ValueError('Choose discovered maps')
    if paired and value['model_dir'] not in assets['models']: raise ValueError('Choose discovered VGL model')
    if not paired and value['model_dir']: raise ValueError('VO localization does not need model_dir')
    if (value['lane_id'],value['line_type']) not in hdmap_choices(value['map_file']): raise ValueError('Unknown lane/line')
    return dict(value)


def pipeline_command(selection):
    return ['ros2','launch','kart_bringup','sim_drive.launch.py']+[f'{k}:={v}' for k,v in selection.items() if v]


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from kart_interfaces.msg import OperationModeRequest, OperationModeState

    class Operator(Node):
        def __init__(self):
            super().__init__('sim_operator')
            for key,default in [('map_root','/workspaces/map'),('model_root','/workspaces/models'),
                                ('heartbeat_timeout_s',2.),('status_period_s',.2),
                                ('studio_enabled',True),('studio_script','/workspaces/tools/app/start.sh'),('studio_port',8766)]:self.declare_parameter(key,default)
            self.timeout=float(self.get_parameter('heartbeat_timeout_s').value)
            period=float(self.get_parameter('status_period_s').value)
            if not math.isfinite(self.timeout) or not math.isfinite(period) or min(self.timeout,period)<=0:raise ValueError('Invalid operator timing')
            self.studio=None;self.studio_error='';self.studio_port=int(self.get_parameter('studio_port').value)
            if not 1<=self.studio_port<=65535:raise ValueError('Invalid studio_port')
            if self.get_parameter('studio_enabled').value:
                script=Path(self.get_parameter('studio_script').value)
                with socket.socket() as sock:
                    occupied=sock.connect_ex(('127.0.0.1',self.studio_port))==0
                if not occupied:
                    if script.is_file():self.studio=subprocess.Popen(['bash',str(script),'--port',str(self.studio_port)],start_new_session=True)
                    else:self.studio_error=f'Map Studio script missing: {script}'
            self.assets=inventory(self.get_parameter('map_root').value,self.get_parameter('model_root').value)
            self.child=None; self.pending=None; self.stopping=None; self.error=''; self.selection={}
            self.heartbeat=0.; self.sim_settings={}; self.localized=False; self.diag_time=0.; self.odom_time=0.; self.tracking=''; self.tracking_time=0.
            self.mode=3; self.mode_time=0.; self.line_request=None; self.line_future=None
            self.pub=self.create_publisher(String,'sim/operator/status',10)
            self.mode_pub=self.create_publisher(OperationModeRequest,'operation_mode/request',10)
            self.lane_pub=self.create_publisher(String,'hdmap/select_lane',1)
            self.line_client=self.create_client(SetParameters,'hdmap_server/set_parameters')
            self.create_subscription(String,'sim/operator/request',self.request,10)
            self.create_subscription(String,'sim/operator/heartbeat',self.beat,10)
            self.create_subscription(OperationModeState,'operation_mode/state',self.mode_cb,1)
            self.create_subscription(Odometry,'visual_slam/tracking/odometry',lambda _:setattr(self,'odom_time',time.monotonic()),qos_profile_sensor_data)
            self.create_subscription(String,'control/tracking_status',self.tracking_cb,1)
            self.create_subscription(DiagnosticArray,'localization/vslam/diagnostics',self.diag,10)
            self.timer=self.create_timer(period,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

        def beat(self,msg):
            try:
                settings=json.loads(msg.data)
                if not isinstance(settings,dict):return
                self.sim_settings=settings;self.heartbeat=time.monotonic()
            except (ValueError,TypeError):pass
        def sim_ready(self):
            return (time.monotonic()-self.heartbeat<self.timeout and self.sim_settings.get('camera_enabled') is True
                    and self.sim_settings.get('publish_truth_tf') is False
                    and isinstance(self.sim_settings.get('max_steering_rad'),(int,float))
                    and abs(self.sim_settings['max_steering_rad']-.45)<1e-6)
        def mode_cb(self,m):self.mode=m.mode; self.mode_time=time.monotonic()
        def tracking_cb(self,m):self.tracking=m.data; self.tracking_time=time.monotonic()
        def diag(self,msg):
            rows=[dict((v.key,v.value) for v in row.values) for row in msg.status if row.hardware_id=='visual_slam']
            self.localized=any(row.get('localized_in_exist_map')=='Yes' and row.get('vo_status')=='OK' for row in rows)
            self.diag_time=time.monotonic()
        def ready(self):
            now=time.monotonic()
            return bool(self.sim_ready() and self.child and self.child.poll() is None and self.stopping is None and self.localized
                        and now-self.diag_time<self.timeout and now-self.odom_time<self.timeout
                        and now-self.tracking_time<self.timeout and self.tracking in ('tracking','zero_speed_target')
                        and now-self.mode_time<self.timeout)
        def mode_request(self,mode):
            msg=OperationModeRequest();msg.header.stamp=self.get_clock().now().to_msg();msg.mode=mode;msg.source='sim_monitor'
            self.mode_pub.publish(msg)
        def request(self,msg):
            try:
                data=json.loads(msg.data);action=data['action'];value=data.get('value')
                if action=='kart_configure':
                    if not self.sim_ready():raise ValueError('Start ROS sim with camera_enabled=true, publish_truth_tf=false, max steering 0.45 rad')
                    selection=validate_selection(value,self.assets)
                    self.mode_request(3);self.pending=selection;self.shutdown_child()
                elif action=='kart_line':
                    if not self.child or self.child.poll() is not None or self.stopping is not None:raise ValueError('Pipeline is not running')
                    if self.line_future is not None:raise ValueError('Line change already pending')
                    if not isinstance(value,list) or len(value)!=2 or tuple(value) not in hdmap_choices(self.selection['map_file']):raise ValueError('Unknown lane/line')
                    self.mode_request(3)
                    if not self.line_client.service_is_ready():raise ValueError('HDMap parameter service unavailable')
                    req=SetParameters.Request(); req.parameters=[Parameter(name='line_type',value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=value[1]))]
                    self.line_request=value;self.line_future=self.line_client.call_async(req)
                elif action=='kart_auto':
                    if not self.ready() or time.monotonic()-self.heartbeat>self.timeout:raise ValueError('Wait for map localization and tracking readiness')
                    self.mode_request(1)
                elif action=='kart_stop':self.mode_request(3)
                elif action=='kart_unload':self.pending=None;self.mode_request(3);self.shutdown_child()
                elif action=='kart_refresh':self.assets=inventory(self.get_parameter('map_root').value,self.get_parameter('model_root').value)
                else:raise ValueError('Unknown operator action')
                self.error=''
            except (ValueError,KeyError,TypeError,OSError) as exc:self.error=str(exc);self.mode_request(3)
        def shutdown_child(self):
            if self.line_future is not None:self.line_future.cancel()
            self.line_future=None;self.line_request=None
            if self.child and self.child.poll() is None and self.stopping is None:
                try:os.killpg(self.child.pid,signal.SIGINT)
                except ProcessLookupError:pass
                self.stopping=time.monotonic()
        def tick(self):
            if self.line_future is not None and self.line_future.done():
                try:
                    result=self.line_future.result()
                    if not result.results or not all(r.successful for r in result.results):raise ValueError('HDMap rejected line change')
                    lane,line=self.line_request;self.lane_pub.publish(String(data=lane));self.selection.update(lane_id=lane,line_type=line)
                except Exception as exc:self.error=str(exc)
                self.line_future=None;self.line_request=None
            if self.child and self.child.poll() is not None:
                code=self.child.returncode;self.child=None;self.stopping=None;self.localized=False;self.mode=3
                if self.pending is None and code:self.error=f'Pipeline exited ({code}); see terminal 2'
            if self.child and self.stopping is not None and time.monotonic()-self.stopping>8:
                os.killpg(self.child.pid,signal.SIGKILL)
            if self.pending is not None and self.child is None:
                self.selection=self.pending;self.pending=None;self.localized=False;self.tracking='';self.diag_time=self.odom_time=self.tracking_time=self.mode_time=0.
                try:self.child=subprocess.Popen(pipeline_command(self.selection),start_new_session=True)
                except OSError as exc:self.error=str(exc)
            if self.mode==1 and (time.monotonic()-self.heartbeat>self.timeout or not self.ready()):self.mode_request(3)
            try:lines=[list(x) for x in hdmap_choices(self.selection['map_file'])] if self.selection else []
            except (OSError,ValueError,KeyError,TypeError) as exc:
                lines=[];self.error=str(exc);self.mode_request(3)
            state={'connected':True,'running':bool(self.child and self.child.poll() is None),'stopping':self.stopping is not None,
                   'ready':self.ready(),'sim_ready':self.sim_ready(),'mode':self.mode,'localized':self.localized,'tracking':self.tracking,
                   'studio_port':self.studio_port if self.get_parameter('studio_enabled').value else None,
                   'studio_error':self.studio_error if not self.studio or self.studio.poll() is None else f'Studio exited ({self.studio.returncode})',
                   'selection':self.selection,'assets':self.assets,'error':self.error,
                   'lines':lines}
            self.pub.publish(String(data=json.dumps(state)))

    rclpy.init(args=args);node=Operator()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.mode_request(3)
        node.pending=None;node.shutdown_child()
        if node.child:
            try:node.child.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(node.child.pid,signal.SIGKILL);node.child.wait(timeout=2)
        if node.studio and node.studio.poll() is None:
            os.killpg(node.studio.pid,signal.SIGINT)
            try:node.studio.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(node.studio.pid,signal.SIGKILL);node.studio.wait(timeout=2)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
