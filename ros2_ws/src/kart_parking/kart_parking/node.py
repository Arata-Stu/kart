"""Parking reference owner. Planning runs off-executor; no actuator publisher."""
import concurrent.futures
import copy
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from diagnostic_msgs.msg import DiagnosticArray
from kart_interfaces.msg import ReferenceLine, OperationModeState, OperationModeRequest
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from .geometry import wrap
from .planner import Settings, load_site, prepare, activate
from .mission import Mission


def yaw(q):
    values=(q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1)>.01:
        raise ValueError('Invalid quaternion')
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class ParkingNode(Node):
    def __init__(self):
        super().__init__('parking_planner')
        defaults=dict(site_file='',slot='P1',map_frame='map',tracking_frame='rear_axle',
                      input_timeout_s=.3,line_timeout_s=2.5,localization_timeout_s=2.5,
                      publish_rate_hz=20.,**vars(Settings()))
        for name,value in defaults.items():
            self.declare_parameter(name,value,ParameterDescriptor(read_only=True))
        self.p={k:self.get_parameter(k).value for k in defaults}
        self.c=Settings(**{k:self.p[k] for k in vars(Settings())}).validate()
        for key in ('input_timeout_s','line_timeout_s','localization_timeout_s','publish_rate_hz'):
            if not math.isfinite(self.p[key]) or self.p[key]<=0:
                raise ValueError(key+' must be positive')
        for key in ('map_frame','tracking_frame'):
            if not self.p[key] or self.p[key].startswith('/'):
                raise ValueError('Invalid frame')
        self.world,self.zone=load_site(self.p['site_file'],self.p['map_frame'],self.p['slot'],self.c)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output=self.create_publisher(ReferenceLine,'planning/reference_line',qos)
        self.preview=self.create_publisher(Path,'parking/path',qos)
        self.status=self.create_publisher(String,'parking/status',1)
        self.stop=self.create_publisher(OperationModeRequest,'operation_mode/request',10)
        self.create_subscription(ReferenceLine,'planning/course_reference',self.line,qos)
        self.create_subscription(Odometry,'visual_slam/tracking/odometry',lambda m:self.receive('odom',m),qos_profile_sensor_data)
        self.create_subscription(OperationModeState,'operation_mode/state',lambda m:self.receive('mode',m),1)
        self.create_subscription(DiagnosticArray,'localization/vslam/diagnostics',self.diagnostics,10)
        self.create_subscription(PoseStamped,'parking/goal',self.goal,1)
        self.create_service(Trigger,'parking/start',self.start)
        self.create_service(Trigger,'parking/reset',self.reset)
        self.inputs={}; self.base=None; self.signature=None; self.prepared=None
        self.mission=None; self.error='waiting_for_course'; self.localized=False
        self.pool=concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.future=None; self.generation=0; self.future_generation=-1
        self.last_clock=None; self.locked=False
        self.create_timer(1/self.p['publish_rate_hz'],self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Parking settings: '+json.dumps(self.p))

    def receive(self,key,msg):
        self.inputs[key]=(msg,time.monotonic())

    def fresh(self,key,timeout):
        if key not in self.inputs: return False
        m,received=self.inputs[key]
        stamp=rclpy.time.Time.from_msg(m.header.stamp).nanoseconds/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        return stamp>0 and -.05<=age<=timeout and time.monotonic()-received<=timeout

    def diagnostics(self,msg):
        found=False; ok=False
        for s in msg.status:
            if s.hardware_id=='visual_slam':
                found=True
                values={v.key:v.value for v in s.values}
                ok=values.get('localized_in_exist_map')=='Yes' and values.get('vo_status')=='OK'
        if found:
            self.localized=ok; self.receive('diag',msg)

    def line(self,msg):
        self.receive('line',msg)
        signature=(msg.header.frame_id,msg.lane_id,msg.line_type,msg.closed,
                   tuple((p.x,p.y,p.z) for p in msg.points),tuple(msg.speeds))
        self.base=copy.deepcopy(msg)  # Refresh only from the actual source heartbeat.
        if signature==self.signature: return
        self.signature=signature
        self.generation+=1; self.prepared=None
        if self.mission:
            self.mission.fault('course_changed'); self.locked=True
        self.error='preparing'

    def submit(self):
        if self.base is None or self.prepared is not None or self.future is not None or self.error!='preparing': return
        m=self.base
        if m.header.frame_id!=self.p['map_frame'] or any(abs(p.z)>1e-6 or not math.isfinite(p.z) for p in m.points):
            self.error='invalid_course_frame_or_height'; return
        self.future_generation=self.generation
        self.future=self.pool.submit(prepare,[(p.x,p.y) for p in m.points],list(m.speeds),m.closed,
                                     copy.deepcopy(self.zone),self.world,self.c)

    def stopped_mode(self):
        return self.fresh('mode',self.p['input_timeout_s']) and self.inputs['mode'][0].mode==OperationModeState.STOP

    def goal(self,msg):
        try:
            if not self.stopped_mode() or self.mission or self.locked:
                raise ValueError('Goal change requires STOP and parking/reset')
            if (msg.header.frame_id!=self.p['map_frame'] or not math.isfinite(msg.pose.position.z)
                    or abs(msg.pose.position.z)>1e-6
                    or abs(msg.pose.orientation.x)+abs(msg.pose.orientation.y)>1e-6):
                raise ValueError('Goal must be planar in map frame')
            p=msg.pose.position; goal=(p.x,p.y,yaw(msg.pose.orientation))
            if not all(math.isfinite(v) for v in goal) or not self.world.clear(goal,self.zone['area']) or not self.world.clear(goal,self.zone['maneuver']):
                raise ValueError('Goal footprint/clearance outside selected slot')
            self.zone=dict(self.zone,goal=goal); self.generation+=1
            self.prepared=None; self.error='preparing'
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def measurement(self):
        if not self.fresh('odom',self.p['input_timeout_s']): raise ValueError('odom_stale')
        if not self.localized or not self.fresh('diag',self.p['localization_timeout_s']): raise ValueError('not_localized')
        m=self.inputs['odom'][0]
        stamp=rclpy.time.Time.from_msg(m.header.stamp)
        t=self.buffer.lookup_transform(self.p['map_frame'],self.p['tracking_frame'],stamp)
        q=t.transform.rotation; p=t.transform.translation
        pose=(p.x,p.y,yaw(q))
        # A rigid-body stop requires both translation and angular velocity small;
        # use full child-origin speed plus |omega|*lever arm as a conservative
        # upper bound for rear-axle speed. No frame relabeling of twist is needed.
        lever=self.buffer.lookup_transform(self.p['tracking_frame'],m.child_frame_id,stamp)
        yaw(lever.transform.rotation)
        v=m.twist.twist.linear; w=m.twist.twist.angular; r=lever.transform.translation
        values=(*pose,v.x,v.y,v.z,w.x,w.y,w.z,r.x,r.y,r.z)
        if not all(math.isfinite(x) for x in values): raise ValueError('nonfinite_measurement')
        omega=math.sqrt(w.x*w.x+w.y*w.y+w.z*w.z)
        speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)+omega*math.sqrt(r.x*r.x+r.y*r.y+r.z*r.z)
        return pose,speed,omega

    def start(self,request,response):
        try:
            if self.locked or self.mission: raise ValueError('Reset in STOP before another mission')
            if self.prepared is None: raise ValueError(self.error)
            if not self.fresh('line',self.p['line_timeout_s']): raise ValueError('course_stale')
            if not self.fresh('mode',self.p['input_timeout_s']) or self.inputs['mode'][0].mode not in (OperationModeState.STOP,OperationModeState.AUTO):
                raise ValueError('Fresh STOP or AUTO required')
            pose,_,_=self.measurement()
            route=activate(self.prepared,pose,self.c)
            self.mission=Mission(route,self.zone,self.world,self.c,len(route)-len(self.prepared.connector))
            self.publish_path(route)
            response.success=True; response.message='Parking armed; forward course to selected bay. AUTO remains operator-owned.'
        except Exception as exc:
            response.success=False; response.message=str(exc)
        return response

    def reset(self,request,response):
        if not self.stopped_mode():
            response.success=False; response.message='Fresh STOP required'; return response
        self.mission=None; self.locked=False
        self.prepared=None; self.generation+=1; self.error='preparing'
        self.publish_path([])
        response.success=True; response.message='Parking cleared; normal course output resumes in STOP'
        return response

    def publish_path(self,route):
        msg=Path(); msg.header.frame_id=self.p['map_frame']; msg.header.stamp=self.get_clock().now().to_msg()
        for x,y,a,_ in route:
            p=PoseStamped(); p.header=msg.header; p.pose.position.x=x; p.pose.position.y=y
            p.pose.orientation.z=math.sin(a/2); p.pose.orientation.w=math.cos(a/2); msg.poses.append(p)
        self.preview.publish(msg)

    def request_stop(self,reason):
        # Retain the latch even if the operator subsequently presses AUTO.
        m=OperationModeRequest(); m.header.stamp=self.get_clock().now().to_msg()
        m.mode=OperationModeRequest.STOP; m.source='parking:'+reason; self.stop.publish(m)

    def tick(self):
        now=self.get_clock().now(); seconds=now.nanoseconds/1e9
        if self.last_clock is not None and seconds<self.last_clock:
            self.prepared=None; self.generation+=1; self.error='preparing'
            if self.mission: self.mission.fault('clock_jump'); self.locked=True
        self.last_clock=seconds
        if self.future is not None and self.future.done():
            try:
                result=self.future.result()
                if self.future_generation==self.generation:
                    self.prepared=result; self.error='ready'
            except Exception as exc:
                if self.future_generation==self.generation: self.error=str(exc)
            self.future=None
        self.submit()
        out=ReferenceLine(); out.header.frame_id=self.p['map_frame']; out.header.stamp=now.to_msg()
        state=self.error
        if self.mission:
            try:
                if not self.fresh('line',self.p['line_timeout_s']): raise ValueError('course_stale')
                if self.count_publishers('planning/reference_line')!=1: raise ValueError('reference_publisher_conflict')
                if not self.fresh('mode',self.p['input_timeout_s']): raise ValueError('mode_stale')
                pose,speed,omega=self.measurement()
                self.mission.update(pose,speed,omega,seconds,self.inputs['mode'][0].mode==OperationModeState.AUTO)
            except Exception as exc:
                self.mission.fault(str(exc))
            state=self.mission.state
            out.lane_id=self.p['slot']; out.closed=False
            approaching=self.mission.approaching_parking()
            out.line_type='parking' if approaching else self.base.line_type
            for x,y,_,v in self.mission.window():
                out.points.append(Point(x=float(x),y=float(y),z=0.)); out.speeds.append(float(v))
            if state in ('settling','parked','failed'):
                self.locked=True
                self.request_stop(self.mission.reason or state)
                # Empty reference invalidates pursuit; speed controller requests
                # STOP too. Do not restart torque if pose drifts out of tolerance.
        elif self.base is not None and self.fresh('line',self.p['line_timeout_s']):
            out=copy.deepcopy(self.base)  # Preserve source stamp; never rejuvenate stale input.
        self.output.publish(out)
        self.status.publish(String(data=json.dumps(dict(state=state,slot=self.p['slot'],
            goal=self.zone['goal'],reason=self.mission.reason if self.mission else self.error,
            progress_index=self.mission.index if self.mission else None,
            parked=state=='parked',parking_speed_mps=self.c.parking_speed_mps))))

    def destroy_node(self):
        self.pool.shutdown(wait=False,cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=ParkingNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
