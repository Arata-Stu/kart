"""Experimental single-authority route router; never publishes actuator commands."""
from dataclasses import asdict, fields, replace
import hashlib
import json
import math
from pathlib import Path as FilePath
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from kart_interfaces.msg import GroundDetections, OpponentGroundPoints, OperationModeState, ReferenceLine
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from kart_hdmap.document import load
from kart_hdmap.tuning import StopGate
from .behavior import Planner, Settings
from .behavior_inputs import Motion, Observations, fresh, stamp_ns, transform, yaw


class BehaviorNode(Node):
    def __init__(self):
        super().__init__('racing_behavior')
        fixed = ParameterDescriptor(read_only=True)
        self.mode = self.declare_parameter('mode', 'shadow', fixed).value
        self.map_file = self.declare_parameter('map_file', '', fixed).value
        if self.mode not in ('shadow', 'active') or not FilePath(self.map_file).is_absolute():
            raise ValueError('mode=shadow/active and absolute map_file are required')
        defaults = Settings()
        names = [field.name for field in fields(Settings)]
        for name in names:
            self.declare_parameter(name, getattr(defaults, name))
        self.settings = Settings(**{name: self.get_parameter(name).value for name in names})
        self.planner = Planner(self.settings)
        self.inputs, self.ego_motion, self.gate = Observations(), Motion(), StopGate()
        self.document = load(self.map_file, 'map')
        self.document_at = time.monotonic()
        self.reference = None
        self.reference_at = self.pose_at = self.pose_stamp = None
        self.ego = None
        self.pending_pose = self.pending_rc = None
        self.signature = ''
        self.last_result = {}
        self.last_clock,self.started = None,False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.candidate = self.create_publisher(ReferenceLine, 'racing/candidate_reference', latched)
        self.control = (self.create_publisher(ReferenceLine, 'planning/reference_line', latched)
                        if self.mode == 'active' else None)
        self.status_pub = self.create_publisher(String, 'racing/behavior_status', 1)
        self.path_pub = self.create_publisher(Path, 'racing/behavior_path', 1)
        self.marker_pub = self.create_publisher(MarkerArray, 'racing/behavior_markers', 1)
        self.base_sub = self.create_subscription(ReferenceLine, 'baseline_reference', self.on_reference, latched)
        self.create_subscription(String, 'hdmap/document', self.on_document, latched)
        self.create_subscription(GroundDetections, '/racing/map_points', self.on_map, qos_profile_sensor_data)
        self.create_subscription(OpponentGroundPoints, '/racing/ground_points', self.on_rc, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(OperationModeState, '/operation_mode/state', self.on_mode, 1)
        self.create_service(Trigger, 'racing/behavior/reset', self.reset)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.add_post_set_parameters_callback(self.apply_parameters)
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.tick, clock=self.wall)

    def on_mode(self, msg):
        now, wall = self.get_clock().now().nanoseconds, time.monotonic()
        if fresh(stamp_ns(msg.header), wall, now, wall, .3):
            self.gate.update_mode(msg.mode)
        else:
            self.gate.mode = None

    def on_document(self, msg):
        try:
            if len(msg.data.encode()) > 3*1024*1024:
                raise ValueError('map_document_too_large')
            document = json.loads(msg.data)
            if document.get('schema') != 'kart.hdmap.visualization.v1' or document.get('frame_id') != 'map':
                raise ValueError('invalid_map_document')
            self.document, self.document_at = document, time.monotonic()
            self.configure_reference()
        except (ValueError, KeyError, TypeError) as error:
            self.document_at = None
            self.get_logger().warning(str(error), throttle_duration_sec=2.)

    def on_reference(self, msg):
        self.reference, self.reference_at = msg, time.monotonic()
        try:
            if (msg.header.frame_id != 'map' or not 2 <= len(msg.points) <= 5000
                    or len(msg.points) != len(msg.speeds) or not msg.lane_id):
                raise ValueError('invalid_baseline_reference')
            if any(not all(math.isfinite(v) for v in (p.x,p.y,p.z)) or abs(p.z)>1e-6 for p in msg.points):
                raise ValueError('invalid_baseline_geometry')
            self.configure_reference()
        except (ValueError, KeyError, TypeError) as error:
            self.reference_at = None
            self.get_logger().warning(str(error), throttle_duration_sec=2.)

    def configure_reference(self):
        if self.reference is None or self.document_at is None:
            return
        msg = self.reference
        points = [(p.x,p.y) for p in msg.points]
        value = dict(points=points, speeds=list(msg.speeds), closed=msg.closed, lane=msg.lane_id,
                     line=msg.line_type, document=self.document)
        signature = hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
        if signature == self.signature:
            return
        self.planner.set_reference(points,list(msg.speeds),msg.closed,self.document,msg.lane_id,signature,
                                   line_type=msg.line_type)
        self.signature = signature

    def on_map(self, msg):
        self.inputs.map_points(msg, time.monotonic())

    def on_rc(self, msg):
        self.retry_rc()
        if self.pending_rc is not None:
            return
        self.pending_rc=(msg,time.monotonic())
        self.retry_rc()

    def retry_rc(self):
        if self.pending_rc is None:
            return
        msg,received=self.pending_rc
        wall,now=time.monotonic(),self.get_clock().now().nanoseconds
        if not fresh(stamp_ns(msg.header),received,now,wall,self.settings.object_timeout):
            self.pending_rc,self.inputs.rc_valid=None,False
            return
        try:
            tf = self.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            self.inputs.rc_points(msg, tf, received)
            self.pending_rc=None
        except TransformException:
            pass  # A later steady tick retries without extending source/receipt age.
        except (ValueError, TypeError):
            self.pending_rc=None
            self.inputs.rc_valid = False

    def on_pose(self, msg):
        self.retry_pose()
        if self.pending_pose is not None:
            return
        self.pending_pose=(msg,time.monotonic())
        self.retry_pose()

    def retry_pose(self):
        if self.pending_pose is None:
            return
        msg,received=self.pending_pose
        now, wall, stamp = self.get_clock().now().nanoseconds, time.monotonic(), stamp_ns(msg.header)
        if not fresh(stamp,received,now,wall,.2):
            self.pending_pose=None
            self.ego = None
            return
        try:
            tf = self.buffer.lookup_transform('map','rear_axle',Time.from_msg(msg.header.stamp)).transform
            x,y,_ = transform((0.,0.,0.),tf)
            heading = yaw(tf)
            self.pending_pose=None
            if not self.ego_motion.add(stamp,x,y):
                return
            velocity = self.ego_motion.estimate(.1)
            if velocity is None:
                self.ego = None
                return
            speed = velocity[0]*math.cos(heading)+velocity[1]*math.sin(heading)
            if speed < -.05:
                raise ValueError('reverse_not_supported')
            self.ego = (x,y,heading,max(0.,speed))
            self.pose_stamp,self.pose_at = stamp,received
            self.gate.update_speed(speed)
        except TransformException:
            pass
        except ValueError:
            self.pending_pose=None
            self.ego = None

    def validate_parameters(self, parameters):
        try:
            changes = {p.name:p.value for p in parameters if p.name in asdict(self.settings)}
            if changes:
                issue = self.gate.issue()
                if issue:
                    raise ValueError(issue)
                replace(self.settings,**changes)
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False,reason=str(error))

    def apply_parameters(self, parameters):
        changes = {p.name:p.value for p in parameters if p.name in asdict(self.settings)}
        if changes:
            self.settings = replace(self.settings,**changes)
            # Geometry remains latched while parameters change. An explicit
            # stopped reset is needed before taking another route.
            self.planner.settings = self.settings
            if self.reference is None:
                self.planner = Planner(self.settings)
                self.signature = ''
                return
            try:
                self.planner.set_reference([(p.x,p.y) for p in self.reference.points],list(self.reference.speeds),
                    self.reference.closed,self.document,self.reference.lane_id,self.signature+':parameters',
                    line_type=self.reference.line_type)
                self.signature += ':parameters'
            except (ValueError,TypeError,AttributeError) as error:
                self.reference_at = None
                self.get_logger().error('parameter accepted, reference invalid: '+str(error))

    def reset(self, request, response):
        issue = self.gate.issue()
        now,wall = self.get_clock().now().nanoseconds,time.monotonic()
        if issue or self.ego is None or not fresh(self.pose_stamp,self.pose_at,now,wall,.2):
            response.success,response.message = False,issue or 'fresh_pose_required'
            return response
        if (self.reference is None or self.document_at is None
                or not fresh(stamp_ns(self.reference.header),self.reference_at,now,wall,2.5)
                or not 0 <= wall-self.document_at <= 2.5):
            response.success,response.message = False,'fresh_baseline_and_map_required'
            return response
        # Reset only near the normal line, not while stopped beside an object.
        try:
            points=[(p.x,p.y) for p in self.reference.points]
            segments=list(zip(points,points[1:]))+([(points[-1],points[0])] if self.reference.closed else [])
            segments=[(a,b) for a,b in segments if math.dist(a,b)>1e-9]
            if not segments:
                raise ValueError('invalid_baseline_geometry')
            def distance(segment):
                (ax,ay),(bx,by)=segment
                dx,dy=bx-ax,by-ay
                ratio=max(0.,min(1.,((self.ego[0]-ax)*dx+(self.ego[1]-ay)*dy)/(dx*dx+dy*dy)))
                return math.hypot(self.ego[0]-ax-ratio*dx,self.ego[1]-ay-ratio*dy)
            if min(map(distance,segments)) > .15:
                raise ValueError('基準ライン近傍へ戻してからresetしてください')
            planner = Planner(self.settings)
            msg=self.reference
            value=dict(points=points,speeds=list(msg.speeds),closed=msg.closed,lane=msg.lane_id,
                       line=msg.line_type,document=self.document)
            signature=hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
            planner.set_reference(points,list(msg.speeds),msg.closed,self.document,msg.lane_id,signature,
                                  line_type=msg.line_type)
            self.planner,self.signature,self.started = planner,signature,False
            response.success,response.message = True,'行動計画をリセットしました。AUTOは変更しません'
        except (ValueError, TypeError, AttributeError) as error:
            response.success,response.message = False,str(error)
        return response

    def issue(self, now, wall):
        if self.ego is None or not fresh(self.pose_stamp,self.pose_at,now,wall,.2):
            return 'pose_unavailable'
        if (self.reference is None or not self.signature or self.document_at is None
                or not fresh(stamp_ns(self.reference.header),self.reference_at,now,wall,2.5)
                or not 0 <= wall-self.document_at <= 2.5):
            return 'baseline_or_map_unavailable'
        if not fresh(self.inputs.map_stamp,self.inputs.map_at,now,wall,self.settings.object_timeout):
            return 'perception_timeout'
        if not self.inputs.map_valid:
            return 'projection_'+self.inputs.map_reason
        if self.inputs.raw_rc and (not self.inputs.rc_valid or not fresh(
                self.inputs.rc_stamp,self.inputs.rc_at,now,wall,self.settings.object_timeout)):
            return 'confirmed_rc_tracks_unavailable'
        if self.mode == 'active' and self.count_publishers('planning/reference_line') != 1:
            return 'reference_publisher_conflict'
        return ''

    def tick(self):
        self.retry_pose()
        self.retry_rc()
        now,wall = self.get_clock().now().nanoseconds,time.monotonic()
        issue = self.issue(now,wall)
        if self.last_clock is not None and now < self.last_clock:
            issue = 'clock_reversed'
        self.last_clock = now
        if issue:
            result = (self.planner.invalidate(issue) if self.started else
                      dict(state='FAULT',reason=issue,path=[],speeds=[],valid=False,target_speed=0.,candidates=[]))
        else:
            try:
                objects = self.inputs.objects(wall,self.settings.object_timeout,self.settings.static_memory_s)
                result = self.planner.update(wall,self.ego,objects,geometry_revision=self.signature)
                if result.get('state') != 'FAULT' and len(result.get('path',[])) >= 2:
                    self.started = True
                expired=self.issue(self.get_clock().now().nanoseconds,time.monotonic())
                if expired and self.started:
                    result=self.planner.invalidate(expired)
            except (ValueError, TypeError, KeyError) as error:
                result = (self.planner.invalidate('planner_error:'+str(error)[:128]) if self.started else
                          dict(state='FAULT',reason='planner_error:'+str(error)[:128],path=[],speeds=[],valid=False,
                               target_speed=0.,candidates=[]))
        self.last_result = result
        stamps = [v for v in (self.pose_stamp,self.inputs.map_stamp) if v is not None and v > 0]
        if self.inputs.raw_rc and self.inputs.rc_stamp:
            stamps.append(self.inputs.rc_stamp)
        source = min(stamps) if stamps else now
        result['processing_ms']=(time.monotonic()-wall)*1000.
        self.publish(result, source,self.get_clock().now().nanoseconds)

    def publish(self, result, source, now):
        message = ReferenceLine()
        message.header.frame_id = 'map'
        message.header.stamp = Time(nanoseconds=max(0,source)).to_msg()
        message.lane_id = self.reference.lane_id if self.reference else ''
        message.line_type = 'behavior'
        message.closed = False
        points,speeds = result.get('path',[]),result.get('speeds',[])
        valid_geometry = (isinstance(points,list) and isinstance(speeds,list) and len(points)==len(speeds)
                          and 2<=len(points)<=300 and all(isinstance(p,(list,tuple)) and len(p)==2
                          and all(type(v) in (int,float) and math.isfinite(v) for v in p) for p in points)
                          and all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in speeds))
        if valid_geometry:
            for (x,y),speed in zip(points,speeds):
                if not all(math.isfinite(v) for v in (x,y,speed)) or speed < 0:
                    message.points,message.speeds = [],[]
                    break
                message.points.append(Point(x=float(x),y=float(y),z=0.))
                message.speeds.append(min(float(speed),self.settings.speed_cap))
        self.candidate.publish(message)
        if self.control:
            self.control.publish(message)
        path = Path(header=message.header)
        for point in message.points:
            pose = PoseStamped(header=message.header)
            pose.pose.position,pose.pose.orientation.w = point,1.
            path.poses.append(pose)
        self.path_pub.publish(path)
        markers = MarkerArray()
        # Clear previous alternatives immediately on a source/planner fault.
        markers.markers.append(Marker(header=message.header,action=3))  # DELETEALL
        candidates=[] if result.get('state')=='FAULT' or not valid_geometry else result.get('candidates',[])[:2]
        for index,candidate in enumerate(candidates):
            marker = Marker(header=message.header,ns='candidate',id=index,type=Marker.LINE_STRIP,action=Marker.ADD)
            marker.pose.orientation.w = 1.
            marker.scale.x = .025
            feasible=candidate.get('feasible') and result.get('state')!='BLOCKED'
            marker.color.r,marker.color.g,marker.color.b,marker.color.a = ((.2,.7,1.,.8) if feasible else (1.,.25,.2,.7))
            marker.lifetime.nanosec = 300000000
            marker.points = [Point(x=float(p[0]),y=float(p[1]),z=.02) for p in candidate.get('path',[])[:300]]
            markers.markers.append(marker)
        self.marker_pub.publish(markers)
        payload = dict(result,mode=self.mode,frame_id='map',stamp_ns=str(source),published_at_ns=str(now),
                       valid=bool(message.points and result.get('state') != 'FAULT'),parameters=asdict(self.settings))
        if not valid_geometry:
            payload.update(state='FAULT',reason=result.get('reason','invalid_planner_path'),target_speed=0.,
                           valid=False,path=[])
        if payload.get('state') == 'FAULT':
            payload['candidates']=[dict(row,feasible=False,reason='計画無効: '+str(payload['reason']))
                                   for row in payload.get('candidates',[])[:2]]
        elif payload.get('state') == 'BLOCKED':
            payload['candidates']=[dict(row,feasible=False) for row in payload.get('candidates',[])[:2]]
        payload.pop('speeds',None)
        raw = json.dumps(payload,ensure_ascii=False,allow_nan=False,separators=(',',':'))
        if len(raw.encode()) <= 128*1024:
            self.status_pub.publish(String(data=raw))


def main():
    rclpy.init()
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
