"""Image-stamped section selector and offline map-pose recorder; never commands actuators."""
import json
import math
from pathlib import Path
import time
from collections import deque


def localized(statuses):
    """cuVSLAM diagnostics must explicitly confirm registered-map localization."""
    return any(s.level==0 and {v.key:v.value for v in s.values}.get('vo_status')=='OK'
               and {v.key:v.value for v in s.values}.get('localized_in_exist_map')=='Yes' for s in statuses)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.duration import Duration
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import CameraInfo
    from kart_interfaces.msg import E2EHeadSelection
    from tf2_ros import Buffer, TransformListener, TransformException
    from .sections import Router, load
    from .contract import routing_settings

    class Selector(Node):
        def __init__(self):
            super().__init__('e2e_section_selector')
            names=dict(model_dir='',routing_file='',trajectory_output='',
                       camera_info_topic='/realsense/color/camera_info',
                       diagnostics_topic='/localization/vslam/diagnostics',
                       output_topic='/e2e/head_selection',diagnostics_timeout_s=1.5)
            values={k:self.declare_parameter(k,v).value for k,v in names.items()}
            config=load(values['routing_file']) if values['routing_file'] else routing_settings(values['model_dir'])
            if config is None: raise ValueError('Section routing required')
            self.router=Router(config);self.config=config
            self.buffer=Buffer(cache_time=Duration(seconds=10.))
            self.listener=TransformListener(self.buffer,self)
            self.good=False;self.diagnostic_at=0.;self.timeout=values['diagnostics_timeout_s']
            if not math.isfinite(self.timeout) or self.timeout<=0: raise ValueError('Invalid diagnostics timeout')
            self.last_stamp=0
            self.stream=None
            if values['trajectory_output']:
                path=Path(values['trajectory_output']);path.parent.mkdir(parents=True,exist_ok=True)
                self.stream=path.open('x')
            self.publisher=self.create_publisher(E2EHeadSelection,values['output_topic'],10)
            self.diag=self.create_subscription(DiagnosticArray,values['diagnostics_topic'],self.diagnostics,10)
            self.pending=deque(maxlen=128)
            self.timer=self.create_timer(.01,self.process)
            self.images=self.create_subscription(CameraInfo,values['camera_info_topic'],self.image,qos_profile_sensor_data)

        def diagnostics(self,msg):
            self.good=localized(msg.status)
            self.diagnostic_at=time.monotonic()
            if not self.good:self.router.reset()

        def image(self,msg):
            if len(self.pending)==self.pending.maxlen:
                self.good=False
            self.pending.append((msg,time.monotonic()))

        def process(self):
            while self.pending:
                msg,received=self.pending[0]
                ready=self.buffer.can_transform(self.config['frame'],self.config['base_frame'],Time.from_msg(msg.header.stamp))
                if not ready and time.monotonic()-received<self.config['max_pose_age_s']:
                    return
                self.pending.popleft()
                self.select_image(msg)

        def select_image(self,msg):
            ns=Time.from_msg(msg.header.stamp).nanoseconds
            state=E2EHeadSelection();state.header=msg.header;state.head_index=-1
            state.routing_sha256=self.router.sha
            row=dict(stamp_ns=ns,frame=self.config['frame'],base_frame=self.config['base_frame'],routing_sha256=self.router.sha,valid=False)
            reason='unlocalized_or_stale_diagnostics'
            try:
                if ns<=self.last_stamp:
                    self.router.reset()
                    raise ValueError('nonmonotonic_image_stamp')
                self.last_stamp=ns
                if not self.good or time.monotonic()-self.diagnostic_at>self.timeout:
                    raise ValueError(reason)
                # Latest TF is never substituted for the requested image time.
                transform=self.buffer.lookup_transform(self.config['frame'],self.config['base_frame'],Time.from_msg(msg.header.stamp))
                p,q=transform.transform.translation,transform.transform.rotation
                norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
                if not math.isfinite(norm) or abs(norm-1.)>.01: raise ValueError('invalid_quaternion')
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                if not all(math.isfinite(v) for v in (p.x,p.y,yaw)): raise ValueError("nonfinite_pose")
                row.update(x=p.x,y=p.y,yaw=yaw)
                result=self.router.select(p.x,p.y,yaw)
                if result is None: raise ValueError('outside_ambiguous_or_discontinuous_section')
                row.update(result,valid=True)
                state.valid=True;state.section_id=result['section_id'];state.head_id=result['head_id'];state.head_index=result['head_index']
                reason='selected'
            except (TransformException, ValueError) as error:
                reason=str(error)
            state.reason=reason;row['reason']=reason
            self.publisher.publish(state)
            if self.stream:
                self.stream.write(json.dumps(row,allow_nan=False)+'\n');self.stream.flush()

        def destroy_node(self):
            if self.stream:self.stream.close()
            return super().destroy_node()

    rclpy.init()
    node=Selector()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__': main()
