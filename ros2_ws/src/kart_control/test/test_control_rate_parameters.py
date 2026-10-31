"""ROS integration: committed rate changes alter the actual output timer."""
import os
import shutil
import signal
import subprocess
import time
import unittest
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from kart_interfaces.msg import OperationModeState, ControlCommand
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError as error:
    raise unittest.SkipTest(f'ROS/generated interfaces unavailable: {error}') from error


class ControlRateTests(unittest.TestCase):
    def test_both_nodes_stop_gate_atomic_rejection_and_timer_update(self):
        if not shutil.which('ros2'): self.skipTest('ros2 unavailable')
        rclpy.init();self.addCleanup(rclpy.shutdown)
        for executable, topic, message in [('pure_pursuit','control/tracking_status',String),
                                           ('speed_controller','auto/control_cmd',ControlCommand)]:
            with self.subTest(node=executable):
                namespace=f'/rate_test_{os.getpid()}_{executable}'
                process=subprocess.Popen(['ros2','run','kart_control',executable,'--ros-args','-r',f'__ns:={namespace}'],
                                         stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
                node=Node('rate_client',namespace=namespace)
                try:
                    mode=[OperationModeState.STOP];speed=[0.];count=[0];enabled=[True]
                    mode_pub=node.create_publisher(OperationModeState,'operation_mode/state',10)
                    odom_pub=node.create_publisher(Odometry,'visual_slam/tracking/odometry',10)
                    def publish():
                        if not enabled[0]: return
                        m=OperationModeState();m.header.stamp=node.get_clock().now().to_msg();m.mode=mode[0];mode_pub.publish(m)
                        o=Odometry();o.header.stamp=m.header.stamp;o.header.frame_id='map';o.child_frame_id='rear_axle'
                        o.pose.pose.orientation.w=1.;o.twist.twist.linear.x=speed[0];odom_pub.publish(o)
                    timer=node.create_timer(.02,publish)
                    sub=node.create_subscription(message,topic,lambda _:count.__setitem__(0,count[0]+1),100)
                    setter=node.create_client(SetParametersAtomically,f'{executable}/set_parameters_atomically')
                    getter=node.create_client(GetParameters,f'{executable}/get_parameters')
                    self.assertTrue(setter.wait_for_service(timeout_sec=10))
                    self.assertTrue(getter.wait_for_service(timeout_sec=3))
                    def spin(seconds):
                        end=time.monotonic()+seconds
                        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
                    def call(client,request):
                        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                        self.assertTrue(future.done());return future.result()
                    def change(value,extra=None):
                        req=SetParametersAtomically.Request()
                        req.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in {'control_rate_hz':value,**(extra or {})}.items()]
                        return call(setter,req).result.successful
                    def current():
                        req=GetParameters.Request();req.names=['control_rate_hz']
                        return call(getter,req).values[0].double_value
                    def frequency():
                        count[0]=0;spin(1);return count[0]
                    spin(.5);initial=frequency()
                    self.assertTrue(change(20.));spin(.25);slower=frequency()
                    self.assertEqual(current(),20.)
                    self.assertLess(slower,initial*.65)
                    for invalid in (0.,9.,101.,float('nan'),50):self.assertFalse(change(invalid))
                    extra={'lookahead_m':-1.} if executable=='pure_pursuit' else {'kp':-1.}
                    self.assertFalse(change(100.,extra));self.assertEqual(current(),20.)
                    self.assertLess(frequency(),initial*.65)
                    mode[0]=OperationModeState.AUTO;spin(.2);self.assertFalse(change(100.))
                    mode[0]=OperationModeState.STOP;speed[0]=.1;spin(.2);self.assertFalse(change(100.))
                    speed[0]=0.;spin(.2);enabled[0]=False;spin(.5);self.assertFalse(change(100.))
                    enabled[0]=True;spin(.3);self.assertTrue(change(100.));spin(.25)
                    self.assertGreater(frequency(),slower*1.5)
                finally:
                    node.destroy_node()
                    if process.poll() is None:
                        os.killpg(process.pid,signal.SIGINT)
                        try:process.wait(timeout=3)
                        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
