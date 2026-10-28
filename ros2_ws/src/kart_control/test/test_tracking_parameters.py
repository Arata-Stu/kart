"""Native ROS parameter service check; skipped without ROS/generated interfaces."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import unittest

try:
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from rclpy.parameter import Parameter
    from kart_interfaces.msg import OperationModeState
    from nav_msgs.msg import Odometry
except ImportError as error:
    raise unittest.SkipTest(f'ROS/generated interfaces unavailable: {error}') from error


class TrackingParametersTest(unittest.TestCase):
    def test_native_set_requires_fresh_mode_and_preserves_rejected_values(self):
        if not shutil.which('ros2'):
            self.skipTest('ros2 executable unavailable')
        namespace=f'/tracking_parameter_test_{os.getpid()}'
        process=subprocess.Popen(['ros2','run','kart_control','pure_pursuit','--ros-args','-r',f'__ns:={namespace}'],
                                 stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        def cleanup():
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
        self.addCleanup(cleanup)
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node=Node('parameter_test_client',namespace=namespace)
        self.addCleanup(node.destroy_node)
        pub=node.create_publisher(OperationModeState,'operation_mode/state',1)
        odom_pub=node.create_publisher(Odometry,'visual_slam/tracking/odometry',10)
        measured=[0.]
        mode=[None]
        def publish():
            if mode[0] is not None:
                message=OperationModeState();message.header.stamp=node.get_clock().now().to_msg();message.mode=mode[0]
                pub.publish(message)
                odom=Odometry();odom.header.stamp=message.header.stamp
                odom.header.frame_id='map';odom.child_frame_id='rear_axle'
                odom.pose.pose.orientation.w=1.;odom.twist.twist.linear.x=measured[0]
                odom_pub.publish(odom)
        timer=node.create_timer(.03,publish)
        setter=node.create_client(SetParametersAtomically,'pure_pursuit/set_parameters_atomically')
        getter=node.create_client(GetParameters,'pure_pursuit/get_parameters')
        self.assertTrue(setter.wait_for_service(timeout_sec=5),'pure_pursuit parameter service did not start')
        self.assertTrue(getter.wait_for_service(timeout_sec=3))
        def call(client,request):
            future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=3)
            self.assertTrue(future.done())
            return future.result()
        def change(value):
            return change_many({'lookahead_m':value})
        def change_many(values):
            request=SetParametersAtomically.Request()
            request.parameters=[Parameter(name,value=value).to_parameter_msg() for name,value in values.items()]
            return call(setter,request).result
        self.assertFalse(change(1.1).successful)
        mode[0]=OperationModeState.STOP
        end=time.monotonic()+.3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        self.assertTrue(change(1.1).successful)
        self.assertFalse(change(-1.).successful)
        request=GetParameters.Request();request.names=['lookahead_m']
        self.assertEqual(call(getter,request).values[0].double_value,1.1)
        mode[0]=OperationModeState.AUTO
        end=time.monotonic()+.15
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        self.assertTrue(change(.9).successful)  # Fresh AUTO permits live tracking adjustment.
        self.assertEqual(call(getter,request).values[0].double_value,.9)
        self.assertFalse(change_many({'lookahead_low_speed_m':3.,'lookahead_high_speed_m':2.}).successful)
        request.names=['lookahead_low_speed_m','lookahead_high_speed_m']
        self.assertEqual([v.double_value for v in call(getter,request).values],[.8,2.])
        self.assertTrue(change_many({'lookahead_low_speed_m':2.5,'lookahead_high_speed_m':3.}).successful)
        self.assertTrue(change_many({'lookahead_low_speed_m':.8,'lookahead_high_speed_m':2.,
                                    'lookahead_mode':'speed_interpolated'}).successful)
        for invalid in (-1.,0.,float('nan'),True,1):
            self.assertFalse(change_many({'lookahead_low_speed_m':invalid}).successful)
        self.assertFalse(change_many({'lookahead_mode':'unknown'}).successful)
        request.names=['lookahead_mode']
        self.assertEqual(call(getter,request).values[0].string_value,'speed_interpolated')
        self.assertTrue(change_many({'lookahead_mode':'fixed'}).successful)
        self.assertTrue(change_many({'mpc_horizon_s':1.2,'mpc_tracking_weight':12.}).successful)
        self.assertFalse(change_many({'mpc_horizon_s':2.1}).successful)
        self.assertFalse(change_many({'mpc_smoothness_weight':0.}).successful)
        self.assertFalse(change_many({'lateral_controller':'invalid'}).successful)
        self.assertFalse(change_many({'lateral_controller':'mpc'}).successful)  # AUTO switch prohibited
        request.names=['lateral_controller','mpc_horizon_s']
        result=call(getter,request)
        self.assertEqual(result.values[0].string_value,'pure_pursuit')
        self.assertEqual(result.values[1].double_value,1.2)

        mode[0]=OperationModeState.STOP;measured[0]=.2
        end=time.monotonic()+.2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        self.assertFalse(change_many({'lateral_controller':'mpc'}).successful)
        measured[0]=0.
        end=time.monotonic()+.2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        self.assertTrue(change_many({'lateral_controller':'mpc'}).successful)
        request.names=['lateral_controller']
        self.assertEqual(call(getter,request).values[0].string_value,'mpc')
        self.assertTrue(change_many({'lateral_controller':'pure_pursuit'}).successful)
