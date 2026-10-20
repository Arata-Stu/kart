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
except ImportError as error:
    raise unittest.SkipTest(f'ROS/generated interfaces unavailable: {error}') from error


class TrackingParametersTest(unittest.TestCase):
    def test_native_set_requires_stop_and_preserves_rejected_values(self):
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
        mode=[None]
        def publish():
            if mode[0] is not None:
                message=OperationModeState();message.header.stamp=node.get_clock().now().to_msg();message.mode=mode[0]
                pub.publish(message)
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
            request=SetParametersAtomically.Request()
            request.parameters=[Parameter('lookahead_m',value=value).to_parameter_msg()]
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
        self.assertFalse(change(.9).successful)
        self.assertEqual(call(getter,request).values[0].double_value,1.1)
