"""Real Pure Pursuit + delayed TF regression; no vehicle or hardware nodes."""
import os
import shutil
import signal
import subprocess
import time
import unittest

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Point, TransformStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from kart_interfaces.msg import ReferenceLine
except ImportError as error:
    raise unittest.SkipTest(f'ROS/generated interfaces unavailable: {error}') from error


class TrackingTfWaitTest(unittest.TestCase):
    def test_delayed_exact_tf_tracks_but_missing_tf_fails(self):
        if not shutil.which('ros2'):
            self.skipTest('ros2 executable unavailable')
        namespace = f'/tracking_tf_wait_{os.getpid()}'
        map_frame, rear_frame = namespace[1:] + '_map', namespace[1:] + '_rear'
        process = subprocess.Popen([
            'ros2', 'run', 'kart_control', 'pure_pursuit', '--ros-args',
            '-r', f'__ns:={namespace}', '-p', f'map_frame:={map_frame}',
            '-p', f'tracking_frame:={rear_frame}', '-p', 'wheelbase_m:=0.257',
            '-p', 'max_steering_rad:=0.38', '-p', 'tf_lookup_timeout_s:=0.05'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

        def cleanup():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        self.addCleanup(cleanup)
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = Node('tf_wait_client', namespace=namespace)
        self.addCleanup(node.destroy_node)
        line_pub = node.create_publisher(ReferenceLine, 'planning/reference_line',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        odom_pub = node.create_publisher(Odometry, 'visual_slam/tracking/odometry', qos_profile_sensor_data)
        diag_pub = node.create_publisher(DiagnosticArray, 'localization/vslam/diagnostics', 10)
        broadcaster = TransformBroadcaster(node)
        statuses, pending = [], []
        node.create_subscription(String, 'control/tracking_status', lambda m: statuses.append(m.data), 100)
        enabled = [True]

        def publish_inputs():
            stamp = node.get_clock().now().to_msg()
            line = ReferenceLine()
            line.header.stamp, line.header.frame_id = stamp, map_frame
            line.points = [Point(x=0.0), Point(x=10.0)]
            line.speeds = [0.5, 0.5]
            line_pub.publish(line)
            odom = Odometry()
            odom.header.stamp = stamp
            odom_pub.publish(odom)
            diag = DiagnosticArray()
            diag.header.stamp = stamp
            status = DiagnosticStatus(hardware_id='visual_slam')
            status.values = [KeyValue(key='localized_in_exist_map', value='Yes'),
                             KeyValue(key='vo_status', value='OK')]
            diag.status = [status]
            diag_pub.publish(diag)
            if enabled[0]:
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, map_frame, rear_frame
                tf.transform.translation.x = 1.0
                tf.transform.rotation.w = 1.0
                pending.append((time.monotonic() + .015, tf))

        def publish_delayed_tf():
            while pending and pending[0][0] <= time.monotonic():
                broadcaster.sendTransform(pending.pop(0)[1])
        node.create_timer(1 / 30, publish_inputs)
        node.create_timer(.001, publish_delayed_tf)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.005)
            self.assertIsNone(process.poll(), 'Pure Pursuit exited unexpectedly')

        spin(2)  # Discovery and initial TF history.
        statuses.clear()
        spin(2)
        self.assertGreater(len(statuses), 20)
        self.assertEqual(set(statuses), {'tracking'})
        enabled[0] = False
        pending.clear()
        statuses.clear()
        spin(.4)
        self.assertIn('tf_unavailable', statuses)
        self.assertEqual(statuses[-1], 'tf_unavailable')
