"""Short read-only sample of Joy and board status arrival times in the ROS graph."""
import time
import uuid


def measure():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Int32MultiArray

    rclpy.init(args=[])
    node = rclpy.create_node('studio_link_timing_' + uuid.uuid4().hex[:12])
    try:
        channels = {'joy': ('/joy', 'sensor_msgs/msg/Joy', Joy),
                    'board': ('/vehicle/output_channels', 'std_msgs/msg/Int32MultiArray', Int32MultiArray)}
        samples = {key: [] for key in channels}
        status = {}
        # Allow both publishers to enter the graph before choosing a namespace.
        deadline = time.monotonic() + 1
        while True:
            graph = node.get_topic_names_and_types()
            matches = {key: [topic for topic, types in graph if topic.endswith(suffix) and kind in types]
                       for key, (suffix, kind, _) in channels.items()}
            if all(matches.values()) or time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        for key, names in matches.items():
            if len(names) != 1 or node.count_publishers(names[0]) != 1:
                status[key] = '配信元なし／一意に特定できません'
                continue
            node.create_subscription(channels[key][2], names[0],
                                     lambda _, seq=samples[key]: seq.append(time.monotonic()),
                                     qos_profile_sensor_data)
            status[key] = names[0]
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=max(0, deadline-time.monotonic()))
        result = {'measured_ms': time.time_ns() // 1_000_000}
        for key, seq in samples.items():
            gaps = [(b-a)*1000 for a, b in zip(seq, seq[1:])]
            result[key] = dict(count=len(seq), max_gap_ms=max(gaps, default=None), topic=status[key])
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()
