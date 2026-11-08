"""One mode request in the selected ROS graph, with state feedback."""
import time
import uuid


def request(action):
    import rclpy
    from kart_interfaces.msg import OperationModeRequest as Request, OperationModeState as State

    modes = {'mode_auto': Request.AUTO, 'mode_manual': Request.MANUAL, 'mode_stop': Request.STOP}
    desired = modes.get(action)
    if desired is None and action != 'mode_status':
        raise ValueError('Unknown mode action')
    source = 'kart_studio:' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('kart_studio_mode_' + source.split(':')[1])
    try:
        def wait(condition, seconds):
            deadline = time.monotonic() + seconds
            while not condition():
                if time.monotonic() >= deadline:
                    return False
                rclpy.spin_once(node, timeout_sec=0.05)
            return True

        states = []
        suffix = '/operation_mode/state'
        def topics():
            return [name for name, kinds in node.get_topic_names_and_types()
                    if name.endswith(suffix) and 'kart_interfaces/msg/OperationModeState' in kinds]
        if not wait(topics, 3):
            raise ValueError('モード管理ノードが見つかりません')
        names = topics()
        if len(names) != 1 or node.count_publishers(names[0]) != 1:
            raise ValueError('対象のモード管理ノードを一意に特定できません')
        topic = names[0]
        node.create_subscription(State, topic, states.append, 1)
        pub = node.create_publisher(Request, topic[:-len('state')] + 'request', 10)
        if not wait(lambda: states and (desired is None or pub.get_subscription_count()), 2):
            raise ValueError('モード管理ノードから応答がありません')
        if desired is not None:
            msg = Request()
            msg.mode, msg.source = desired, source
            # Use the manager clock, including simulated time. The manager rejects stale requests.
            msg.header.stamp = states[-1].header.stamp
            states.clear()
            pub.publish(msg)
            if not wait(lambda: any(s.source == source and s.mode == desired for s in states), 1):
                raise ValueError('モード要求の反映は未確認です。状態を再取得してください')
            # Allow downstream arming checks to report an immediate STOP.
            until = time.monotonic() + 0.35
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.05)
            states.clear()
            if not wait(lambda: states, 0.5):
                raise ValueError('要求後に状態配信が途切れました。現在のモードは未確認です')
        state = states[-1]
        labels = {State.AUTO:'AUTO', State.MANUAL:'MANUAL', State.STOP:'STOP', State.PROPO:'PROPO'}
        return dict(mode=labels.get(state.mode, 'UNKNOWN'), source=state.source)
    finally:
        node.destroy_node()
        rclpy.shutdown()
