"""One bounded ROS service call inside the selected container. Never retries commands."""
import json
import time
import uuid

STATUS_SUFFIX = '/bag/recording_status'
STATUS_TYPE = 'std_srvs/srv/Trigger'


def select_service(services, expected=None):
    candidates = sorted(name for name, types in services
                        if name.endswith(STATUS_SUFFIX) and STATUS_TYPE in types)
    if len(candidates) > 1:
        raise ValueError('録画managerが複数あります。対象を一意にしてから操作してください')
    if not candidates:
        return None
    if expected is not None and candidates[0] != expected:
        raise ValueError('録画managerが変更されています。状態を再取得してください')
    return candidates[0]


def request(action, expected_service=None):
    if action not in ('recording_status', 'recording_start', 'recording_stop'):
        raise ValueError('Unknown recording action')
    if action != 'recording_status' and not expected_service:
        raise ValueError('録画状態を取得してから操作してください')
    import rclpy
    from std_srvs.srv import Trigger, SetBool
    context = rclpy.Context()
    rclpy.init(context=context, args=[])
    node = None
    try:
        node = rclpy.create_node('kart_studio_recording_' + uuid.uuid4().hex[:12], context=context)
        deadline = time.monotonic() + 3.0
        service = None
        while time.monotonic() < deadline:
            service = select_service(node.get_service_names_and_types(), expected_service)
            if service:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        if service is None:
            if action == 'recording_status':
                return dict(available=False, message='録画manager未検出。録画対応bringupの起動とkart_bag_managerの更新・ビルドを確認してください')
            raise ValueError('録画managerに接続できません。操作は送信していません')
        status_only = action == 'recording_status'
        kind = Trigger if status_only else SetBool
        endpoint = service if status_only else service[:-len('recording_status')] + 'set_recording'
        client = node.create_client(kind, endpoint)
        if not client.wait_for_service(timeout_sec=1.0):
            raise ValueError('録画サービス未接続。操作は送信していません')
        message = kind.Request()
        if not status_only:
            message.data = action == 'recording_start'
        future = client.call_async(message)  # Exactly once. No ros2 CLI parsing or pub retries.
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if not future.done():
            future.cancel()
            raise ValueError('録画操作の結果は未確認です。自動再送せず、状態を再取得してください')
        response = future.result()
        value = json.loads(response.message)
        if value.get('schema') != 'kart.recording.v1':
            raise ValueError('録画managerの応答形式が一致しません。両側のコードを更新してください')
        return dict(available=True, service=service, accepted=response.success, **value)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown(context=context)
