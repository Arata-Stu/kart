"""Single explicit behavior reset, progressed by the existing HDMap executor."""
import time


class BehaviorBridge:
    def __init__(self, bridge):
        from std_srvs.srv import Trigger
        self.bridge, self.type, self.pending = bridge, Trigger, None
        self.client = bridge.node.create_client(Trigger, '/racing/behavior/reset')

    def begin(self, body, reply, deadline):
        try:
            if self.pending is not None:
                raise ValueError('行動計画リセットの応答待ちです')
            if time.monotonic() >= deadline:
                raise ValueError('行動計画リセット要求の期限切れです')
            if body.get('map_id') != self.bridge.identity:
                raise ValueError('NotebookとJetsonの地図が一致しません')
            if body.get('expected_revision') != self.bridge.revision():
                raise ValueError('適用版が変わりました。状態を再取得してください')
            if getattr(getattr(self.bridge, 'parameters', None), 'pending', None) is not None:
                raise ValueError('パラメータ処理中です。結果を確認してからリセットしてください')
            issue = self.bridge.issue() or self.bridge.gate.issue()
            if issue:
                raise ValueError(issue)
            node = self.bridge.node
            status = self.bridge.behavior.view(node.count_publishers('/racing/behavior_status') == 1,
                                               node.get_clock().now().nanoseconds)
            if not status['fresh'] or status['status'].get('mode') not in ('shadow', 'active'):
                raise ValueError('行動計画ノード未起動・状態未受信・受信停止です')
            if not self.client.service_is_ready():
                raise ValueError('行動計画reset service未起動です')
            self.pending = dict(future=self.client.call_async(self.type.Request()), reply=reply, deadline=deadline)
        except Exception as error:
            reply.put({'error': str(error)})

    def tick(self):
        pending = self.pending
        if pending is None:
            return
        if pending['future'].done():
            try:
                result = pending['future'].result()
                response = dict(map_id=self.bridge.identity, success=result.success, message=result.message)
            except Exception as error:
                response = {'error': str(error)}
        elif time.monotonic() >= pending['deadline']:
            response = {'error': '行動計画リセットの結果が不明です。状態を確認してください。自動再送しません'}
        else:
            return
        self.pending = None
        pending['reply'].put(response)
