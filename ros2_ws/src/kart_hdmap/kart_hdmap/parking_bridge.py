"""One-shot parking service requests, progressed without blocking the ROS executor."""
import json
import time


class ParkingBridge:
    def __init__(self, bridge):
        from std_srvs.srv import Trigger
        from std_msgs.msg import String
        self.bridge=bridge; self.type=Trigger; self.pending=None; self.received=0.; self.data=None
        self.enabled=bridge.node.settings.get('tuning_parking_enabled',False)
        self.clients={a:bridge.node.create_client(Trigger,'parking/'+a) for a in ('start','reset')}
        self.subscription=bridge.node.create_subscription(String,'parking/status',self.status,1)

    def status(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict): return
            self.data=data;self.received=time.monotonic()
        except (ValueError,TypeError): pass

    def view(self):
        return dict(enabled=self.enabled,available=self.enabled and self.data is not None and time.monotonic()-self.received<1.,
                    status=self.data if time.monotonic()-self.received<1. else None,pending=self.pending is not None)

    def begin(self,action,body,reply,deadline):
        try:
            if self.pending: raise ValueError('駐車操作の応答待ちです')
            if time.monotonic()>=deadline: raise ValueError('駐車要求の期限切れです')
            if body.get('map_id')!=self.bridge.identity: raise ValueError('地図が一致しません')
            if body.get('expected_revision')!=self.bridge.revision(): raise ValueError('適用版が変わりました。状態を再取得してください')
            if not self.view()['available']: raise ValueError('駐車planner未起動・状態未受信です')
            kind={'/parking_start':'start','/parking_reset':'reset'}[action]
            if kind=='reset':
                issue=self.bridge.gate.issue()
            else:
                issue=self.bridge.gate.issue(allow_running=True)
                if self.bridge.gate.mode not in (1,3): issue='駐車開始はAUTOまたはSTOPで行ってください'
            if issue: raise ValueError(issue)
            client=self.clients[kind]
            if not client.service_is_ready(): raise ValueError('駐車service未起動です')
            self.pending=dict(future=client.call_async(self.type.Request()),reply=reply,deadline=deadline)
        except Exception as exc: reply.put({'error':str(exc)})

    def tick(self):
        p=self.pending
        if not p:return
        if p['future'].done():
            try:
                result=p['future'].result()
                response=dict(map_id=self.bridge.identity,success=result.success,message=result.message)
            except Exception as exc: response={'error':str(exc)}
        elif time.monotonic()>p['deadline']:
            response={'error':'駐車操作の結果が不明です。状態を確認してください。自動再送しません'}
        else:return
        self.pending=None;p['reply'].put(response)
