"""ROS-independent control ownership and supervisor heartbeat contract."""
import time


class OperatorLink:
    def __init__(self,timeout=2.):
        self.timeout=timeout;self.received=None;self.state={};self.send=None
    def update(self,state):
        if not isinstance(state,dict):raise ValueError('Invalid operator status')
        self.state=state;self.received=time.monotonic()
    def connected(self):
        return self.send is not None and self.received is not None and time.monotonic()-self.received<self.timeout
    def ready(self):return self.connected() and self.state.get('ready',False)
    def request(self,action,value=None):
        if self.send is None:raise ValueError('kart operations require ROS sim + terminal 2 sim_bringup')
        if action=='kart_auto' and not self.ready():raise ValueError('Wait for kart localization and tracking readiness')
        self.send(action,value)
    def status(self):
        return {**self.state,'connected':self.connected(),'ready':self.ready()}
