"""Simulation actuator gate; explicit clocks allow ROS-free expiry testing."""
import math

class CommandGate:
    def __init__(self, command_timeout=.2, mode_timeout=.5, require_mode=True):
        if not math.isfinite(command_timeout) or not math.isfinite(mode_timeout) or min(command_timeout,mode_timeout)<=0:
            raise ValueError('Timeouts must be finite positive values')
        self.command_timeout=command_timeout; self.mode_timeout=mode_timeout; self.require_mode=require_mode
        self.reset()

    def reset(self):
        self.command=(0.,0.,0.,0.); self.received=None; self.source=None
        self.mode=None; self.mode_received=None; self.mode_source=None

    def invalidate_command(self):
        self.received=None

    def accept_command(self,values,source,sim_time,steady_time):
        valid=(all(math.isfinite(v) for v in values) and abs(values[0])<=1 and
               all(0<=v<=1 for v in values[1:]) and sum(v>0 for v in values[1:])<=1 and
               math.isfinite(source) and -1e-6<=sim_time-source<=self.command_timeout)
        if not valid: self.received=None; return False
        self.command=tuple(values); self.source=source; self.received=steady_time
        return True

    def accept_mode(self,mode,source,sim_time,steady_time):
        if not math.isfinite(source) or not -1e-6<=sim_time-source<=self.mode_timeout:
            self.mode_received=None; self.received=None; return
        if mode!=self.mode or mode not in (1,2): self.received=None
        self.mode=mode; self.mode_source=source; self.mode_received=steady_time

    def output(self,sim_time,steady_time):
        permitted=(not self.require_mode or
            (self.mode in (1,2) and self.mode_received is not None and
             0<=steady_time-self.mode_received<=self.mode_timeout and
             -1e-6<=sim_time-self.mode_source<=self.mode_timeout))
        fresh=(self.received is not None and 0<=steady_time-self.received<=self.command_timeout and
               -1e-6<=sim_time-self.source<=self.command_timeout)
        return self.command if permitted and fresh else (0.,0.,0.,0.)
