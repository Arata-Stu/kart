"""Monotone route progress and persistent parking completion, independent of ROS."""
import math
from .geometry import distance, point_segment, wrap


class Mission:
    def __init__(self, route, zone, world, settings, departure_index=0):
        self.route,self.zone,self.world,self.c=route,zone,world,settings
        self.arc=[0.]
        for a,b in zip(route,route[1:]):
            self.arc.append(self.arc[-1]+distance(a,b))
        self.departure_index=departure_index
        self.index=0
        self.state='approach'
        self.reason=''
        self.settled=None
        self.last_progress=None
        self.progress_anchor=route[0]
        self.last_time=None
        self.last_pose=None

    def fault(self, reason):
        self.state='failed'; self.reason=reason; self.settled=None

    def update(self, pose, speed, yaw_rate, now, active):
        if self.state=='failed':
            return
        if not all(math.isfinite(v) for v in (*pose,speed,yaw_rate,now)):
            self.fault('invalid_measurement'); return
        if self.last_time is not None and now<self.last_time:
            self.fault('clock_jump'); return
        self.last_time=now
        if not self.world.clear(pose):
            self.fault('vehicle_clearance'); return
        goal=self.zone['goal']
        at_goal=(distance(pose,goal)<=self.c.goal_position_m
                 and abs(wrap(pose[2]-goal[2]))<=self.c.goal_yaw_rad
                 and self.world.clear(pose,self.zone['area']))
        # Never finish from a nearby pass on the course or an earlier lap.
        near_end=self.arc[-1]-self.arc[self.index]<.5
        if self.state in ('settling','parked') or (at_goal and near_end):
            if not at_goal:
                self.fault('left_goal_after_arrival'); return
            self.state='parked' if self.state=='parked' else 'settling'
            stopped=abs(speed)<=self.c.stopped_speed_mps and abs(yaw_rate)<=self.c.stopped_yaw_rate_rps
            if self.state=='parked' and not stopped:
                self.fault('motion_after_parking'); return
            if stopped:
                if self.settled is None: self.settled=now
                if now-self.settled>=self.c.settle_s: self.state='parked'
            else:
                self.settled=None
            return
        if self.last_pose is not None and distance(pose,self.last_pose)>.5:
            self.fault('localization_jump'); return
        self.last_pose=pose
        # Only nearby forward segments can advance progress. A full next lap may
        # overlap spatially with the exit; never run a global nearest-point query.
        end=self.index+1; length=0.
        while end<len(self.route)-1 and length<.5:
            length+=distance(self.route[end-1],self.route[end]); end+=1
        candidates=[(point_segment(pose,self.route[i],self.route[i+1])[0],i)
                    for i in range(self.index,end)]
        error,index=min(candidates)
        if error>self.c.max_cross_track_m:
            self.fault('off_route'); return
        if abs(wrap(pose[2]-self.route[index][2]))>1.:
            self.fault('wrong_heading'); return
        self.index=max(self.index,index)
        if not active:
            self.last_progress=now; self.progress_anchor=pose
        elif self.last_progress is None or distance(pose,self.progress_anchor)>.04:
            self.last_progress=now; self.progress_anchor=pose
        elif now-self.last_progress>self.c.stall_s:
            self.fault('stalled_before_goal'); return
        self.state='parking' if self.index>=self.departure_index else 'approach'

    def approaching_parking(self):
        return self.arc[self.departure_index]-self.arc[self.index]<=1.

    def window(self):
        if self.state in ('settling','parked','failed'):
            return []
        end=self.index+1; length=0.
        while end<len(self.route)-1 and length<self.c.horizon_m:
            length+=distance(self.route[end-1],self.route[end]); end+=1
        return self.route[self.index:end+1]
