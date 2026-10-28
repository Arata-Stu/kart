"""Offline shared parking planner and optional ideal controller/mission trial."""
import json
import math
from pathlib import Path
import sys
import tempfile


def preview(document, lane_id, kind, slot, pose, options=None, initial_speed=0.):
    root=Path(__file__).resolve().parents[3]/'ros2_ws/src'
    for name in ('kart_parking','kart_hdmap'):
        path=str(root/name)
        if path not in sys.path: sys.path.insert(0,path)
    from kart_parking.planner import Settings, load_site, prepare, activate
    from kart_parking.geometry import wrap
    from dataclasses import replace
    c=Settings()
    if options: c=replace(c,wheelbase_m=float(options["wheelbase"]),max_steering_rad=float(options["maxSteer"])).validate()
    lane=next(l for l in document['lanes'] if l['id']==lane_id)
    line=lane['lines'][kind];points=line['points']
    speeds=[lane.get('centerline_speed_mps',1.5)]*len(points) if kind=='centerline' else [p[5] for p in line['profile']]
    with tempfile.TemporaryDirectory(prefix='kart-parking-preview-') as folder:
        file=Path(folder)/'map.json';file.write_text(json.dumps(dict(document,schema='kart.hdmap.v2')))
        world,zone=load_site(file,document.get('frame','map'),slot,c)
    plan=prepare(points,speeds,lane['closed'],zone,world,c)
    route=activate(plan,pose,c)
    if options:
        from .parking_follow import follow
        result=follow(route,zone,world,c,len(route)-len(plan.connector),pose,options,initial_speed)
        return dict(result,route=[list(p) for p in route],parking_speed_mps=c.parking_speed_mps)
    # Follow the planned poses with their scheduled speeds. This intentionally
    # does not claim PP/MPC tracking, PID settling, or real vehicle success.
    frames=[];time=0.;distance_on_segment=0.;index=0
    while len(frames)<18000:
        a=route[index];b=route[min(index+1,len(route)-1)]
        length=math.hypot(b[0]-a[0],b[1]-a[1]);t=min(1,distance_on_segment/length) if length else 0
        x=a[0]+(b[0]-a[0])*t;y=a[1]+(b[1]-a[1])*t;yaw=a[2]+wrap(b[2]-a[2])*t
        speed=a[3]+(b[3]-a[3])*t
        final=index==len(route)-1
        frames.append([time,x,y,yaw,0 if final else speed,0,0,0,b[0],b[1],0,0 if final else speed,0])
        if final: return dict(frames=frames,reason='planned_goal',route=[list(p) for p in route],parking_speed_mps=c.parking_speed_mps)
        if speed<=1e-6:return dict(frames=frames,reason='stopped_target',route=[list(p) for p in route],parking_speed_mps=c.parking_speed_mps)
        distance_on_segment+=max(speed,.01)/30;time+=1/30
        while index<len(route)-1:
            a,b=route[index:index+2];length=math.hypot(b[0]-a[0],b[1]-a[1])
            if distance_on_segment<length:break
            distance_on_segment-=length;index+=1
    return dict(frames=frames,reason='time_limit',route=[list(p) for p in route],parking_speed_mps=c.parking_speed_mps)
