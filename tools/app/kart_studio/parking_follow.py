"""Shared Mission + production C++ lateral control with ideal speed tracking."""
import hashlib
import math
from pathlib import Path
import shutil
import subprocess
import time
from .tracking_preview import _cache, _lock


def follow(route, zone, world, settings, departure, pose, options, initial_speed=0.):
    from kart_parking.mission import Mission
    from kart_parking.geometry import wrap, point_segment
    root=Path(__file__).resolve().parents[3]
    source=root/'tools/app/native/parking_follow.cpp';include=root/'ros2_ws/src/kart_control/include'
    digest=hashlib.sha256(source.read_bytes()+b''.join(p.read_bytes() for p in sorted((include/'kart_control').glob('*.hpp')))).hexdigest()
    binary=_cache/digest
    with _lock:
        if not binary.exists():
            compiler=shutil.which('c++')
            if not compiler:raise ValueError('駐車試走にはC++コンパイラが必要です')
            result=subprocess.run([compiler,'-std=c++17','-O2','-I'+str(include),str(source),'-o',str(binary)],capture_output=True,text=True,timeout=60)
            if result.returncode:raise ValueError('駐車試走のコンパイル失敗: '+result.stderr[-1000:])
    if options.get('controller') not in ('pure_pursuit','mpc'):raise ValueError('未知の横制御方式です')
    keys=[('speed',.01,20),('wheelbase',.05,2),('maxSteer',.01,1.2),('horizon',.3,2),('tracking',.1,100),('smoothness',.1,100),('steeringRate',.1,5)]
    values=[options[k] for k,_,_ in keys]
    if any(type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi for v,(_,lo,hi) in zip(values,keys)):
        raise ValueError('駐車試走の設定が範囲外です')
    if not math.isfinite(initial_speed) or not 0<=initial_speed<=20:raise ValueError('試走速度が不正です')
    mission=Mission(route,zone,world,settings,departure);speed=initial_speed;omega=steering=0.;frames=[];maxerror=0.
    proc=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    try:
        proc.stdin.write(' '.join(map(str,[len(route),int(options['controller']=='mpc'),*values]))+'\n')
        for x,y,_,v in route:proc.stdin.write(f'{x} {y} {v}\n')
        proc.stdin.flush();deadline=time.monotonic()+25
        for tick in range(9000):
            now=tick/30;mission.update(pose,speed,omega,now,True)
            target=zone['goal'];ms=lookahead=command=0.
            if mission.state not in ('settling','parked','failed'):
                window=mission.window()
                proc.stdin.write(' '.join(map(str,[mission.index,mission.index+len(window),*pose,speed,steering,int(mission.approaching_parking())]))+'\n');proc.stdin.flush()
                output=list(map(float,proc.stdout.readline().split()))
                if len(output)!=7:raise ValueError('駐車試走の制御計算に失敗しました')
                valid,steering,command,tx,ty,ms,lookahead=output;target=(tx,ty)
                if not valid:mission.fault('controller_invalid')
            segment=min(mission.index,len(route)-2)
            error=point_segment(pose,route[segment],route[segment+1])[0];maxerror=max(maxerror,error)
            frames.append([now,*pose,speed,steering,error,maxerror,*target[:2],ms,command,lookahead])
            if mission.state in ('parked','failed'):break
            if time.monotonic()>deadline:return dict(frames=frames,reason='time_limit')
            # Ideal longitudinal response; does not simulate the real speed PID.
            speed=command;omega=speed*math.tan(steering)/settings.wheelbase_m
            pose=(pose[0]+speed*math.cos(pose[2]+omega/60)/30,pose[1]+speed*math.sin(pose[2]+omega/60)/30,wrap(pose[2]+omega/30))
        return dict(frames=frames,reason='simulated_parked' if mission.state=='parked' else mission.reason or 'time_limit')
    finally:
        proc.stdin.close()
        try:proc.wait(timeout=2)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        proc.stdout.close()
