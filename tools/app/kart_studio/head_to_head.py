"""Offline two-car experiment. Perfect observations, ideal longitudinal response.

The ego uses the production lateral controller. The opponent follows the exact
reference geometry. No perception, vehicle outputs, or map writes are involved.
"""
import bisect
import hashlib
import math
from pathlib import Path
import selectors
import shutil
import subprocess
import time

from shapely.geometry import Polygon
from .preview_lookahead import lookahead_values
from .tracking_preview import _cache, _lock


def number(data, key, default, lo, hi):
    value = data.get(key, default)
    if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{key}: {lo}〜{hi}の有限値が必要です')
    return value


class Course:
    def __init__(self, line, closed, speed):
        self.closed = closed
        self.points = line['points']
        if not 3 <= len(self.points) <= 5000:
            raise ValueError('試走ラインは3〜5000点が必要です')
        if any(len(p) != 2 or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v)>1e6 for v in p) for p in self.points):
            raise ValueError('試走ラインの座標が不正です')
        profile = line.get('profile')
        if profile is not None and (len(profile)!=len(self.points) or any(len(p)<6 for p in profile)):
            raise ValueError('速度profileが不正です')
        self.speeds = [number({'v':p[5]}, 'v', speed, 0, 20) for p in profile] if profile is not None else [speed]*len(self.points)
        self.step = line.get('speed_mode') == 'step'
        self.stations = [0.]
        for a,b in zip(self.points,self.points[1:]+self.points[:1]):
            length = math.dist(a,b)
            if length < 1e-5: raise ValueError('ラインの重複点を除去してください')
            self.stations.append(self.stations[-1]+length)
        self.total = self.stations[-1]
        if not 3<=self.total<=20000:raise ValueError('ライン長は3〜20000 mにしてください')

    def sample(self, s):
        s %= self.total
        i = min(len(self.points)-1,bisect.bisect_right(self.stations,s)-1)
        a,b = self.points[i],self.points[(i+1)%len(self.points)]
        f=(s-self.stations[i])/(self.stations[i+1]-self.stations[i])
        speed=self.speeds[i] if self.step else self.speeds[i]+f*(self.speeds[(i+1)%len(self.points)]-self.speeds[i])
        return a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]),math.atan2(b[1]-a[1],b[0]-a[0]),speed

    def project(self, pose, previous, reach):
        # Local unwrapped search prevents switching to a nearby hairpin branch.
        best=(float('inf'),previous)
        for i,(a,b) in enumerate(zip(self.points,self.points[1:]+self.points[:1])):
            length=self.stations[i+1]-self.stations[i]
            for lap in (math.floor(previous/self.total)-1,math.floor(previous/self.total),math.floor(previous/self.total)+1):
                start=self.stations[i]+lap*self.total
                lo=max(0.,(previous-.2-start)/length);hi=min(1.,(previous+reach-start)/length)
                if lo>hi:continue
                dx,dy=b[0]-a[0],b[1]-a[1]
                f=max(lo,min(hi,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length**2))
                d=math.hypot(pose[0]-a[0]-f*dx,pose[1]-a[1]-f*dy)
                if d<best[0]:best=d,start+f*length
        return max(previous,best[1]),best[0]


def area(lane):
    rings=[]
    for key in ('left','right'):
        pts=lane.get(key,[])
        if not 3<=len(pts)<=5000 or any(len(p)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in p) for p in pts):
            raise ValueError('左右境界が必要です')
        ring=Polygon(pts)
        if not ring.is_valid or ring.area<=0:raise ValueError('左右境界が不正です')
        rings.append(ring)
    a,b=rings
    if a.contains(b):return a.difference(b)
    if b.contains(a):return b.difference(a)
    raise ValueError('閉コースの内周・外周が必要です')


def body_shape(pose, length, width, rear):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return Polygon([(x+c*u-s*v,y+s*u+c*v) for u,v in ((-rear,-width/2),(length-rear,-width/2),(length-rear,width/2),(-rear,width/2))])


def sweep(a,b,pa,pb,radius):
    # Convex hull of endpoint bodies plus a bound on rotational arc sagitta.
    angle=abs(math.atan2(math.sin(pb[2]-pa[2]),math.cos(pb[2]-pa[2])))
    return a.union(b).convex_hull.buffer(radius*(1-math.cos(angle/2))+1e-6)


def executable():
    root=Path(__file__).resolve().parents[3]
    source=root/'tools/app/native/head_to_head.cpp';include=root/'ros2_ws/src/kart_control/include'
    digest=hashlib.sha256(source.read_bytes()+b''.join(p.read_bytes() for p in sorted((include/'kart_control').glob('*.hpp')))).hexdigest()
    binary=_cache/digest
    with _lock:
        if not binary.exists():
            compiler=shutil.which('c++')
            if not compiler:raise ValueError('試走にはC++コンパイラが必要です')
            result=subprocess.run([compiler,'-std=c++17','-O2','-I'+str(include),str(source),'-o',str(binary)],capture_output=True,text=True,timeout=60)
            if result.returncode:raise ValueError('試走のビルド失敗: '+result.stderr[-1500:])
    return binary


def predictive_executable():
    root=Path(__file__).resolve().parents[3]
    source=root/'tools/app/native/predictive_spliner.cpp'
    include=root/'ros2_ws/src/kart_racing/include'
    digest=hashlib.sha256(source.read_bytes()+b''.join(p.read_bytes() for p in sorted(include.rglob('*.hpp')))).hexdigest()
    binary=_cache/digest
    with _lock:
        if not binary.exists():
            compiler=shutil.which('c++')
            if not compiler:raise ValueError('試走にはC++コンパイラが必要です')
            result=subprocess.run([compiler,'-std=c++17','-O2','-I'+str(include),str(source),'-o',str(binary)],capture_output=True,text=True,timeout=60)
            if result.returncode:raise ValueError('Predictive plannerビルド失敗: '+result.stderr[-1500:])
    return binary


def preview(request):
    lane=request['lane'];options=request['options'];cfg=request['head_to_head']
    planner_kind=cfg.get('planner','baseline')
    if planner_kind not in ('baseline','predictive_spliner'):raise ValueError('追い抜きplannerが不正です')
    if lane.get('closed') is not True:raise ValueError('Head-to-head初版は閉コース専用です')
    method=options.get('controller')
    if method not in ('pure_pursuit','mpc'):raise ValueError('横制御方式が不正です')
    lookahead_config=lookahead_values(options)
    vmax=number(options,'speed',1.5,.1,20)
    wb=number(options,'wheelbase',.257,.05,2);steer_limit=number(options,'maxSteer',.384,.01,1.2)
    controls=[number(options,k,d,lo,hi) for k,d,lo,hi in [('horizon',1,.3,2),('tracking',10,.1,100),('smoothness',1,.1,100),('steeringRate',1.5,.1,5)]]
    laps=number(options,'laps',3,1,20)
    if int(laps)!=laps:raise ValueError('周回数は整数で指定してください')
    line=lane['lines'][request['line_type']]
    course=Course(line,True,vmax);surface=area(lane)
    obstacles=request.get('obstacles',[])
    if len(obstacles)>500:raise ValueError('障害物数が上限を超えています')
    for item in obstacles:
        points=item['polygon']
        if not 3<=len(points)<=5000 or any(len(p)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in p) for p in points):raise ValueError('障害物が不正です')
        shape=Polygon(points)
        if not shape.is_valid:raise ValueError('障害物が不正です')
        surface=surface.difference(shape)
    settings=line.get('settings',{})
    length=number(settings,'vehicle_length',.35,.1,2);width=number(settings,'vehicle_width',.19,.05,1)
    rear=number(settings,'rear_axle_to_rear',.1065,0,length)
    margin=number(cfg,'margin',.08,.01,.5)
    usable=surface.buffer(-margin)
    opponent_cap=number(cfg,'opponent_speed',2,0,20)
    initial_gap=number(cfg,'initial_gap',3,.5,min(100,course.total-length-1))
    headway=number(cfg,'headway',.7,.2,3)
    min_gap=number(cfg,'min_gap',1,.2,10)
    observe=number(cfg,'observe_seconds',3,.5,30)
    zone_start=number(cfg,'zone_start',0,0,course.total)
    zone_end=number(cfg,'zone_end',0,0,course.total)
    enabled=cfg.get('overtake',False)
    if type(enabled) is not bool:raise ValueError('追い抜き設定が不正です')
    if enabled and not 5<=zone_end-zone_start<=500:raise ValueError('追い抜き区間は始点からの距離で5〜500 m、周回始点を跨がず指定してください')
    side=cfg.get('side','left')
    if side not in ('left','right'):raise ValueError('追い抜き側が不正です')
    offset=number(cfg,'offset',.65,width+2*margin,3)*(1 if side=='left' else -1)
    duration=number(cfg,'duration',120,5,300)
    accel=1.5;brake=2.5;dt=1/30
    s=0.;os=initial_gap+length;v=ov=steer=0.
    pose=course.sample(s)[:3];opose=course.sample(os)[:3]
    radius=math.hypot(max(rear,length-rear),width/2)
    shape=body_shape(pose,length,width,rear);oshape=body_shape(opose,length,width,rear)
    if not usable.contains(shape) or not usable.contains(oshape) or shape.distance(oshape)<=margin:
        raise ValueError('開始位置の車体が境界・障害物・相手と干渉しています')
    frames=[];rivals=[];metrics=[];events=[];plan=None;stable=0.;estimate=None;state='FOLLOW_OBSERVE';reason='time_limit';maxerror=0.;passed=False;note='車間が安定するまで観測';minimum=initial_gap

    def offset_at(station, candidate):
        if not candidate:return 0.
        start,end,ramp=candidate
        u=min(1.,max(0.,(station-start)/ramp),max(0.,(end-station)/ramp))
        return offset*(10*u**3-15*u**4+6*u**5)

    def route_point(station,candidate):
        x,y,yaw,speed=course.sample(station);d=offset_at(station,candidate)
        return x-math.sin(yaw)*d,y+math.cos(yaw)*d,speed

    def clear_plan(candidate):
        start,end,_=candidate
        direction=course.sample(start)[2]
        previous=None;previous_pose=None
        for i in range(math.ceil((end-start)/.1)+1):
            station=min(end,start+i*.1)
            yaw_ref=course.sample(station)[2]
            if abs(math.atan2(math.sin(yaw_ref-direction),math.cos(yaw_ref-direction)))>.1:return False
            a=route_point(station,candidate);b=route_point(station+.02,candidate)
            yaw=math.atan2(b[1]-a[1],b[0]-a[0]);p=(a[0],a[1],yaw)
            shape=body_shape(p,length,width,rear)
            if not usable.contains(shape):return False
            if previous is not None:
                angle=abs(math.atan2(math.sin(yaw-previous_pose[2]),math.cos(yaw-previous_pose[2])))
                distance=math.dist(p[:2],previous_pose[:2])
                if angle>math.tan(steer_limit)/wb*max(distance,1e-6)*.8:return False
                if not usable.contains(sweep(previous,shape,previous_pose,p,radius)):return False
            previous=shape;previous_pose=p
        return True

    predictor=None;prediction_selector=None;prediction=None
    predictor_binary=predictive_executable() if planner_kind=='predictive_spliner' else None
    proc=subprocess.Popen([str(executable())],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    selector=selectors.DefaultSelector();selector.register(proc.stdout,selectors.EVENT_READ)
    deadline=time.monotonic()+60
    try:
        if predictor_binary:
            predictor=subprocess.Popen([str(predictor_binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
            prediction_selector=selectors.DefaultSelector();prediction_selector.register(predictor.stdout,selectors.EVENT_READ)
            predictor.stdin.write(' '.join(map(str,[course.total,length,width,margin,min_gap,headway,.8*math.tan(steer_limit)/wb]))+'\n');predictor.stdin.flush()
        proc.stdin.write(' '.join(map(str,[int(method=='mpc'),int(course.step),wb,steer_limit,*controls,*lookahead_config]))+'\n');proc.stdin.flush()
        terminal_time=0.
        for tick in range(int(duration*30)+1):
            now=tick*dt;terminal_time=now;gap=os-s-length
            minimum=min(minimum,gap) if not plan and not passed else minimum
            if predictor:
                local=s%course.total
                # Width is a search bound only: exact geometry is checked below.
                x0,y0,yaw0,_=course.sample(s)
                ego_d=-(pose[0]-x0)*math.sin(yaw0)+(pose[1]-y0)*math.cos(yaw0)
                end=s-local+zone_end
                cap=min(vmax,min(course.sample(s+i*max(0,end-s)/100)[3] for i in range(101)))
                allowed=enabled and (plan is not None or zone_start<=local<zone_end)
                predictor.stdin.write(' '.join(map(str,[now,s,ego_d,v,os,0,ov,cap,end,abs(offset)+width if side=='left' else 0,abs(offset)+width if side=='right' else 0,int(allowed)]))+'\n');predictor.stdin.flush()
                if not prediction_selector.select(2):raise ValueError('Predictive planner応答timeout')
                values=predictor.stdout.readline().split()
                if len(values)!=12:raise ValueError('Predictive planner出力不正')
                if not all(math.isfinite(float(values[i])) for i in (0,1,2,5,6,7,8,9,10,11)):raise ValueError('Predictive planner出力が非有限です')
                prediction=dict(valid=bool(int(values[0])),speed=float(values[1]),coverage=float(values[2]),state=values[3],reason=values[4],active=bool(int(values[5])),start=float(values[6]),end=float(values[7]),ramp=float(values[8]),offset=float(values[9]),roc=[float(values[10]),float(values[11])])
                if not prediction['valid']:
                    reason=prediction['reason'];note='予測失効・試走中断（実車退避未検証）';break
                if prediction['state']=='PASSED':
                    state='PASSED';reason='overtake_completed';passed=True;break
                if not plan:
                    state=prediction['state'];note=f"GP観測被覆 {prediction['coverage']:.0%} / {prediction['reason']}"
                    if prediction['active']:
                        candidate=(prediction['start'],prediction['end'],prediction['ramp'])
                        offset=prediction['offset']
                        if not clear_plan(candidate):
                            reason='predictive_geometry_rejected';note='予測経路の車体境界・曲率検査で中断';break
                        plan=candidate;events.append(dict(time=now,state='PASS',station=s,roc=prediction['roc']))
            if not predictor and not plan and not passed:
                desired_gap=min_gap+headway*v
                stable=stable+dt if abs(gap-desired_gap)<.4 and abs(v-ov)<.2 else 0.
                # Perfect course observations; no simulated detector confidence.
                estimate=ov if stable>=observe else None
                state='OPPORTUNITY_READY' if estimate is not None and vmax-estimate>=.5 else 'FOLLOW_OBSERVE'
                note='速度優位なし' if estimate is not None and vmax-estimate<.5 else ('観測完了・区間待ち' if estimate is not None else '車間が安定するまで観測')
                local=s%course.total
                if enabled and state=='OPPORTUNITY_READY' and zone_start<=local<zone_end:
                    end=s-local+zone_end
                    ramp=max(3.,vmax,math.sqrt(8*abs(offset)*wb/math.tan(steer_limit)))
                    own_cap=min(vmax,min(course.sample(s+i*(end-s)/100)[3] for i in range(101)))
                    advantage=own_cap-opponent_cap
                    # Conservative: ignore positive relative gain while accelerating,
                    # and reserve both lane transitions in addition to catching up.
                    required=gap+2*length+min_gap+headway*opponent_cap
                    needed=own_cap*(max(0,own_cap-v)/accel+max(0,required)/max(.01,advantage))+2*ramp
                    candidate=(s,end,ramp)
                    if advantage<.5 or needed>end-s or end-s<2*ramp+1:
                        note='残距離または速度差不足・追従継続'
                    elif not clear_plan(candidate):note='追い抜き経路の幅・曲率不足'
                    else:
                        plan=candidate;state='PASS';note='モデル上の追い抜き実行';events.append(dict(time=now,state=state,station=s))
            if plan:
                start,end,ramp=plan
                state='MERGE' if s>=end-ramp else 'PASS'
                if state=='MERGE' and s-os-length<min_gap:
                    reason='merge_unavailable';note='合流車間不足・試走中断（退避制御は未実装）';break
                if s>=end:
                    if abs(offset_at(s,plan))<.01 and math.dist(pose[:2],course.sample(s)[:2])<.15:
                        passed=True;state='PASSED';reason='overtake_completed';events.append(dict(time=now,state=state,station=s))
                        break
                    reason='merge_deadline';note='合流期限でライン復帰未完了';break
            if s>=course.total*laps:reason='completed';break
            target_cap=vmax
            if not plan and not passed:
                target_cap=min(vmax,max(0.,ov+.8*(gap-(min_gap+headway*v))),math.sqrt(max(0.,ov*ov+2*brake*(gap-min_gap))))
            if prediction is not None:target_cap=prediction['speed']
            # Feed a local route into the production PP/MPC. The open-route end is
            # well beyond stopping distance; course zero-speed samples remain active.
            horizon=max(8.,vmax*vmax/(2*brake)+5.,max(lookahead_config[1:])+2.)
            count=min(900,math.ceil((horizon+1)/.25)+1)
            route=[route_point(s-1+i*(horizon+1)/(count-1),plan) for i in range(count)]
            proc.stdin.write(' '.join(map(str,[len(route),*pose,v,steer,max(.001,target_cap)]))+'\n')
            proc.stdin.write(''.join(f'{x} {y} {speed}\n' for x,y,speed in route));proc.stdin.flush()
            if not selector.select(max(.001,min(2.,deadline-time.monotonic()))):raise ValueError('横制御計算が時間内に応答しませんでした')
            output=proc.stdout.readline().split()
            if len(output)!=8:raise ValueError('横制御出力が不正です')
            valid,steer,target,tx,ty,ms,lookahead=map(float,output[:7])
            if not all(math.isfinite(a) for a in (valid,steer,target,tx,ty,ms,lookahead)):raise ValueError('横制御出力が非有限です')
            if not valid:reason='controller_invalid';note=output[7];break
            target=min(target,target_cap)
            s,error=course.project(pose,s,max(1.,v*dt*3));maxerror=max(maxerror,error)
            frames.append([now,*pose,v,steer,error,maxerror,tx,ty,ms,target,lookahead])
            rivals.append([*opose,ov])
            metrics.append(dict(state=state,gap=gap,opponent_speed=ov,estimate=estimate,stable_seconds=stable,note=note,station=s%course.total,coverage=prediction['coverage'] if prediction else None))
            if time.monotonic()>deadline:reason='compute_timeout';break
            if tick==int(duration*30):break
            # Substeps bound translation and heading changes for swept geometry.
            substeps=max(2,math.ceil(max(v,ov,vmax,opponent_cap)*dt/.03))
            fault=None
            for substep in range(substeps):
                h=dt/substeps
                nv=max(0.,v+max(-brake*h,min(accel*h,target-v)))
                oref=min(opponent_cap,course.sample(os)[3],course.sample(os+ov*ov/(2*brake)+.1)[3])
                nov=max(0.,ov+max(-brake*h,min(accel*h,oref-ov)))
                turn=nv/wb*math.tan(steer)*h
                newpose=(pose[0]+nv*math.cos(pose[2]+turn/2)*h,pose[1]+nv*math.sin(pose[2]+turn/2)*h,pose[2]+turn)
                nos=os+nov*h;newopose=course.sample(nos)[:3]
                newshape=body_shape(newpose,length,width,rear);newoshape=body_shape(newopose,length,width,rear)
                ego_sweep=sweep(shape,newshape,pose,newpose,radius);other_sweep=sweep(oshape,newoshape,opose,newopose,radius)
                if not usable.contains(ego_sweep) or not usable.contains(other_sweep):fault='boundary_violation'
                elif ego_sweep.distance(other_sweep)<=margin:fault='vehicle_conflict'
                pose,opose,shape,oshape,v,ov,os=newpose,newopose,newshape,newoshape,nv,nov,nos
                terminal_time=now+(substep+1)*h
                if fault:break
            s,_=course.project(pose,s,max(1.,v*dt*3))
            if fault:reason=fault;note='車体の掃引領域・余裕の抵触';break
        if not frames:raise ValueError('試走開始失敗: '+reason+' '+note)
        # Final frame corresponds to the actual terminal state, not a stale sample.
        terminal=list(frames[-1]);terminal[:5]=[terminal_time,*pose,v]
        if terminal_time<=frames[-1][0]+1e-9:
            frames.pop();rivals.pop();metrics.pop()
        frames.append(terminal);rivals.append([*opose,ov]);metrics.append(dict(state=state,gap=os-s-length,opponent_speed=ov,estimate=estimate,stable_seconds=stable,note=note,station=s%course.total,coverage=prediction['coverage'] if prediction else None))
        return dict(planner=planner_kind,observation='ideal',frames=frames,opponents=rivals,h2h=metrics,events=events,reason=reason,controller=method,total=course.total,zone=[zone_start,zone_end],minimum_follow_gap=minimum,vehicle=dict(length=length,width=width,rear=rear),planned_route=[list(route_point(plan[0]+i*(plan[1]-plan[0])/200,plan)[:2]) for i in range(201)] if plan else [])
    finally:
        if predictor is not None:
            if prediction_selector:prediction_selector.close()
            predictor.stdin.close()
            try:predictor.wait(timeout=2)
            except subprocess.TimeoutExpired:predictor.kill();predictor.wait()
            predictor.stdout.close()
        selector.close()
        proc.stdin.close()
        try:proc.wait(timeout=2)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        proc.stdout.close()
