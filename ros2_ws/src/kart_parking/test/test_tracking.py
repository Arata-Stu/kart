"""Nominal kinematic integration with the production C++ pursuit calculation.
No tire dynamics, actuator lag, ROS scheduling or localization noise is modeled.
"""
import math
import shutil
import subprocess
from pathlib import Path
import pytest
from kart_parking.planner import Settings, load_site, curve, activate, prepare
from kart_parking.mission import Mission
from kart_parking.geometry import World, polygon, distance, wrap


@pytest.fixture(scope='module')
def probe(tmp_path_factory):
    compiler=shutil.which('c++')
    if not compiler: pytest.skip('C++ compiler unavailable')
    root=Path(__file__).resolve().parents[2]
    binary=tmp_path_factory.mktemp('probe')/'pursuit'
    subprocess.run([compiler,'-std=c++17','-O2','-I'+str(root/'kart_control/include'),
                    str(Path(__file__).with_name('pursuit_probe.cpp')),'-o',str(binary)],check=True)
    return binary


@pytest.mark.parametrize("passed", [False, True])
def test_generated_parking_curve_reaches_goal(probe, passed):
    c=Settings()
    site=Path(__file__).resolve().parents[1]/'config/site.example.json'
    world,zone=load_site(site,'map','P1',c)
    course=[(0.,0.),(7.,0.),(7.,3.),(-2.,3.),(-2.,0.)]
    points=prepare(course,[1.]*len(course),True,zone,world,c).connector
    assert points
    route=[(*p,c.parking_speed_mps) for p in points];route[-1]=(*points[-1],0.)
    for i in range(len(route)-2,-1,-1):
        route[i]=(*route[i][:3],min(route[i][3],math.sqrt(route[i+1][3]**2+2*c.deceleration_mps2*distance(route[i],route[i+1]))))
    departure=0
    if passed:
        world=World([[-4,-3],[5,-3],[5,6],[-4,6]],[],c.front_m,c.rear_m,c.width_m,c.margin_m)
        zone=dict(zone,maneuver=polygon([[-2,-2],[3.5,-2],[3.5,1.5],[-2,1.5]]))
        course=[(2*math.sin(2*math.pi*i/240),2-2*math.cos(2*math.pi*i/240)) for i in range(240)]
        plan=prepare(course,[.5]*len(course),True,zone,world,c)
        pose=plan.course[(plan.departure+20)%len(plan.course)][:3]
        route=activate(plan,pose,c)
        departure=len(route)-len(plan.connector)
    else:
        pose=points[0]
    mission=Mission(route,zone,world,c,departure)
    speed=0.;omega=0.;dt=.02
    with subprocess.Popen([str(probe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as proc:
        proc.stdin.write(str(len(route))+'\n')
        for x,y,a,v in route: proc.stdin.write(f'{x} {y} {v}\n')
        proc.stdin.flush()
        for tick in range(4000):
            mission.update(pose,speed,omega,1+tick*dt,True)
            assert mission.state!='failed',mission.reason
            if mission.state=='parked': break
            if mission.state=='settling':
                speed=omega=0.;continue
            window=mission.window()
            lookahead=.25 if mission.approaching_parking() else .8
            proc.stdin.write(f'{mission.index} {mission.index+len(window)} {pose[0]} {pose[1]} {pose[2]} {lookahead}\n');proc.stdin.flush()
            valid,steering,speed=map(float,proc.stdout.readline().split())
            assert valid
            omega=speed*math.tan(steering)/c.wheelbase_m
            pose=(pose[0]+dt*speed*math.cos(pose[2]),pose[1]+dt*speed*math.sin(pose[2]),wrap(pose[2]+dt*omega))
        proc.stdin.close()
        proc.wait(timeout=5)
    assert mission.state=='parked'
    assert distance(pose,zone['goal'])<c.goal_position_m
    assert abs(wrap(pose[2]-zone['goal'][2]))<c.goal_yaw_rad
