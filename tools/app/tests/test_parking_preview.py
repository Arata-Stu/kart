import math
import pytest
from kart_studio.parking_preview import preview


def document():
    points=[[-4,0],[6,0],[6,4],[-4,4]]
    return dict(schema='kart.hdmap.v2',frame='map',lanes=[dict(id='main',closed=True,centerline_speed_mps=.5,
        left=[[-3,1],[5,1],[5,3],[-3,3]],right=[[-5,-1],[7,-1],[7,5],[-5,5]],lines={'centerline':{'points':points}})],
        parking_areas=[dict(id='P1',polygon=[[1,-2.5],[3,-2.5],[3,-1],[1,-1]],entrance=[[1,-1],[3,-1]],goal_pose=[2,-1.8,-math.pi/2])])


def test_parking_preview_reaches_goal_and_passed_bay_takes_lap():
    result=preview(document(),'main','centerline','P1',[5,0,0])
    assert result['reason']=='planned_goal'
    assert max(p[1] for p in result['route'])>=4
    assert result['frames'][-1][1:4]==pytest.approx([2,-1.8,-math.pi/2])
    assert result['frames'][-1][4]==0
    assert result['parking_speed_mps']==.25
    assert all(p[3]<=.25 for p in result['route'] if p[1]<-1)


def test_wrong_heading_rejected():
    with pytest.raises(ValueError,match='aligned'):
        preview(document(),'main','centerline','P1',[0,0,math.pi])


@pytest.mark.parametrize('method',['pure_pursuit','mpc'])
def test_actual_controller_and_mission_reach_parking_goal(method):
    from pathlib import Path
    from kart_parking.planner import Settings,load_site,prepare
    from kart_studio.parking_follow import follow
    c=Settings()
    path=Path(__file__).resolve().parents[3]/'ros2_ws/src/kart_parking/config/site.example.json'
    world,zone=load_site(path,'map','P1',c)
    course=[(0.,0.),(7.,0.),(7.,3.),(-2.,3.),(-2.,0.)]
    plan=prepare(course,[1.]*len(course),True,zone,world,c)
    route=[(*p,.25) for p in plan.connector];route[-1]=(*route[-1][:3],0.)
    cfg=dict(controller=method,speed=1.5,wheelbase=.257,maxSteer=.3839724354,horizon=1,tracking=10,smoothness=1,steeringRate=1.5)
    result=follow(route,zone,world,c,0,route[0][:3],cfg)
    assert result['reason']=='simulated_parked',result['reason']
    assert result['frames'][-1][4]==0
    assert max(f[4] for f in result['frames'])<=.25
