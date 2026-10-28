import json
import math
from dataclasses import replace

import pytest

from kart_parking.geometry import World, polygon, wrap, distance
from kart_parking.planner import (Settings, load_site, prepare, activate, connect,
                                  curve, arc, directed, course_samples)
from kart_parking.mission import Mission


@pytest.fixture
def setup():
    c=Settings()
    world=World([[-3,-3],[5,-3],[5,4],[-3,4]],[],c.front_m,c.rear_m,c.width_m,c.margin_m)
    zone=dict(goal=(2.,-1.,-math.pi/2),forward_yaw=0.,
              area=polygon([[1.5,-1.5],[2.5,-1.5],[2.5,-.5],[1.5,-.5]]),
              maneuver=polygon([[-1,-2],[3.5,-2],[3.5,.6],[-1,.6]]))
    points=[(0.,0.),(4.,0.),(4.,3.),(-2.,3.),(-2.,0.)]
    return c,world,zone,points


def test_prepare_has_forward_curved_connector(setup):
    c,w,z,p=setup
    plan=prepare(p,[1.]*len(p),True,z,w,c)
    assert distance(plan.connector[-1],z['goal'])<1e-8
    assert abs(wrap(plan.connector[-1][2]-z['goal'][2]))<1e-8
    assert w.swept(plan.connector,z['maneuver'])
    assert all(directed(s,z) for s in plan.connector)
    assert all(b[0]>=a[0]-1e-8 for a,b in zip(plan.connector,plan.connector[1:]))
    for a,b in zip(plan.connector,plan.connector[1:]):
        assert abs(wrap(b[2]-a[2]))/distance(a,b)<=c.curvature*1.01


def test_passed_departure_goes_forward_next_lap(setup):
    c,w,z,p=setup
    plan=prepare(p,[1.]*len(p),True,z,w,c)
    after=plan.course[(plan.departure+10)%len(plan.course)][:3]
    route=activate(plan,after,c)
    assert len(route)>len(plan.course)*.9
    assert max(x[1] for x in route)>2.9
    assert all(x[3]>=0 for x in route)
    assert route[-1][3]==0
    for a,b in zip(route,route[1:]):
        assert a[3]**2<=b[3]**2+2*c.deceleration_mps2*distance(a,b)+1e-8


def test_too_close_requires_lap(setup):
    c,w,z,p=setup
    plan=prepare(p,[1.]*len(p),True,z,w,c)
    route=activate(plan,plan.course[plan.departure][:3],c)
    assert len(route)>len(plan.course)


def test_wrong_course_heading_rejected(setup):
    c,w,z,p=setup
    plan=prepare(p,[1.]*len(p),True,z,w,c)
    with pytest.raises(ValueError,match='aligned'):
        activate(plan,(0.,0.,math.pi),c)


def test_wall_blocks_straight_and_turning_connectors(setup):
    c,w,z,p=setup
    w.obstacles=[polygon([[-1,-.6],[3.5,-.6],[3.5,-.4],[-1,-.4]])]
    assert curve((0.,0.,0.),z['goal'],c,w,z) is None
    with pytest.raises(ValueError,match='No forward'):
        prepare(p,[1.]*len(p),True,z,w,replace(c,search_timeout_s=.1,max_expansions=20))


def test_goal_backwards_is_rejected(setup):
    c,w,z,p=setup
    z=dict(z,goal=(2.,-1.,math.pi))
    with pytest.raises(ValueError):
        prepare(p,[1.]*len(p),True,z,w,replace(c,search_timeout_s=.1))


def test_swept_footprint_not_only_center():
    w=World([[-2,-2],[2,-2],[2,2],[-2,2]],[[[.2,-.1],[.3,-.1],[.3,.1],[.2,.1]]],.25,.1,.2,.02)
    assert not w.clear((0.,0.,0.))  # Rear axle is free, front of vehicle hits wall.
    assert not w.swept([(-.5,0.,0.),(.5,0.,0.)])  # Obstacle between samples.


def test_concave_boundary_and_enclosed_obstacle():
    w=World([[0,0],[3,0],[3,3],[2,3],[2,1],[1,1],[1,3],[0,3]],[],.8,.8,.1,.01)
    assert not w.clear((1.5,2.,0.))
    w=World([[-3,-3],[3,-3],[3,3],[-3,3]],[[[-.01,-.01],[.01,-.01],[.01,.01],[-.01,.01]]],.3,.3,.3,.01)
    assert not w.clear((0.,0.,0.))


def test_bad_polygons():
    with pytest.raises(ValueError): polygon([[0,0],[1,1],[1,0],[0,1]])
    with pytest.raises(ValueError): polygon([[0,0],[1,0],[0,0]])


def test_site_frame_and_goal_fit(tmp_path,setup):
    c,w,z,p=setup
    doc=dict(schema='kart.parking.v1',frame_id='map',drivable=w.outer,obstacles=[],slots={'P1':z})
    f=tmp_path/'site.json'; f.write_text(json.dumps(doc))
    load_site(f,'map','P1',c)
    with pytest.raises(ValueError,match='frame'): load_site(f,'odom','P1',c)
    doc['slots']['P1']['goal']=(2.,-.5,-math.pi/2);f.write_text(json.dumps(doc))
    with pytest.raises(ValueError,match='footprint'): load_site(f,'map','P1',c)


def mission_at_end(setup):
    c,w,z,p=setup
    route=[(2.,-.7,-math.pi/2,.2),(2.,-.85,-math.pi/2,.1),(2.,-1.,-math.pi/2,0.)]
    return Mission(route,z,w,c)


def test_requires_stop_dwell_and_keeps_latch(setup):
    m=mission_at_end(setup); goal=m.zone['goal']
    m.update(goal,.2,0.,1.,True)
    assert m.state=='settling' and not m.window()
    m.update(goal,0.,0.,2.,True); assert m.state=='settling'
    m.update(goal,0.,0.,2.9,False); assert m.state=='settling'
    m.update(goal,0.,0.,3.1,False); assert m.state=='parked'
    m.update(goal,.1,0.,3.2,True); assert m.state=='failed'
    m.update(goal,0.,0.,10.,False); assert m.state=='failed'


def test_rotation_does_not_count_as_stopped(setup):
    m=mission_at_end(setup); goal=m.zone['goal']
    m.update(goal,0.,.2,1.,True);m.update(goal,0.,.2,3.,True)
    assert m.state=='settling'


def test_drift_after_arrival_fails_without_restarting(setup):
    m=mission_at_end(setup)
    m.update(m.zone['goal'],0.,0.,1.,True)
    m.update((2.,-1.2,-math.pi/2),0.,0.,2.,False)
    assert m.state=='failed' and m.window()==[]


def test_wrong_pose_stall_not_parking_success(setup):
    m=mission_at_end(setup)
    m.update((2.,-.7,-math.pi/2),0.,0.,1.,True)
    m.update((2.,-.7,-math.pi/2),0.,0.,7.,True)
    assert m.state=='failed' and m.reason=='stalled_before_goal'


def test_clock_jump_invalidates(setup):
    m=mission_at_end(setup)
    m.update(m.zone['goal'],0.,0.,2.,True); m.update(m.zone['goal'],0.,0.,1.,True)
    assert m.reason=='clock_jump'


def test_lap_overlap_does_not_skip_to_goal(setup):
    c,w,z,p=setup;plan=prepare(p,[1.]*len(p),True,z,w,c)
    route=activate(plan,plan.course[plan.departure][:3],c)
    m=Mission(route,z,w,c,len(route)-len(plan.connector))
    m.update(route[0][:3],0.,0.,1.,True)
    assert m.index<3 and m.state=='approach'
    assert len(m.window())<len(route)


def test_invalid_inputs(setup):
    c,w,z,p=setup
    with pytest.raises(ValueError): course_samples(p,[1.]*len(p),False,c)
    with pytest.raises(ValueError): course_samples(p,[-1.]*len(p),True,c)
    with pytest.raises(ValueError): replace(c,max_steering_rad=float('nan')).validate()


def test_arc_curvature_and_forward(setup):
    c,*_=setup
    a=arc((0,0,0),c.curvature,.5,.01)
    assert a[-1][0]>0
    assert abs(a[-1][2]-.5*c.curvature)<1e-10


def test_lattice_detours_when_tangent_connections_hit_obstacle(setup):
    import time
    c,w,z,p=setup
    z=dict(z,maneuver=polygon([[-1,-2],[3.5,-2],[3.5,1.5],[-1,1.5]]))
    w.obstacles=[polygon([[.72,-.23],[.88,-.23],[.88,-.07],[.72,-.07]])]
    start=(0.,0.,0.)
    assert curve(start,z['goal'],c,w,z) is None
    path=connect(start,z,w,c,time.monotonic()+8.)
    assert path and w.swept(path,z['maneuver'])
    assert distance(path[-1],z['goal'])<1e-8
    assert all(directed(p,z) for p in path)


def test_auto_course_direction_tracks_selected_line_order():
    from kart_parking.planner import course_forward
    points=[(-4,0),(6,0),(6,4),(-4,4)]
    gate=[(1,-1),(3,-1)]
    assert course_forward(points,gate)==pytest.approx(0)
    assert abs(course_forward(list(reversed(points)),gate))==pytest.approx(math.pi)
