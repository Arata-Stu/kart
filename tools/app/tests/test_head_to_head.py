import math
import pytest
from kart_studio.head_to_head import preview, Course


def request():
    # CCW stadium: long straights and explicitly separated inner/outer rings.
    def ring(radius):
        return ([[i,6-radius] for i in range(0,101,2)] +
                [[100+radius*math.cos(-math.pi/2+i*math.pi/60),6+radius*math.sin(-math.pi/2+i*math.pi/60)] for i in range(1,61)] +
                [[i,6+radius] for i in range(98,-1,-2)] +
                [[radius*math.cos(math.pi/2+i*math.pi/60),6+radius*math.sin(math.pi/2+i*math.pi/60)] for i in range(1,60)])
    points=ring(6)
    return dict(lane=dict(closed=True,left=ring(4),right=ring(8),lines=dict(centerline=dict(points=points))),line_type='centerline',obstacles=[],options=dict(controller='pure_pursuit',speed=5,wheelbase=.257,maxSteer=.384,laps=1),head_to_head=dict(planner='baseline',opponent_speed=2,initial_gap=3,observe_seconds=1,duration=30,zone_start=10,zone_end=90,overtake=True))


@pytest.mark.parametrize('method',['pure_pursuit','mpc'])
def test_pass_and_merge(method):
    body=request();body['options']['controller']=method
    result=preview(body)
    assert result['reason']=='overtake_completed', (result['reason'], result['h2h'][-1])
    states={row['state'] for row in result['h2h']}
    assert {'FOLLOW_OBSERVE','PASS','MERGE','PASSED'}<=states
    assert result['h2h'][-1]['gap'] < -1
    assert len(result['frames'])==len(result['opponents'])==len(result['h2h'])
    assert all(math.isfinite(x) for row in result['frames'] for x in row)


def test_follow_only():
    body=request();body['head_to_head'].update(overtake=False,duration=15)
    result=preview(body)
    assert result['reason']=='time_limit'
    assert result['minimum_follow_gap']>=1
    assert 'PASS' not in {m['state'] for m in result['h2h']}
    last=result['h2h'][-1]
    assert last['estimate']==pytest.approx(2)
    assert result['frames'][-1][4]==pytest.approx(2,abs=.1)


@pytest.mark.parametrize('case',['faster','short','blocked'])
def test_decline(case):
    body=request();body['head_to_head']['duration']=18
    if case=='faster':body['head_to_head']['opponent_speed']=6
    if case=='short':body['head_to_head']['zone_end']=20
    if case=='blocked':body['head_to_head']['offset']=2.5
    result=preview(body)
    assert 'PASS' not in {m['state'] for m in result['h2h']}
    assert result['reason']=='time_limit'


def test_stopped_opponent():
    body=request();body['head_to_head'].update(opponent_speed=0,overtake=False,duration=10)
    result=preview(body)
    assert result['reason']=='time_limit'
    assert result['minimum_follow_gap']>=.9
    assert result['frames'][-1][4]<.05


def test_obstacle_failure():
    body=request();body['head_to_head'].update(overtake=False,duration=10)
    body['obstacles']=[dict(polygon=[[8,-2],[9,-2],[9,2],[8,2]])]
    result=preview(body)
    assert result['reason']=='boundary_violation'
    diagnostic=result['diagnostic']
    assert diagnostic['hits']
    assert all(hit['actor'] in ('自車','相手') for hit in diagnostic['hits'])
    assert any(hit['outside']['coordinates'] for hit in diagnostic['hits'])
    assert diagnostic['surface']['type'] in ('Polygon','MultiPolygon')
    import json
    json.dumps(diagnostic,allow_nan=False)


@pytest.mark.parametrize('key,value',[('opponent_speed',float('nan')),('initial_gap',-1),('zone_end',1000),('side','unknown'),('overtake',1)])
def test_invalid(key,value):
    body=request();body['head_to_head'][key]=value
    with pytest.raises(ValueError):preview(body)


def test_project_lap_boundary():
    body=request();course=Course(body['lane']['lines']['centerline'],True,5)
    previous=course.total-.05
    pose=course.sample(course.total+.1)[:3]
    s,error=course.project(pose,previous,1)
    assert s==pytest.approx(course.total+.1)
    assert error<1e-6


def test_saved_speed_profile_limits_passing():
    body=request();points=body['lane']['lines']['centerline']['points']
    body['lane']['lines']['centerline']['profile']=[[0,0,0,0,0,2] for p in points]
    body['head_to_head']['duration']=15
    result=preview(body)
    assert max(f[4] for f in result['frames'])<=2+1e-8
    assert 'PASS' not in {m['state'] for m in result['h2h']}


def test_swept_conflict_between_samples():
    from kart_studio.head_to_head import body_shape, sweep
    a=(-1,0,0);b=(1,0,0)
    sa=body_shape(a,.35,.19,.1065);sb=body_shape(b,.35,.19,.1065)
    obstacle=body_shape((0,0,0),.35,.19,.1065)
    assert not sa.intersects(obstacle) and not sb.intersects(obstacle)
    assert sweep(sa,sb,a,b,.3).intersects(obstacle)


def test_open_course_rejected():
    body=request();body['lane']['closed']=False
    with pytest.raises(ValueError,match='閉コース'):preview(body)


def test_h2h_fixed_lookahead():
    body=request();body['options'].update(lookahead_mode='fixed',lookahead_m=1.1)
    body['head_to_head'].update(overtake=False,duration=5)
    result=preview(body)
    assert all(row[12]==pytest.approx(1.1) for row in result['frames'])


def test_predictive_learn_pass_merge():
    body=request();body['options']['laps']=3
    body['head_to_head'].update(planner='predictive_spliner',duration=300,initial_gap=6)
    result=preview(body)
    assert result['planner']=='predictive_spliner'
    assert result['observation']=='ideal'
    assert result['reason']=='overtake_completed',result['h2h'][-1]
    assert result['h2h'][0]['coverage']<.1
    assert result['h2h'][-1]['coverage']>=.9
    evaluated=[h['distance_plan'] for h in result['h2h'] if h.get('distance_plan')]
    assert evaluated and any(d['accepted'] for d in evaluated)
    assert all(d['ramp']>0 and d['end']>d['start'] for d in evaluated)
    assert result['events'][0]['time']>100  # cannot pass before observing the lap
    assert result['h2h'][-1]['gap']< -1
    assert {'FOLLOW_OBSERVE','PASS','MERGE','PASSED'}<={m['state'] for m in result['h2h']}


def test_predictive_untrained_follow():
    body=request();body['head_to_head'].update(planner='predictive_spliner',duration=10)
    result=preview(body)
    assert result['reason']=='time_limit'
    assert not result['events']
    assert result['minimum_follow_gap']>=1
    assert all(m['state']=='FOLLOW_OBSERVE' for m in result['h2h'])


def test_unknown_planner_rejected():
    body=request();body['head_to_head']['planner']='unknown'
    with pytest.raises(ValueError,match='planner'):preview(body)


def test_studio_overtake_sample():
    import json
    from pathlib import Path
    source=(Path(__file__).parents[1]/'frontend/samples/overtake.js').read_text()
    body=json.loads(source.removeprefix('export default ').strip().removesuffix(';'))
    result=preview(body)
    assert result['reason']=='overtake_completed',result['h2h'][-1]
    assert {'FOLLOW_OBSERVE','PASS','MERGE','PASSED'}<={h['state'] for h in result['h2h']}
    assert result['h2h'][-1]['coverage']>=.9
    assert result['h2h'][-1]['gap']< -1


def test_wrapped_passing_window():
    from kart_studio.head_to_head import passing_window
    assert passing_window(95,100,90,20)==(True,120)
    assert passing_window(105,100,90,20)==(True,120)
    assert passing_window(120,100,90,20)==(False,120)
    assert passing_window(195,100,90,20)==(True,220)


def test_predictive_pass_across_seam():
    body=request()
    for key in ('left','right'):
        pts=body['lane'][key];body['lane'][key]=pts[25:]+pts[:25]
    line=body['lane']['lines']['centerline'];pts=line['points'];line['points']=pts[25:]+pts[:25]
    total=Course(line,True,5).total
    body['options']['laps']=4
    body['head_to_head'].update(planner='predictive_spliner',duration=300,initial_gap=6,opponent_speed=3,zone_start=total-10,zone_end=20)
    result=preview(body)
    assert result['reason']=='overtake_completed',result['h2h'][-1]
    assert result['events'][0]['station']%total>total-10
    assert result['h2h'][-1]['station']<20


def test_short_zone_configurable():
    body=request();body['head_to_head'].update(zone_start=10,zone_end=12,duration=5)
    with pytest.raises(ValueError,match='5〜500'):preview(body)
    body['head_to_head']['min_zone_length']=1
    result=preview(body)
    assert result['reason']=='time_limit'
    assert not result['events']  # Input acceptance does not bypass feasibility.


def test_stopped_opponent_bypass_without_gp_lap():
    body = request()
    body["head_to_head"].update(planner="predictive_spliner", opponent_speed=0, initial_gap=8,
                                zone_start=0, zone_end=40)
    result = preview(body)
    assert result["reason"] == "overtake_completed", result["h2h"][-1]
    assert {"AVOID_STATIC", "MERGE", "PASSED"} <= {h["state"] for h in result["h2h"]}
    assert result["h2h"][-1]["coverage"] < .1


@pytest.mark.parametrize("sides,chosen", [([], "left"), ([1], "right"), ([-1], "left"), ([1, -1], None)])
def test_box_blocks_one_or_both_passes(sides, chosen):
    body = request()
    body["head_to_head"].update(planner="predictive_spliner", opponent_speed=0, initial_gap=8,
                                zone_start=0, zone_end=40, side="left")
    body["obstacles"] = [{"polygon": [[x, sign*y] for x,y in
                         [(7.8,.24),(8.3,.24),(8.3,.64),(7.8,.64)]]} for sign in sides]
    result = preview(body)
    if chosen:
        assert result["reason"] == "overtake_completed"
        first = result["events"][0]
        assert first["side"] == chosen
        assert first["candidates"] == {"left": 1 not in sides, "right": -1 not in sides}
    else:
        assert result["reason"] == "blocked_stopped"
        assert not result["events"]
        assert result["h2h"][-1]["state"] == "BLOCKED"
        assert result["frames"][-1][4] < .001
