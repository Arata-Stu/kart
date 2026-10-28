import math
import pytest
from kart_studio.tracking_preview import preview


def request(method='mpc'):
    return dict(closed=True,line=dict(points=[[3*math.cos(i*2*math.pi/100),3*math.sin(i*2*math.pi/100)] for i in range(100)]),options=dict(controller=method,speed=1.5,lookahead=.8,wheelbase=.257,maxSteer=.384,horizon=1,tracking=10,smoothness=1,steeringRate=1.5))


@pytest.mark.parametrize('method',['pure_pursuit','mpc'])
def test_circle(method):
    result=preview(request(method))
    assert result['reason']=='completed'
    assert max(row[6] for row in result['frames'])<.15
    assert all(abs(row[5])<=.384+1e-9 for row in result['frames'])


def test_stopped_and_invalid():
    body=request();body['options']['speed']=0
    result=preview(body)
    assert result['reason']=='stopped_target'
    assert all(row[4]==0 for row in result['frames'])
    body['options']['horizon']=float('nan')
    with pytest.raises(ValueError): preview(body)


def test_step_profile():
    body=request('pure_pursuit')
    body['line']['speed_mode']='step'
    body['line']['profile']=[[0,0,0,0,0,.5 if i<50 else 1.] for i in range(100)]
    result=preview(body)
    assert result['reason']=='completed'
    assert set(row[11] for row in result['frames'])=={.5,1.}
