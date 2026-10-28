import math
from kart_studio.lines import profile, settings
from kart_studio.speed_sections import sample
from kart_studio.lanes import validate
from kart_hdmap.document import load
from kart_studio.storage import atomic_json


def test_step_boundary_and_no_curve_or_acceleration_regulation():
    points=[[0,0],[2,0],[2,2],[0,2]]
    limits=[dict(start=0,end=2,speed=2),dict(start=2,end=8,speed=.5)]
    line=sample(points,True,.1,limits)
    p=settings(dict(max_speed=1.,lateral_accel=.1,accel=.1,decel=.1))
    rows=profile(line,True,p,limits,True)
    for row in rows:
        assert row[5]==(2 if row[0]<2-1e-8 else .5)
    regulated=profile(line,True,p,limits)
    assert max(r[5] for r in regulated)<=1


def test_step_normalization_and_export_metadata(tmp_path):
    from test_studio import track
    from kart_studio.maps import Maps
    maps=Maps(tmp_path)
    atomic_json(tmp_path/'course/map.json',dict(track(),title='course',frame='map',revision=1,lines={},point_count=0,updated=0))
    maps.generate('course','centerline',{},1,lambda:None)
    d=maps.load('course');d['lanes'][0]['custom_speed_mode']='step'
    d=maps.save('course',d)
    maps.generate('course','customline',{},d['revision'],lambda:None)
    exported=maps.export('course')
    for path in (tmp_path/'course/map.json',exported/'hd_map.yaml'):
        assert load(path)['lanes'][0]['lines']['customline']['speed_mode']=='step'
    d=maps.load('course');d['lanes'][0]['custom_speed_mode']='profile'
    assert 'customline' not in maps.save('course',d)['lanes'][0]['lines']
