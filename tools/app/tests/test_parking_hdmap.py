"""Studio save/export/registration → shared HDMap → planner contract."""
import copy
import math
from pathlib import Path
import pytest
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json, read_json
from kart_studio import parking, registration
from kart_hdmap.document import load
from kart_parking.planner import load_site, Settings
from kart_hdmap.parking import entrance_path
from test_studio import track


def parking_fields():
    return dict(parking_drivable=[[-10,-10],[10,-10],[10,10],[-10,10]],parking_areas=[
        dict(id='P1',polygon=[[1.5,-1.5],[2.5,-1.5],[2.5,-.5],[1.5,-.5]],
             goal_pose=[2,-1,-math.pi/2],entrance=[[1.5,-.5],[2.5,-.5]],
             maneuver=[[-1,-2],[3.5,-2],[3.5,.6],[-1,.6]],forward_yaw=0.)])


@pytest.mark.parametrize("derived", [False, True])
def test_roundtrip_save_export_planner_and_registration(tmp_path, derived):
    maps=Maps(tmp_path)
    seed=track();seed['left']=[[x*.5,y*.5] for x,y in seed['left']]
    atomic_json(tmp_path/'course/map.json',dict(seed,title='course',frame='map',revision=1,lines={},point_count=0,updated=0))
    maps.generate('course','centerline',{},1,lambda:None)
    original=maps.load('course')
    original.update(parking_fields())
    if derived: original['parking_drivable']=[]
    saved=maps.save('course',original)
    assert saved['lanes'][0]['lines']  # Parking edits do not invalidate racing geometry.
    export=maps.export('course')
    for path in (tmp_path/'course/map.json',export/'hd_map.yaml'):
        doc=load(path)
        assert doc['parking_areas'][0]['id']=='P1'
        world,zone=load_site(path,'map','P1',Settings())
        assert zone['goal']==(2.,-1.,-math.pi/2)
        assert zone['entrance']==[[1.5,-.5],[2.5,-.5]]
        assert world.clear(zone['goal'],zone['area'])
    assert read_json(export/'metadata.json')['parking_warnings']
    atomic_json(tmp_path/'target/map.json',maps.load('course'))
    proposal=registration.preview(maps,'target',dict(source='course',revision=saved['revision'],x=1,y=2,yaw=90))
    transformed=proposal['document']['parking_areas'][0]
    assert transformed['goal_pose']==pytest.approx([2,4,0])
    assert 'forward_yaw' not in transformed
    # A client omitting optional attributes must not erase existing parking data.
    old_client=maps.load('course');old_client.pop('parking_areas');old_client.pop('parking_drivable')
    assert maps.save('course',old_client)['parking_areas']


def test_bad_gate_goal_wall_and_duplicate():
    for mutate in (
        lambda d:d['parking_areas'][0].update(entrance=[[1.5,-.5],[2.5,-1.5]]),
        lambda d:d['parking_areas'][0].update(goal_pose=[9,9,0]),
        lambda d:d.update(obstacles=[dict(id='wall',polygon=[[1,-.6],[3,-.6],[3,-.4],[1,-.4]])]),
        lambda d:d['parking_areas'].append(copy.deepcopy(d['parking_areas'][0])),
        lambda d:d.update(parking_drivable=[]),
    ):
        d=parking_fields();mutate(d)
        with pytest.raises(ValueError): parking.validate(d)


def test_missing_parking_is_warning_not_export_prohibition():
    assert parking.validate({})==dict(parking_areas=[],parking_drivable=[])
    assert 'P1' in parking.warnings({})[0]
    d=parking_fields();d['parking_areas']=[dict(d['parking_areas'][0],id=k) for k in ('P1','P2','P3')]
    assert not parking.warnings(d)


def test_entrance_required_and_wrong_side_rejected():
    d=parking_fields()['parking_areas'][0]
    z=dict(area=d['polygon'],entrance=d['entrance'])
    assert entrance_path([(2,0,0),(2,-.6,0),(2,-1,0)],z)
    assert not entrance_path([(1,-1,0),(1.6,-1,0),(2,-1,0)],z)
    assert not entrance_path([(2,0,0),(2,-.6,0),(2,0,0),(2,-1,0)],z)


@pytest.mark.parametrize("derived", [False, True])
def test_embedded_map_to_forward_parking_plan(tmp_path, derived):
    from kart_parking.planner import prepare, activate
    doc=dict(schema='kart.hdmap.v2',frame='map',**parking_fields())
    row=doc['parking_areas'][0]
    row.update(polygon=[[1,-2.5],[3,-2.5],[3,-1],[1,-1]],
               entrance=[[1,-1],[3,-1]],goal_pose=[2,-1.8,-math.pi/2])
    row.pop('maneuver');row.pop('forward_yaw')
    if derived:
        doc['parking_drivable']=[]
        doc['lanes']=[dict(id='main',closed=True,
            left=[[-3,1],[5,1],[5,3],[-3,3]],
            right=[[-5,-1],[7,-1],[7,5],[-5,5]])]
    path=tmp_path/'map.json';atomic_json(path,doc)
    world,zone=load_site(path,'map','P1',Settings())
    course=[(-4,0),(6,0),(6,4),(-4,4)]
    prepared=prepare(course,[.5]*len(course),True,zone,world,Settings())
    assert entrance_path(prepared.connector,zone)
    start=prepared.course[(prepared.departure+20)%len(prepared.course)][:3]
    route=activate(prepared,start,Settings())
    assert max(p[1] for p in route)>3.9  # next forward lap
    assert route[-1][3]==0.
    assert route[-1][:3]==pytest.approx(zone['goal'])


def derived_doc():
    d=dict(schema='kart.hdmap.v2',frame='map',**parking_fields())
    d['parking_drivable']=[]
    d['lanes']=[dict(id='main',closed=True,
        left=[[-4,0],[4,0],[4,8],[-4,8]],
        right=[[-5,-.5],[5,-.5],[5,9],[-5,9]])]
    return d


def test_derived_surface_holes_gate_walls_and_export_keys(tmp_path):
    from kart_hdmap.parking_space import surface
    from shapely.geometry import Point
    d=derived_doc()
    parking.validate(d)
    path=tmp_path/'derived.json';atomic_json(path,d)
    world,zone=load_site(path,'map','P1',Settings())
    assert world.clear((0,-.25,0))
    assert world.clear((2,-.5,-math.pi/2))  # seam is open
    assert world.clear(zone['goal'],zone['area'])
    assert not world.clear((0,4,0))  # middle island is not free
    assert not world.clear((8,4,0))
    d['obstacles']=[dict(polygon=[[1.9,-.7],[2.1,-.7],[2.1,-.6],[1.9,-.6]])]
    atomic_json(path,d)
    world,_=load_site(path,'map','P1',Settings())
    assert not world.clear((2,-.5,-math.pi/2))  # obstacles survive union
    lane=d['lanes'][0]
    exported=dict(d,lanes=[dict(id='main',closed_loop=True,left_bound=lane['left'],right_bound=lane['right'])])
    assert surface(d,d['parking_areas']).equals(surface(exported,exported['parking_areas']))
    assert not surface(d,d['parking_areas']).contains(Point(0,4))


def test_derived_rejects_disconnected_and_intersecting_bounds():
    d=derived_doc()
    d['parking_areas'][0]['maneuver']=[[1.4,-1.6],[2.6,-1.6],[2.6,-.51],[1.4,-.51]]
    with pytest.raises(ValueError): parking.validate(d)
    d=derived_doc();d['lanes'][0]['left'][0]=[-6,0]
    with pytest.raises(ValueError): parking.validate(d)


def test_derived_open_bound_orientation():
    from kart_hdmap.parking_space import surface
    from shapely.geometry import Point
    d=dict(lanes=[dict(closed=False,left=[[0,1],[5,1]],right=[[5,-1],[0,-1]])])
    s=surface(d,[])
    assert s.contains(Point(2,0))
    assert not s.contains(Point(2,2))


def test_shapely_import_error_is_actionable(monkeypatch):
    import builtins
    from kart_hdmap.parking_space import surface
    original = builtins.__import__

    def broken(name, *args, **kwargs):
        if name.startswith('shapely'):
            raise ImportError('numpy.core.multiarray failed to import')
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', broken)
    with pytest.raises(ValueError, match='描き直しは不要'):
        surface({}, [])


def test_default_space_and_optional_restriction(tmp_path):
    from kart_hdmap.parking_space import surface
    d=derived_doc();row=d['parking_areas'][0]
    row.pop('forward_yaw');row.pop('maneuver')
    fields=parking.validate(d)
    assert fields['parking_areas'][0]['maneuver']==[]
    assert 'forward_yaw' not in fields['parking_areas'][0]
    default=surface(d,[row])
    row['maneuver']=[[-100,-100],[100,-100],[100,100],[-100,100]]
    assert surface(d,[row]).equals(default)  # Purple cannot authorize extra space.
    path=tmp_path/'map.json';atomic_json(path,d)
    world,zone=load_site(path,'map','P1',Settings())
    assert not world.clear((8,4,0),zone['maneuver'])
    row['polygon']=[[x,y-.1] for x,y in row['polygon']]
    row['entrance']=[[x,y-.1] for x,y in row['entrance']]
    row['goal_pose'][1]-=.1
    with pytest.raises(ValueError,match='入口が通常コースに接続'):
        parking.validate(d)  # Purple must not bridge a gap, even in old files.
