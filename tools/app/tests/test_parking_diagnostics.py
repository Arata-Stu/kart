from shapely.geometry import Polygon
from kart_hdmap.parking_geometry import SurfaceWorld
from kart_studio.parking_diagnostics import clearance_diagnostic


def world(obstacles=(), margin=.1):
    surface = Polygon([(-3,-3),(3,-3),(3,3),(-3,3)], holes=[[(1,1),(2,1),(2,2),(1,2)]])
    return SurfaceWorld(surface, list(obstacles), .3, .1, .2, margin)


def test_clear_and_hole():
    w = world()
    assert w.clear((0,0,0))
    assert clearance_diagnostic(w,(0,0,0))['hits'] == []
    pose = (1.2,1.5,0)
    assert not w.clear(pose)
    d = clearance_diagnostic(w,pose)
    assert len(d['surface']['coordinates']) == 2
    assert d['hits'][0]['kind'] == '走行可能領域の境界'
    assert d['hits'][0]['outside']['coordinates']


def test_margin_without_contact():
    w = world()
    pose = (2.65,0,0)
    assert not w.clear(pose)
    hit = clearance_diagnostic(w,pose)['hits'][0]
    assert 0 < hit['distance_m'] < w.margin
    assert not hit['outside']['coordinates']


def test_enclosed_obstacle_and_goal_domain():
    w = world([[(-.01,-.01),(.01,-.01),(.01,.01),(-.01,.01)]])
    assert not w.clear((0,0,0))
    assert clearance_diagnostic(w,(0,0,0))['hits'][0]['kind'] == '障害物 1'
    w = world()
    domain = [(-.05,-.05),(.05,-.05),(.05,.05),(-.05,.05)]
    assert not w.clear((0,0,0),domain)
    assert clearance_diagnostic(w,(0,0,0),domain)['hits'][0]['kind'] == '駐車枠の境界'
