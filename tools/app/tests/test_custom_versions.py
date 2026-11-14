import copy
import json
from unittest.mock import patch

import pytest

from kart_studio import custom_versions, registration
from kart_studio.storage import atomic_json
from kart_studio.maps import Maps
from kart_studio.tuning import Tuning
from kart_studio.copies import copy_map
from types import SimpleNamespace


def controls(y=0):
    return dict(custom=[[0, y], [5, y]], custom_speeds=[], custom_speed_mode='profile')


def setup_map(tmp_path):
    maps = Maps(tmp_path / 'map')
    lane = dict(id='main', left=[[0, 1], [5, 1]], right=[[0, -1], [5, -1]],
                closed=False, lines={}, **controls())
    atomic_json(maps.root / 'course/map.json', dict(schema='kart.hdmap.v2', frame='map', revision=1,
                lanes=[lane], point_count=0, updated=0))
    atomic_json(maps.root / 'course/snapshot.json', {'frame': 'map'})
    for kind in ('centerline', 'customline'):
        maps.generate('course', kind, {'spacing': .5}, maps.load('course')['revision'], lambda: None)
    return maps


def add_version(maps, name, key):
    doc = maps.load('course'); lane = doc['lanes'][0]
    lane.setdefault('custom_versions', []).append(dict(id=key, name=name,
        **{field: copy.deepcopy(lane.get(field, 'profile' if field == 'custom_speed_mode' else [])) for field in custom_versions.FIELDS}))
    lane['custom_version'] = key
    return maps.save('course', doc)


def test_persist_generate_switch_and_live_snapshot(tmp_path):
    maps = setup_map(tmp_path)
    doc = add_version(maps, '守り', 'defense')
    first = copy.deepcopy(doc['lanes'][0]['lines']['customline'])
    assert doc['lanes'][0]['custom_versions'][0]['line'] == first
    doc['lanes'][0].update(**controls(.2), custom_version='')
    doc = maps.save('course', doc)
    assert 'customline' not in doc['lanes'][0]['lines']
    doc = add_version(maps, '攻め', 'attack')
    maps.generate('course', 'customline', {'spacing': .5, 'max_speed': .6}, doc['revision'], lambda: None)
    doc = maps.load('course'); lane = doc['lanes'][0]
    second = lane['custom_versions'][1]['line']
    assert second['settings']['max_speed'] == .6
    assert second != first
    lane.update(**controls(), custom_version='defense')
    # Generated geometry from a network client cannot override the trusted copy.
    lane['custom_versions'][0]['line']['points'][0][0] = 1234
    lane['lines'] = {}
    doc = maps.save('course', doc)
    assert doc['lanes'][0]['lines']['customline'] == first
    lane = doc['lanes'][0]
    lane.update(**controls(.2), custom_version='attack')
    lane['lines'] = {}
    doc = maps.save('course', doc)
    assert doc['lanes'][0]['lines']['customline'] == second
    # A real preparation validates/exports the selected line; the bank stays local.
    envelope = Tuning(maps).prepare(dict(id='course', document_revision=doc['revision'],
        lane_id='main', line_type='customline', centerline_speed_mps=.3))
    snapshot = json.loads(envelope['snapshot_json'])
    assert 'custom_versions' not in snapshot['document']['lanes'][0]
    assert snapshot['document']['lanes'][0]['lines']['customline'] == second
    tuning = Tuning(maps)
    with patch('kart_studio.tuning.remote', return_value={'map_id': snapshot['map_id']}) as remote:
        tuning.request(dict(id='course', snapshot=envelope['snapshot_json'],
            expected_revision='previous', connection={}), 'apply')
    assert remote.call_count == 1


def test_boundary_and_obstacle_edits_invalidate_all_generated_versions(tmp_path):
    maps = setup_map(tmp_path)
    doc = add_version(maps, '守り', 'defense')
    doc['obstacles'] = [dict(id='wall', polygon=[[10, 10], [11, 10], [11, 11]])]
    doc = maps.save('course', doc)
    lane = doc['lanes'][0]
    assert 'line' not in lane['custom_versions'][0]
    assert not lane['lines']
    maps.generate('course', 'customline', {'spacing': .5}, doc['revision'], lambda: None)
    doc = maps.load('course')
    assert 'line' in doc['lanes'][0]['custom_versions'][0]
    doc['lanes'][0]['left'][0][1] += .1
    doc = maps.save('course', doc)
    assert 'line' not in doc['lanes'][0]['custom_versions'][0]
    assert not doc['lanes'][0]['lines']


def test_untrusted_result_and_bad_versions_are_rejected(tmp_path):
    maps = setup_map(tmp_path)
    doc = add_version(maps, '守り', 'defense')
    lane = doc['lanes'][0]
    lane['custom_versions'][0].update(**controls(.5), line=lane['lines']['customline'])
    result = maps.save('course', doc)
    assert 'line' not in result['lanes'][0]['custom_versions'][0]
    for change in ('duplicate', 'selected', 'name', 'limit'):
        bad = copy.deepcopy(result)
        lane = bad['lanes'][0]
        if change == 'duplicate': lane['custom_versions'] *= 2
        elif change == 'selected': lane['custom_version'] = 'absent'
        elif change == 'name': lane['custom_versions'][0]['name'] = '\n'
        else: lane['custom_versions'] *= 33
        with pytest.raises(ValueError): maps.save('course', bad)


def test_registration_transforms_versions_and_requires_regeneration(tmp_path):
    maps = setup_map(tmp_path)
    doc = add_version(maps, '守り', 'defense')
    atomic_json(maps.root / 'target/map.json', doc)
    proposal = registration.preview(maps, 'target', dict(source='course', revision=doc['revision'], x=3, y=4, yaw=0))
    version = proposal['document']['lanes'][0]['custom_versions'][0]
    assert version['custom'] == [[3, 4], [8, 4]]
    assert 'line' not in version
    assert maps.load('course')['lanes'][0]['custom_versions'][0]['custom'] == [[0, 0], [5, 0]]


def test_copy_preserves_independent_version_bank(tmp_path):
    maps = setup_map(tmp_path)
    doc = add_version(maps, '守り', 'defense')
    atomic_json(maps.root / 'course/cloud.json', {})
    job = SimpleNamespace(check=lambda: None, log=lambda message: None)
    copy_map(maps, job, 'course', 'copy', doc['revision'])
    copied = maps.load('copy')
    assert copied['lanes'][0]['custom_versions'] == doc['lanes'][0]['custom_versions']
    copied['lanes'][0]['custom_versions'][0]['name'] = '変更版'
    maps.save('copy', copied)
    assert maps.load('course')['lanes'][0]['custom_versions'][0]['name'] == '守り'
