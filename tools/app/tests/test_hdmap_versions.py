import copy
import json
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from kart_studio.copies import copy_map
from kart_studio.hdmap_versions import HDMapVersions
from kart_studio.storage import atomic_json
from kart_studio.tuning import Tuning
from test_custom_versions import setup_map


def create(versions, title='通常'):
    return versions.create(dict(id='course', name=title, revision=versions.maps.load('course')['revision'],
                                lane_id='main', line_type='customline'))


def restore(versions, version_id, key='course'):
    return versions.restore(dict(id=key, version_id=version_id, revision=versions.maps.load(key)['revision'],
                                 lane_id='main', line_type='customline'))


def test_whole_map_snapshots_restore_all_lanes_and_preserve_localization(tmp_path):
    maps = setup_map(tmp_path); tuning = Tuning(maps)
    versions = HDMapVersions(maps, tuning.identity)
    doc = maps.load('course')
    doc['lanes'].append(dict(copy.deepcopy(doc['lanes'][0]), id='secondary'))
    doc['obstacles'] = [dict(id='wall', polygon=[[10, 10], [11, 10], [11, 11]])]
    doc.update(parking_drivable=[[-1, -2], [6, -2], [6, 2], [-1, 2]], parking_areas=[dict(
        id='P1', polygon=[[1, -1.5], [2, -1.5], [2, -.5], [1, -.5]],
        goal_pose=[1.5, -1, 0], entrance=[[1, -.5], [2, -.5]])])
    # Valid server-owned input also includes generated lines for both lanes.
    atomic_json(maps.root / 'course/map.json', doc)
    original = maps.load('course')
    normal = create(versions)
    assert normal['generated'] and normal['lane_count'] == 2
    snapshot_bytes = (maps.root / 'course/snapshot.json').read_bytes()
    changed = copy.deepcopy(original)
    changed['lanes'] = [changed['lanes'][0]]
    changed['lanes'][0]['custom'][0][1] = .2
    changed['obstacles'] = []
    changed.update(parking_areas=[], parking_drivable=[])
    changed = maps.save('course', changed)
    attack = create(versions, '攻め')
    result = restore(versions, normal['id'])
    assert result['document']['revision'] == changed['revision'] + 1
    assert result['document']['lanes'] == original['lanes']
    assert result['document']['obstacles'] == original['obstacles']
    assert result['document']['parking_areas'] == original['parking_areas']
    assert result['document']['parking_drivable'] == original['parking_drivable']
    assert result['document'].get('title') == original.get('title')
    assert (maps.root / 'course/snapshot.json').read_bytes() == snapshot_bytes
    assert {p.suffix for p in versions.folder('course').iterdir()} == {'.json'}
    previous = versions.read('course', 'previous')
    assert previous['document']['lanes'] == changed['lanes']
    back = restore(versions, 'previous')
    assert back['document']['lanes'] == changed['lanes']
    assert len(versions.list('course')) == 3  # Two named versions and one reversible working copy.
    assert versions.read('course', normal['id'])['document']['lanes'] == original['lanes']
    assert versions.read('course', attack['id'])['document']['lanes'] == changed['lanes']


def test_restored_snapshot_uses_existing_validated_live_apply_and_stale_guards(tmp_path):
    maps = setup_map(tmp_path); tuning = Tuning(maps)
    versions = HDMapVersions(maps, tuning.identity)
    normal = create(versions)
    doc = maps.load('course'); doc['lanes'][0]['centerline_speed_mps'] = .7
    maps.save('course', doc)
    restored = restore(versions, normal['id'])['document']
    envelope = tuning.prepare(dict(id='course', document_revision=restored['revision'],
        lane_id='main', line_type='customline', centerline_speed_mps=.3))
    candidate = json.loads(envelope['snapshot_json'])
    assert candidate['document']['hdmap_version']['name'] == '通常'
    with patch('kart_studio.tuning.remote', return_value=dict(map_id=tuning.identity('course'))) as remote:
        tuning.request(dict(id='course', snapshot=envelope['snapshot_json'], expected_revision='r1', connection={}), 'apply')
        assert remote.call_args.args[2]['expected_revision'] == 'r1'
    maps.save('course', maps.load('course'))
    with patch('kart_studio.tuning.remote') as remote, pytest.raises(ValueError, match='地図が変わりました'):
        tuning.request(dict(id='course', snapshot=envelope['snapshot_json'], expected_revision='r1', connection={}), 'apply')
    remote.assert_not_called()


def test_versions_reject_stale_tampered_foreign_maps_and_bad_names(tmp_path):
    maps = setup_map(tmp_path); tuning = Tuning(maps)
    versions = HDMapVersions(maps, tuning.identity)
    normal = create(versions)
    for title in ('', ' 通常', '通常', 'a\nb'):
        with pytest.raises(ValueError): create(versions, title)
    with pytest.raises(ValueError):
        versions.restore(dict(id='course', version_id=normal['id'], revision=0, lane_id='main', line_type='customline'))
    with pytest.raises(ValueError): restore(versions, '../other')
    row = versions.read('course', normal['id'])
    row['map_id'] = 'other-map'
    atomic_json(versions.folder('course') / (normal['id'] + '.json'), row)
    assert not versions.list('course')[0]['compatible']
    with pytest.raises(ValueError, match='自己位置推定用地図'): restore(versions, normal['id'])
    row['document']['lanes'][0]['left'][0][0] = 100
    atomic_json(versions.folder('course') / (normal['id'] + '.json'), row)
    with pytest.raises(ValueError, match='変更されています'): versions.read('course', normal['id'])


def test_independent_map_copies_keep_shared_map_versions(tmp_path):
    maps = setup_map(tmp_path); tuning = Tuning(maps)
    versions = HDMapVersions(maps, tuning.identity)
    normal = create(versions)
    atomic_json(maps.root / 'course/cloud.json', {})
    job = SimpleNamespace(check=lambda: None, log=lambda message: None)
    copy_map(maps, job, 'course', 'copy', maps.load('course')['revision'])
    copied_versions = versions.list('copy')
    assert copied_versions[0]['compatible']
    result = restore(versions, normal['id'], key='copy')
    assert result['document']['revision'] == 2
    assert 'hdmap_version' not in maps.load('course')
