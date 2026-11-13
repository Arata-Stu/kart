import copy
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_studio.tuning import Tuning, remote, _protocol
from kart_studio.maps import Maps
from kart_studio.storage import atomic_json
from kart_studio.lines import generate


CONNECTION = dict(user='kart',host='192.168.1.2',port=2222,record_root='/record',map_root='/map')


class TuningBackendTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.maps = Maps(root)
        folder = root/'course'
        folder.mkdir()
        lane = dict(id='main', left=[[0,1],[5,1]],right=[[0,-1],[5,-1]],closed=False,custom=[[0,0],[5,0]],lines={})
        for kind in ('centerline','customline'):
            lane['lines'][kind] = generate(lane, kind, dict(spacing=.5))
        self.doc = dict(schema='kart.hdmap.v2',frame='map',revision=1,lanes=[lane],obstacles=[])
        atomic_json(folder/'map.json', self.doc)
        atomic_json(folder/'snapshot.json', dict(frame='map'))
        self.tuning = Tuning(self.maps)
        self.body = dict(id='course',document_revision=1,lane_id='main',line_type='customline',centerline_speed_mps=.3,connection=CONNECTION)

    def test_prepare_reuses_export_validation_and_preserves_section_profile(self):
        prepared = self.tuning.prepare(self.body)
        self.assertEqual(set(prepared), {'snapshot_json'})
        candidate = json.loads(prepared['snapshot_json'])
        self.assertEqual(candidate['document']['lanes'][0]['lines']['customline']['profile'],self.doc['lanes'][0]['lines']['customline']['profile'])
        self.assertTrue((self.maps.folder('course')/'exports').is_dir())
        with self.assertRaises(ValueError): self.tuning.prepare({**self.body,'document_revision':0})

    def test_stale_preview_never_sends_and_status_remains_small(self):
        serialized = self.tuning.prepare(self.body)['snapshot_json']
        candidate = json.loads(serialized)
        with patch('kart_studio.tuning.remote') as send:
            identity = candidate['map_id']
            send.return_value = dict(map_id=identity,revision='r2')
            self.tuning.request({**self.body,'known_revision':'r1','pose_stream':'session','pose_after':90},'status')
            payload = send.call_args.args[2]
            self.assertEqual(payload['known_revision'],'r1')
            self.assertEqual(payload['pose_stream'],'session')
            self.assertEqual(payload['pose_after'],90)
            self.assertNotIn('snapshot',payload)
            self.doc['revision']=2
            atomic_json(self.maps.folder('course')/'map.json',self.doc)
            send.reset_mock()
            with self.assertRaises(ValueError):
                self.tuning.request({**self.body,'snapshot':serialized,'expected_revision':'r1'},'apply')
            send.assert_not_called()

    def test_regenerated_raceline_survives_browser_envelope_without_numeric_changes(self):
        # Server metadata can include integer-valued floats, signed zero, and
        # exponent notation that JSON.stringify would rewrite in an object.
        self.doc['generation_metadata'] = dict(values=[0.0, -0.0, 1e-7, 1e20])
        atomic_json(self.maps.folder('course')/'map.json', self.doc)
        generated = self.maps.generate('course', 'raceline', dict(spacing=.5), 1, lambda: None)
        body = {**self.body, 'document_revision': generated['revision'], 'line_type': 'raceline'}
        prepared = self.tuning.prepare(body)
        serialized = prepared['snapshot_json']
        original = json.loads(serialized)
        self.assertIn('-0.0', serialized)
        self.assertEqual(original['document']['revision'], generated['revision'])
        profile = original['document']['lanes'][0]['lines']['raceline']['profile']
        self.assertTrue(profile)

        # The browser parses the outer response/request envelopes, but the
        # hashed snapshot itself remains an opaque string throughout.
        browser = json.loads(json.dumps(prepared))
        request = json.loads(json.dumps({**body, 'snapshot': browser['snapshot_json'],
                                       'expected_revision': 'r1'}))
        with patch('kart_studio.tuning.remote') as send:
            send.return_value = dict(map_id=original['map_id'], revision='r2')
            result = self.tuning.request(request, 'apply')
            forwarded = send.call_args.args[2]
            self.assertEqual(result['revision'], 'r2')
            self.assertEqual(forwarded['expected_revision'], 'r1')
            self.assertEqual(_protocol.encoded(forwarded['snapshot']), serialized.encode())
            self.assertEqual(forwarded['snapshot']['document']['lanes'][0]['lines']['raceline']['profile'], profile)
            _protocol.validate(forwarded['snapshot'], original['map_id'])

    def test_modified_snapshot_is_rejected_before_remote_send(self):
        candidate = json.loads(self.tuning.prepare(self.body)['snapshot_json'])
        for field in ('geometry', 'speed', 'revision'):
            changed = copy.deepcopy(candidate)
            if field == 'geometry':
                changed['document']['lanes'][0]['lines']['customline']['points'][0][0] += .1
            elif field == 'speed':
                changed['centerline_speed_mps'] += .1
            else:
                changed['revision'] = 'invalid'
            with self.subTest(field=field), patch('kart_studio.tuning.remote') as send:
                with self.assertRaisesRegex(ValueError, 'Snapshot revision mismatch'):
                    self.tuning.request({**self.body, 'snapshot': json.dumps(changed),
                                         'expected_revision': 'r1'}, 'apply')
                send.assert_not_called()

    def test_invalid_or_oversized_serialized_snapshot_never_sends(self):
        invalid = [None, {}, '', '{', 'null', '[]', '0']
        for value in invalid:
            with self.subTest(value_type=type(value).__name__, size=len(value) if isinstance(value, str) else None), \
                    patch('kart_studio.tuning.remote') as send:
                with self.assertRaises(ValueError):
                    self.tuning.request({**self.body, 'snapshot': value,
                                         'expected_revision': 'r1'}, 'apply')
                send.assert_not_called()

        serialized = self.tuning.prepare(self.body)['snapshot_json']
        oversized = [serialized + ' ' * _protocol.MAX_BYTES,
                     '"' + 'あ' * (_protocol.MAX_BYTES // 3) + '"']
        for value in oversized:
            with self.subTest(utf8_bytes=len(value.encode())), \
                    patch('kart_studio.tuning.json.loads') as decode, \
                    patch('kart_studio.tuning.remote') as send:
                with self.assertRaises(ValueError):
                    self.tuning.request({**self.body, 'snapshot': value,
                                         'expected_revision': 'r1'}, 'apply')
                decode.assert_not_called()
                send.assert_not_called()

    def test_esc_transport_is_mapless_and_uses_separate_loopback_port(self):
        result = subprocess.CompletedProcess([], 0, b'{"stop_requested":true}', b'')
        with patch('kart_studio.tuning.subprocess.run', return_value=result) as run:
            response = remote(CONNECTION, 'esc_stop', {})
            self.assertTrue(response['stop_requested'])
            self.assertIn('127.0.0.1:8783/stop', run.call_args.args[0][-1])
            self.assertEqual(run.call_count, 1)

    def test_transport_uses_existing_port_fixed_endpoint_and_no_retry(self):
        result=subprocess.CompletedProcess([],0,b'{"map_id":"x"}',b'')
        with patch('kart_studio.tuning.subprocess.run',return_value=result) as run:
            self.assertEqual(remote(CONNECTION,'status',{'map_id':'x'})['map_id'],'x')
            args=run.call_args.args[0]
            self.assertIn('2222',args)
            self.assertIn('kart@192.168.1.2',args)
            self.assertIn('http://127.0.0.1:8782/status',args[-1])
        with patch('kart_studio.tuning.subprocess.run',side_effect=subprocess.TimeoutExpired('ssh',10)) as run:
            with self.assertRaisesRegex(ValueError,'自動再送'):
                remote(CONNECTION,'apply',{})
            self.assertEqual(run.call_count,1)
        with self.assertRaises(ValueError):remote({**CONNECTION,'host':'$(bad)'},'status',{})

    def test_behavior_reset_forwards_map_revision_without_a_snapshot_or_automatic_retry(self):
        identity = self.tuning.identity('course')
        with patch('kart_studio.tuning.remote', return_value=dict(map_id=identity, success=True, message='reset')) as send:
            self.tuning.request({**self.body, 'expected_revision': 'r1'}, 'behavior_reset')
            self.assertEqual(send.call_count, 1)
            self.assertEqual(send.call_args.args[1], 'behavior_reset')
            self.assertEqual(send.call_args.args[2]['expected_revision'], 'r1')
            self.assertEqual(send.call_args.args[2]['map_id'], identity)
            self.assertNotIn('snapshot', send.call_args.args[2])
        result = subprocess.CompletedProcess([], 0, b'{"map_id":"x","success":true}', b'')
        with patch('kart_studio.tuning.subprocess.run', return_value=result) as run:
            self.assertTrue(remote(CONNECTION, 'behavior_reset', dict(map_id='x', expected_revision='r1'))['success'])
            self.assertIn('http://127.0.0.1:8782/behavior_reset', run.call_args.args[0][-1])
            self.assertEqual(run.call_count, 1)

    def test_behavior_reset_http_route_uses_the_existing_tuning_adapter(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from kart_studio.server import Studio
        studio = Studio.__new__(Studio)
        studio.tuning = SimpleNamespace(request=Mock(return_value=dict(success=True)))
        body = dict(id='course', connection=CONNECTION, expected_revision='r1')
        self.assertEqual(studio.post('/api/tuning/behavior_reset', body), dict(success=True))
        studio.tuning.request.assert_called_once_with(body, 'behavior_reset')
