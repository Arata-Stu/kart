import copy
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
        candidate = self.tuning.prepare(self.body)
        self.assertEqual(candidate['document']['lanes'][0]['lines']['customline']['profile'],self.doc['lanes'][0]['lines']['customline']['profile'])
        self.assertTrue((self.maps.folder('course')/'exports').is_dir())
        with self.assertRaises(ValueError): self.tuning.prepare({**self.body,'document_revision':0})

    def test_stale_preview_never_sends_and_status_remains_small(self):
        candidate = self.tuning.prepare(self.body)
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
                self.tuning.request({**self.body,'snapshot':candidate,'expected_revision':'r1'},'apply')
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
