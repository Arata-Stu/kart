import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kart_studio.bringup import Bringup, remote
from kart_studio import bringup_host

CONNECTION = dict(user='kart',host='jetson',port=22,record_root='/record',map_root='/map')
ID = 'a'*64


class BringupTests(unittest.TestCase):
    def test_transport_fixed_script_json_no_retry(self):
        payload=dict(action='start',container=ID,values={'mode':'esc-trial','device':'$(bad); x'})
        with patch('kart_studio.bringup.subprocess.run',return_value=subprocess.CompletedProcess([],0,'{"ok":true}','')) as run:
            self.assertTrue(remote(CONNECTION,payload)['ok'])
            self.assertEqual(json.loads(run.call_args.kwargs['input']),payload)
            self.assertNotIn('$(bad)',run.call_args.args[0][-1])
        with patch('kart_studio.bringup.subprocess.run',side_effect=subprocess.TimeoutExpired('ssh',60)) as run:
            with self.assertRaisesRegex(ValueError,'自動再送'):
                remote(CONNECTION,payload)
            self.assertEqual(run.call_count,1)

    def test_saved_settings_do_not_start_and_are_target_scoped(self):
        with tempfile.TemporaryDirectory() as folder, patch('kart_studio.bringup.remote') as send:
            service=Bringup(Path(folder))
            preset=dict(container=ID,values={'mode':'esc-trial'})
            body=dict(connection=CONNECTION,preset=preset)
            service.request(body,'preset_save')
            self.assertEqual(service.request(body,'preset_get')['preset'],preset)
            self.assertIsNone(service.request(dict(connection={**CONNECTION,'host':'other'}),'preset_get')['preset'])
            send.assert_not_called()
            with self.assertRaises(ValueError):service.request(dict(connection=CONNECTION,container='--privileged'),'start')

    def test_host_uses_admin_and_rejects_other_launch(self):
        item=dict(id=ID,running=True)
        with patch.object(bringup_host,'containers',return_value=[item]), patch.object(bringup_host,'command',return_value='{"phase":"idle"}') as run:
            bringup_host.dispatch(dict(action='status',container=ID))
            argv=run.call_args.args[0]
            self.assertEqual(argv[:7],['docker','exec','-i','-u','admin','-w','/workspaces'])
            self.assertIn('KART_BRINGUP_TMUX_SOCKET=kart-bringup-' + ID[:12], argv)
            with patch.object(bringup_host,'launches',return_value=[{'pid':42}]):
                with self.assertRaisesRegex(ValueError,'二重起動'):
                    bringup_host.dispatch(dict(action='start',container=ID))
            with self.assertRaises(ValueError):bringup_host.dispatch(dict(action='status',container='b'*64))

    def test_container_start_never_creates_or_rebuilds(self):
        with patch.object(bringup_host,'containers',return_value=[dict(id=ID,running=False)]), patch.object(bringup_host,'command') as run:
            bringup_host.dispatch(dict(action='container_start',container=ID))
            run.assert_called_once_with(['docker','start',ID])
