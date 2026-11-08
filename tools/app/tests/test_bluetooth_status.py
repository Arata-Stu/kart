import socket
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from kart_studio.bringup_host import controller_rssi, sample_reports


class BluetoothStatusTest(unittest.TestCase):
    def test_input_reports_and_silent_window(self):
        reader, writer = socket.socketpair(type=socket.SOCK_DGRAM)
        with reader, writer:
            writer.send(bytes([0x31]))
            writer.send(bytes([0xff]))
            writer.send(bytes([0x01]))
            result = sample_reports(reader.fileno(), 0.02)
            self.assertEqual(result['reports'], 2)
            self.assertGreater(result['last_report_ms'], 0)
            self.assertGreaterEqual(result['max_gap_ms'], 0)
            silent = sample_reports(reader.fileno(), 0.01)
            self.assertEqual(silent['reports'], 0)
            self.assertIsNone(silent['last_report_ms'])

    @patch('kart_studio.bringup_host.subprocess.run')
    def test_rssi_zero_and_unavailable(self, run):
        for value, expected in [(0, 0), (-8, -8), (127, None)]:
            run.return_value = SimpleNamespace(returncode=0, stdout=f'RSSI return value: {value}')
            self.assertEqual(controller_rssi('00:11:22:33:44:55'), expected)
        run.side_effect = FileNotFoundError
        self.assertIsNone(controller_rssi('00:11:22:33:44:55'))
