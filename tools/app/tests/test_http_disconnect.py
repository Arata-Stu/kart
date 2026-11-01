"""A disconnected HTTP client must not trigger a second response or action."""
import unittest
from unittest.mock import Mock

from kart_studio.server import Handler


class ReplyTests(unittest.TestCase):
    def test_disconnect_during_headers_or_body(self):
        for stage in ('end_headers', 'write'):
            for error in (BrokenPipeError, ConnectionResetError):
                with self.subTest(stage=stage, error=error):
                    handler = Mock()
                    target = handler.end_headers if stage == 'end_headers' else handler.wfile.write
                    target.side_effect = error()
                    Handler.reply(handler, 400, {'error': 'original failure'})
                    self.assertTrue(handler.close_connection)
                    handler.send_response.assert_called_once_with(400)
                    target.assert_called_once()
