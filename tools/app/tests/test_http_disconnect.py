"""A disconnected HTTP client must not trigger a second response or action."""
import io
import json
import unittest
from unittest.mock import Mock

from kart_studio.server import Handler
from kart_studio.tuning import MAX_APPLY_BYTES, _protocol


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


class TuningEnvelopeTests(unittest.TestCase):
    def test_escaped_snapshot_can_exceed_standard_http_limit_only_on_apply(self):
        snapshot = json.dumps({'notes': '"' * (_protocol.MAX_BYTES // 2 - 32)})
        raw = json.dumps({'snapshot': snapshot}).encode()
        self.assertLessEqual(len(snapshot.encode()), _protocol.MAX_BYTES)
        self.assertGreater(len(raw), 4 * 1024 * 1024)
        self.assertLessEqual(len(raw), MAX_APPLY_BYTES)
        for path in ('/api/tuning/apply', '/api/save'):
            with self.subTest(path=path):
                handler = Mock()
                handler.path = path
                handler.headers = {'Content-Type': 'application/json', 'Content-Length': str(len(raw))}
                handler.rfile = io.BytesIO(raw)
                handler.server.studio.post.return_value = {'accepted': True}
                Handler.do_POST(handler)
                if path == '/api/tuning/apply':
                    handler.server.studio.post.assert_called_once_with(path, {'snapshot': snapshot})
                    handler.reply.assert_called_once_with(200, {'accepted': True})
                else:
                    handler.server.studio.post.assert_not_called()
                    handler.reply.assert_called_once_with(400, {'error': 'Body size limit exceeded'})

    def test_oversized_apply_envelope_rejected_before_read(self):
        handler = Mock()
        handler.path = '/api/tuning/apply'
        handler.headers = {'Content-Type': 'application/json', 'Content-Length': str(MAX_APPLY_BYTES + 1)}
        Handler.do_POST(handler)
        handler.rfile.read.assert_not_called()
        handler.server.studio.post.assert_not_called()
        handler.reply.assert_called_once_with(400, {'error': 'Body size limit exceeded'})
