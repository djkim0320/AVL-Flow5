"""HTTP routing and error classification of the studio server."""

import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from unittest import mock
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from dbf_studio import model_registry, server


class _Pool:
    _max_workers = 2


class _Jobs:
    def list(self):
        return []

    def status(self, job_id):
        raise FileNotFoundError('해석 기록이 없습니다.')


class ServerRoutes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
        cls.httpd.pool = _Pool()
        cls.httpd.jobs = _Jobs()
        cls.thread = threading.Thread(target=cls.httpd.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f'http://127.0.0.1:{cls.httpd.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.server_close()

    def call(self, path, method='GET', body=None, headers=None):
        try:
            with urlopen(Request(self.base + path, data=body, method=method, headers=headers or {})) as r:
                return r.status, json.loads(r.read())
        except HTTPError as e:
            return e.code, json.loads(e.read())

    def test_health_and_unknown_routes(self):
        self.assertEqual(self.call('/api/health'), (200, dict(status='ready', step_workers=2)))
        self.assertEqual(self.call('/api/nothing')[0], 404)
        self.assertEqual(self.call('/api/nothing', 'POST', b'{}')[0], 404)
        self.assertEqual(self.call('/../pyproject.toml')[0], 404)

    def test_bad_input_is_400_with_the_message(self):
        status, value = self.call('/api/analysis/catalog')
        self.assertEqual(status, 400)
        self.assertEqual(value['error'], '등록 모델을 선택하세요.')
        self.assertEqual(self.call('/api/analysis/jobs/0123456789abcdef0123456789abcdef')[0], 400)
        status, value = self.call('/api/projects', 'POST', b'{"schema": "x"}')
        self.assertEqual(status, 400)
        self.assertIn('DBF 배치 파일 형식', value['error'])
        status, value = self.call('/api/models/match', 'POST', b'{}')
        self.assertEqual(status, 400)
        self.assertIn('project', value['error'])

    def test_server_fault_is_500_not_blamed_on_the_input(self):
        with mock.patch.object(model_registry, 'list_models', side_effect=RuntimeError('boom')):
            status, value = self.call('/api/models')
        self.assertEqual(status, 500)
        self.assertIn('RuntimeError', value['error'])
        self.assertEqual(self.call('/api/models'), (200, model_registry.list_models()))

    def test_foreign_origin_and_empty_body_are_rejected(self):
        self.assertEqual(self.call('/api/projects', 'POST', b'{}', {'Origin': 'http://evil.example'})[0], 403)
        self.assertEqual(self.call('/api/projects', 'POST', b'')[0], 400)


if __name__ == '__main__':
    unittest.main()
