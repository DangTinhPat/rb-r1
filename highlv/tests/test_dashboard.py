import json
from pathlib import Path
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from rb_runtime.dashboard import DashboardServer

ROOT = Path(__file__).resolve().parents[1]


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.status = self.root / "status.json"
        self.status.write_text(
            json.dumps({"mode": "DISARMED", "updated_monotonic": time.monotonic()})
        )
        self.web = DashboardServer(
            "127.0.0.1",
            0,
            self.root / "token",
            self.status,
            str(self.root / "commands.sock"),
            ROOT / "web/index.html",
        )
        self.web.start()
        self.url = f"http://127.0.0.1:{self.web.server.server_port}"

    def tearDown(self):
        self.web.close()
        self.directory.cleanup()

    def request(self, path, **kwargs):
        request = Request(
            self.url + path,
            headers={"Authorization": "Bearer " + self.web.token},
            **kwargs,
        )
        with urlopen(request, timeout=2.0) as response:
            return json.load(response)

    def test_status_requires_token_and_detects_stale_runtime(self):
        with self.assertRaises(HTTPError) as error:
            urlopen(self.url + "/api/status", timeout=2.0)
        self.assertEqual(error.exception.code, 401)
        self.assertTrue(self.request("/api/status")["runtime_online"])
        self.status.write_text(
            json.dumps(
                {"mode": "DISARMED", "updated_monotonic": time.monotonic() - 4.0}
            )
        )
        self.assertFalse(self.request("/api/status")["runtime_online"])

    def test_page_loads_from_the_new_asset_directory(self):
        with urlopen(self.url + "/", timeout=2.0) as response:
            self.assertIn(b"RB", response.read())

    def test_command_failure_is_reported_when_runtime_socket_is_absent(self):
        with self.assertRaises(HTTPError) as error:
            self.request("/api/command", data=b'{"command":"ARM"}', method="POST")
        self.assertEqual(error.exception.code, 503)
        self.assertFalse(json.load(error.exception)["accepted"])

    def test_malformed_posts_never_reach_the_command_socket(self):
        for body in (b'[]', b'null', b'not-json', b'{"command":"ARM extra"}', b'{}'):
            with self.subTest(body=body), self.assertRaises(HTTPError) as error:
                self.request('/api/command', data=body, method='POST')
            self.assertEqual(error.exception.code, 400)
        self.assertTrue(self.request('/api/status')['runtime_online'])

    def test_invalid_status_timestamp_and_body_are_offline(self):
        for status in ('[]', '{"updated_monotonic":NaN}', '{"updated_monotonic":Infinity}'):
            self.status.write_text(status)
            self.assertFalse(self.request('/api/status')['runtime_online'])

    def test_post_is_sent_once_even_if_runtime_drops_its_reply(self):
        import socket
        import threading
        path = str(self.root / 'commands.sock')
        seen = []
        with socket.socket(socket.AF_UNIX) as listener:
            listener.bind(path)
            listener.listen(2)
            def accept():
                with listener.accept()[0] as client:
                    seen.append(client.recv(128))
                    # Command may already be applied: do not retry after EOF.
            thread = threading.Thread(target=accept)
            thread.start()
            with self.assertRaises(HTTPError) as error:
                self.request('/api/command', data=b'{"command":"ARM"}', method='POST')
            thread.join(timeout=2)
            self.assertFalse(thread.is_alive())
            result = json.load(error.exception)
            self.assertFalse(result['confirmed'])
            listener.settimeout(.05)
            with self.assertRaises(TimeoutError):
                listener.accept()
        self.assertEqual(seen, [b'ARM\n'])

    def test_partial_clients_are_bounded_and_recovery_needs_no_command(self):
        import socket
        web = DashboardServer('127.0.0.1', 0, self.root/'limited-token', self.status,
                              str(self.root/'unused.sock'), ROOT/'web/index.html',
                              request_limit=2, request_timeout=.25)
        web.start()
        clients = []
        try:
            for _ in range(2):
                client = socket.create_connection(web.server.server_address, timeout=1)
                client.sendall(b'GET /api/status HTTP/1.0\r\n')
                clients.append(client)
            # A third connection gets backpressure instead of growing threads.
            with self.assertRaises(HTTPError) as error:
                urlopen(f'http://127.0.0.1:{web.server.server_port}/api/status', timeout=1)
            self.assertEqual(error.exception.code, 503)
            # Both partial clients time out; the bounded server then serves again.
            for client in clients:
                self.assertEqual(client.recv(1), b'')
            request = Request(f'http://127.0.0.1:{web.server.server_port}/api/status',
                              headers={'Authorization':'Bearer '+web.token})
            with urlopen(request, timeout=1) as response:
                self.assertTrue(json.load(response)['runtime_online'])
        finally:
            for client in clients:
                client.close()
            web.close()


class DashboardProcessTests(unittest.TestCase):
    def test_http_runs_in_separate_non_realtime_process_and_survives_reload(self):
        import os
        import signal
        import socket
        from rb_runtime.dashboard import DashboardProcess, load_or_create_token
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            status = root/'status.json'
            status.write_text(json.dumps({'mode':'MODEL', 'updated_monotonic':time.monotonic()}))
            token = load_or_create_token(root/'token')
            web = DashboardProcess('127.0.0.1',0,root/'token',status,str(root/'commands.sock'),ROOT/'web/index.html')
            try:
                web.start()
                self.assertNotEqual(web.process.pid, os.getpid())
                self.assertEqual(os.sched_getscheduler(web.process.pid), os.SCHED_OTHER)
                def read():
                    request = Request(f'http://127.0.0.1:{web.port}/api/status',
                                      headers={'Authorization':'Bearer '+token})
                    with urlopen(request, timeout=1) as response:
                        return json.load(response)
                for _ in range(3):
                    self.assertEqual(read()['mode'],'MODEL')
                # A stopped web process cannot hold the parent's GIL or operator
                # socket. No commands are generated while it is stopped/reloaded.
                with socket.socket(socket.AF_UNIX) as operator:
                    operator.bind(str(root/'commands.sock'))
                    operator.listen(1)
                    operator.settimeout(.05)
                    web.process.send_signal(signal.SIGSTOP)
                    try:
                        status.write_text(json.dumps({'mode':'STANDUP','updated_monotonic':time.monotonic()}))
                        with self.assertRaises(TimeoutError):
                            operator.accept()
                    finally:
                        web.process.send_signal(signal.SIGCONT)
                    self.assertEqual(read()['mode'],'STANDUP')
                    with self.assertRaises(TimeoutError):
                        operator.accept()
            finally:
                web.close()

    def test_bind_failure_cleans_up_the_child(self):
        import socket
        from rb_runtime.dashboard import DashboardProcess
        with tempfile.TemporaryDirectory() as directory, socket.socket() as occupied:
            occupied.bind(('127.0.0.1',0));occupied.listen(1)
            root = Path(directory)
            web = DashboardProcess('127.0.0.1',occupied.getsockname()[1],root/'token',root/'status',
                                   str(root/'commands.sock'),ROOT/'web/index.html')
            with self.assertRaises(RuntimeError):
                web.start()
            self.assertIsNone(web.process)
