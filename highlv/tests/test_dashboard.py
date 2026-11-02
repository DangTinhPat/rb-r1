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
