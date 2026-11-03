import importlib.util
import json
import os
from pathlib import Path
import pty
import select
import sys
import threading
import time
import tty
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from server import Bridge, make_handler, validate

TEL = '#T 1000 1100 0 0' + ' 1,0,0,00,10.0,0.0,10.0,0'*6


class Validation(unittest.TestCase):
    def test_command_boundaries(self):
        for s in ['STOP', 'RESET', 'MOTOR_HOME', 'ON 63', 'OFF 1', 'W 0 -950',
                  'W 3 0', 'MOVE 1 -5 3', 'JOG 5 -1 30', 'G 2 10 .036']:
            validate(s)
        for s in ['', 'ON 0', 'ON 64', 'ON 1.2', 'W 1 1', 'W 0 nan', 'W 0 inf',
                  'W 0 951', 'MOVE 1 361 3', 'MOVE 0 5 1', 'MOVE 2 5 0',
                  'JOG 2 0.5 3', 'G 2 -1 1', 'RESET extra', 'W 0 2\nSTOP']:
            with self.subTest(s=s), self.assertRaises(ValueError):
                validate(s)

    def test_stale_browser_discards_commands_and_ownership(self):
        b = Bridge(-1)
        now = time.monotonic()
        b.hello, b.telemetry_at = True, now
        b.connect('a', now)
        b.command('a', 'ON 1')
        with self.assertRaises(ValueError):
            b.connect('b', now)
        with b.lock:
            b.expire(now + .301)
        self.assertFalse(b.pending)
        self.assertFalse(b.owner)
        self.assertTrue(b.stop_pending)
        with self.assertRaises(ValueError):
            b.pulse('a', now + .302)
        with self.assertRaises(ValueError):
            b.command('a', 'ON 1')

    def test_telemetry_validation(self):
        b = Bridge(-1)
        b.receive(TEL)
        self.assertEqual(len(b.motors), 6)
        for line in [TEL.replace('10.0', 'nan'), '#S 2 0 63', '#G 2 nan 0 1 1']:
            with self.assertRaises(ValueError):
                b.receive(line)


class MockTransport(unittest.TestCase):
    def test_http_uart_roundtrip_and_disconnect(self):
        # Temporary local pseudo-UART; never opens /dev/ttyS2 or an MCU.
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(slave, False)
        b = Bridge(slave)
        http = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(b, 'test-token'))
        worker = threading.Thread(target=b.run)
        web = threading.Thread(target=http.serve_forever, kwargs={'poll_interval': .01})
        worker.start(); web.start()
        received = bytearray()
        def collect(until, timeout=1):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if select.select([master], [], [], .01)[0]:
                    received.extend(os.read(master, 4096))
                if until in received:
                    return
            self.fail(f'Missing {until!r}: {received!r}')
        base = f'http://127.0.0.1:{http.server_port}'
        def post(path, body, token='test-token'):
            req = urllib.request.Request(base+'/api/'+path, json.dumps(body).encode(),
                {'X-Bench-Token': token, 'Content-Type': 'application/json'})
            return json.loads(urllib.request.urlopen(req, timeout=1).read())
        try:
            collect(b'HELLO\n')
            os.write(master, ('#HELLO MOTOR_BENCH_1\n#S 1 0 63\n'+TEL+'\n').encode())
            end = time.monotonic()+1
            while not b.telemetry_at and time.monotonic()<end:
                time.sleep(.001)
            with self.assertRaises(urllib.error.HTTPError) as e:
                post('command', {'client':'a', 'command':'ON 1'}, 'wrong')
            self.assertEqual(e.exception.code, 403)
            live = post('connect', {'client':'a'})
            self.assertTrue(live['connected'])
            self.assertTrue(live['controlling'])
            self.assertEqual(live['online_count'], 6)
            with self.assertRaises(urllib.error.HTTPError) as old:
                post('pulse', {'client':'a'})
            self.assertEqual(old.exception.code, 409)
            post('command', {'client':'a', 'command':'RESET'})
            collect(b' RESET\n')
            self.assertIn(b'P ', received)
            os.write(master, b'#A 2 QUEUED\n')
            end = time.monotonic()+1
            while not b.ack and time.monotonic()<end:
                time.sleep(.001)
            self.assertEqual(b.ack, '#A 2 QUEUED')
            received.clear()
            collect(b'STOP\n')  # No new telemetry: LF must stop, clear commands and ownership.
            self.assertFalse(b.owner)
            self.assertFalse(b.pending)
        finally:
            b.running=False; worker.join(1)
            http.shutdown(); web.join(1); http.server_close()
            os.close(master); os.close(slave)


if __name__ == '__main__':
    unittest.main()
