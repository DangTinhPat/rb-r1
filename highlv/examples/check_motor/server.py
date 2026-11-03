#!/usr/bin/env python3
"""LF web motor bench. Standard library only. UART is owned by one worker.

Lumi-Quantum motor commands / telemetry, adapted for a separate LF UART bench.
This is not the production HCR1 runtime. No motor is automatically enabled.
"""
import argparse
import base64
import hashlib
import struct
from collections import deque
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import select
import socket
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from rb_runtime.transport import open_serial
from gui_diagnostics import parse_telemetry

BROWSER_TTL = .3
TELEMETRY_TTL = .15
HEARTBEAT_PERIOD = .005


def validate(command):
    """Canonicalize the small bench command language; no raw UART from HTTP."""
    if not isinstance(command, str) or len(command) > 80:
        raise ValueError('Lệnh không hợp lệ')
    f = command.split()
    if not f:
        raise ValueError('Thiếu lệnh')
    if f[0] in ('STOP', 'RESET', 'MOTOR_HOME', 'GG') and len(f) == 1:
        return f[0]
    try:
        v = [float(x) for x in f[1:]]
    except ValueError:
        raise ValueError('Giá trị không hợp lệ') from None
    if not all(math.isfinite(x) for x in v):
        raise ValueError('Giá trị phải hữu hạn')
    cmd = f[0]
    if cmd in ('ON', 'OFF') and len(v) == 1 and v[0].is_integer() and 1 <= v[0] <= 63:
        return f'{cmd} {int(v[0])}'
    if not v or not v[0].is_integer() or not 0 <= v[0] < 6:
        raise ValueError('Motor phải từ 0 đến 5')
    m = int(v[0])
    valid = (cmd == 'W' and m % 3 == 0 and len(v) == 2 and abs(v[1]) <= 950 or
             cmd == 'MOVE' and m % 3 != 0 and len(v) == 3 and abs(v[1]) <= 360 and 0 < v[2] <= 30 or
             cmd == 'JOG' and m % 3 != 0 and len(v) == 3 and v[1] in (-1, 0, 1) and 0 < v[2] <= 30 or
             cmd == 'G' and m % 3 != 0 and len(v) == 3 and all(0 <= x <= 65.535 for x in v[1:]))
    if not valid:
        raise ValueError('Sai lệnh, loại motor hoặc giới hạn')
    return ' '.join([cmd, str(m)] + [format(x, '.9g') for x in v[1:]])


class Bridge:
    def __init__(self, fd):
        self.fd = fd
        self.lock = threading.Lock()
        self.owner = ''
        self.pulse_at = self.telemetry_at = 0.
        self.gui_gap_max_ms = 0.
        self.pending = deque()
        self.logs = deque(maxlen=60)
        self.gains = {}
        self.motors = []
        self.counters = []
        self.fault, self.homed, self.off = True, 0, 0
        self.stop_pending = True
        self.hello = False
        self.error = ''
        self.ack = ''
        self.stop_reason = 'Mới khởi động; cần RESET và HOME'
        self.scheduler = 'chưa khởi động'
        self.last_heartbeat_at = 0.
        self.heartbeat_gap_max_ms = 0.
        self.commands_sent = 0
        self.acks_received = 0
        self.running = True
        self.sequence = 0

    def _stop(self, reason='STOP'):
        self.stop_reason = reason
        self.logs.append('LF: ' + reason)
        self.pending.clear()
        self.last_heartbeat_at = 0.
        self.owner = ''
        self.stop_pending = True

    def expire(self, now):
        # Caller holds lock. Browser and LF/ESP freshness are separate leases.
        if self.owner and (now - self.pulse_at >= BROWSER_TTL or
                           now - self.telemetry_at >= TELEMETRY_TTL):
            self._stop('Mất heartbeat GUI → STOP' if now - self.pulse_at >= BROWSER_TTL
                       else 'Mất telemetry ESP → STOP')

    def connect(self, client, now=None):
        now = time.monotonic() if now is None else now
        if not isinstance(client, str) or not 1 <= len(client) <= 80:
            raise ValueError('Thiếu mã phiên GUI')
        with self.lock:
            self.expire(now)
            if self.owner and self.owner != client:
                raise ValueError('Một GUI khác đang điều khiển')
            if self.error or not self.hello or now - self.telemetry_at >= TELEMETRY_TTL:
                raise ValueError('Chưa có firmware motor bench / telemetry ESP mới')
            self.owner, self.pulse_at = client, now

    def pulse(self, client, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            self.expire(now)
            if not self.owner or self.owner != client:
                raise ValueError('Phiên GUI đã hết hạn; bấm Kết nối GUI lại')
            if self.error or not self.hello or now - self.telemetry_at >= TELEMETRY_TTL:
                self._stop('Mất telemetry ESP → STOP')
                raise ValueError('Mất telemetry ESP')
            self.gui_gap_max_ms = max(self.gui_gap_max_ms, (now-self.pulse_at)*1000)
            self.pulse_at = now

    def disconnect(self, client, reason):
        with self.lock:
            if self.owner == client:
                self._stop(reason)

    def command(self, client, command, reason='STOP trên GUI'):
        command = validate(command)
        with self.lock:
            if command == 'STOP':
                self._stop(str(reason)[:120])
                return
            self.expire(time.monotonic())
            if not self.owner or self.owner != client:
                raise ValueError('Bấm Kết nối GUI trước')
            if len(self.pending) >= 16:
                self._stop()
                raise ValueError('Hàng đợi đầy → STOP')
            self.pending.append(command)

    def receive(self, line):
        with self.lock:
            if line == '#HELLO MOTOR_BENCH_1':
                self.hello = True
            elif line.startswith('#T '):
                self.counters, self.motors = parse_telemetry(line)
                self.telemetry_at = time.monotonic()
            elif line.startswith('#S '):
                f = line.split()
                if len(f) != 4 or f[1] not in ('0', '1') or not all(0 <= int(x) <= 63 for x in f[2:]):
                    raise ValueError('Sai #S')
                self.fault, self.homed, self.off = bool(int(f[1])), int(f[2]), int(f[3])
            elif line.startswith('#G '):
                f = line.split()
                if len(f) != 6 or not 0 <= int(f[1]) < 6:
                    raise ValueError('Sai #G')
                v = [float(x) for x in f[2:]]
                if not all(math.isfinite(x) and x >= 0 for x in v):
                    raise ValueError('Sai gain')
                self.gains[f[1]] = v
            elif line.startswith('#A '):
                self.acks_received += 1
                self.ack = line
                self.logs.append(line)
            elif line:
                self.logs.append(line)

    def snapshot(self, client):
        with self.lock:
            now = time.monotonic()
            self.expire(now)
            return dict(connected=self.hello and now - self.telemetry_at < TELEMETRY_TTL and not self.error,
                        controlling=bool(client and self.owner == client), fault=self.fault,
                        homed=self.homed, off=self.off, motors=self.motors, counters=self.counters,
                        gains=self.gains.copy(), ack=self.ack, error=self.error,
                        logs=list(self.logs), stop_reason=self.stop_reason,
                        commands_sent=self.commands_sent, acks_received=self.acks_received,
                        gui_gap_max_ms=round(self.gui_gap_max_ms, 2),
                        scheduler=self.scheduler, uart_pid=os.getpid(),
                        heartbeat_gap_max_ms=round(self.heartbeat_gap_max_ms, 2),
                        online_count=sum(m['online'] for m in self.motors), age_ms=round((now - self.telemetry_at)*1000) if self.telemetry_at else None)

    def write(self, data):
        encoded = data.encode('ascii')
        # Never retain/replay delayed motor commands after transport backpressure.
        if os.write(self.fd, encoded) != len(encoded):
            raise OSError('UART short write')

    def run(self, ipc=None):
        # Linux schedules this UART thread ahead of HTTP/log threads. 0 means
        # the calling thread, not the whole process. Blocking select yields CPU.
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))
            self.scheduler = 'SCHED_FIFO:20'
        except (OSError, AttributeError) as exc:
            self.scheduler = 'SCHED_OTHER: ' + str(exc)
            with self.lock:
                self.logs.append('UART chưa có ưu tiên realtime: ' + str(exc))
        buf = bytearray()
        try:
            self.write('STOP\nHELLO\n')
            next_beat = time.monotonic()
            while self.running:
                now = time.monotonic()
                tx = ''
                with self.lock:
                    self.expire(now)
                    if self.stop_pending:
                        tx, self.stop_pending = 'STOP\n', False
                    if self.owner and now >= next_beat:
                        if self.last_heartbeat_at:
                            self.heartbeat_gap_max_ms = max(self.heartbeat_gap_max_ms,
                                                           (now-self.last_heartbeat_at)*1000)
                        self.last_heartbeat_at = now
                        self.sequence += 1
                        tx += f'P {self.sequence}\n'
                        next_beat = now + HEARTBEAT_PERIOD
                        if self.pending:
                            self.sequence += 1
                            cmd = self.pending.popleft()
                            self.commands_sent += 1
                            tx += f'C {self.sequence} {cmd}\n'
                            self.logs.append(f'LF → ESP: {cmd}')
                if tx:
                    self.write(tx)
                ready, _, _ = select.select([self.fd] + ([ipc] if ipc else []), [], [], .001)
                if self.fd in ready:
                    data = os.read(self.fd, 4096)
                    if not data:
                        raise OSError('UART đóng')
                    buf.extend(data)
                    while b'\n' in buf:
                        raw, _, rest = buf.partition(b'\n')
                        buf = bytearray(rest)
                        if len(raw) > 1024:
                            raise OSError('UART line too long')
                        try:
                            self.receive(raw.decode('ascii').strip())
                        except (ValueError, UnicodeError):
                            with self.lock:
                                self.logs.append('Bỏ telemetry sai định dạng')
                    if len(buf) > 1024:
                        raise OSError('UART overflow')
                if ipc and ipc in ready:
                    self.handle_ipc(ipc)
        except OSError as exc:
            with self.lock:
                self.error = str(exc)
                self._stop('Lỗi UART: ' + str(exc))
        finally:
            try:
                self.write('STOP\n')
            except OSError:
                pass


    def handle_ipc(self, ipc):
        packet = ipc.recv(65536)
        if not packet:
            self._stop('Web process đã đóng → STOP')
            self.running = False
            return
        try:
            request = json.loads(packet)
            operation, args = request['operation'], request['args']
            if operation not in ('snapshot', 'connect', 'pulse', 'disconnect', 'command', 'shutdown'):
                raise ValueError('Unknown local operation')
            if operation == 'shutdown':
                self._stop('Dừng web server → STOP')
                self.running = False
                result = None
            else:
                result = getattr(self, operation)(*args)
            reply = {'ok': True, 'value': result}
        except (ValueError, TypeError, KeyError) as exc:
            reply = {'ok': False, 'error': str(exc)}
        ipc.send(json.dumps(reply, ensure_ascii=False).encode())


class BridgeProcess:
    """UART lives in a separate interpreter: no HTTP/log GIL or shared locks.

    Local seqpacket RPC is bounded and serialized. If the web process dies,
    socket EOF stops the UART child; loss of child also lets ESP's lease expire.
    """
    def __init__(self, port=None, baud=921600, fd=None):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.lock = threading.Lock()
        self.running = True
        self.pid = os.fork()  # before starting any HTTP or diagnostic threads
        if self.pid == 0:
            parent.close()
            try:
                uart_fd = fd if fd is not None else open_serial(port, baud)
                child.setblocking(False)
                bridge = Bridge(uart_fd)
                bridge.run(child)
                os.close(uart_fd)
                code = 0
            except BaseException as exc:
                print('UART worker failed: ' + str(exc), file=sys.stderr, flush=True)
                code = 1
            child.close()
            os._exit(code)
        child.close()
        if fd is not None:
            os.close(fd)
        self.peer = parent
        self.peer.settimeout(2)
        try:
            self.snapshot('')  # fail startup if UART cannot be opened exclusively
        except BaseException:
            self.close()
            raise
        self.peer.settimeout(.2)

    def request(self, operation, *args):
        with self.lock:
            try:
                self.peer.send(json.dumps({'operation': operation, 'args': args}, ensure_ascii=False).encode())
                raw = self.peer.recv(65536)
                if not raw:
                    raise OSError('UART worker đã thoát')
                reply = json.loads(raw)
            except (OSError, ValueError) as exc:
                # A timed-out response must never be reused for the next RPC.
                self.peer.close()
                self.running = False
                raise RuntimeError('UART IPC failed: ' + str(exc)) from exc
            if not reply['ok']:
                raise ValueError(reply['error'])
            return reply['value']

    def snapshot(self, client):
        return self.request('snapshot', client)

    def connect(self, client):
        return self.request('connect', client)

    def pulse(self, client):
        return self.request('pulse', client)

    def disconnect(self, client, reason):
        return self.request('disconnect', client, reason)

    def command(self, client, command, reason='STOP trên GUI'):
        return self.request('command', client, command, reason)

    def close(self):
        if self.running:
            try:
                self.request('shutdown')
            except (OSError, RuntimeError, ValueError):
                pass
        self.running = False
        self.peer.close()
        until = time.monotonic() + 1
        while time.monotonic() < until:
            try:
                if os.waitpid(self.pid, os.WNOHANG)[0]:
                    return
            except ChildProcessError:
                return
            time.sleep(.01)
        os.kill(self.pid, signal.SIGTERM)
        os.waitpid(self.pid, 0)


def make_handler(bridge, token):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        disable_nagle_algorithm = True
        def log_message(self, *_):
            pass

        def reply(self, code, body, content_type='application/json'):
            data = (json.dumps(body, ensure_ascii=False) if content_type == 'application/json' else body).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(data)))
            if code >= 400:
                self.close_connection = True
                self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(data)

        def authorized(self):
            return hmac.compare_digest(self.headers.get('X-Bench-Token', ''), token)

        def ws_read(self):
            def exact(size):
                data = self.rfile.read(size)
                if len(data) != size:
                    raise OSError('WebSocket EOF')
                return data
            first, second = exact(2)
            if first & 0x70 or not first & 0x80 or not second & 0x80:
                raise ValueError('WebSocket requires a final masked frame')
            opcode, size = first & 15, second & 127
            if size == 126:
                size = struct.unpack('!H', exact(2))[0]
            elif size == 127:
                raise ValueError('WebSocket frame too large')
            if size > 512 or (opcode >= 8 and size > 125):
                raise ValueError('WebSocket frame too large')
            mask = exact(4)
            data = exact(size)
            return opcode, bytes(v ^ mask[i % 4] for i, v in enumerate(data))

        def ws_send(self, body, opcode=1):
            payload = json.dumps(body).encode() if opcode == 1 else body
            header = bytes([0x80 | opcode, len(payload)]) if len(payload) < 126 else bytes([0x80 | opcode, 126]) + struct.pack('!H', len(payload))
            self.wfile.write(header + payload)
            self.wfile.flush()

        def websocket(self):
            client = None
            self.close_connection = True
            if self.headers.get('Upgrade', '').lower() != 'websocket' or self.headers.get('Sec-WebSocket-Version') != '13':
                return self.reply(400, {'error': 'WebSocket upgrade required'})
            key = self.headers.get('Sec-WebSocket-Key', '')
            try:
                if len(base64.b64decode(key, validate=True)) != 16:
                    raise ValueError('Invalid websocket key')
            except (ValueError, TypeError):
                return self.reply(400, {'error': 'Invalid websocket key'})
            accept = base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            self.send_response(101)
            self.send_header('Upgrade', 'websocket')
            self.send_header('Connection', 'Upgrade')
            self.send_header('Sec-WebSocket-Accept', accept)
            self.end_headers()
            self.connection.settimeout(.3)
            try:
                opcode, data = self.ws_read()
                auth = json.loads(data)
                if opcode != 1 or auth.get('type') != 'auth' or not hmac.compare_digest(auth.get('token', ''), token):
                    raise ValueError('WebSocket authentication failed')
                client = auth['client']
                bridge.pulse(client)  # renewal only; never claims an expired session
                self.ws_send({'type': 'ready'})
                sequence = 0
                while True:
                    opcode, data = self.ws_read()
                    if opcode == 8:
                        break
                    if opcode == 9:
                        self.ws_send(data, 10)
                        continue
                    if opcode != 1:
                        raise ValueError('Unsupported websocket opcode')
                    message = json.loads(data)
                    seq = message.get('seq')
                    if message.get('type') != 'heartbeat' or type(seq) is not int or seq <= sequence:
                        raise ValueError('Invalid heartbeat sequence')
                    bridge.pulse(client)
                    sequence = seq
                    self.ws_send({'type': 'pong', 'seq': seq})
            except (OSError, ValueError, KeyError, TypeError, RuntimeError):
                pass
            finally:
                if client:
                    try:
                        bridge.disconnect(client, 'Kênh heartbeat GUI đã đóng → STOP')
                    except (OSError, RuntimeError):
                        pass
                try:
                    self.ws_send(struct.pack('!H', 1000), 8)
                except OSError:
                    pass

        def do_GET(self):
            if self.path == '/api/stream':
                return self.websocket()
            if self.path == '/heartbeat.js':
                return self.reply(200, Path(__file__).with_name('heartbeat.js').read_text(), 'application/javascript')
            if self.path == '/':
                return self.reply(200, Path(__file__).with_name('index.html').read_text(), 'text/html')
            if not self.authorized():
                return self.reply(403, {'error': 'Sai token'})
            if self.path == '/api/state':
                try:
                    return self.reply(200, bridge.snapshot(self.headers.get('X-Bench-Client', '')))
                except (OSError, RuntimeError) as exc:
                    return self.reply(503, {'error': str(exc)})
            self.reply(404, {'error': 'Không có đường dẫn này'})

        def do_POST(self):
            if not self.authorized():
                return self.reply(403, {'error': 'Sai token'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 512 or self.headers.get('Content-Type') != 'application/json':
                    raise ValueError('Cần JSON, tối đa 512 byte')
                body = json.loads(self.rfile.read(size))
                client = body.get('client', '')
                if self.path == '/api/connect':
                    bridge.connect(client)
                    return self.reply(200, bridge.snapshot(client))
                elif self.path in ('/api/heartbeat', '/api/pulse'):
                    return self.reply(409, {'error': 'GUI cũ: nhấn Ctrl+F5 để tải bản WebSocket mới'})
                elif self.path == '/api/command':
                    bridge.command(client, body.get('command'), body.get('reason', 'STOP trên GUI'))
                else:
                    return self.reply(404, {'error': 'Không có đường dẫn này'})
                self.reply(200, {'queued_on_lf': True})
            except (ValueError, TypeError, AttributeError) as exc:
                self.reply(400, {'error': str(exc)})
            except (OSError, RuntimeError) as exc:
                self.reply(503, {'error': str(exc)})
    return Handler


def configure_wifi(interface):
    """Wi-Fi latency setting for this SBC bench; no network restart needed."""
    if not interface or not Path('/sys/class/net', interface).exists():
        return 'Wi-Fi power save: interface not present (skipped)'
    try:
        subprocess.run(['iw', 'dev', interface, 'set', 'power_save', 'off'],
                       check=True, capture_output=True, timeout=2)
        result = subprocess.run(['iw', 'dev', interface, 'get', 'power_save'],
                                check=True, capture_output=True, text=True, timeout=2)
        return interface + ': ' + result.stdout.strip()
    except (OSError, subprocess.SubprocessError) as exc:
        return 'Không đặt được Wi-Fi power_save=off: ' + str(exc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyS2')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--http-port', type=int, default=8082)
    parser.add_argument('--wifi-interface', default='wlan0', help='disable Wi-Fi power saving; empty skips')
    parser.add_argument('--status-file', default='/tmp/rb-check-motor-status.json')
    parser.add_argument('--token-file', default='/tmp/rb-check-motor.token')
    args = parser.parse_args()
    sys.setswitchinterval(.001)  # bound HTTP thread GIL holds on the single-core LF
    print(configure_wifi(args.wifi_interface), flush=True)
    token_path = Path(args.token_file)
    if token_path.exists():
        token = token_path.read_text().strip()
    else:
        token = secrets.token_urlsafe(24)
        token_fd = os.open(str(token_path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(token_fd, 'w') as handle:
            handle.write(token + '\n')
    bridge = BridgeProcess(port=args.port, baud=args.baud)
    def diagnostics():
        while bridge.running:
            try:
                path = Path(args.status_file)
                temp = path.with_name(path.name + '.tmp')
                state = bridge.snapshot('')
                state.pop('controlling', None)
                status_fd = os.open(str(temp), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
                with os.fdopen(status_fd, 'w') as handle:
                    json.dump(state, handle, ensure_ascii=False)
                os.replace(str(temp), str(path))
            except (OSError, RuntimeError, ValueError):
                pass
            time.sleep(1)
    try:
        server = ThreadingHTTPServer((args.bind, args.http_port), make_handler(bridge, token))
        threading.Thread(target=diagnostics, daemon=True).start()
        print(f'Mở http://<IP-LF>:{args.http_port}/#{token}', flush=True)
        print('Ctrl+C để dừng; firmware sẽ cần RESET → MOTOR_HOME trước khi bật motor.', flush=True)
        try:
            server.serve_forever(poll_interval=.1)
        except KeyboardInterrupt:
            pass
        finally:
            bridge.close()
            server.server_close()
    finally:
        bridge.close()


if __name__ == '__main__':
    main()
