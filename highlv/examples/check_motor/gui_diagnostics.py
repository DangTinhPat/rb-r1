"""Session recording and counters; no device writes or control commands."""
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import re
import threading
import time
import urllib.request


def redact(text):
    text = re.sub(r'(?i)(\bAUTH\s+)\S+', r'\1[REDACTED]', str(text))
    return re.sub(r'(?i)(X-OTA-Key\s*[:=]\s*)\S+', r'\1[REDACTED]', text)


class Journal:
    """Bounded asynchronous writer; GUI/control thread never waits for disk."""
    def __init__(self, root=None, rotate_bytes=10*1024*1024, max_parts=10):
        self.started = time.monotonic()
        self.path = Path(root or Path(__file__).resolve().parents[2]/'logs') / (
            datetime.now().strftime('%Y%m%d_%H%M%S_%f')+f'_{os.getpid()}')
        self.rotate_bytes, self.max_parts = rotate_bytes, max_parts
        self.pending = queue.Queue(maxsize=4096)
        self.error, self.dropped, self.written = '', 0, 0
        self.closed = False
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def write(self, kind, **fields):
        entry = {'time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
                 'elapsed_s': round(time.monotonic()-self.started, 3), 'type': kind, **fields}
        if self.closed or self.error:
            return entry
        try:
            self.pending.put_nowait(entry)
        except queue.Full:
            self.dropped += 1
        return entry

    def _run(self):
        try:
            self.path.mkdir(parents=True, exist_ok=False)
            part, size, handle = 1, 0, None
            try:
                handle = (self.path/f'session_{part:03d}.jsonl').open('x', encoding='utf-8')
                while True:
                    try:
                        entry = self.pending.get(timeout=.5)
                    except queue.Empty:
                        handle.flush()
                        continue
                    if entry is None:
                        break
                    line = json.dumps(entry, ensure_ascii=False)+'\n'
                    if size and size+len(line.encode('utf-8')) > self.rotate_bytes:
                        handle.close()
                        part += 1
                        if part > self.max_parts:
                            raise OSError('Đã đạt giới hạn 100 MiB/phiên; mở phiên GUI mới để ghi tiếp')
                        handle = (self.path/f'session_{part:03d}.jsonl').open('x', encoding='utf-8')
                        size = 0
                    handle.write(line)
                    size += len(line.encode('utf-8'))
                    self.written += 1
                    if self.pending.empty():
                        handle.flush()
            finally:
                if handle and not handle.closed:
                    handle.close()
        except OSError as exc:
            self.error = str(exc)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.worker.is_alive():
            try:
                self.pending.put(None, timeout=1)
            except queue.Full:
                self.error = self.error or 'Hàng đợi ghi chưa được lưu hết khi đóng'
            self.worker.join(timeout=2)
            if self.worker.is_alive():
                self.error = self.error or 'Chưa xác nhận đã ghi hết log khi đóng'


class BusCounter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.total = self.baseline = self.window_total = None
        self.window_at = None
        self.rate = None
        self.session_errors = 0

    def update(self, total, now):
        if total < 0:
            raise ValueError('negative bus counter')
        if self.total is None:
            self.total = self.baseline = self.window_total = total
            self.window_at = now
            return None
        if total < self.total:
            self.reset()
            self.update(total, now)
            return {'reset': True}
        self.total = total
        self.session_errors = total-self.baseline
        elapsed = now-self.window_at
        if elapsed >= 1:
            delta = total-self.window_total
            self.rate = delta/elapsed
            self.window_at, self.window_total = now, total
            return {'delta': delta, 'seconds': elapsed, 'rate': self.rate}
        return None


def parse_telemetry(line):
    fields = line.split()
    if len(fields) != 11 or fields[0] != '#T':
        raise ValueError('khung #T phải có đủ 6 motor')
    counters = [int(v) for v in fields[1:5]]
    if any(v < 0 for v in counters):
        raise ValueError('bộ đếm âm')
    motors = []
    for text in fields[5:]:
        f = text.split(',')
        if len(f) != 8 or any(v not in ('0', '1') for v in f[:3]):
            raise ValueError('trường trạng thái motor không hợp lệ')
        status, miss = int(f[3], 16), int(f[7])
        values = [float(v) for v in f[4:7]]
        if not 0 <= status <= 255 or miss < 0 or not all(math.isfinite(v) for v in values):
            raise ValueError('giá trị motor không hợp lệ')
        motors.append(dict(online=f[0]=='1', on=f[1]=='1', moving=f[2]=='1',
                           status=status, pos=values[0], vel=values[1], tgt=values[2], miss=miss))
    return counters, motors


class StatusPoller:
    """Read-only HTTP diagnostics on a separate thread. Never uses console socket."""
    def __init__(self, host, key, mac, generation, events):
        self.host, self.key, self.mac = host, key, mac
        self.generation, self.events = generation, events
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def _run(self):
        while not self.stop_event.is_set():
            result, error = None, ''
            try:
                req = urllib.request.Request(f'http://{self.host}:8081/status',
                                             headers={'X-OTA-Key': self.key})
                with urllib.request.urlopen(req, timeout=1.5) as response:
                    result = json.loads(response.read(32768))
                if result.get('mac', '').lower() != self.mac.lower():
                    raise ValueError('MAC của endpoint chẩn đoán không khớp ESP đã provision')
                if (result.get('firmware') != '3in1-ota' or len(result.get('motors', [])) != 6
                        or not all(isinstance(m, dict) for m in result['motors'])):
                    raise ValueError('endpoint không phải 3in1-ota / thiếu dữ liệu motor')
            except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
                result, error = None, str(exc)
            self.events.put(('diagnostics', (self.generation, result, error)))
            self.stop_event.wait(1)

    def stop(self):
        self.stop_event.set()
