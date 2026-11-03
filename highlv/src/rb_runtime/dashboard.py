"""Small authenticated HTTP dashboard for the LF hybrid runtime."""

import hmac
import json
import math
import os
import secrets
import socket
import select
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


def load_or_create_token(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        token = path.read_text().strip()
    except FileNotFoundError:
        token = secrets.token_urlsafe(32)
        fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            stream.write(token + "\n")
    if len(token) < 32:
        raise ValueError("web access token must contain at least 32 characters")
    os.chmod(str(path), 0o600)
    return token


class DashboardServer:
    def __init__(self, host, port, token_file, status_file, command_socket, page_file,
                 *, request_limit=8, request_timeout=2.0):
        self.token = load_or_create_token(token_file)
        self.status_file = Path(status_file)
        self.command_socket = command_socket
        self.page = Path(page_file).read_bytes()
        app = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "RBHybridUI/1.1"

            def setup(self):
                self.request.settimeout(request_timeout)
                super().setup()

            def handle(self):
                # A browser can close/reload during any response. This is an HTTP
                # failure only; it must not terminate the server or issue a command.
                try:
                    super().handle()
                except (OSError, TimeoutError):
                    pass

            def _json(self, code, value):
                body = json.dumps(value, separators=(",", ":")).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _authorized(self):
                supplied = self.headers.get("Authorization", "")
                prefix = "Bearer "
                return supplied.startswith(prefix) and hmac.compare_digest(
                    supplied[len(prefix) :].encode("utf-8"), app.token.encode("utf-8")
                )

            def do_GET(self):
                if self.path == "/" or self.path == "/index.html":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(app.page)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(app.page)
                    return
                if self.path != "/api/status":
                    self._json(404, {"error": "not found"})
                    return
                if not self._authorized():
                    self._json(401, {"error": "access token required"})
                    return
                try:
                    status = json.loads(app.status_file.read_text())
                    age = time.monotonic() - float(status["updated_monotonic"])
                    if not math.isfinite(age):
                        raise ValueError("invalid status timestamp")
                    status["runtime_online"] = 0 <= age < 3.0
                    status["status_age_s"] = max(0.0, age)
                    self._json(200, status)
                except (OSError, ValueError, KeyError, TypeError):
                    self._json(200, {"runtime_online": False, "mode": "OFFLINE"})

            def do_POST(self):
                if self.path != "/api/command":
                    self._json(404, {"error": "not found"})
                    return
                if not self._authorized():
                    self._json(401, {"error": "access token required"})
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 1 <= length <= 256:
                        raise ValueError("invalid request size")
                    request = json.loads(self.rfile.read(length).decode("utf-8"))
                    if not isinstance(request, dict):
                        raise ValueError("command request must be an object")
                    command = request.get("command")
                    if command not in ("ARM", "DISARM", "RESET", "ESTOP", "RESET_ENCODERS", "CAL_IMU"):
                        raise ValueError("unsupported command")
                except (UnicodeError, ValueError, TypeError) as error:
                    self._json(400, {"accepted": False, "error": str(error)})
                    return
                try:
                    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    client.settimeout(2.0)
                    try:
                        client.connect(app.command_socket)
                        client.sendall((command + "\n").encode("ascii"))
                        response = bytearray()
                        while b"\n" not in response and len(response) < 512:
                            chunk = client.recv(512 - len(response))
                            if not chunk:
                                break
                            response.extend(chunk)
                    finally:
                        client.close()
                    if b"\n" not in response:
                        raise RuntimeError("runtime returned no complete command result")
                    result = json.loads(response.split(b"\n", 1)[0].decode())
                    if not isinstance(result, dict) or not isinstance(result.get("accepted"), bool):
                        raise RuntimeError("invalid runtime command result")
                    self._json(200, result)
                except (
                    OSError,
                    UnicodeError,
                    ValueError,
                    TypeError,
                    json.JSONDecodeError,
                    RuntimeError,
                ) as error:
                    # A socket failure after send does not prove non-application. Never retry.
                    self._json(503, {"accepted": False, "confirmed": False,
                                     "error": str(error)})

            def log_message(self, _format, *_args):
                pass

        class Server(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True
            block_on_close = False

            def __init__(self, *args):
                self.slots = threading.BoundedSemaphore(request_limit)
                super().__init__(*args)

            def process_request(self, request, address):
                if not self.slots.acquire(blocking=False):
                    try:
                        request.settimeout(.1)
                        body = b'{"error":"dashboard busy"}'
                        request.sendall(b"HTTP/1.0 503 Service Unavailable\r\n"
                                        b"Content-Type: application/json\r\n"
                                        b"Retry-After: 1\r\nConnection: close\r\n"
                                        + ("Content-Length: %d\r\n\r\n" % len(body)).encode()
                                        + body)
                    except OSError:
                        pass
                    finally:
                        self.shutdown_request(request)
                    return
                try:
                    super().process_request(request, address)
                except BaseException:
                    self.slots.release()
                    raise

            def process_request_thread(self, request, address):
                try:
                    super().process_request_thread(request, address)
                finally:
                    self.slots.release()

        self.server = Server((host, port), Handler)
        self.thread = None

    def start(self):
        self.thread = threading.Thread(
            target=self.server.serve_forever, name="hybrid-web", daemon=True
        )
        self.thread.start()

    def close(self):
        if self.thread:
            self.server.shutdown()
            self.thread.join(timeout=2.0)
        self.server.server_close()


class DashboardProcess:
    """Web is expendable: separate GIL/scheduler, no UART or controller objects.

    Launch with exec rather than fork: RKNN may already own native threads.
    Only a fresh command socket connects the child to the control process.
    """

    def __init__(self, host, port, token_file, status_file, command_socket, page_file):
        self.arguments = ["--host", host, "--port", str(port),
                          "--token-file", str(token_file), "--status-file", str(status_file),
                          "--command-socket", str(command_socket), "--page-file", str(page_file)]
        self.process = None
        self.port = None

    def start(self):
        if self.process is not None:
            raise RuntimeError("dashboard process already started")
        env = os.environ.copy()
        source = str(Path(__file__).resolve().parents[1])
        env["PYTHONPATH"] = source + os.pathsep + env.get("PYTHONPATH", "")
        self.process = subprocess.Popen(
            [sys.executable, "-u", "-m", "rb_runtime.dashboard", *self.arguments],
            env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            close_fds=True,
        )
        try:
            # Startup only, before entering the control loop. Never wait for HTTP
            # from the telemetry path, or restart the web synchronously there.
            if not select.select([self.process.stdout], [], [], 5.0)[0]:
                raise RuntimeError("dashboard process startup timeout")
            result = json.loads(self.process.stdout.readline(1024))
            if "error" in result:
                raise RuntimeError(result["error"])
            self.port = int(result["port"])
            self.process.stdout.close()
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        if self.process.stdout:
            self.process.stdout.close()
        self.process = None


def main():
    import argparse
    import signal

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, required=True)
    for name in ("token-file", "status-file", "command-socket", "page-file"):
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args()
    web = None
    announced = False
    try:
        # The runtime uses SCHED_FIFO. HTTP must not inherit its RT priority.
        os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
        web = DashboardServer(args.host, args.port, args.token_file, args.status_file,
                              args.command_socket, args.page_file)
        def terminate(*_):
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, terminate)
        signal.signal(signal.SIGINT, terminate)
        print(json.dumps({"port": web.server.server_port}), flush=True)
        announced = True
        web.server.serve_forever(poll_interval=.2)
    except Exception as error:
        print(json.dumps({"error": str(error)}),
              file=sys.stderr if announced else sys.stdout, flush=True)
        return 1
    finally:
        if web:
            web.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
