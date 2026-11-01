"""Small authenticated HTTP dashboard for the LF hybrid runtime."""

import hmac
import json
import os
import secrets
import socket
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
    def __init__(self, host, port, token_file, status_file, command_socket, page_file):
        self.token = load_or_create_token(token_file)
        self.status_file = Path(status_file)
        self.command_socket = command_socket
        self.page = Path(page_file).read_bytes()
        app = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "RBHybridUI/1.0"

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
                    supplied[len(prefix) :], app.token
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
                    command = request.get("command")
                    if command not in ("ARM", "DISARM", "RESET", "ESTOP"):
                        raise ValueError("unsupported command")
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
                    if not response:
                        raise RuntimeError("runtime returned no command result")
                    self._json(200, json.loads(response.split(b"\n", 1)[0].decode()))
                except (
                    OSError,
                    UnicodeError,
                    ValueError,
                    TypeError,
                    json.JSONDecodeError,
                    RuntimeError,
                ) as error:
                    self._json(503, {"accepted": False, "error": str(error)})

            def log_message(self, _format, *_args):
                pass

        class Server(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        self.server = Server((host, port), Handler)
        self.thread = None

    def start(self):
        import threading

        self.thread = threading.Thread(
            target=self.server.serve_forever, name="hybrid-web", daemon=True
        )
        self.thread.start()

    def close(self):
        if self.thread:
            self.server.shutdown()
            self.thread.join(timeout=2.0)
        self.server.server_close()
