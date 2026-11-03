#!/usr/bin/env python3
"""Run the CPU runtime against a temporary Unix MCU peer; never open UART."""

import argparse
from collections import deque
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from rb_runtime import protocol as p


class McuPeer:
    def __init__(self, path):
        self.socket = socket.socket(socket.AF_UNIX)
        self.socket.settimeout(2.0)
        self.socket.connect(str(path))
        self.parser = p.Parser()
        self.frames = deque()
        self.sequence = 0

    def receive(self, predicate):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            while self.frames:
                frame = self.frames.popleft()
                if predicate(frame):
                    return frame
            data = self.socket.recv(4096)
            if not data:
                raise RuntimeError("runtime disconnected from the mock MCU")
            self.frames.extend(self.parser.feed(data))
        raise TimeoutError("waiting for runtime frame")

    def send(self, kind, payload, corrupt=False):
        self.sequence += 1
        frame = p.Frame(kind, self.sequence, time.monotonic_ns() // 1000, payload)
        raw = frame.encode()
        if corrupt:
            raw = raw[:-1] + bytes([raw[-1] ^ 0x55])
        self.socket.sendall(raw)
        return frame.seq

    def handshake(self):
        hello = self.receive(lambda frame: frame.kind == p.HELLO)
        self.epoch = p.HELLO_DATA.unpack(hello.payload)[0]
        self.send(p.HELLO_ACK, hello.payload)
        self.receive(
            lambda frame: (
                frame.kind == p.STATUS and p.STATUS_DATA.unpack(frame.payload)[1] == 1
            )
        )
        self.sample = p.Telemetry(
            self.epoch,
            7 | p.TARGET_VALID,
            0,
            [0.0] * 3,
            [0.0, 0.0, -1.0],
            [0.0, 0.0, 9.81],
            [2.35706715, 2.35706715, 0.0, 0.0, 0.0, 0.0],
            [0.0] * 6,
            [0.0, 0.0],
            [2.35706715] * 4 + [0.0, 0.0],
        )

    def tick(self):
        seq = self.send(p.TELEMETRY, self.sample.encode())
        frame = self.receive(
            lambda frame: (
                frame.kind == p.TARGET and p.TARGET_DATA.unpack(frame.payload)[1] == seq
            )
        )
        target = p.TARGET_DATA.unpack(frame.payload)
        self.sample.applied_seq = frame.seq
        self.sample.applied_target = list(target[4:10])
        return target


def operator(path, data):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(2.0)
        client.connect(str(path))
        client.sendall(data)
        response = bytearray()
        while b"\n" not in response:
            part = client.recv(512)
            if not part:
                raise RuntimeError("operator command received no result")
            response.extend(part)
        return json.loads(response)


# The shipped model only runs on the board's NPU, so the host smoke test starts the real runtime
# with a stand-in policy. Everything else (UART/socket loop, core, web) is the production code.
FAKE_POLICY_RUNTIME = """
import sys
import rb_runtime.app as app

class FakePolicy:
    def __init__(self, *args, **kwargs):
        self.reset()

    def reset(self):
        self.steps, self.last_ms, self.hidden = 0, 0.1, [0.0] * 512

    def step(self, obs):
        self.steps += 1
        return [0.0] * 6

    def close(self):
        pass

app.Policy = FakePolicy
app.main()
"""


def run_smoke_test(with_web=False):
    checks = []
    with tempfile.TemporaryDirectory(prefix="rb-runtime-smoke-") as temp:
        temp = Path(temp)
        wire, command = temp / "mcu.sock", temp / "command.sock"
        status_dir = temp / "status"
        status_dir.mkdir()
        env = dict(
            os.environ, PYTHONPATH=str(ROOT / "src"), PYTHONDONTWRITEBYTECODE="1"
        )
        with (temp / "runtime.log").open("w+") as log:
            process = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    FAKE_POLICY_RUNTIME,
                    "--socket",
                    str(wire),
                    "--command-socket",
                    str(command),
                    "--status-file",
                    str(status_dir / "status.json"),
                    "--realtime-priority",
                    "0",
                    *(["--web-host", "127.0.0.1", "--web-port", "0",
                       "--web-token-file", str(temp / "web.token")] if with_web else ["--no-web"]),
                ],
                cwd=ROOT,
                env=env,
                stdout=log,
                stderr=log,
            )
            peer = None
            try:
                deadline = time.monotonic() + 10.0
                ready = None
                def ready_event():
                    for line in (temp / "runtime.log").read_text().splitlines():
                        try:
                            event = json.loads(line)
                        except ValueError:
                            continue
                        if event.get("event") == "ready":
                            return event
                    return None
                while not wire.exists() or not command.exists() or (with_web and not ready):
                    if with_web:
                        ready = ready_event()
                    if process.poll() is not None or time.monotonic() > deadline:
                        log.seek(0)
                        raise RuntimeError(log.read())
                    time.sleep(0.01)
                peer = McuPeer(wire)
                peer.handshake()
                assert peer.tick()[2] == p.DISABLED
                checks.append("HELLO/ACK and DISARMED")
                result = operator(command, b"ARM extra\n")
                assert result["accepted"] is False
                assert peer.tick()[2] == p.DISABLED
                checks.append("malformed operator command rejected")
                # A partial local command must not be executed before the line arrives.
                with socket.socket(socket.AF_UNIX) as client:
                    client.settimeout(2.0)
                    client.connect(str(command))
                    client.sendall(b"A")
                    time.sleep(0.001)
                    assert peer.tick()[2] == p.DISABLED
                    client.sendall(b"RM\n")
                    result = json.loads(client.recv(512))
                    assert result["accepted"] and result["mode"] == "HOME"
                # Diagnostics share MCU sequence ordering, but never generate
                # a TARGET or replace the following TELEMETRY source sequence.
                peer.send(p.EXECUTION_DIAG,
                          p.EXECUTION_DIAG_DATA.pack(peer.epoch, 1, *([0] * 22)))
                for _ in range(4):
                    time.sleep(0.005)
                    assert peer.tick()[2] == p.HOME_PD
                checks.append("fragmented local ARM and HOME targets")
                checks.append("optional MCU diagnostics interleaved with HOME samples")
                if with_web:
                    import signal
                    from urllib.request import Request, urlopen
                    # Freeze the actual HTTP process while the real LF core
                    # continues receiving MCU samples. Only temporary sockets.
                    web_pid = ready["web_pid"]
                    assert web_pid and web_pid != process.pid
                    os.kill(web_pid, signal.SIGSTOP)
                    try:
                        for _ in range(8):
                            time.sleep(.005)
                            assert peer.tick()[2] == p.HOME_PD
                    finally:
                        os.kill(web_pid, signal.SIGCONT)
                    checks.append("web process frozen >TTL; control and applied ACK continue")
                    token = (temp / "web.token").read_text().strip()
                    for _ in range(2):
                        request = Request(ready["web"] + "/api/status",
                                          headers={"Authorization": "Bearer " + token})
                        with urlopen(request, timeout=1.0) as response:
                            assert json.load(response)["mode"] == "HOME"
                        assert peer.tick()[2] == p.HOME_PD
                    checks.append("HTTP resumes/reloads without changing HOME or replaying commands")
                peer.send(p.TELEMETRY, peer.sample.encode(), corrupt=True)
                time.sleep(0.005)
                assert peer.tick()[2] == p.HOME_PD
                checks.append("bad CRC ignored")

                def ticking(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        time.sleep(0.005)
                        assert peer.tick()[2] == p.HOME_PD

                # The status file is rewritten once per second; wait for one written after the CRC error.
                ticking(1.3)
                status = json.loads((status_dir / "status.json").read_text())
                assert status["parser_errors"] > 0 and status["mode"] == "HOME"
                assert set(status["link_quality"]) == {"gaps", "lost_frames", "max_gap_us", "max_burst"}
                loop = status["loop_stats"]
                assert loop["reads"] > 20 and sum(loop["handler_ms_histogram"]) > 20
                assert loop["status_write_failures"] == 0
                checks.append("status reports link quality, parser errors and loop timing")
                # The dashboard is expendable: losing the status file must not stop the controller.
                shutil.rmtree(status_dir)
                ticking(1.3)
                assert process.poll() is None
                checks.append("controller keeps running after the status file disappears")
                peer.sample.flags |= p.ESTOP
                time.sleep(0.005)
                assert peer.tick()[2] == p.DISABLED
                checks.append("MCU E-stop disables output")
                assert not operator(command, b"ARM\n")["accepted"]
                checks.append("FAULT remains latched on ARM")
                previous = peer.epoch
                assert operator(command, b"RESET\n")["accepted"]
                peer.handshake()
                assert peer.epoch != previous and peer.tick()[2] == p.DISABLED
                checks.append("RESET creates new epoch and stays DISARMED")
                return {
                    "passed": True,
                    "backend": "stand-in policy (the model needs the board NPU)",
                    "transport": "temporary Unix sockets",
                    "physical_hardware": False,
                    "checks": checks,
                }
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                if peer:
                    peer.socket.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--with-web", action="store_true", help="also check isolated HTTP outage/recovery")
    args = parser.parse_args()
    report = run_smoke_test(args.with_web)
    text = json.dumps(report, indent=2) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text)
    print(text, end="")


if __name__ == "__main__":
    main()
