#!/usr/bin/env python3
"""Run the RB-R1 hybrid runtime with UART or a local MCU emulator connection."""

import argparse
import bisect
import ctypes
import fcntl
import gc
import json
import os
from pathlib import Path
import select
import signal
import socket
import stat
import sys
import time

from .hybrid_core import HybridCore
from .fold import FoldLogic
from .policy import Policy
from .protocol import Parser, TELEMETRY, ENCODER_HOME_ACK
from .transport import Writer, open_serial
from .dashboard import DashboardProcess
from .paths import PROJECT_ROOT
from .commands import parse_command

ROOT = PROJECT_ROOT


def log(event, **data):
    print(json.dumps({"event": event, **data}), file=sys.stderr, flush=True)


HANDLER_EDGES_MS = (0.25, 0.5, 1, 2, 5, 15)
MCL_CURRENT, MCL_FUTURE = 1, 2


class LoopStats:
    """What the process itself saw. Together with HybridCore.link_quality (MCU clock gaps) this
    separates "the wire lost frames" from "LF was too slow to read them"."""

    def __init__(self):
        self.last_iteration = None
        self.max_iteration_gap_ms = 0.0
        self.stalls_over_10ms = 0
        self.reads = 0
        self.max_frames_per_read = 0
        self.max_read_to_write_ms = 0.0
        self.late_writes_over_5ms = 0
        self.max_status_write_ms = 0.0
        self.status_write_failures = 0
        self.handler_ms_histogram = [0] * (len(HANDLER_EDGES_MS) + 1)

    def iteration(self, now):
        if self.last_iteration is not None:
            gap = (now - self.last_iteration) * 1000
            self.max_iteration_gap_ms = max(self.max_iteration_gap_ms, gap)
            self.stalls_over_10ms += gap > 10
        self.last_iteration = now

    def handler(self, ms):
        self.handler_ms_histogram[bisect.bisect_right(HANDLER_EDGES_MS, ms)] += 1

    def snapshot(self):
        data = dict(vars(self))
        del data["last_iteration"]
        data["handler_ms_edges"] = list(HANDLER_EDGES_MS)
        return data


def lock_memory():
    """A page fault inside the 5 ms tick is a latency spike the scheduler cannot fix."""
    try:
        if ctypes.CDLL(None, use_errno=True).mlockall(MCL_CURRENT | MCL_FUTURE) != 0:
            raise OSError(ctypes.get_errno(), "mlockall failed")
    except (OSError, AttributeError) as error:
        log("memory_not_locked", reason=str(error))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(ROOT / "config/runtime.json"))
    ap.add_argument("--serial", help="physical UART; overrides configured transport")
    ap.add_argument("--socket", help="Unix socket for MCU emulator")
    ap.add_argument(
        "--listen-tcp", type=int, help="loopback-only HIL port, use SSH forwarding"
    )
    ap.add_argument("--backend", choices=["rknn"])
    ap.add_argument("--status-file")
    ap.add_argument(
        "--command-socket",
        default="/tmp/rb-hybrid-command.sock",
        help="local operator command socket; independent of HCR1 UART",
    )
    ap.add_argument(
        "--web-host",
        default="127.0.0.1",
        help="dashboard bind address; default is loopback for SSH forwarding",
    )
    ap.add_argument("--web-port", type=int, default=8080)
    ap.add_argument("--web-token-file", default="/tmp/rb-hybrid-web.token")
    ap.add_argument(
        "--no-web", action="store_true", help="disable the operator dashboard"
    )
    ap.add_argument("--realtime-priority", type=int)
    args = ap.parse_args()
    cfg = json.loads(Path(args.config).read_text())
    if args.backend:
        cfg["backend"] = args.backend
    if args.serial:
        cfg["transport"], cfg["serial_port"] = "serial", args.serial
    if args.socket:
        cfg["transport"], cfg["socket"] = "unix", args.socket
    if args.status_file:
        cfg["status_file"] = args.status_file
    if args.realtime_priority is not None:
        cfg["realtime_priority"] = args.realtime_priority
    if not (
        0 <= cfg["home_hold_s"] < cfg["warmup_start_s"] < cfg["handoff_earliest_s"]
    ):
        raise ValueError("invalid phase timings")
    if cfg["warmup_steps"] < 50 or cfg["inference_deadline_ms"] > 10:
        raise ValueError(
            "policy requires at least 50 warm-up ticks and a 10 ms deadline"
        )
    if (
        not 10 <= cfg["inference_hard_deadline_ms"] <= 15
        or not 1 <= cfg["max_consecutive_deadline_misses"] <= 3
    ):
        raise ValueError("invalid bounded jitter budget")
    lock = open(cfg["status_file"] + ".lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    listener = connection = command_listener = command_connection = web = None
    command_buffer = bytearray()
    command_deadline = 0.0
    fd = None
    policy = logic = None
    stop = False

    def terminate(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    try:
        priority = cfg.get("realtime_priority", 0)
        if priority:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
            lock_memory()
        policy = Policy(ROOT, cfg["backend"])
        logic = FoldLogic()
        core = HybridCore(policy, logic, cfg)
        # Control loop owns no growing Python reference cycles; avoid periodic GC stalls.
        gc.collect()
        gc.disable()
        if args.listen_tcp is not None:
            listener = socket.socket()
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", args.listen_tcp))
        elif cfg["transport"] == "unix":
            path = Path(cfg["socket"])
            if path.exists():
                if not stat.S_ISSOCK(path.stat().st_mode):
                    raise RuntimeError("socket path occupied by a regular file")
                probe = socket.socket(socket.AF_UNIX)
                try:
                    probe.connect(str(path))
                except ConnectionRefusedError:
                    path.unlink()
                else:
                    raise RuntimeError("socket is already served")
                finally:
                    probe.close()
            listener = socket.socket(socket.AF_UNIX)
            listener.bind(str(path))
            os.chmod(path, 0o600)
        elif cfg["transport"] == "serial" and cfg["serial_port"]:
            fd = open_serial(cfg["serial_port"], cfg["baud"])
        else:
            raise ValueError(
                "UART port must be configured before selecting serial transport"
            )
        command_path = Path(args.command_socket)
        if command_path.exists():
            if not stat.S_ISSOCK(command_path.stat().st_mode):
                raise RuntimeError("command socket path occupied by a regular file")
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                probe.connect(str(command_path))
            except ConnectionRefusedError:
                command_path.unlink()
            else:
                raise RuntimeError("operator command socket is already served")
            finally:
                probe.close()
        command_listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        command_listener.bind(str(command_path))
        os.chmod(command_path, 0o600)
        command_listener.listen(2)
        command_listener.setblocking(False)
        if not args.no_web:
            web = DashboardProcess(
                args.web_host,
                args.web_port,
                args.web_token_file,
                cfg["status_file"],
                args.command_socket,
                ROOT / "web/index.html",
            )
            try:
                web.start()
            except (OSError, ValueError, RuntimeError) as error:
                log("web_unavailable", reason=str(error))
                web.close()
                web = None
        if listener:
            listener.listen(1)
            listener.setblocking(False)
        writer = Writer(fd) if fd is not None else None
        parser = Parser()
        stats = LoopStats()
        last_hello = last_status = 0.0
        last_mode = None
        max_control_ms = 0.0
        max_telemetry_ms = 0.0
        transport_name = (
            "loopback-tcp" if args.listen_tcp is not None else cfg["transport"]
        )
        log(
            "ready",
            backend=cfg["backend"],
            transport=transport_name,
            web=None if web is None else f"http://{args.web_host}:{web.port}",
            web_pid=None if web is None else web.process.pid,
            token_file=None if args.no_web else args.web_token_file,
        )
        while not stop:
            now = time.monotonic()
            stats.iteration(now)
            if command_connection and now >= command_deadline:
                command_connection.close()
                command_connection = None
                command_buffer.clear()
            if writer:
                try:
                    writer.add(core.watchdog(now))
                    if not core.link and now - last_hello >= 1:
                        writer.add([core.hello()])
                        last_hello = now
                    if now - last_status >= 1:
                        writer.add([core.status()])
                    writer.flush()
                except (OSError, RuntimeError) as error:
                    core.fail("transport_fault", str(error))
                    if listener:
                        connection.close()
                        connection = None
                        fd = None
                        writer = None
                    else:
                        raise
            if now - last_status >= 1 or core.mode != last_mode:
                snapshot = core.snapshot()
                snapshot.update(
                    transport=transport_name,
                    web_process_alive=web is not None and web.process.poll() is None,
                    parser_errors=parser.errors,
                    max_handler_ms=max_control_ms,
                    max_telemetry_ms=max_telemetry_ms,
                    loop_stats=stats.snapshot(),
                    updated_monotonic=now,
                )
                # The dashboard is expendable: a full or missing /tmp must not stop the controller.
                try:
                    temp = Path(cfg["status_file"] + ".tmp")
                    temp.write_text(json.dumps(snapshot, indent=2) + "\n")
                    temp.replace(cfg["status_file"])
                except OSError as error:
                    if stats.status_write_failures % 60 == 0:
                        log("status_write_failed", reason=str(error))
                    stats.status_write_failures += 1
                stats.max_status_write_ms = max(stats.max_status_write_ms, (time.monotonic() - now) * 1000)
                if core.mode != last_mode:
                    log("mode", **snapshot)
                last_mode = core.mode
                last_status = now
            reads = [command_listener]
            if fd is not None:
                reads.append(fd)
            elif listener:
                reads.append(listener)
            if command_connection:
                reads.append(command_connection)
            ready, _, _ = select.select(reads, [], [], 0.005)
            if not ready:
                continue
            for ready_fd in ready:
                if ready_fd is command_listener:
                    client, _ = command_listener.accept()
                    if command_connection:
                        client.close()
                    else:
                        command_connection = client
                        command_connection.setblocking(False)
                        command_buffer.clear()
                        command_deadline = time.monotonic() + 1.0
                    continue
                if ready_fd is command_connection:
                    try:
                        data = command_connection.recv(129)
                        command_buffer.extend(data)
                        if len(command_buffer) > 128:
                            raise ValueError("command exceeds 128 bytes")
                        if b"\n" not in command_buffer and data:
                            continue
                        if command_buffer:
                            line, _, tail = command_buffer.partition(b"\n")
                            if tail.strip():
                                raise ValueError("send one command per connection")
                            op, vx, wz = parse_command(line)
                            if op == "RESET_ENCODERS":
                                last_status = 0.0
                            rejected = core.counters["rejected"]
                            reply = (
                                core.estop()
                                if op == "ESTOP"
                                else core.command(op, time.monotonic(), vx, wz)
                            )
                            if writer:
                                writer.add(reply)
                                writer.flush()
                            accepted = core.counters["rejected"] == rejected
                            command_connection.sendall(
                                (
                                    json.dumps(
                                        {
                                            "accepted": accepted,
                                            "mode": core.mode,
                                            "fault": core.reason,
                                            "error": core.last_command_error
                                            if not accepted
                                            else None,
                                        }
                                    )
                                    + "\n"
                                ).encode()
                            )
                        command_connection.close()
                        command_connection = None
                    except (OSError, UnicodeError, ValueError, IndexError) as error:
                        try:
                            command_connection.sendall(
                                (
                                    json.dumps({"accepted": False, "error": str(error)})
                                    + "\n"
                                ).encode()
                            )
                        except OSError:
                            pass
                        command_connection.close()
                        command_connection = None
                    continue
                if fd is not None and ready_fd == fd:
                    try:
                        data = os.read(fd, 4096)
                        if not data:
                            raise ConnectionError("MCU connection closed")
                        read_at = time.monotonic()
                        frames = parser.feed(data)
                        stats.reads += 1
                        stats.max_frames_per_read = max(stats.max_frames_per_read, len(frames))
                        for frame in frames:
                            begin = time.monotonic()
                            reply = core.receive(frame, begin)
                            duration = (time.monotonic() - begin) * 1000
                            max_control_ms = max(max_control_ms, duration)
                            if frame.kind == TELEMETRY:
                                max_telemetry_ms = max(max_telemetry_ms, duration)
                                stats.handler(duration)
                            if (
                                frame.kind == TELEMETRY
                                and duration > 15
                                and core.mode in ("HOME", "STANDUP", "WARMUP", "MODEL")
                            ):
                                reply = [
                                    core.fail(
                                        "control_deadline",
                                        f"telemetry handler {duration:.3f} ms exceeds 15 ms",
                                    ),
                                    core.status(),
                                ]
                            if frame.kind == ENCODER_HOME_ACK:
                                last_status = 0.0
                            # Send each reply as soon as it exists, so a later frame in the same
                            # read never delays an earlier target.
                            writer.add(reply)
                            writer.flush()
                            sent_ms = (time.monotonic() - read_at) * 1000
                            stats.max_read_to_write_ms = max(stats.max_read_to_write_ms, sent_ms)
                            if frame.kind == TELEMETRY and sent_ms > 5:
                                stats.late_writes_over_5ms += 1
                    except (OSError, RuntimeError) as error:
                        if core.mode in ("DISARMED", "WAIT_LINK"):
                            core.new_epoch()
                        else:
                            core.fail("transport_fault", str(error))
                        if listener:
                            connection.close()
                            connection = None
                            fd = None
                            writer = None
                        else:
                            raise
                elif fd is None and listener is not None and ready_fd is listener:
                    connection, _ = listener.accept()
                    connection.setblocking(False)
                    fd = connection.fileno()
                    writer = Writer(fd)
                    parser = Parser()
                    core.new_epoch()
                    last_hello = 0.0
        if writer:
            writer.add([core.fail("transport_fault", "service stopping")])
            writer.flush()
    finally:
        if web:
            web.close()
        if connection:
            connection.close()
        elif fd is not None:
            os.close(fd)
        if listener:
            listener.close()
        if command_connection:
            command_connection.close()
        if command_listener:
            command_listener.close()
            try:
                command_path.unlink()
            except FileNotFoundError:
                pass
        if policy:
            policy.close()
        lock.close()


if __name__ == "__main__":
    main()
