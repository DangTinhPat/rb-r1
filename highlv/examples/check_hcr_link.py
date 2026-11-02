#!/usr/bin/env python3
"""UART example: continuously send hello and receive im here; Ctrl+C stops."""
import argparse
import math
import os
from pathlib import Path
import select
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from rb_runtime.transport import open_serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyS2")
    parser.add_argument("--timeout", type=float, default=8)
    parser.add_argument("--interval", type=float, default=1, help="seconds between hello messages")
    parser.add_argument("--once", action="store_true", help="exit after the first reply")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    if not math.isfinite(args.interval) or not 0 < args.interval < args.timeout:
        parser.error("--interval must be positive and less than --timeout")
    fd = open_serial(args.port, 921600)
    pending, received = bytearray(), bytearray()
    deadline = time.monotonic() + args.timeout
    next_hello = 0.0
    sent, replies = 0, 0
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if not pending and now >= next_hello:
                pending.extend(b"hello\n")
                next_hello = now + 1
            readable, writable, _ = select.select(
                [fd], [fd] if pending else [], [], 0.05
            )
            if writable:
                try:
                    count = os.write(fd, pending)
                    del pending[:count]
                    if not pending:
                        sent += 1
                        print(f"LF -> ESP: hello [{sent}]", flush=True)
                except BlockingIOError:
                    pass
            if readable:
                received.extend(os.read(fd, 256))
                while b"\n" in received:
                    line, _, rest = received.partition(b"\n")
                    received[:] = rest
                    if line.strip() == b"im here":
                        replies += 1
                        deadline = time.monotonic() + args.timeout
                        print(f"ESP -> LF: im here [{replies}]", flush=True)
                        if args.once:
                            print("PASS: UART hai chieu", flush=True)
                            return
                if len(received) > 256:
                    received.clear()
        raise SystemExit("FAIL: khong nhan duoc 'im here' tu ESP")
    except KeyboardInterrupt:
        print(f"\nDa dung: gui {sent} hello, nhan {replies} im here", flush=True)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
