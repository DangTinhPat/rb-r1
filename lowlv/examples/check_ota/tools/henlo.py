#!/usr/bin/env python3
"""Phía máy tính của check_ota: gửi "henlo" tới ESP32 qua TCP, in mọi dòng ESP gửi lại.

    python3 examples/check_ota/tools/henlo.py                    # rb1-check.local:8023
    python3 examples/check_ota/tools/henlo.py --host 192.168.1.50
    python3 examples/check_ota/tools/henlo.py --once             # gửi một lần, kiểm tra rồi thoát

Gõ chữ vào terminal rồi Enter để gửi tuỳ ý sang ESP32 (hiện trên Serial Monitor).
Mã thoát với --once: 0 nếu ESP trả đúng "henlo", 1 nếu không.
"""
import argparse
import socket
import sys
import threading
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="rb1-check.local")
    ap.add_argument("--port", type=int, default=8023)
    ap.add_argument("--interval", type=float, default=1.0, help="giây giữa hai lần gửi henlo")
    ap.add_argument("--once", action="store_true", help="gửi một lần, chờ phản hồi rồi thoát")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    try:
        sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
    except OSError as exc:
        print(f"Không nối được {args.host}:{args.port}: {exc}")
        return 1
    sock.settimeout(None)
    print(f"Đã nối {args.host}:{args.port}")

    replies = []
    got_reply = threading.Event()
    sent_at = {"t": 0.0}

    def receiver() -> None:
        buf = b""
        while True:
            try:
                data = sock.recv(256)
            except OSError:
                return
            if not data:
                print("ESP32 đã đóng kết nối")
                got_reply.set()
                return
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                rtt = (time.monotonic() - sent_at["t"]) * 1000.0
                if text == "henlo":
                    print(f"[RX] {text}   (khứ hồi {rtt:.0f} ms)")
                    replies.append(text)
                    got_reply.set()
                else:
                    print(f"[RX] {text}")

    threading.Thread(target=receiver, daemon=True).start()

    def send(text: str) -> None:
        sent_at["t"] = time.monotonic()
        sock.sendall(text.encode() + b"\n")
        print(f"[TX] {text}")

    if args.once:
        send("henlo")
        ok = got_reply.wait(args.timeout) and replies == ["henlo"]
        print("KẾT QUẢ: " + ("ĐẠT, ESP32 trả lời henlo" if ok else "KHÔNG ĐẠT, không nhận được henlo"))
        sock.close()
        return 0 if ok else 1

    def typer() -> None:
        for line in sys.stdin:
            line = line.strip()
            if line:
                try:
                    send(line)
                except OSError:
                    return

    threading.Thread(target=typer, daemon=True).start()
    try:
        while True:
            send("henlo")
            time.sleep(args.interval)
    except (KeyboardInterrupt, OSError):
        pass
    sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
