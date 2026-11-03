#!/usr/bin/env python3
"""Send one operator command to the runtime's local Unix socket."""

import argparse
import json
import socket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command", choices=("ARM", "DISARM", "RESET", "ESTOP", "VELOCITY", "RESET_ENCODERS", "CAL_IMU")
    )
    parser.add_argument("values", nargs="*")
    parser.add_argument("--socket", default="/tmp/rb-hybrid-command.sock")
    args = parser.parse_args()
    if args.command == "VELOCITY":
        if len(args.values) != 2:
            parser.error("VELOCITY requires vx and wz")
        line = "VELOCITY " + " ".join(args.values)
    elif args.values:
        parser.error(f"{args.command} takes no extra values")
    else:
        line = args.command
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(3.0)
        client.connect(args.socket)
        client.sendall((line + "\n").encode("ascii"))
        response = bytearray()
        while b"\n" not in response and len(response) < 512:
            part = client.recv(512 - len(response))
            if not part:
                break
            response.extend(part)
        if not response:
            parser.exit(2, "runtime returned no command result\n")
        result = json.loads(response)
        print(json.dumps(result))
        return 0 if result.get("accepted") else 2


if __name__ == "__main__":
    raise SystemExit(main())
